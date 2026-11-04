"""
kv260_view_tracker.py

view_tracker.py for the KV260. Same tracker (tracker.py), same drawing
(view_tracker.render), but:
  - frames come from the FPGA through kv260Frames.FrameGrabber
  - the "window" is a live web page, because the Kria has no screen

    sudo python3 kv260_view_tracker.py --ball-radius 9
    python3 kv260_view_tracker.py --fake                  # no hardware

  then on the laptop:
    ssh -L 8080:localhost:8080 <user>@<kria-ip>           # keep it open
    open http://localhost:8080

Keys on the page are the same as view_tracker: b = learn background (clear the
table first), r = reset, m = mask view. The thresh slider works live. The
final detector and tracker configs are printed when you stop it (Ctrl+C).

Threads:
    grabber thread   FPGA handshake + BRAM copy   (kv260Frames)
    tracking thread  trk.update() on EVERY frame, in order
    render thread    render() + JPEG at --display-fps, so drawing never slows tracking
    web threads      one per browser connection
"""

import argparse
import collections
import math
import threading
import time
import types
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

from kv260Frames import FakeSource, FpgaSource, FrameGrabber, parse_crop
from tracker import BallDetector, BallTracker, DetectorConfig, TrackerConfig, SEARCH
from view_tracker import learn_background, render


class Live:
    """Everything the threads share."""

    def __init__(self):
        self.lock = threading.Lock()
        self.cond = threading.Condition(self.lock)
        self.snap = None              # latest state for the render thread
        self.jpeg = None              # latest rendered picture
        self.jpeg_id = 0
        self.commands = collections.deque()   # keys pressed on the web page
        self.running = True


# ============================================================================
# Tracking thread: the same loop as view_tracker.main(), minus the window
# ============================================================================

def tracking_loop(grabber, det, trk, live, args):
    trail = collections.deque(maxlen=args.trail)
    proc = collections.deque(maxlen=400)
    last_gray, last_state, show_mask = None, None, False

    while live.running:
        # keys from the web page
        while live.commands:
            k = live.commands.popleft()
            if k == "b":
                learn_background(grabber, det)
                trk.reset()
                trail.clear()
            elif k == "r":
                trk.reset()
                trail.clear()
            elif k == "m":
                show_mask = not show_mask

        ready = det.cfg.mode != "diff" or det.background is not None
        for _seq, stamp, gray in grabber.wait_frames(0.05):
            last_gray = gray
            if not ready:
                continue
            t0 = time.perf_counter()
            last_state = trk.update(gray, stamp)
            proc.append((time.perf_counter() - t0) * 1000.0)
            if last_state.state == SEARCH:
                trail.clear()
            elif last_state.x is not None:
                trail.append((last_state.x, last_state.y))
            if args.log_file:
                s = last_state
                args.log_file.write(f"{stamp:.6f},{s.state},{s.x},{s.y},{s.vx},{s.vy},{int(s.measured)}\n")

        if last_gray is None:
            continue

        # A frozen copy of what render() reads from the tracker, so the render
        # thread never sees the tracker half-way through an update.
        ahead = trk.lookahead(0.020)
        view = types.SimpleNamespace(last_roi=trk.last_roi, last_cands=list(trk.last_cands),
                                     cfg=trk.cfg, lookahead=lambda _sec, a=ahead: a)
        with grabber.lock:
            iv = list(grabber.live_intervals)
            dropped = grabber.dropped
        info = {"cam_fps": 1000.0 / np.mean(iv) if iv else 0.0,
                "proc_ms": float(np.mean(proc)) if proc else 0.0,
                "dropped": dropped}
        with live.lock:
            live.snap = (last_gray, last_state, view, list(trail), info, show_mask, ready)


# ============================================================================
# Render thread: draw + JPEG at display_fps
# ============================================================================

def render_loop(det, grabber, live, args):
    period = 1.0 / args.display_fps
    while live.running:
        t0 = time.monotonic()
        with live.lock:
            snap = live.snap
        if snap is not None:
            gray, s, view, trail, info, show_mask, ready = snap
            img = render(gray, s, view, det, trail, info, args.scale, show_mask, ready)
            if grabber.error:
                cv2.putText(img, "FPGA: " + grabber.error[:60], (8, 80),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (60, 60, 255), 1, cv2.LINE_AA)
            ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality])
            if ok:
                with live.cond:
                    live.jpeg = jpg.tobytes()
                    live.jpeg_id += 1
                    live.cond.notify_all()
        time.sleep(max(0.0, period - (time.monotonic() - t0)))


# ============================================================================
# Web page
# ============================================================================

PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>FoosPGA Tracker</title>
<style>
  :root { --bg:#101214; --panel:#1a1d21; --line:#2b3036; --text:#e7eaee; --muted:#8c95a0; }
  body { margin:0; background:var(--bg); color:var(--text);
         font:14px/1.45 system-ui,-apple-system,"Segoe UI",sans-serif; }
  main { max-width:1300px; margin:0 auto; padding:16px; }
  h1 { font-size:18px; font-weight:600; margin:0 0 12px; }
  img { width:100%; height:auto; max-height:78vh; object-fit:contain; display:block; background:#000;
        border:1px solid var(--line); border-radius:6px; image-rendering:pixelated; }
  .bar { display:flex; flex-wrap:wrap; gap:10px 18px; align-items:center; margin-top:12px;
         background:var(--panel); border:1px solid var(--line); border-radius:6px; padding:10px 12px; }
  label { display:flex; align-items:center; gap:8px; color:var(--muted); }
  input[type=range] { width:200px; }
  button { background:#2a3038; color:var(--text); border:1px solid #3a414b; border-radius:5px;
           padding:6px 10px; font:inherit; cursor:pointer; }
  button:hover { background:#343b45; }
  kbd { font:12px ui-monospace,monospace; border:1px solid #3a414b; border-radius:3px; padding:0 4px; }
</style></head>
<body><main>
  <h1>FoosPGA tracker &middot; KV260</h1>
  <img src="/stream" alt="live tracker view">
  <div class="bar">
    <button data-k="b"><kbd>b</kbd> learn background</button>
    <button data-k="r"><kbd>r</kbd> reset tracker</button>
    <button data-k="m"><kbd>m</kbd> mask view</button>
    <label>thresh <input id="t" type="range" min="0" max="255" value="__THRESH__">
      <output id="tv">__THRESH__</output></label>
    <span style="color:var(--muted)">mode: __MODE__</span>
  </div>
</main>
<script>
  const send = (k) => fetch("/cmd?k=" + k);
  document.querySelectorAll("button[data-k]").forEach(b => b.onclick = () => send(b.dataset.k));
  document.addEventListener("keydown", e => { if ("brm".includes(e.key)) send(e.key); });
  const t = document.getElementById("t"), tv = document.getElementById("tv");
  t.oninput = () => { tv.textContent = t.value; fetch("/set?thresh=" + t.value); };
</script></body></html>"""


def make_handler(live, det, args):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def small(self, body, ctype):
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            url = urlparse(self.path)
            q = parse_qs(url.query)
            if url.path == "/":
                page = (PAGE.replace("__THRESH__", str(det.cfg.thresh))
                            .replace("__MODE__", det.cfg.mode))
                self.small(page.encode(), "text/html; charset=utf-8")
            elif url.path == "/cmd":
                k = q.get("k", [""])[0]
                if k in ("b", "r", "m"):
                    live.commands.append(k)
                self.small(b"ok", "text/plain")
            elif url.path == "/set":
                if "thresh" in q:
                    det.cfg.thresh = max(0, min(255, int(q["thresh"][0])))
                self.small(b"ok", "text/plain")
            elif url.path == "/snapshot.jpg":
                with live.lock:
                    jpg = live.jpeg
                if jpg:
                    self.small(jpg, "image/jpeg")
                else:
                    self.send_error(503)
            elif url.path == "/stream":
                self.stream()
            else:
                self.send_error(404)

        def stream(self):
            # MJPEG: one long response, each part is a new JPEG. Browsers show
            # it in a plain <img> tag.
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            last = 0
            try:
                while live.running:
                    with live.cond:
                        live.cond.wait_for(lambda: live.jpeg_id != last, timeout=1.0)
                        jpg, last = live.jpeg, live.jpeg_id
                    if jpg is None:
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                     b"Content-Length: %d\r\n\r\n" % len(jpg))
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass                     # browser tab closed

    return Handler


# ============================================================================
# main
# ============================================================================

def main():
    ap = argparse.ArgumentParser(description="Live tracker viewer for the KV260 (web page)")
    ap.add_argument("--fake", action="store_true", help="no hardware: fake bouncing ball")
    ap.add_argument("--gpio-base", type=lambda s: int(s, 0), default=0xB0020000)
    ap.add_argument("--bram-base", type=lambda s: int(s, 0), default=0xB0040000)
    ap.add_argument("--safe-copy", action="store_true", help="use if you get 'Bus error'")
    ap.add_argument("--crop", metavar="X0:X1",
                    help="keep only columns X0..X1, e.g. 200:480; --bounds is then in cropped pixels")
    ap.add_argument("--host", default="127.0.0.1",
                    help="127.0.0.1 = only through the SSH tunnel; 0.0.0.0 = whole network")
    ap.add_argument("--port", type=int, default=8080)
    # same tracker options as view_tracker.py
    ap.add_argument("--mode", choices=["diff", "bright", "dark"], default="diff")
    ap.add_argument("--thresh", type=int, default=30)
    ap.add_argument("--min-area", type=int, default=12)
    ap.add_argument("--max-area", type=int, default=400)
    ap.add_argument("--ball-radius", type=float, default=5.0, help="ball radius in pixels")
    ap.add_argument("--bounds", help="x0,y0,x1,y1: region the ball CENTER can reach")
    ap.add_argument("--px-scale", type=float, default=1.0,
                    help="multiply TrackerConfig's pixel distances (gate, ROI, reacquire pad); "
                         "~2 if the ball looks twice as big as it did at 320x240")
    ap.add_argument("--scale", type=int, default=1, help="display magnification (640x400 already)")
    ap.add_argument("--display-fps", type=float, default=30.0)
    ap.add_argument("--jpeg-quality", type=int, default=80)
    ap.add_argument("--trail", type=int, default=120)
    ap.add_argument("--log", help="CSV of every tracker output: stamp,state,x,y,vx,vy,measured")
    args = ap.parse_args()

    bounds = tuple(float(v) for v in args.bounds.split(",")) if args.bounds else None
    crop = parse_crop(args.crop)
    src = FakeSource(crop=crop) if args.fake else \
        FpgaSource(args.gpio_base, args.bram_base, args.safe_copy, crop)
    grabber = FrameGrabber(src)
    if not grabber.first_frame.wait(5.0):
        print("no frames within 5 s:", grabber.error)
        grabber.release()
        return 1
    print(f"frames: {grabber.w} x {grabber.h} from {'fake source' if args.fake else 'FPGA'}")

    det = BallDetector(DetectorConfig(mode=args.mode, thresh=args.thresh,
                                      min_area=args.min_area, max_area=args.max_area))
    base = TrackerConfig()
    trk = BallTracker(det, TrackerConfig(
        ball_area=math.pi * args.ball_radius ** 2,
        gate_min_px=base.gate_min_px * args.px_scale,
        reacquire_pad_px=int(round(base.reacquire_pad_px * args.px_scale)),
        roi_min_px=int(round(base.roi_min_px * args.px_scale))), bounds=bounds)

    args.log_file = open(args.log, "w", buffering=1) if args.log else None
    if args.log_file:
        args.log_file.write("stamp,state,x,y,vx,vy,measured\n")

    live = Live()
    threading.Thread(target=tracking_loop, args=(grabber, det, trk, live, args), daemon=True).start()
    threading.Thread(target=render_loop, args=(det, grabber, live, args), daemon=True).start()

    server = ThreadingHTTPServer((args.host, args.port), make_handler(live, det, args))
    server.daemon_threads = True
    print(f"live view on http://{args.host}:{args.port}   (Ctrl+C to stop)")
    if args.host == "127.0.0.1" and not args.fake:
        print(f"laptop: ssh -L {args.port}:localhost:{args.port} <user>@<kria-ip>, "
              f"then open http://localhost:{args.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        live.running = False
        grabber.release()
        print()
        print(det.cfg)
        print(trk.cfg)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
