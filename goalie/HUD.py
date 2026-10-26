"""
HUD.py

Everything the viewer draws and serves: the dashboard (camera view with
tracking overlays beside a live stats panel), the Renderer thread that builds
it, the local OpenCV window, the MJPEG browser stream, and the key map both
viewers share.

Nothing here touches tracking state except through the locks the tracker and
grabber expose, so the viewer can never block detection.
"""

import collections
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

from cameraSetup import EXPOSURE_NAMES, set_ctrl


# ============================================================================
# Dashboard rendering
# ============================================================================

PANEL_W = 330
VIEW_H = 480
FONT = cv2.FONT_HERSHEY_SIMPLEX

C_BG = (30, 26, 26)
C_TXT = (225, 225, 225)
C_DIM = (140, 140, 140)
C_HEAD = (240, 190, 110)
C_OK = (110, 210, 120)
C_WARN = (0, 170, 255)
C_BAD = (70, 70, 235)
C_ROI = (230, 190, 0)
C_FRAME = (70, 70, 70)


def _txt(img, text, org, color=C_TXT, scale=0.42, thick=1):
    cv2.putText(img, text, org, FONT, scale, color, thick, cv2.LINE_AA)


def _label_box(img, text, org, color, scale=0.5):
    (tw, th), _ = cv2.getTextSize(text, FONT, scale, 1)
    x, y = org
    cv2.rectangle(img, (x - 6, y - th - 7), (x + tw + 6, y + 7), (0, 0, 0), -1)
    _txt(img, text, (x, y), color, scale)


def _med(v):
    return float(np.median(v)) if len(v) else 0.0


def _rate_color(actual, target):
    if target <= 0:
        return C_TXT
    r = actual / target
    return C_OK if r >= 0.95 else C_WARN if r >= 0.8 else C_BAD


def _ms_color(v, budget):
    return C_OK if v < 0.5 * budget else C_WARN if v < budget else C_BAD


def _sparkline(img, x, y, w, h, vals, ymax, ref=None):
    cv2.rectangle(img, (x, y), (x + w, y + h), C_FRAME, 1)
    if ref is not None and ref < ymax:
        yy = y + h - 1 - int(ref / ymax * (h - 3))
        cv2.line(img, (x + 1, yy), (x + w - 1, yy), (105, 105, 105), 1)
    if len(vals) < 2:
        return
    v = np.asarray(vals[-(w - 2):], np.float32)
    xs = (x + 1 + np.linspace(0, w - 3, len(v))).astype(np.int32)
    ys = (y + h - 1 - np.minimum(v, ymax) / ymax * (h - 3)).astype(np.int32)
    pts = np.stack([xs, ys], axis=1).reshape(-1, 1, 2)
    cv2.polylines(img, [pts], False, C_OK, 1, cv2.LINE_AA)
    for xi in xs[v > ymax]:                     # clipped stalls: red ticks
        cv2.line(img, (int(xi), y + 1), (int(xi), y + 7), C_BAD, 2)


def _histogram(img, x, y, w, h, gray, thr):
    cv2.rectangle(img, (x, y), (x + w, y + h), C_FRAME, 1)
    small = np.ascontiguousarray(gray[::4, ::4])
    hist = np.log1p(cv2.calcHist([small], [0], None, [64], [0, 256]).ravel())
    top = hist.max() if hist.max() > 0 else 1.0
    bw = (w - 2) / 64.0
    for i, hv in enumerate(hist):
        bh = int(hv / top * (h - 3))
        if bh <= 0:
            continue
        xa = int(x + 1 + i * bw)
        xb = max(xa, int(x + 1 + (i + 1) * bw) - 1)
        col = C_OK if i * 4 + 3 > thr else (150, 150, 150)
        cv2.rectangle(img, (xa, y + h - 1 - bh), (xb, y + h - 1), col, -1)
    tx = int(x + 1 + thr / 256.0 * (w - 2))
    cv2.line(img, (tx, y), (tx, y + h), C_WARN, 1)


def _level_bar(img, x, y, w, h, mean, peak, thr):
    cv2.rectangle(img, (x, y), (x + w, y + h), C_FRAME, 1)
    pw = int(peak / 255.0 * (w - 2))
    if pw > 0:
        cv2.rectangle(img, (x + 1, y + 1), (x + 1 + pw, y + h - 1),
                      C_OK if peak > thr else C_BAD, -1)
    mx = x + 1 + int(mean / 255.0 * (w - 2))
    cv2.line(img, (mx, y - 2), (mx, y + h + 2), C_TXT, 1)
    tx = x + 1 + int(thr / 255.0 * (w - 2))
    cv2.line(img, (tx, y - 3), (tx, y + h + 3), C_WARN, 2)


def draw_dashboard(gray, st, trail, iv, dec, det, age, hits, lt, skipped, s, meta):
    """Camera view with overlays on the left, live stats panel on the right."""
    period = meta["period_ms"]

    if gray is None:
        canvas = np.full((VIEW_H, 640 + PANEL_W, 3), C_BG, np.uint8)
        _txt(canvas, "waiting for frames...", (20, VIEW_H // 2), C_DIM, 0.7)
        return canvas

    # ---- camera view ---------------------------------------------------------
    h, w = gray.shape[:2]
    sc = VIEW_H / h
    cam_w = int(round(w * sc))
    if sc != 1:
        interp = cv2.INTER_NEAREST if sc > 1 else cv2.INTER_AREA
        cam = cv2.resize(gray, (cam_w, VIEW_H), interpolation=interp)
    else:
        cam = gray
    cam = cv2.cvtColor(cam, cv2.COLOR_GRAY2BGR)

    if s.show_mask:
        mask = (gray > s.threshold).astype(np.uint8)
        if sc != 1:
            mask = cv2.resize(mask, (cam_w, VIEW_H), interpolation=cv2.INTER_NEAREST)
        overlay = cam.copy()
        overlay[mask.astype(bool)] = (60, 60, 255)
        cam = cv2.addWeighted(overlay, 0.55, cam, 0.45, 0)

    if s.show_trail and len(trail) > 1:
        n = len(trail)
        for i in range(1, n):
            p, q = trail[i - 1], trail[i]
            if p is None or q is None:
                continue
            f = i / n
            cv2.line(cam, (int(p[0] * sc), int(p[1] * sc)), (int(q[0] * sc), int(q[1] * sc)),
                     (0, int(110 + 130 * f), int(255 * f)), 2, cv2.LINE_AA)

    if st.get("have_track"):
        roi = st.get("roi")
        if roi:
            x0, y0, x1, y1 = roi
            cv2.rectangle(cam, (int(x0 * sc), int(y0 * sc)), (int(x1 * sc), int(y1 * sc)),
                          C_ROI, 1)
        c = (int(st["cx"] * sc), int(st["cy"] * sc))
        rr = max(3, int(st["r"] * sc))
        cv2.circle(cam, c, rr + 3, C_OK, 2, cv2.LINE_AA)
        cv2.drawMarker(cam, c, C_OK, cv2.MARKER_CROSS, 8, 1)
        vx, vy = st["vx"], st["vy"]
        if vx or vy:
            e = (int((st["cx"] + vx * 0.03) * sc), int((st["cy"] + vy * 0.03) * sc))
            cv2.arrowedLine(cam, c, e, C_BAD, 2, cv2.LINE_AA, tipLength=0.25)

    label, col = ("TRACKING", C_OK) if st.get("have_track") else ("NO BALL", C_BAD)
    if s.paused:
        label += "  |  PAUSED"
    _label_box(cam, label, (14, 26), col)
    if s.show_mask:
        _label_box(cam, f"mask: pixels > {s.threshold}", (14, 56), (90, 90, 255), 0.45)
    if s.message and time.monotonic() < s.message_until:
        _label_box(cam, s.message, (14, VIEW_H - 16), C_TXT, 0.5)

    # ---- stats panel ---------------------------------------------------------
    panel = np.full((VIEW_H, PANEL_W, 3), C_BG, np.uint8)
    x1, x2 = 12, 112
    y = 24
    _txt(panel, "FoosPGA vision", (x1, y), C_HEAD, 0.6)
    y += 8

    def section(title):
        nonlocal y
        y += 18
        _txt(panel, title, (x1, y), C_HEAD)

    def row(label, value, color=C_TXT):
        nonlocal y
        y += 15
        _txt(panel, label, (x1 + 6, y), C_DIM)
        _txt(panel, value, (x2, y), color)

    tail = iv[-200:]
    cap_fps = 1000.0 / float(np.mean(tail)) if tail else 0.0
    proc_fps = ((len(lt) - 1) / (lt[-1] - lt[0])
                if len(lt) >= 2 and lt[-1] > lt[0] else 0.0)
    det_rate = 100.0 * sum(hits) / len(hits) if hits else 0.0
    med_dec, med_det, med_age = _med(dec), _med(det), _med(age)

    section("CAMERA")
    row("capture", f"{cap_fps:6.1f} fps  / {meta['fps']}", _rate_color(cap_fps, meta["fps"]))
    row("format", meta["format"])
    row("exposure", f"{s.exp_units * 100} us    gain {s.gain}",
        C_BAD if s.exp_units * 0.1 > period else C_TXT)
    row("threshold", f"{s.threshold}")

    section("PIPELINE")
    row("process", f"{proc_fps:6.1f} fps  skip {skipped}", _rate_color(proc_fps, meta["fps"]))
    row("decode", f"{med_dec:5.2f} ms", _ms_color(med_dec, period))
    row("detect", f"{med_det:5.2f} ms", _ms_color(med_det, period))
    row("frame age", f"{med_age:5.2f} ms", _ms_color(med_age, 2 * period))

    section("TRACK")
    row("detected", f"{det_rate:5.1f} %  (recent)",
        C_OK if det_rate >= 90 else C_WARN if det_rate >= 50 else C_BAD)
    if st.get("have_track"):
        spd = (st["vx"] ** 2 + st["vy"] ** 2) ** 0.5
        row("position", f"({st['cx']:5.1f}, {st['cy']:5.1f}) px")
        row("radius", f"{st['r']:4.1f} px   {spd:5.0f} px/s")
    else:
        row("position", "--", C_DIM)
        row("radius", "--", C_DIM)

    mean = float(cv2.mean(gray)[0])
    peak = float(cv2.minMaxLoc(gray)[1])
    section("BRIGHTNESS")
    y += 15
    _txt(panel, f"mean {mean:4.0f}   peak {peak:4.0f}   thr {s.threshold}", (x1 + 6, y),
         C_OK if peak > s.threshold else C_BAD)
    y += 8
    bar_w = PANEL_W - 2 * x1 - 6
    _level_bar(panel, x1 + 6, y, bar_w, 10, mean, peak, s.threshold)
    y += 10

    y += 20
    _txt(panel, f"capture interval   med {_med(iv):.2f} ms", (x1, y), C_HEAD)
    y += 6
    _sparkline(panel, x1 + 6, y, bar_w, 36, iv, max(2.5 * period, 1.0), ref=period)
    y += 36

    y += 18
    _txt(panel, "histogram (log)   green = above thr", (x1, y), C_HEAD)
    y += 6
    _histogram(panel, x1 + 6, y, bar_w, 40, gray, s.threshold)

    _txt(panel, "q quit  m mask  p pause  t trail  s snap  r reset",
         (x1, VIEW_H - 26), C_DIM, 0.36)
    _txt(panel, "+/- thresh    e/E exposure    g/G gain",
         (x1, VIEW_H - 10), C_DIM, 0.36)

    return np.hstack([cam, panel])


class Renderer:
    """Builds dashboard frames at a fixed rate in its own thread."""

    def __init__(self, grab, tracker, settings, meta, view_fps, encode_jpeg):
        self.grab = grab
        self.tracker = tracker
        self.settings = settings
        self.meta = meta
        self.view_fps = max(1, view_fps)
        self.encode_jpeg = encode_jpeg
        self.cond = threading.Condition()
        self.latest = None
        self.latest_jpeg = None
        self.frame_id = 0
        self.last_gray = None
        self.frozen = None
        self.render_ms = collections.deque(maxlen=200)
        self._err_reported = False
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def _run(self):
        dt = 1.0 / self.view_fps
        while not self.settings.quit.is_set():
            t0 = time.perf_counter()
            img = jpg = None
            try:
                img = self.render()
                if self.encode_jpeg:
                    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 75])
                    jpg = buf.tobytes() if ok else None
            except Exception as e:                 # a drawing bug must never kill the run
                if not self._err_reported:
                    print(f"viewer render error (tracking unaffected): {e!r}")
                    self._err_reported = True
            self.render_ms.append((time.perf_counter() - t0) * 1000.0)
            with self.cond:
                if img is not None:
                    self.latest = img
                    self.latest_jpeg = jpg
                    self.frame_id += 1
                self.cond.notify_all()
            time.sleep(max(0.0, dt - (time.perf_counter() - t0)))
        with self.cond:
            self.cond.notify_all()

    def render(self):
        g, t, s = self.grab, self.tracker, self.settings
        gray, _stamp, _seq = g.latest()
        with g.lock:
            iv = list(g.live_intervals)
            dec = list(g.live_decode)
        with t.lock:
            st = dict(t.state)
            trail = list(t.trail)
            det = list(t.live_det)
            age = list(t.live_age)
            hits = list(t.live_hits)
            lt = list(t.live_t)
            skipped = t.consumer_skipped
        if s.paused:
            if self.frozen is None:
                self.frozen = (gray, st, trail)
            gray, st, trail = self.frozen
        else:
            self.frozen = None
        self.last_gray = gray
        return draw_dashboard(gray, st, trail, iv, dec, det, age, hits, lt,
                              skipped, s, self.meta)

    def snapshot(self):
        os.makedirs("snapshots", exist_ok=True)
        ts = time.strftime("%Y%m%d-%H%M%S")
        with self.cond:
            view = self.latest
        if view is not None:
            cv2.imwrite(f"snapshots/{ts}_view.png", view)
        if self.last_gray is not None:
            cv2.imwrite(f"snapshots/{ts}_raw.png", self.last_gray)
        self.settings.flash(f"saved snapshots/{ts}_*.png")


# ============================================================================
# Viewers: local window and browser
# ============================================================================

WINDOW = "FoosPGA vision"


def open_window():
    """Create the viewer window. Returns (ok, reason)."""
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        return False, "no display found (SSH session without a monitor?)"
    try:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        return True, ""
    except cv2.error:
        return False, "this OpenCV build has no GUI support"


def run_window(renderer, settings, on_key):
    """Main-thread loop for the local window. Returns when the run should end."""
    last = -1
    shown = False
    while not settings.quit.is_set():
        with renderer.cond:
            img, fid = renderer.latest, renderer.frame_id
        if img is not None and fid != last:
            cv2.imshow(WINDOW, img)
            last, shown = fid, True
        key = cv2.waitKey(15)
        if key not in (-1, 255):
            on_key(chr(key & 0xFF))
        if shown and cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
            settings.quit.set()                     # window closed with the X button
    cv2.destroyAllWindows()


PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><title>FoosPGA vision</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
 body{background:#141414;color:#ccc;font:14px system-ui,sans-serif;margin:0;padding:12px}
 img{max-width:100%;height:auto;display:block;border:1px solid #333}
 .k{margin-top:10px;display:flex;flex-wrap:wrap;gap:6px}
 button{background:#2a2a2a;color:#ddd;border:1px solid #444;border-radius:4px;
        padding:6px 10px;cursor:pointer;font:13px system-ui,sans-serif}
 button:hover{background:#383838}
 p{color:#888;margin:8px 0 0}
</style></head><body>
<img src="/stream" alt="live view">
<div class="k">
 <button onclick="k('m')">m &nbsp;mask</button>
 <button onclick="k('p')">p &nbsp;pause</button>
 <button onclick="k('t')">t &nbsp;trail</button>
 <button onclick="k('s')">s &nbsp;snapshot</button>
 <button onclick="k('r')">r &nbsp;reset stats</button>
 <button onclick="k('-')">&minus; &nbsp;threshold</button>
 <button onclick="k('+')">+ &nbsp;threshold</button>
 <button onclick="k('e')">e &nbsp;exposure &minus;</button>
 <button onclick="k('E')">E &nbsp;exposure +</button>
 <button onclick="k('g')">g &nbsp;gain &minus;</button>
 <button onclick="k('G')">G &nbsp;gain +</button>
 <button onclick="if(confirm('Stop the run and print the report?'))k('q')">q &nbsp;quit</button>
</div>
<p>Keyboard shortcuts work while this tab is focused. Snapshots save on the board.</p>
<script>
 function k(c){fetch('/key?k='+encodeURIComponent(c));}
 document.addEventListener('keydown',e=>{
   if(e.ctrlKey||e.metaKey||e.altKey)return;
   if(e.key==='q'&&!confirm('Stop the run and print the report?'))return;
   if(e.key.length===1||e.key==='Escape'){k(e.key);e.preventDefault();}
 });
</script>
</body></html>"""


def start_web(port, renderer, settings, on_key):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            pass

        def do_GET(self):
            u = urlparse(self.path)
            if u.path == "/":
                body = PAGE.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif u.path == "/key":
                on_key(parse_qs(u.query).get("k", [""])[0])
                self.send_response(204)
                self.end_headers()
            elif u.path == "/stream":
                self._stream()
            else:
                self.send_error(404)

        def _stream(self):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.end_headers()
            last = -1
            try:
                while not settings.quit.is_set():
                    with renderer.cond:
                        renderer.cond.wait_for(
                            lambda: renderer.frame_id != last or settings.quit.is_set(),
                            timeout=1.0)
                        jpg, last = renderer.latest_jpeg, renderer.frame_id
                    if jpg is None:
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                     b"Content-Length: " + str(len(jpg)).encode()
                                     + b"\r\n\r\n")
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass

    srv = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


def local_ip():
    """Best-guess LAN address for the URL hint. Sends no traffic."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return "<board-ip>"


# ============================================================================
# Key handling (shared by the window and the browser)
# ============================================================================

class KeyHandler:
    """One key map shared by the window and the browser."""

    def __init__(self, settings, device, controls_enabled, period_ms):
        self.s = settings
        self.dev = device
        self.ctl = controls_enabled
        self.period_ms = period_ms
        self.renderer = None
        self.lock = threading.Lock()   # window and browser can both send keys

    def __call__(self, k):
        s = self.s
        if not k:
            return
        if k in ("q", "Q", "\x1b", "Escape"):
            s.flash("quitting")
            s.quit.set()
        elif k == "m":
            s.show_mask = not s.show_mask
            s.flash("threshold mask " + ("on" if s.show_mask else "off"))
        elif k == "p":
            s.paused = not s.paused
            s.flash("view paused (tracking continues)" if s.paused else "view resumed")
        elif k == "t":
            s.show_trail = not s.show_trail
            s.flash("trail " + ("on" if s.show_trail else "off"))
        elif k in ("+", "="):
            s.threshold = min(254, s.threshold + 5)
            s.flash(f"threshold {s.threshold}")
        elif k in ("-", "_"):
            s.threshold = max(1, s.threshold - 5)
            s.flash(f"threshold {s.threshold}")
        elif k in ("e", "E"):
            self._exposure(-1 if k == "e" else +1)
        elif k in ("g", "G"):
            self._gain(-5 if k == "g" else +5)
        elif k == "s":
            if self.renderer is not None:
                self.renderer.snapshot()
        elif k == "r":
            s.reset_requested = True
            s.flash("stats reset")

    def _exposure(self, step):
        if not self.ctl:
            self.s.flash("camera controls disabled (--no-controls)")
            return
        with self.lock:
            want = int(np.clip(self.s.exp_units + step, 1, 5000))
            got = set_ctrl(self.dev, EXPOSURE_NAMES, want)
            if got is None:
                self.s.flash("exposure control not available")
                return
            self.s.exp_units = got
            self.s.changes += 1
            self.s.reset_requested = True
            note = "" if got == want else " (camera refused)"
            if got * 0.1 > self.period_ms:
                note += "  > frame period, FPS will drop"
            self.s.flash(f"exposure {got * 100} us{note} -- stats reset")

    def _gain(self, step):
        if not self.ctl:
            self.s.flash("camera controls disabled (--no-controls)")
            return
        with self.lock:
            want = int(np.clip(self.s.gain + step, 0, 100))
            got = set_ctrl(self.dev, ("gain",), want)
            if got is None:
                self.s.flash("gain control not available")
                return
            self.s.gain = got
            self.s.changes += 1
            self.s.reset_requested = True
            self.s.flash(f"gain {got} -- stats reset")
