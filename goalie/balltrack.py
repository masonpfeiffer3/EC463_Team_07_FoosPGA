#!/usr/bin/env python3
"""
ball_track.py  (v3)

Ball tracking + camera characterization for the autonomous foosball project.
Runs on the KV260 against the Arducam ezBOX (UVC, mono, global shutter).

v3 adds a live viewer: the camera feed with tracking overlays beside a panel
of live stats, a capture-interval trace and a brightness histogram against the
threshold. Keys adjust threshold, exposure and gain while it runs.

Viewer
------
    --display   window on a monitor plugged into the KV260
    --web       same view in a browser at http://<board-ip>:8080
                (use this over SSH -- no X forwarding needed)

    --display with no usable display falls back to --web automatically.

Keys (window, or the browser tab when focused)
----------------------------------------------
    q  quit            m  threshold-mask overlay     p  pause view
    t  trail on/off    s  save snapshot              r  reset stats
    + / -   threshold +-5     e / E   exposure -+100 us     g / G   gain -+5

Changing exposure or gain resets the statistics, so the final report always
describes a single set of camera settings.

Usage
-----
    python3 ball_track.py --web                       # live view in a browser
    python3 ball_track.py --display                   # live view on attached monitor
    python3 ball_track.py                             # headless 30 s characterization
    python3 ball_track.py --exposure 200 --gain 80    # target settings (needs lights)
    python3 ball_track.py --save-raw frames/          # dump frames as test vectors
    python3 ball_track.py --list                      # formats the camera supports

The viewer runs in its own thread at --view-fps and never blocks tracking, but
it does use CPU. For clean characterization numbers, run headless.


Some measurements:
@36in height, 48 inches viewable laterally
@48in height, 64 inches viewable laterally
"""

import argparse
import collections
import os
import sys
import threading
import time

import cv2
import numpy as np

from cameraSetup import (FrameGrabber, apply_controls, check_cpu_governor,
                         desired_controls, device_label, exposure_units,
                         list_formats, live_exposure)
from HUD import KeyHandler, Renderer, local_ip, open_window, run_window, start_web


# ============================================================================
# Detection
# ============================================================================

KERNEL3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))


def detect_ball(gray, threshold, min_r, max_r, max_aspect, morph, roi=None):
    """
    Bright-blob detection via one connected-components pass.

    Candidates are filtered on equivalent radius, bounding-box aspect ratio
    (motion blur elongates the ball; rods and shadows are far longer), and
    fill ratio (a disc fills pi/4 of its box). The best-scoring survivor wins.

    Invert the threshold if you ever backlight through a translucent field,
    where the ball becomes a dark silhouette.

    Returns (cx, cy, radius, area) in full-frame pixels, or None.
    """
    x0 = y0 = 0
    if roi is not None:
        x0, y0, x1, y1 = roi
        gray = gray[y0:y1, x0:x1]

    _, mask = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)
    if morph:
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL3)

    n, _labels, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if n <= 1:
        return None

    s = stats[1:]                                   # drop label 0 (background)
    area = s[:, cv2.CC_STAT_AREA].astype(np.float32)
    bw = s[:, cv2.CC_STAT_WIDTH].astype(np.float32)
    bh = s[:, cv2.CC_STAT_HEIGHT].astype(np.float32)

    r_eq = np.sqrt(area / np.pi)
    aspect = np.maximum(bw, bh) / np.maximum(np.minimum(bw, bh), 1.0)
    fill = area / np.maximum(bw * bh, 1.0)

    ok = (r_eq >= min_r) & (r_eq <= max_r) & (aspect <= max_aspect) & (fill >= 0.5)
    if not ok.any():
        return None

    score = -np.abs(fill - np.pi / 4) - 0.1 * (aspect - 1.0)
    score[~ok] = -np.inf
    i = int(np.argmax(score))
    cx, cy = cents[i + 1]
    return float(cx + x0), float(cy + y0), float(r_eq[i]), float(area[i])


def predict_roi(cx, cy, vx, vy, shape, dt, pad):
    h, w = shape
    nx, ny = cx + vx * dt, cy + vy * dt
    x0, y0 = int(max(0, nx - pad)), int(max(0, ny - pad))
    x1, y1 = int(min(w, nx + pad)), int(min(h, ny + pad))
    if x1 - x0 < 8 or y1 - y0 < 8:
        return None
    return (x0, y0, x1, y1)


# ============================================================================
# Shared runtime settings
# ============================================================================

class Settings:
    """Values the viewer can change while the tracker is running."""

    def __init__(self, args):
        self.threshold = args.threshold
        self.exp_units = exposure_units(args.exposure)
        self.gain = args.gain
        self.show_mask = False
        self.show_trail = True
        self.paused = False
        self.reset_requested = False
        self.changes = 0
        self.message = ""
        self.message_until = 0.0
        self.quit = threading.Event()

    def flash(self, msg, secs=2.5):
        self.message = msg
        self.message_until = time.monotonic() + secs


# ============================================================================
# Tracker (runs in its own thread)
# ============================================================================

class Tracker:
    LIVE = 400

    def __init__(self, grab, args, settings, period_ms, duration):
        self.grab = grab
        self.args = args
        self.settings = settings
        self.period_ms = period_ms
        self.duration = duration
        self.lock = threading.Lock()
        self.trail = collections.deque(maxlen=40)
        self.state = {"cx": 0.0, "cy": 0.0, "r": 0.0, "vx": 0.0, "vy": 0.0,
                      "have_track": False, "roi": None}
        self._reset()
        self.t_run_start = time.monotonic()
        self.t_end = None

    def _reset(self):
        self.det_ms = collections.deque(maxlen=50000)
        self.ages = collections.deque(maxlen=50000)
        self.radii = collections.deque(maxlen=50000)
        self.bg_mean = []
        self.frame_max = []
        self.frames = 0
        self.hits = 0
        self.consumer_skipped = 0
        self.live_det = collections.deque(maxlen=self.LIVE)
        self.live_age = collections.deque(maxlen=self.LIVE)
        self.live_hits = collections.deque(maxlen=200)
        self.live_t = collections.deque(maxlen=self.LIVE)
        self.t_start = time.monotonic()

    def run(self):
        a, s, grab = self.args, self.settings, self.grab
        period_s = self.period_ms / 1000.0
        _, _, last_seq = grab.latest()

        cx = cy = vx = vy = r = 0.0
        have_track = False
        last_hit_stamp = None
        roi = None
        t_report = time.monotonic()

        try:
            while not s.quit.is_set():
                if s.reset_requested:
                    s.reset_requested = False
                    grab.reset_stats()
                    with self.lock:
                        self._reset()
                    _, _, last_seq = grab.latest()
                    t_report = time.monotonic()

                gray, stamp, seq = grab.latest()
                if gray is None or seq == last_seq:
                    time.sleep(0.0002)
                    continue
                skipped = seq - last_seq - 1 if seq - last_seq > 1 else 0
                last_seq = seq

                t_now = time.monotonic()
                age = (t_now - stamp) * 1000.0
                thr = s.threshold

                tp = time.perf_counter()
                hit = detect_ball(gray, thr, a.min_radius, a.max_radius,
                                  a.max_aspect, a.morph, roi)
                if hit is None and roi is not None:
                    hit = detect_ball(gray, thr, a.min_radius, a.max_radius,
                                      a.max_aspect, a.morph, None)
                det = (time.perf_counter() - tp) * 1000.0

                if hit is not None:
                    nx, ny, r, _area = hit
                    if have_track and last_hit_stamp is not None:
                        dt = stamp - last_hit_stamp
                        if 0 < dt < 0.05:
                            vx = 0.6 * vx + 0.4 * ((nx - cx) / dt)
                            vy = 0.6 * vy + 0.4 * ((ny - cy) / dt)
                        else:
                            vx = vy = 0.0
                    cx, cy = nx, ny
                    last_hit_stamp = stamp
                    have_track = True
                    roi = predict_roi(cx, cy, vx, vy, gray.shape, period_s, a.roi_pad)
                else:
                    have_track = False
                    roi = None

                sample = None
                with self.lock:
                    self.frames += 1
                    frames = self.frames
                    self.consumer_skipped += skipped
                    self.det_ms.append(det)
                    self.ages.append(age)
                    self.live_det.append(det)
                    self.live_age.append(age)
                    self.live_hits.append(hit is not None)
                    self.live_t.append(t_now)
                    if hit is not None:
                        self.hits += 1
                        self.radii.append(r)
                        self.trail.append((cx, cy))
                    elif self.trail and self.trail[-1] is not None:
                        self.trail.append(None)          # break the trail at a loss
                    self.state.update(cx=cx, cy=cy, r=r, vx=vx, vy=vy,
                                      have_track=have_track, roi=roi)

                # Cheap brightness sampling for the light-budget diagnosis.
                if frames % 100 == 1:
                    sample = (float(cv2.mean(gray)[0]), float(cv2.minMaxLoc(gray)[1]))
                    with self.lock:
                        self.bg_mean.append(sample[0])
                        self.frame_max.append(sample[1])

                if a.save_raw and frames % a.save_every == 0:
                    cv2.imwrite(os.path.join(a.save_raw, f"f{frames:06d}.png"), gray)

                if t_now - t_report >= 2.0:
                    el = max(t_now - self.t_start, 1e-6)
                    with grab.lock:
                        dec = list(grab.live_decode)
                        cap = grab.grab_count
                    with self.lock:
                        dl = list(self.live_det)
                        hits, fr = self.hits, self.frames
                    print(f"[{t_now - self.t_run_start:6.1f}s] capture {cap/el:6.1f} fps  "
                          f"process {fr/el:6.1f} fps  detect {100.0*hits/max(fr,1):5.1f}%  "
                          f"decode {np.median(dec) if dec else 0:5.2f} ms  "
                          f"detect {np.median(dl) if dl else 0:5.2f} ms")
                    t_report = t_now

                if self.duration and (t_now - self.t_run_start) >= self.duration:
                    break
        finally:
            self.t_end = time.monotonic()
            s.quit.set()


# ============================================================================
# Reporting
# ============================================================================

def pct(values, p):
    return float(np.percentile(np.asarray(values), p)) if len(values) else 0.0


def stat_line(label, values, unit="ms"):
    if not len(values):
        return f"  {label:<26} (no data)"
    v = np.asarray(values)
    return (f"  {label:<26} median {np.median(v):7.3f}  p95 {pct(v,95):7.3f}  "
            f"p99 {pct(v,99):7.3f}  max {v.max():8.3f} {unit}")


# ============================================================================
# Main
# ============================================================================

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", type=int, default=0)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=200)
    ap.add_argument("--fourcc", default="MJPG", choices=["MJPG", "YUYV"])
    ap.add_argument("--exposure", type=int, default=1000,
                    help="microseconds, 100 us steps (target 200 once lighting is installed)")
    ap.add_argument("--gain", type=int, default=50, help="0-100")
    ap.add_argument("--no-controls", action="store_true",
                    help="leave camera controls exactly as they are")
    ap.add_argument("--no-raw", action="store_true",
                    help="let OpenCV decode to BGR instead of decoding straight to gray")
    ap.add_argument("--threshold", type=int, default=200)
    ap.add_argument("--min-radius", type=float, default=4.0)
    ap.add_argument("--max-radius", type=float, default=30.0)
    ap.add_argument("--max-aspect", type=float, default=2.5,
                    help="max bbox aspect ratio; >1 tolerates motion blur")
    ap.add_argument("--morph", action="store_true", help="add a 3x3 opening pass")
    ap.add_argument("--roi-pad", type=float, default=60.0)
    ap.add_argument("--warmup", type=float, default=0.5,
                    help="seconds discarded after controls are applied")
    ap.add_argument("--duration", type=float, default=None,
                    help="seconds; default 30 headless, until quit with a viewer; 0 = until quit")
    ap.add_argument("--display", action="store_true", help="live window on an attached monitor")
    ap.add_argument("--web", action="store_true", help="live view in a browser")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--view-fps", type=int, default=20)
    ap.add_argument("--save-raw", default=None, metavar="DIR")
    ap.add_argument("--save-every", type=int, default=10)
    ap.add_argument("--report", action="store_true",
                    help="(accepted for compatibility; the full report is always printed)")
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()

    dev = device_label(args.device)
    period_ms = 1000.0 / args.fps

    if args.list:
        list_formats(dev)
        return 0

    settings = Settings(args)
    exp_us = settings.exp_units * 100

    # ---- preflight -----------------------------------------------------------
    print(f"device       {dev}")
    if args.exposure % 100:
        print(f"  note: exposure is set in 100 us steps; using {exp_us} us")
    if exp_us / 1000.0 > period_ms:
        print(f"  WARNING: {exp_us} us exposure exceeds the {period_ms:.2f} ms frame "
              f"period; framerate will be capped near {1e6/exp_us:.0f} FPS")
    gov = check_cpu_governor()
    if gov:
        print(f"cpu governor {gov}")
        if gov != "performance":
            print("  frequency scaling can cause multi-hundred-ms stalls. For")
            print("  characterization runs:")
            print("    echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor")

    if args.display:
        ok, why = open_window()
        if not ok:
            print(f"viewer       --display unavailable: {why}")
            print("             falling back to the browser viewer")
            args.display = False
            args.web = True
    viewer = args.display or args.web
    duration = args.duration if args.duration is not None else (0.0 if viewer else 30.0)

    # ---- open stream ---------------------------------------------------------
    grab = FrameGrabber(args.device, args.width, args.height, args.fps,
                        args.fourcc, raw=not args.no_raw)
    got = grab.negotiated()
    print(f"requested    {args.width}x{args.height} @ {args.fps} {args.fourcc}")
    print(f"negotiated   {got['width']}x{got['height']} @ {got['fps']:.0f} {got['fourcc']}")
    if got["fourcc"] != args.fourcc:
        print("  WARNING: driver did not honor the requested format.")

    if not grab.first_frame.wait(timeout=5.0):
        print("no frames within 5 s -- check the cable, the device index, and --list")
        grab.release()
        return 1

    decode_path = ("raw -> grayscale decode" if grab.raw_active
                   else "OpenCV BGR decode -> cvtColor")
    print(f"decode path  {decode_path}")

    # ---- controls: only now that the stream is actually running ------------
    controls_enabled = not args.no_controls
    if not controls_enabled:
        print("controls     left unchanged (--no-controls)")
    else:
        print("controls     applying after stream start")
        if not apply_controls(dev, desired_controls(settings)):
            print("  WARNING: not every control took. Numbers below may not mean")
            print("  what you think.")

    # Discard frames captured before the new controls took effect.
    time.sleep(args.warmup)
    grab.reset_stats()

    if args.save_raw:
        os.makedirs(args.save_raw, exist_ok=True)

    # ---- start tracking ------------------------------------------------------
    tracker = Tracker(grab, args, settings, period_ms, duration)
    tthread = threading.Thread(target=tracker.run, daemon=True)
    tthread.start()

    renderer = srv = None
    keys = KeyHandler(settings, dev, controls_enabled, period_ms)
    if viewer:
        meta = {
            "fps": args.fps,
            "period_ms": period_ms,
            "format": f"{got['width']}x{got['height']} {got['fourcc']}, "
                      f"{'raw>gray' if grab.raw_active else 'BGR>gray'}",
        }
        renderer = Renderer(grab, tracker, settings, meta, args.view_fps,
                            encode_jpeg=args.web)
        keys.renderer = renderer
        renderer.start()
        if args.web:
            try:
                srv = start_web(args.port, renderer, settings, keys)
                print(f"viewer       open http://{local_ip()}:{args.port} in a browser")
            except OSError as e:
                print(f"viewer       could not start web server on port {args.port}: {e}")
                print("             try --port 8081")
        if args.display:
            print("viewer       window open -- press q in the window to finish")
    if duration:
        print(f"running      {duration:.0f} s")
    else:
        print("running      until quit (q in the viewer, or Ctrl-C here)")

    try:
        if args.display:
            run_window(renderer, settings, keys)
        while tthread.is_alive():
            tthread.join(0.2)
    except KeyboardInterrupt:
        settings.quit.set()
    tthread.join(2.0)

    # Read controls back WHILE STILL STREAMING -- the definitive check.
    live_ae, live_ex = (None, None) if not controls_enabled else live_exposure(dev)

    if renderer is not None:
        renderer.thread.join(timeout=1.0)
    if srv is not None:
        srv.shutdown()
    grab.release()

    # ---- report --------------------------------------------------------------
    t = tracker
    elapsed = max((t.t_end or time.monotonic()) - t.t_start, 1e-6)
    iv = list(grab.intervals)
    dec = list(grab.decode_ms)
    det_ms = list(t.det_ms)
    cap_fps = grab.grab_count / elapsed
    proc_fps = t.frames / elapsed
    med_iv = float(np.median(iv)) if iv else 0.0
    stalls = [x for x in iv if med_iv and x > 3 * med_iv]
    exp_us = settings.exp_units * 100

    print("\n" + "=" * 78)
    print("CAMERA")
    print(f"  duration                   {elapsed:.1f} s")
    print(f"  format                     {got['width']}x{got['height']} {got['fourcc']}, "
          f"{decode_path}")
    if controls_enabled:
        exp_live = f"{live_ex * 100} us" if live_ex is not None else "unknown"
        print(f"  exposure (live, streaming) {exp_live}  auto_exposure={live_ae}  "
              f"[requested {exp_us} us, manual=1]  gain {settings.gain}")
    print(f"  capture rate               {cap_fps:.1f} fps  (requested {args.fps})")
    print(f"  read / decode failures     {grab.read_fails} / {grab.decode_fails}")
    print(stat_line("capture interval", iv))
    if iv:
        print(f"  jitter (p95 - median)      {pct(iv, 95) - med_iv:.3f} ms")
    print(f"  stalls (> 3x median)       {len(stalls)}"
          + (f", worst {max(stalls):.1f} ms" if stalls else ""))

    print("\nPIPELINE")
    print(f"  process rate               {proc_fps:.1f} fps")
    print(f"  frames skipped by consumer {t.consumer_skipped}")
    print(stat_line("decode (grabber thread)", dec))
    print(stat_line("detect (tracker thread)", det_ms))
    print(stat_line("frame age at processing", list(t.ages)))
    if renderer is not None:
        mode = "window" if args.display else "web"
        print(f"  viewer                     {mode}, {np.median(renderer.render_ms) if renderer.render_ms else 0.0:.1f} ms "
              f"per render at {args.view_fps} fps -- these numbers include its CPU load")
    if settings.changes:
        print(f"  settings changed           {settings.changes} time(s); report covers "
              "the period since the last change")

    print("\nIMAGE")
    if t.bg_mean:
        print(f"  frame mean brightness      {np.median(t.bg_mean):6.1f} / 255 (median of samples)")
        print(f"  frame peak brightness      {np.median(t.frame_max):6.1f} / 255 "
              f"(threshold {settings.threshold})")
    print(f"  detection rate             {100.0 * t.hits / max(t.frames, 1):.1f} %")
    if t.radii:
        print(f"  ball radius when detected  {np.median(t.radii):.1f} px median "
              f"(filter {args.min_radius:.0f}-{args.max_radius:.0f})")

    # ---- verdict -------------------------------------------------------------
    print("\nVERDICT")
    findings = []

    if controls_enabled and live_ex is not None and (
            live_ex != settings.exp_units or live_ae != 1):
        findings.append(
            "Camera controls were RESET during streaming. Exposure is not what you "
            "asked for, so every number above is suspect. Re-run; if it repeats, "
            "the driver is resetting controls mid-stream.")

    if cap_fps < 0.9 * args.fps:
        live_exp_ms = (live_ex * 0.1) if live_ex is not None else exp_us / 1000.0
        med_dec = float(np.median(dec)) if dec else 0.0
        if live_exp_ms > 0.9 * period_ms:
            findings.append(
                f"Capture is EXPOSURE-LIMITED: {live_exp_ms:.1f} ms exposure cannot "
                f"fit in a {period_ms:.1f} ms frame. Shorten exposure (and add light).")
        elif med_dec > 0.9 * period_ms:
            findings.append(
                f"Capture is DECODE-LIMITED: {med_dec:.2f} ms median decode vs "
                f"{period_ms:.2f} ms frame period. The grabber thread cannot keep up, "
                "so frames queue in the driver and arrive stale. Try --fourcc YUYV at "
                "320x240 (no decode) to confirm.")
        else:
            findings.append(
                f"Capture is below target ({cap_fps:.0f} of {args.fps} fps) but neither "
                "exposure nor decode explains it. Check the negotiated format line, the "
                "USB port, whether another process holds the camera"
                + (", and try again without the viewer." if renderer else "."))

    if det_ms and pct(det_ms, 95) > period_ms:
        findings.append(
            f"Detection p95 ({pct(det_ms, 95):.2f} ms) exceeds the frame period "
            f"({period_ms:.2f} ms). The tracker cannot keep up at this framerate.")

    if stalls:
        findings.append(
            f"{len(stalls)} capture stall(s), worst {max(stalls):.0f} ms. A third of a "
            "second of blindness mid-play would trip the watchdog. Set the CPU governor "
            "to performance and re-run to see if it repeats.")

    if t.frame_max and np.median(t.frame_max) < settings.threshold:
        findings.append(
            f"Image too DARK: typical peak brightness {np.median(t.frame_max):.0f} never "
            f"reaches threshold {settings.threshold}. Nothing can be detected. Add light, "
            "raise gain, lengthen exposure, or lower --threshold -- in that order of "
            "preference.")
    elif t.frames and t.hits / t.frames < 0.5:
        findings.append(
            "Detection rate is low but the image is bright enough to cross threshold. "
            "Check that the ball is actually in frame and moving, then tune --threshold "
            "and the radius filter against the median radius reported above.")

    if not findings:
        findings.append("No problems detected at these settings.")

    for f in findings:
        print(f"  - {f}")

    print("=" * 78)
    print("Frame age above is time from read() returning to processing start. It is")
    print("NOT exposure-to-detection latency, which includes sensor readout, USB")
    print("transfer and driver queueing. Measure that with the LED transition test.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
