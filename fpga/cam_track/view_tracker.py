"""
view_tracker.py

Live window: the camera feed with the tracker drawn on top. Tracking runs on
EVERY captured frame; only the drawing is throttled (--display-fps), so the
window never slows the tracker down.

    python view_tracker.py --device 0 --width 320 --height 240 --fps 400 --scale 3

What you see
    ring        filter position, colored by state
                  green TRACK   orange COAST   yellow TENTATIVE   red SEARCH
    dot in ring the ball was actually detected on this frame (absent when coasting)
    arrow       velocity (length = 15 ms of travel)
    white dot   where the ball is predicted to be 20 ms from now
    yellow ring every blob the detector found (before gating)
    blue box    the search window the detector was restricted to
    trail       recent filter positions

Keys
    b  learn background (diff mode: clear the table of ball and players first)
    r  reset the tracker          m  toggle diff/mask view (diff mode)
    q or Esc  quit
The "thresh" slider changes the detector threshold live. The final detector
and tracker configs are printed on exit so they can be copied into your code.
"""

import argparse
import collections
import math
import time
import types

import cv2
import numpy as np

from cameraFrames import (IS_MAC, FrameGrabber, apply_controls, desired_controls,
                         device_label, exposure_units)
from tracker import (BallDetector, BallTracker, DetectorConfig, TrackerConfig,
                     SEARCH, TENTATIVE, TRACK, COAST)

COLORS = {SEARCH: (60, 60, 230), TENTATIVE: (0, 220, 255),
          TRACK: (80, 220, 80), COAST: (0, 150, 255)}
FONT = cv2.FONT_HERSHEY_SIMPLEX


def put(img, text, org, color=(255, 255, 255)):
    # Outline from 1 px offsets at the SAME thickness as the fill: a thick
    # outline stroke gets laid out wider than the thin fill and smears the text.
    x, y = org
    for dx, dy in ((-1, -1), (-1, 1), (1, -1), (1, 1), (-1, 0), (1, 0), (0, -1), (0, 1)):
        cv2.putText(img, text, (x + dx, y + dy), FONT, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(img, text, org, FONT, 0.5, color, 1, cv2.LINE_AA)


def render(gray, s, trk, det, trail, info, scale, show_mask, ready=True):
    """Draw one display frame. Pure function of its inputs (easy to test headless)."""
    if show_mask and det.cfg.mode == "diff" and det.background is not None:
        base = cv2.convertScaleAbs(cv2.absdiff(gray, det.background), alpha=4.0)
    else:
        base = gray
    img = cv2.cvtColor(cv2.resize(base, None, fx=scale, fy=scale,
                                  interpolation=cv2.INTER_NEAREST), cv2.COLOR_GRAY2BGR)

    def P(x, y):                       # pixel-center aware image -> display coords
        return int(round((x + 0.5) * scale)), int(round((y + 0.5) * scale))

    if ready:
        if trk.last_roi is not None:
            x0, y0, x1, y1 = trk.last_roi
            cv2.rectangle(img, (x0 * scale, y0 * scale), (x1 * scale, y1 * scale),
                          (255, 140, 40), 1)
        for cx, cy, area in trk.last_cands:
            cv2.circle(img, P(cx, cy), int(math.sqrt(area / math.pi) * scale) + 3,
                       (0, 230, 230), 1, cv2.LINE_AA)

        if len(trail) > 1:
            pts = np.array([P(x, y) for x, y in trail], np.int32)
            cv2.polylines(img, [pts], False, (200, 200, 60), 1, cv2.LINE_AA)

        if s is not None and s.x is not None:
            col = COLORS[s.state]
            ball_r = math.sqrt(trk.cfg.ball_area / math.pi)
            c = P(s.x, s.y)
            cv2.circle(img, c, int(ball_r * scale) + 3, col, 2, cv2.LINE_AA)
            if s.measured:
                cv2.circle(img, c, 2, col, -1, cv2.LINE_AA)
            cv2.arrowedLine(img, c, P(s.x + s.vx * 0.015, s.y + s.vy * 0.015),
                            col, 2, cv2.LINE_AA, tipLength=0.3)
            ahead = trk.lookahead(0.020)
            if ahead is not None:
                cv2.circle(img, P(*ahead), 3, (255, 255, 255), -1, cv2.LINE_AA)

    state = s.state if (s is not None and ready) else "WAITING"
    put(img, state, (8, 20), COLORS.get(state, (255, 255, 255)))
    put(img, f"cam {info['cam_fps']:.0f} fps   track {info['proc_ms']:.2f} ms   "
             f"queue drops {info['dropped']}", (8, 40))
    if not ready:
        put(img, "diff mode: clear the table, then press b", (8, 60), (0, 220, 255))
    elif s is not None and s.x is not None:
        put(img, f"pos {s.x:6.1f},{s.y:6.1f} px   speed {math.hypot(s.vx, s.vy):6.0f} px/s", (8, 60))
    put(img, "b bg   r reset   m mask   q quit", (8, img.shape[0] - 10), (190, 190, 190))
    return img


def learn_background(grabber, det, n=30, step=4):
    """Median of n frames (every `step`-th) taken right now. Table must be empty."""
    got, seen = [], 0
    deadline = time.monotonic() + 5.0
    while len(got) < n and time.monotonic() < deadline:
        for _seq, _t, gray in grabber.wait_frames(0.1):
            seen += 1
            if seen % step == 0:
                got.append(gray.copy())
    if len(got) >= 5:
        det.learn_background(got)
        print(f"background learned from {len(got)} frames")
        return True
    print("could not collect enough frames for a background")
    return False


def main():
    ap = argparse.ArgumentParser(description="Live tracker viewer")
    ap.add_argument("--device", type=int, default=0)
    ap.add_argument("--width", type=int, default=320)
    ap.add_argument("--height", type=int, default=240)
    ap.add_argument("--fps", type=int, default=400)
    ap.add_argument("--fourcc", default="MJPG")
    ap.add_argument("--raw", action="store_true")
    ap.add_argument("--exposure-us", type=float, default=500.0)
    ap.add_argument("--gain", type=int, default=0)
    ap.add_argument("--mode", choices=["diff", "bright", "dark"], default="diff")
    ap.add_argument("--thresh", type=int, default=30)
    ap.add_argument("--min-area", type=int, default=12)
    ap.add_argument("--max-area", type=int, default=400)
    ap.add_argument("--ball-radius", type=float, default=5.0, help="ball radius in pixels")
    ap.add_argument("--bounds", help="x0,y0,x1,y1: region the ball CENTER can reach (enables wall bounces)")
    ap.add_argument("--scale", type=int, default=3, help="display magnification")
    ap.add_argument("--display-fps", type=float, default=30.0)
    ap.add_argument("--trail", type=int, default=120, help="trail length in frames")
    args = ap.parse_args()

    bounds = tuple(float(v) for v in args.bounds.split(",")) if args.bounds else None
    dev = device_label(args.device)

    grabber = FrameGrabber(args.device, args.width, args.height, args.fps,
                           args.fourcc, args.raw)
    print("negotiated:", grabber.negotiated())
    if not grabber.first_frame.wait(5.0):
        print("no frames within 5 s")
        grabber.release()
        return 1
    if not IS_MAC:   # after streaming starts: some UVC cameras reset controls then
        apply_controls(dev, desired_controls(types.SimpleNamespace(
            exp_units=exposure_units(args.exposure_us), gain=args.gain)))

    det = BallDetector(DetectorConfig(mode=args.mode, thresh=args.thresh,
                                      min_area=args.min_area, max_area=args.max_area))
    trk = BallTracker(det, TrackerConfig(ball_area=math.pi * args.ball_radius ** 2),
                      bounds=bounds)

    win = "tracker"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    cv2.createTrackbar("thresh", win, args.thresh, 100,
                       lambda v: setattr(det.cfg, "thresh", v))

    trail = collections.deque(maxlen=args.trail)
    proc = collections.deque(maxlen=400)
    last_gray, last_state, last_render, show_mask = None, None, 0.0, False
    try:
        while True:
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

            now = time.monotonic()
            if last_gray is None or now - last_render < 1.0 / args.display_fps:
                continue
            last_render = now

            with grabber.lock:
                iv = list(grabber.live_intervals)
                dropped = grabber.dropped
            info = {"cam_fps": 1000.0 / np.mean(iv) if iv else 0.0,
                    "proc_ms": float(np.mean(proc)) if proc else 0.0,
                    "dropped": dropped}
            cv2.imshow(win, render(last_gray, last_state, trk, det, trail, info,
                                   args.scale, show_mask, ready))
            k = cv2.waitKey(1) & 0xFF
            if k in (27, ord("q")):
                break
            elif k == ord("b"):
                learn_background(grabber, det)
                trk.reset()
                trail.clear()
            elif k == ord("r"):
                trk.reset()
                trail.clear()
            elif k == ord("m"):
                show_mask = not show_mask
    finally:
        grabber.release()
        cv2.destroyAllWindows()
        print(det.cfg)
        print(trk.cfg)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())