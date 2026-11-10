"""
Arducam_frames.py

kv260_view_tracker.py for the USB Arducam (ezBOX) plugged into the Kria,
instead of the OV9281 on the MIPI port. Same tracker, same drawing, same web
page; only the frame source changes:

    MIPI:  kv260Frames.FrameGrabber   (FPGA -> BRAM -> /dev/mem)
    USB:   cameraFrames.FrameGrabber  (V4L2, MJPEG decoded on the ARM)

No bitstream, xmutil or ov9281_init.py needed. On the Kria:

    v4l2-ctl --list-devices                  # find the Arducam's /dev/videoN
    python3 cameraFrames.py --device 0 --width 640 --height 480 --fps 200   # check fps + decode time
    python3 Arducam_frames.py --device 0     # defaults: 640x480 @ 200 fps

then on the laptop:
    ssh -L 8080:localhost:8080 <user>@<kria-ip>    # keep it open
    open http://localhost:8080

Clear the table and press b (learn background) before tracking. Keys and the
thresh slider work exactly as in kv260_view_tracker.py.
"""

import argparse
import math
import threading
import time
import types
from http.server import ThreadingHTTPServer

import numpy as np

from cameraFrames import (FrameGrabber, apply_controls, check_cpu_governor,
                          desired_controls, device_label, exposure_units)
from kv260_view_tracker import Live, make_handler, render_loop, tracking_loop
from tracker import BallDetector, BallTracker, DetectorConfig, TrackerConfig

STALL_S = 0.5    # no frame for this long -> show a warning on the page


class UsbGrabber:
    """
    cameraFrames.FrameGrabber plus what kv260_view_tracker expects: an optional
    column crop (same meaning as --crop there) and an `error` string that the
    render thread prints on the picture. Everything else is passed through.
    """

    def __init__(self, grabber, crop=None):
        self._g = grabber
        self.x0, self.x1 = crop or (0, grabber.w)
        self.x0 = max(0, min(self.x0, grabber.w - 1))
        self.x1 = max(self.x0 + 1, min(self.x1, grabber.w))
        self.w, self.h = self.x1 - self.x0, grabber.h

    def __getattr__(self, name):            # lock, first_frame, live_intervals, dropped, ...
        return getattr(self._g, name)

    @property
    def error(self):
        stamp = self._g.stamp
        if stamp is None:
            return None
        quiet = time.monotonic() - stamp
        return f"USB camera: no frames for {quiet:.1f} s (unplugged?)" if quiet > STALL_S else None

    def _crop(self, gray):
        if self.x0 == 0 and self.x1 == self._g.w:
            return gray
        return np.ascontiguousarray(gray[:, self.x0:self.x1])

    def latest(self):
        gray, stamp, seq = self._g.latest()
        return (None if gray is None else self._crop(gray)), stamp, seq

    def get_frames(self):
        return [(s, t, self._crop(g)) for s, t, g in self._g.get_frames()]

    def wait_frames(self, timeout=0.1):
        return [(s, t, self._crop(g)) for s, t, g in self._g.wait_frames(timeout)]


def parse_crop(text):
    """'40:280' -> (40, 280). None -> None."""
    if not text:
        return None
    x0, x1 = (int(v) for v in text.split(":"))
    return x0, x1


def main():
    ap = argparse.ArgumentParser(description="Live tracker viewer for the USB Arducam on the Kria")
    # camera (same options as view_tracker.py)
    ap.add_argument("--device", type=int, default=0, help="N in /dev/videoN")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=200)
    ap.add_argument("--fourcc", default="MJPG")
    ap.add_argument("--raw", action="store_true", help="request undecoded buffers")
    ap.add_argument("--exposure-us", type=float, default=500.0)
    ap.add_argument("--gain", type=int, default=0)
    ap.add_argument("--crop", metavar="X0:X1",
                    help="keep only columns X0..X1; --bounds is then in cropped pixels")
    # web page
    ap.add_argument("--host", default="127.0.0.1",
                    help="127.0.0.1 = only through the SSH tunnel; 0.0.0.0 = whole network")
    ap.add_argument("--port", type=int, default=8080)
    # tracker (same options as kv260_view_tracker.py)
    ap.add_argument("--mode", choices=["diff", "bright", "dark"], default="diff")
    ap.add_argument("--thresh", type=int, default=30)
    # size defaults are for 640x480 (2x the 320x240 values, areas 4x)
    ap.add_argument("--min-area", type=int, default=48)
    ap.add_argument("--max-area", type=int, default=1600)
    ap.add_argument("--ball-radius", type=float, default=10.0, help="ball radius in pixels")
    ap.add_argument("--bounds", help="x0,y0,x1,y1: region the ball CENTER can reach")
    ap.add_argument("--px-scale", type=float, default=2.0,
                    help="multiply TrackerConfig's pixel distances; 1 at 320x240, 2 at 640x480")
    ap.add_argument("--scale", type=int, default=1, help="display magnification")
    ap.add_argument("--display-fps", type=float, default=30.0)
    ap.add_argument("--jpeg-quality", type=int, default=80)
    ap.add_argument("--trail", type=int, default=120)
    ap.add_argument("--log", help="CSV of every tracker output: stamp,state,x,y,vx,vy,measured")
    args = ap.parse_args()

    dev = device_label(args.device)
    cam = FrameGrabber(args.device, args.width, args.height, args.fps, args.fourcc, args.raw)
    print("negotiated:", cam.negotiated())
    if not cam.first_frame.wait(5.0):
        print(f"no frames from {dev} within 5 s (wrong --device? check v4l2-ctl --list-devices)")
        cam.release()
        return 1
    # after streaming starts: some UVC cameras reset their controls then
    apply_controls(dev, desired_controls(types.SimpleNamespace(
        exp_units=exposure_units(args.exposure_us), gain=args.gain)))
    gov = check_cpu_governor()
    if gov and gov != "performance":
        print(f"cpu governor is '{gov}': 'performance' gives steadier frame times")

    grabber = UsbGrabber(cam, parse_crop(args.crop))
    print(f"frames: {grabber.w} x {grabber.h} from {dev}")

    bounds = tuple(float(v) for v in args.bounds.split(",")) if args.bounds else None
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
    if args.host == "127.0.0.1":
        print(f"laptop: ssh -L {args.port}:localhost:{args.port} <user>@<kria-ip>, "
              f"then open http://localhost:{args.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        live.running = False
        cam.release()
        if args.log_file:
            args.log_file.close()
        print()
        print(det.cfg)
        print(trk.cfg)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
