"""
metrics_display.py

Numbers for the demo panel, with no web code in it.

MetricsGrabber wraps any FrameGrabber-like object and watches every frame the
tracker pulls through wait_frames()/get_frames(), using the camera timestamps:

    total        frames since start
    last window  one closed 1-second window: frames in it, mean and max
                 interval between consecutive frames, and fps from the mean

A window closes on the first frame stamped >= 1 s after the window started,
so the numbers update once per second (~150 frames at 640x480).

snapshot() gathers everything the page shows into one JSON-ready dict.
"""

import math
import threading

import numpy as np

WINDOW_S = 1.0


class MetricsGrabber:
    def __init__(self, grabber, log_path=None):
        self._g = grabber
        self._mlock = threading.Lock()
        self.total = 0
        self.window = None            # last closed window (dict) or None
        self._last_t = None
        self._ws = None               # current window start stamp
        self._n = 0                   # frames in the current window
        self._gaps = []               # intervals (s) ending in the current window
        self._log = open(log_path, "w", buffering=1) if log_path else None
        if self._log:
            self._log.write("t,total,frames,mean_ms,max_ms,fps\n")

    def __getattr__(self, name):      # w, h, x0, y0, lock, live_intervals, dropped, error, ...
        return getattr(self._g, name)

    def _record(self, frames):
        if not frames:
            return
        with self._mlock:
            for _seq, t, _gray in frames:
                if self._ws is None:
                    self._ws = t
                elif t - self._ws >= WINDOW_S:
                    self._close(t)
                if self._last_t is not None:
                    self._gaps.append(t - self._last_t)
                self._last_t = t
                self._n += 1
                self.total += 1

    def _close(self, t):
        gaps = self._gaps
        mean_ms = 1000.0 * sum(gaps) / len(gaps) if gaps else None
        self.window = {
            "frames": self._n,
            "mean_ms": mean_ms,
            "max_ms": 1000.0 * max(gaps) if gaps else None,
            "fps": 1000.0 / mean_ms if mean_ms else None,
        }
        if self._log:
            w = self.window
            self._log.write(f"{t:.3f},{self.total},{w['frames']},"
                            f"{w['mean_ms'] or 0:.3f},{w['max_ms'] or 0:.3f},{w['fps'] or 0:.1f}\n")
        self._ws, self._n, self._gaps = t, 0, []

    def wait_frames(self, timeout=0.1):
        out = self._g.wait_frames(timeout)
        self._record(out)
        return out

    def get_frames(self):
        out = self._g.get_frames()
        self._record(out)
        return out

    def counts(self):
        with self._mlock:
            return self.total, (dict(self.window) if self.window else None)

    def close(self):
        if self._log:
            self._log.close()
            self._log = None


def _r(v, nd=1):
    return None if v is None else round(float(v), nd)


def snapshot(live, metrics, setup, ui):
    """Everything the panel shows, as a JSON-ready dict."""
    with live.lock:
        snap = live.snap
    total, window = metrics.counts()
    with metrics.lock:
        iv = list(metrics.live_intervals)
        dropped = metrics.dropped
    out = {
        "total": total,
        "window": {k: _r(v, 2) for k, v in window.items()} if window else None,
        "cam_fps": _r(1000.0 / np.mean(iv)) if iv else None,
        "dropped": dropped,
        "proc_ms": None, "state": "WAITING", "ready": False,
        "px": None, "speed_px": None, "table": None, "speed_table": None,
        "setup": setup.status(), "overlay": ui.show_overlay, "error": metrics.error,
    }
    if snap is None:
        return out
    _gray, s, _view, _trail, info, _show_mask, ready = snap
    out["ready"] = bool(ready)
    out["proc_ms"] = _r(info.get("proc_ms"), 2)
    if not ready or s is None:
        return out
    out["state"] = s.state
    if s.x is None:
        return out
    out["px"] = [_r(s.x), _r(s.y)]
    out["speed_px"] = _r(math.hypot(s.vx, s.vy), 0)
    t = setup.to_table(s.x, s.y)
    v = setup.vel_to_table(s.vx, s.vy)
    if t is not None:
        out["table"] = [_r(t[0]), _r(t[1])]
        out["speed_table"] = _r(math.hypot(*v))
    return out
