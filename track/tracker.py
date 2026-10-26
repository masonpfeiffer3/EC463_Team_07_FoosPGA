"""
tracker.py

Ball detection + Kalman tracking for the foosball camera. Works in image
pixels; the pixel -> table-mm mapping (homography) is a separate later stage.

Wiring with cameraFrames.FrameGrabber:

    grabber = FrameGrabber(0, 320, 240, 400, "MJPG", raw=False)
    grabber.first_frame.wait()
    empty = [grabber.latest()[0].copy() for _ in range(30)]   # empty table, spaced out
    det = BallDetector(DetectorConfig(mode="diff", thresh=30))
    det.learn_background(empty)
    trk = BallTracker(det, TrackerConfig(), bounds=(x0, y0, x1, y1))
    while True:
        for seq, stamp, gray in grabber.wait_frames():   # process EVERY frame, in order
            s = trk.update(gray, stamp)
        send(s)                                          # publish only the newest result

Frames from the grabber are shared with its viewer: never modify them in place.
"""

import math
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

SEARCH, TENTATIVE, TRACK, COAST = "SEARCH", "TENTATIVE", "TRACK", "COAST"


# ============================================================================
# Detector
# ============================================================================

@dataclass
class DetectorConfig:
    mode: str = "diff"        # "diff": |frame - background| > thresh
                              # "bright": frame > thresh   "dark": frame < thresh
    thresh: int = 25
    min_area: int = 12        # blob area limits, in pixels
    max_area: int = 400
    min_fill: float = 0.35    # blob area / bbox area (a perfect disc is ~0.785)
    max_aspect: float = 2.5   # long bbox side / short side (rejects rods and arms)
    open_kernel: int = 3      # morphological opening to kill speckle; 1 disables


class BallDetector:
    """Finds ball-shaped blobs, optionally only inside a search window (roi)."""

    def __init__(self, cfg=None):
        self.cfg = cfg or DetectorConfig()
        self.background = None
        k = self.cfg.open_kernel
        self._kernel = np.ones((k, k), np.uint8) if k > 1 else None

    def learn_background(self, frames):
        """Median of several empty-table frames (no ball, no players moving)."""
        self.background = np.median(np.stack(frames), axis=0).astype(np.uint8)

    def detect(self, gray, roi=None):
        """
        roi = (x0, y0, x1, y1) in ints, or None for the full frame.
        Returns [(cx, cy, area), ...] in full-frame pixel coordinates.
        """
        c = self.cfg
        if roi is None:
            x0 = y0 = 0
            sub = gray
        else:
            x0, y0, x1, y1 = roi
            sub = gray[y0:y1, x0:x1]

        if c.mode == "diff":
            if self.background is None:
                raise RuntimeError("diff mode needs learn_background() first")
            bg = self.background if roi is None else self.background[y0:y1, x0:x1]
            _, mask = cv2.threshold(cv2.absdiff(sub, bg), c.thresh, 255, cv2.THRESH_BINARY)
        elif c.mode == "bright":
            _, mask = cv2.threshold(sub, c.thresh, 255, cv2.THRESH_BINARY)
        elif c.mode == "dark":
            _, mask = cv2.threshold(sub, c.thresh, 255, cv2.THRESH_BINARY_INV)
        else:
            raise ValueError(f"unknown detector mode {c.mode!r}")

        if self._kernel is not None:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)

        n, _labels, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
        out = []
        for i in range(1, n):
            x, y, w, h, area = stats[i]
            if not (c.min_area <= area <= c.max_area):
                continue
            if area / float(w * h) < c.min_fill:
                continue
            if max(w, h) / float(min(w, h)) > c.max_aspect:
                continue
            out.append((float(cents[i, 0]) + x0, float(cents[i, 1]) + y0, int(area)))
        return out


# ============================================================================
# Kalman filter
# ============================================================================

class BallKalman:
    """
    Constant-velocity Kalman filter, state [x, y, vx, vy], units px and px/s.
    dt comes from real timestamps, so dropped or late frames are handled
    correctly. accel_std is the process noise: how hard the ball can be
    accelerated by hits and bounces. Too low and the filter lags after a kick;
    too high and the velocity estimate gets noisy.

    bounds = (xmin, ymin, xmax, ymax) is the region the ball CENTER can reach
    (playfield inset by the ball radius). When a prediction leaves it, position
    and velocity are mirrored, which keeps coasting alive through wall bounces.
    """

    def __init__(self, accel_std=8e4, meas_std=0.6, init_vel_std=2000.0,
                 bounds=None, restitution=1.0):
        self.q = accel_std ** 2
        self.R = np.eye(2) * meas_std ** 2
        self.init_vel_std = init_vel_std
        self.bounds = bounds
        self.e = restitution
        self.H = np.zeros((2, 4))
        self.H[0, 0] = self.H[1, 1] = 1.0
        self.x = np.zeros(4)
        self.P = np.eye(4)

    def init(self, z):
        self.x = np.array([z[0], z[1], 0.0, 0.0])
        v2 = self.init_vel_std ** 2
        self.P = np.diag([self.R[0, 0], self.R[1, 1], v2, v2])

    @staticmethod
    def _F(dt):
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        return F

    def _reflect(self, x):
        if self.bounds is None:
            return
        x0, y0, x1, y1 = self.bounds
        for p, v, lo, hi in ((0, 2, x0, x1), (1, 3, y0, y1)):
            if x[p] < lo:
                x[p] = 2 * lo - x[p]
                x[v] = abs(x[v]) * self.e
            elif x[p] > hi:
                x[p] = 2 * hi - x[p]
                x[v] = -abs(x[v]) * self.e

    def predict(self, dt):
        F = self._F(dt)
        Q = np.zeros((4, 4))
        for p, v in ((0, 2), (1, 3)):
            Q[p, p] = self.q * dt ** 4 / 4
            Q[p, v] = Q[v, p] = self.q * dt ** 3 / 2
            Q[v, v] = self.q * dt ** 2
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self._reflect(self.x)

    def project(self, dt):
        """State dt seconds ahead of now, without changing the filter."""
        x = self._F(dt) @ self.x
        self._reflect(x)
        return x

    def gate(self, z):
        """(squared Mahalanobis distance, pixel distance) of z from the prediction."""
        y = np.asarray(z, dtype=float) - self.x[:2]
        S = self.P[:2, :2] + self.R
        return float(y @ np.linalg.solve(S, y)), float(np.hypot(y[0], y[1]))

    def update(self, z):
        y = np.asarray(z, dtype=float) - self.x[:2]
        S = self.P[:2, :2] + self.R
        K = self.P[:, :2] @ np.linalg.inv(S)
        self.x = self.x + K @ y
        I_KH = np.eye(4) - K @ self.H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T   # Joseph form


# ============================================================================
# Tracker (detector + filter + state machine)
# ============================================================================

@dataclass
class TrackerConfig:
    ball_area: float = 64.0       # expected blob area in px (pi * r^2)
    size_weight: float = 2.0      # how much area mismatch counts when picking among candidates
    gate_chi2: float = 9.21       # 99% Mahalanobis gate for 2 DOF
    gate_min_px: float = 12.0     # ...but always accept candidates this close to the prediction
    confirm_hits: int = 3         # consecutive-ish detections before trusting a new track
    coast_time_s: float = 0.30    # give up and re-search after this long without a measurement
    roi_min_px: int = 24          # search window half-size floor
    roi_speed_k: float = 2.0      # window grows by this x (speed * time since last measurement)
    dt_min: float = 1e-4
    dt_max: float = 0.1


@dataclass
class TrackState:
    stamp: float
    state: str
    x: Optional[float] = None
    y: Optional[float] = None
    vx: Optional[float] = None
    vy: Optional[float] = None
    measured: bool = False        # True if this frame's detection updated the filter


class BallTracker:
    """
    SEARCH    no track: full-frame detection each frame.
    TENTATIVE first detection seen; needs confirm_hits to become a track.
    TRACK     detection inside the predicted window updated the filter.
    COAST     no usable detection; output is the filter's prediction.
    After coast_time_s without a measurement the track is dropped to SEARCH.

    Pass your own BallKalman via `kalman`, otherwise one is built with `bounds`.
    """

    def __init__(self, detector, cfg=None, bounds=None, kalman=None):
        self.det = detector
        self.cfg = cfg or TrackerConfig()
        self.kf = kalman or BallKalman(bounds=bounds)
        self.reset()

    def reset(self):
        self.state = SEARCH
        self.hits = 0
        self.last_stamp = None
        self.last_meas = None
        self.last_roi = None       # search window used on the last frame (None = full frame)
        self.last_cands = []       # every blob the detector returned on the last frame

    # -- helpers -------------------------------------------------------------

    def _out(self, stamp, measured):
        x, y, vx, vy = self.kf.x
        return TrackState(stamp, self.state, float(x), float(y), float(vx), float(vy), measured)

    def _roi(self, w, h, t_since_meas):
        c = self.cfg
        px = min(max(self.kf.x[0], 0.0), w - 1.0)
        py = min(max(self.kf.x[1], 0.0), h - 1.0)
        speed = math.hypot(self.kf.x[2], self.kf.x[3])
        sigma = math.sqrt(max(self.kf.P[0, 0], self.kf.P[1, 1]))
        r = c.roi_min_px + c.roi_speed_k * speed * t_since_meas + 4.0 * sigma
        x0, x1 = int(max(0, math.floor(px - r))), int(min(w, math.ceil(px + r)))
        y0, y1 = int(max(0, math.floor(py - r))), int(min(h, math.ceil(py + r)))
        return (x0, y0, max(x1, x0 + 1), max(y1, y0 + 1))

    def _size_cost(self, area):
        return self.cfg.size_weight * abs(math.log(max(area, 1) / self.cfg.ball_area))

    def _associate(self, cands):
        best, best_cost = None, float("inf")
        for cx, cy, area in cands:
            d2, dist = self.kf.gate((cx, cy))
            if d2 > self.cfg.gate_chi2 and dist > self.cfg.gate_min_px:
                continue
            cost = d2 + self._size_cost(area)
            if cost < best_cost:
                best, best_cost = (cx, cy), cost
        return best

    # -- main entry ----------------------------------------------------------

    def update(self, gray, stamp):
        """Process one frame (in capture order). Returns a TrackState."""
        c = self.cfg
        h, w = gray.shape[:2]

        if self.state == SEARCH:
            cands = self.det.detect(gray)
            self.last_roi, self.last_cands = None, cands
            if not cands:
                return TrackState(stamp, SEARCH)
            cx, cy, _ = min(cands, key=lambda k: self._size_cost(k[2]))
            self.kf.init((cx, cy))
            self.state, self.hits = TENTATIVE, 1
            self.last_stamp = self.last_meas = stamp
            return self._out(stamp, True)

        dt = min(max(stamp - self.last_stamp, c.dt_min), c.dt_max)
        self.kf.predict(dt)
        roi = self._roi(w, h, stamp - self.last_meas)
        cands = self.det.detect(gray, roi)
        self.last_roi, self.last_cands = roi, cands
        z = self._associate(cands)
        self.last_stamp = stamp

        if z is not None:
            self.kf.update(z)
            self.last_meas = stamp
            self.hits += 1
            if self.state == TENTATIVE:
                if self.hits >= c.confirm_hits:
                    self.state = TRACK
            else:
                self.state = TRACK
            return self._out(stamp, True)

        if self.state == TENTATIVE:                 # never confirmed: start over
            self.reset()
            return TrackState(stamp, SEARCH)
        if stamp - self.last_meas > c.coast_time_s:  # coasted too long: re-search
            self.reset()
            return TrackState(stamp, SEARCH)
        self.state = COAST
        return self._out(stamp, False)

    def lookahead(self, seconds):
        """(x, y) the ball is predicted to reach `seconds` from the last frame, or None."""
        if self.state not in (TRACK, COAST):
            return None
        x = self.kf.project(seconds)
        return float(x[0]), float(x[1])