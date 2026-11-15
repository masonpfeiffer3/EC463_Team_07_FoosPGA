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

SEARCH, TENTATIVE, TRACK, COAST, HIDDEN = "SEARCH", "TENTATIVE", "TRACK", "COAST", "HIDDEN"


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
    masks: tuple = ()         # ((x0, y0, x1, y1), ...): blobs CENTERED in one are ignored
                              # (rods, heads, glints). Replace the whole tuple to change it.


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
        masks = c.masks           # one read: the web thread may swap the tuple
        for i in range(1, n):
            x, y, w, h, area = stats[i]
            if not (c.min_area <= area <= c.max_area):
                continue
            if area / float(w * h) < c.min_fill:
                continue
            if max(w, h) / float(min(w, h)) > c.max_aspect:
                continue
            cx, cy = float(cents[i, 0]) + x0, float(cents[i, 1]) + y0
            if any(mx0 <= cx < mx1 and my0 <= cy < my1 for mx0, my0, mx1, my1 in masks):
                continue
            out.append((cx, cy, int(area)))
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

    def init(self, z, vel_std=None):
        """Start at z with unknown velocity (std vel_std, default init_vel_std)."""
        self.x = np.array([z[0], z[1], 0.0, 0.0])
        v2 = (self.init_vel_std if vel_std is None else vel_std) ** 2
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
    confirm_hits: int = 3         # detections needed before trusting a new track...
    tentative_max_misses: int = 2 # ...with up to this many missed frames along the way
    coast_time_s: float = 0.30    # give up after this long without a measurement...
    coast_time_slow_s: float = 1.0  # ...or this long if the ball was slower than slow_speed
    slow_speed: float = 150.0     # px/s; a stopped or rolling ball is likely just hidden
    reacquire_time_s: float = 0.5 # after a lost track, search only near where it was lost...
    reacquire_pad_px: int = 40    # ...in a box around the last measurement and final prediction
    roi_min_px: int = 24          # search window half-size floor
    roi_speed_k: float = 2.0      # window grows by this x (speed * time since last measurement)
    rod_zones: tuple = ()         # ((x0, y0, x1, y1), ...): the rod rectangles your mask already uses
    rod_dwell_margin_px: float = 6.0  # a measurement this close to a rectangle counts as "on" it
                                      # (heads poke out past a thin rod mask)
    rod_dwell_s: float = 0.4      # a track on the same rod this long is a head: drop it
                                  # (a real ball crosses a rod in a few ms)
    hidden_margin_px: float = 20.0  # ball under a rod zone: exit box = strip widened by this
                                    # + kick_speed * frame gap, around the y where it went in
    kick_speed_px_s: float = 4000.0 # fastest a foot can send the ball out from under a rod
    drift_speed_px_s: float = 300.0 # exit box grows in y this fast (ball pinned and dragged)...
    drift_max_px: float = 100.0     # ...up to this much
    hidden_max_s: float = 3.0     # parked under a rod this long with no sighting -> SEARCH
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
    SEARCH    no track: full-frame detection each frame, except for the first
              reacquire_time_s after losing a track, when only the area where
              it was lost is searched.
    TENTATIVE first detection seen; needs confirm_hits (allowing up to
              tentative_max_misses missed frames) to become a track.
    TRACK     detection inside the predicted window updated the filter.
    COAST     no usable detection; output is the filter's prediction.
    After coast_time_s without a measurement (coast_time_slow_s if the ball
    was slower than slow_speed) the track is dropped to SEARCH.

    Rod dwell: a track whose measurements stay within rod_dwell_margin_px of
    the same rod rectangle (cfg.rod_zones) for rod_dwell_s is a player head,
    so it is dropped with no reacquire window (which would just grab the
    same head again).

    Under a rod (cfg.rod_zones are masked, so the ball is invisible there):
    entered when a track misses while the ball (prediction or last sighting)
    is within one ball radius of a strip. While under, every frame searches:
      1. the prediction window (normal gate): a ball just passing through;
      2. the exit box: strip +/- (hidden_margin + kick_speed * frame gap),
         entry y +/- the same plus drift_speed * time under (up to drift_max_px).
         A NEW blob here (not in the same spot last frame) is the ball coming
         out, re-initialised with a kick-sized velocity spread;
      3. the escape catcher: blob pairs over two frames, anywhere, that move
         away from the strip and back-project into its y window (a kick too
         fast to be seen in the exit box).
    COAST     (transit) output is still the prediction, so a ball passing
              through keeps moving. Becomes HIDDEN when the prediction leaves
              the exit box unseen, the ball was slow, or coast_time_s passes.
    HIDDEN    parked: output is a fixed guess (strip center x, entry y).
              Nothing for hidden_max_s -> SEARCH.

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
        self.misses = 0
        self.last_stamp = None
        self.last_meas = None
        self.last_z = None         # last measured (x, y)
        self.last_roi = None       # search window used on the last frame (None = full frame)
        self.last_cands = []       # every blob the detector returned on the last frame
        self.lost = None           # (roi, stamp) of the last lost track, for reacquiring
        self.zone = None           # index of the rod zone the last measurement was in
        self.zone_since = None     # stamp of the first consecutive measurement in that zone
        self.under = None          # dict while the ball is under a rod (see _go_under)

    def _zone_at(self, x, y):
        """Index of the rod rectangle (x, y) is in or within the margin of, or None."""
        m = self.cfg.rod_dwell_margin_px
        for i, (x0, y0, x1, y1) in enumerate(self.cfg.rod_zones):
            if x0 - m <= x <= x1 + m and y0 - m <= y <= y1 + m:
                return i
        return None

    def _zone_near(self, x, y, margin):
        """The rod rectangle (x, y) is in or within `margin` of, or None."""
        for r in self.cfg.rod_zones:
            x0, y0, x1, y1 = r
            if x0 - margin <= x <= x1 + margin and y0 - margin <= y <= y1 + margin:
                return r
        return None

    # -- under a rod -------------------------------------------------------------

    def _go_under(self, rect, stamp):
        """The ball went under rod rectangle `rect`. Parks right away if it was slow."""
        self.under = {"rect": rect, "y": self.last_z[1], "t": stamp,
                      "park_t": None, "prev": None, "prev_all": None}
        if math.hypot(self.kf.x[2], self.kf.x[3]) < self.cfg.slow_speed:
            self.under["park_t"] = stamp
            self.state = HIDDEN
        else:
            self.state = COAST

    def _exit_box(self, stamp, dt, w, h):
        """Where a ball leaving the strip can be seen on this frame; also its y window half-size."""
        c, u = self.cfg, self.under
        x0, _, x1, _ = u["rect"]
        mx = c.hidden_margin_px + c.kick_speed_px_s * dt
        my = mx + min(c.drift_speed_px_s * (stamp - u["t"]), c.drift_max_px)
        bx0, bx1 = int(max(0, math.floor(x0 - mx))), int(min(w, math.ceil(x1 + mx)))
        by0, by1 = int(max(0, math.floor(u["y"] - my))), int(min(h, math.ceil(u["y"] + my)))
        return (bx0, by0, max(bx1, bx0 + 1), max(by1, by0 + 1)), my

    def _hidden_out(self, stamp):
        x0, _, x1, _ = self.under["rect"]
        return TrackState(stamp, HIDDEN, (x0 + x1) / 2.0, float(self.under["y"]), 0.0, 0.0, False)

    def _escape(self, outside, stamp, ywin):
        """
        A blob pair (last frame -> this frame), outside the exit box, moving away
        from the strip faster than slow_speed and back-projecting into the strip's
        y window: a kick too fast to be seen in the exit box. Returns (z, v) or None.
        """
        c, u = self.cfg, self.under
        prev, u["prev"] = u["prev"], (stamp, outside)
        if prev is None or not outside:
            return None
        pt, pc = prev
        dtp = stamp - pt
        if dtp <= 0 or not pc:
            return None
        x0, _, x1, _ = u["rect"]
        mid = (x0 + x1) / 2.0
        best, best_err = None, float("inf")
        for cx, cy, _a in outside:
            for px, py, _pa in pc:
                vx, vy = (cx - px) / dtp, (cy - py) / dtp
                speed = math.hypot(vx, vy)
                if not (c.slow_speed < speed <= max(3.0 * c.kick_speed_px_s, 10000.0)):
                    continue
                side = 1.0 if cx > mid else -1.0
                if vx * side <= 0 or abs(cx - mid) <= abs(px - mid):
                    continue                          # not moving away from the strip
                edge = x1 if side > 0 else x0
                t_back = (cx - edge) / vx             # time since it crossed the strip edge
                if t_back < 0 or t_back > stamp - u["t"] + dtp:
                    continue
                err = abs(cy - vy * t_back - u["y"])
                if err <= ywin and err < best_err:
                    best, best_err = ((cx, cy), (vx, vy)), err
        return best

    def _update_under(self, gray, stamp, w, h):
        c, u = self.cfg, self.under
        dt = min(max(stamp - self.last_stamp, c.dt_min), c.dt_max)
        parked = u["park_t"] is not None
        box, ywin = self._exit_box(stamp, dt, w, h)
        if parked:
            draw = box
        else:
            self.kf.predict(dt)
            r = self._roi(w, h, stamp - self.last_meas)
            draw = (min(box[0], r[0]), min(box[1], r[1]), max(box[2], r[2]), max(box[3], r[3]))
        self.last_stamp = stamp
        cands = self.det.detect(gray)               # whole frame: the escape catcher needs it
        self.last_roi, self.last_cands = draw, cands

        z = None
        if not parked:                              # 1. just passing through
            z = self._associate([k for k in cands if draw[0] <= k[0] < draw[2]
                                 and draw[1] <= k[1] < draw[3]])
            if z is not None:
                self.kf.update(z)
        prev_all, u["prev_all"] = u["prev_all"], cands
        if z is None and prev_all is not None:      # 2. coming out of the exit box
                                                    #    (not on the first frame under: no
                                                    #    previous frame to tell new blobs by)
            # only NEW blobs: one sitting in the same spot last frame is a head, not the ball
            inside = [k for k in cands if box[0] <= k[0] < box[2] and box[1] <= k[1] < box[3]
                      and not any(abs(k[0] - q[0]) < 2.0 and abs(k[1] - q[1]) < 2.0
                                  for q in prev_all)]
            if inside:
                gx = (u["rect"][0] + u["rect"][2]) / 2.0
                scale = max(box[2] - box[0], 1.0)
                cx, cy, _ = min(inside, key=lambda k: math.hypot(k[0] - gx, k[1] - u["y"]) / scale
                                + self._size_cost(k[2]))
                z = (cx, cy)
                self.kf.init(z, vel_std=c.kick_speed_px_s)
            else:                                   # 3. kicked out too fast for the box
                outside = [k for k in cands
                           if not (box[0] <= k[0] < box[2] and box[1] <= k[1] < box[3])]
                esc = self._escape(outside, stamp, ywin)
                if esc is not None:
                    z, v = esc
                    self.kf.init(z, vel_std=c.kick_speed_px_s / 4.0)
                    self.kf.x[2], self.kf.x[3] = v
        if z is not None:
            self.under = None
            self.last_meas, self.last_z = stamp, z
            self.hits += 1
            self.zone, self.zone_since = self._zone_at(z[0], z[1]), stamp
            self.state = TRACK
            return self._out(stamp, True)

        if not parked:
            speed = math.hypot(self.kf.x[2], self.kf.x[3])
            if (not box[0] <= self.kf.x[0] < box[2] or speed < c.slow_speed
                    or stamp - self.last_meas > c.coast_time_s):
                u["park_t"] = stamp                 # it should be out by now: it stayed under
                self.state = HIDDEN
                return self._hidden_out(stamp)
            self.state = COAST
            return self._out(stamp, False)
        if stamp - u["park_t"] > c.hidden_max_s:    # never came out: search wider again
            lost = (box, stamp)
            self.reset()
            self.lost = lost
            return TrackState(stamp, SEARCH)
        return self._hidden_out(stamp)

    def _lose(self, stamp, w, h):
        """Drop to SEARCH, remembering where to look first."""
        if self.state == TENTATIVE:
            lost = self.lost       # unconfirmed: keep the window it was found in, if any
        else:
            p = self.cfg.reacquire_pad_px
            (mx, my), (px, py) = self.last_z, self.kf.x[:2]
            x0 = int(min(max(math.floor(min(mx, px) - p), 0), w - 1))
            y0 = int(min(max(math.floor(min(my, py) - p), 0), h - 1))
            x1 = int(min(max(math.ceil(max(mx, px) + p), x0 + 1), w))
            y1 = int(min(max(math.ceil(max(my, py) + p), y0 + 1), h))
            lost = ((x0, y0, x1, y1), stamp)
        self.reset()
        self.lost = lost

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
            roi = None
            if self.lost is not None:
                if stamp - self.lost[1] <= c.reacquire_time_s:
                    roi = self.lost[0]
                else:
                    self.lost = None
            cands = self.det.detect(gray, roi)
            self.last_roi, self.last_cands = roi, cands
            if not cands:
                return TrackState(stamp, SEARCH)
            cx, cy, _ = min(cands, key=lambda k: self._size_cost(k[2]))
            self.kf.init((cx, cy))
            self.state, self.hits, self.misses = TENTATIVE, 1, 0
            self.last_stamp = self.last_meas = stamp
            self.last_z = (cx, cy)
            return self._out(stamp, True)

        if self.under is not None:
            return self._update_under(gray, stamp, w, h)

        dt = min(max(stamp - self.last_stamp, c.dt_min), c.dt_max)
        self.kf.predict(dt)
        roi = self._roi(w, h, stamp - self.last_meas)
        cands = self.det.detect(gray, roi)
        self.last_roi, self.last_cands = roi, cands
        z = self._associate(cands)
        self.last_stamp = stamp

        if z is not None:
            zi = self._zone_at(z[0], z[1])
            if zi != self.zone:
                self.zone, self.zone_since = zi, stamp
            elif zi is not None and stamp - self.zone_since > c.rod_dwell_s:
                self.reset()                          # parked on a rod: it's a head, not the
                return TrackState(stamp, SEARCH)      # ball. No reacquire window on purpose.
            self.kf.update(z)
            self.last_meas = stamp
            self.last_z = z
            self.hits += 1
            if self.state == TENTATIVE:
                if self.hits >= c.confirm_hits:
                    self.state, self.lost = TRACK, None
            else:
                self.state = TRACK
            return self._out(stamp, True)

        if self.state == TENTATIVE:
            self.misses += 1
            if self.misses > c.tentative_max_misses:  # never confirmed: start over
                self._lose(stamp, w, h)
                return TrackState(stamp, SEARCH)
            return self._out(stamp, False)
        if c.rod_zones:                               # lost right at a rod mask: it's under there
            r = math.sqrt(c.ball_area / math.pi)
            rect = (self._zone_near(self.kf.x[0], self.kf.x[1], r)
                    or self._zone_near(self.last_z[0], self.last_z[1], r))
            if rect is not None:
                self._go_under(rect, stamp)
                return self._hidden_out(stamp) if self.state == HIDDEN else self._out(stamp, False)
        slow = math.hypot(self.kf.x[2], self.kf.x[3]) < c.slow_speed
        if stamp - self.last_meas > (c.coast_time_slow_s if slow else c.coast_time_s):
            self._lose(stamp, w, h)                   # coasted too long: re-search
            return TrackState(stamp, SEARCH)
        self.state = COAST
        return self._out(stamp, False)

    def lookahead(self, seconds):
        """(x, y) the ball is predicted to reach `seconds` from the last frame, or None."""
        if self.state not in (TRACK, COAST):
            return None
        x = self.kf.project(seconds)
        return float(x[0]), float(x[1])