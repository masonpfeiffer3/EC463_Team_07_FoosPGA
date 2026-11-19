"""
strategy.py

Goalie strategy: ball state in -> absolute goalie target in mm out.
Pure Python (math, time, dataclasses), so it runs on a laptop from recorded
ball data as well as on the Kria. No camera, FPGA or OpenCV imports.

Contract (FoosPGA Goalie Strategy Interface v1, 2026-11-14):

    y = strategy.update(ball)        # every loop pass; ball may be None
    goalie.set_target_mm(y)          # done by the caller, never in here

The return value is always a finite float inside +/- cfg.clamp_mm.

Strategy frame (mm and mm/s):
    x   distance from OUR goal line, growing toward the opponent
    y   across the table, 0 = centre of our goal
so a ball coming at us has vx < 0. ball_from_track() converts a
tracker.TrackState into this frame.

What update() does:
    no ball, or a ball      hold the last target for lost_hold_s, then move
    that cannot be real     back to home_mm at return_speed_mm_s. "Cannot be
                            real" = off the table, too fast, a stale frame, or
                            a jump the ball could not have made (see _reject).
                            Strategy.rejects counts each reason.
    ball approaching fast   aim at where it will cross the goalie rod
                            (straight line, mirrored at the side walls)
    anything else           follow the ball's y

Wiring on the Kria (after trk.update() in the tracking loop):

    frame = TableFrame.from_setup(setup, goal_at_x0=True, y_sign=1.0)
    strat = Strategy(StrategyConfig(goalie_x_mm=goalie_x_from_masks(setup, frame),
                                    table_half_width_mm=frame.width_mm / 2))
    ...
    y = strat.update(ball_from_track(state, setup, frame))
"""

import math
import time
from dataclasses import dataclass
from typing import Optional

IN_TO_MM = 25.4


# ============================================================================
# Input and settings
# ============================================================================

@dataclass
class Ball:
    x: float                  # mm from our goal line
    y: float                  # mm from the goal centre
    vx: float = 0.0           # mm/s, negative = coming at us
    vy: float = 0.0
    t: float = 0.0            # capture time of the frame, s (time.monotonic)
    confident: bool = True    # False: position is a guess (tracker HIDDEN), so no prediction


@dataclass
class StrategyConfig:
    # Geometry. Goal mouth measured 2026-11-15: 8 in = 203 mm (+/- 101.6 mm).
    clamp_mm: float = 65.0                # limit for the goalie figure CENTER. Placeholder from
                                          # the interface doc; the motor side is measuring it.
    home_mm: float = 0.0                  # where to wait when there is no ball
    goalie_x_mm: Optional[float] = None   # goalie rod's distance from our goal line.
                                          # None = not known yet: follow only, no prediction.
    table_half_width_mm: float = 342.9    # 27 in / 2: side walls, for bounces
    ball_radius_mm: float = 17.5          # ASSUMED (35 mm ball): measure it

    # Intercept prediction: only for a ball that is clearly a shot at us
    min_approach_mm_s: float = 300.0      # slower toward us than this -> follow
    max_predict_s: float = 1.0            # further away in time than this -> follow
    max_bounces: int = 2                  # more wall bounces than this -> follow

    # Following
    max_lead_s: float = 0.05              # lead the ball by the frame's age, up to this much
    deadband_mm: float = 2.0              # ignore target changes smaller than this (jitter)

    # Distrust of the tracker: a ball that fails one of these is treated as "no ball"
    table_length_mm: Optional[float] = None   # None = skip the x part of the off-table check
    margin_mm: float = 50.0               # further outside the table than this = not a ball
    max_ball_speed_mm_s: float = 15000.0  # faster than this = not a ball (design worst case 10 m/s)
    max_age_s: float = 0.10               # frame older than this = stale (loop fell behind)
    jump_margin_mm: float = 30.0          # moved further than max speed allows, plus this...
    jump_confirm: int = 3                 # ...is ignored until it stays there this many frames
    intercept_confirm: int = 2            # frames in a row that must call it a shot

    # Ball lost
    lost_hold_s: float = 0.5              # keep the last target this long...
    return_speed_mm_s: float = 200.0      # ...then move back to home_mm at this speed


# ============================================================================
# Strategy
# ============================================================================

def fold(y, half):
    """
    Mirror y back into [-half, +half] as a ball bouncing between two side
    walls would. Returns (y, number of bounces).
    """
    span = 2.0 * half
    bounces = abs(math.floor((y + half) / span))
    m = (y + half) % (2.0 * span)
    if m > span:
        m = 2.0 * span - m
    return m - half, bounces


def _finite(ball):
    return all(isinstance(v, (int, float)) and math.isfinite(v)
               for v in (ball.x, ball.y, ball.vx, ball.vy, ball.t))


class Strategy:
    def __init__(self, cfg=None):
        self.cfg = cfg or StrategyConfig()
        c = self.cfg
        if not (math.isfinite(c.clamp_mm) and c.clamp_mm >= 0):
            raise ValueError("clamp_mm must be a finite number >= 0")
        if c.table_half_width_mm <= c.ball_radius_mm:
            raise ValueError("table_half_width_mm must be larger than ball_radius_mm")
        self.target = self._clamp(c.home_mm)
        self.mode = "HOME"          # last decision, for logs: FOLLOW / INTERCEPT / HOLD / HOME
        self.reject = None          # why the last ball was thrown away, or None
        self.rejects = {}           # reason -> count since start
        self.frames = 0             # balls offered to update() since start
        self._last_seen = None      # time update() last got a usable ball
        self._last_call = None
        self._prev = None           # (x, y, t) of the last accepted ball
        self._jump = None           # (x, y, t, frames) of a ball waiting to be believed
        self._shots = 0             # frames in a row that _intercept() called a shot

    def _clamp(self, y):
        return min(max(float(y), -self.cfg.clamp_mm), self.cfg.clamp_mm)

    def update(self, ball, now=None):
        """
        ball: Ball or None. Returns the absolute goalie target in mm.
        now: time.monotonic() of this call. Pass it when replaying recorded
        data (e.g. now=ball.t); leave it out on the Kria.
        """
        c = self.cfg
        now = time.monotonic() if now is None else now
        dt = 0.0 if self._last_call is None else max(0.0, now - self._last_call)
        self._last_call = now

        self.reject = None
        if ball is not None:
            self.frames += 1
            self.reject = self._reject(ball, now)
            if self.reject:
                self.rejects[self.reject] = self.rejects.get(self.reject, 0) + 1
        if ball is None or self.reject:
            self._shots = 0
            return self._lost(now, dt)
        self._last_seen = now

        want = self._intercept(ball)
        self._shots = self._shots + 1 if want is not None else 0
        if want is not None and self._shots < c.intercept_confirm:
            want = None                           # one frame is not enough to call it a shot
        if want is None:
            lead = min(max(now - ball.t, 0.0), c.max_lead_s) if ball.confident else 0.0
            want = ball.y + ball.vy * lead
            self.mode = "FOLLOW"
        else:
            self.mode = "INTERCEPT"

        want = self._clamp(want)
        if abs(want - self.target) >= c.deadband_mm:
            self.target = want
        return self.target

    def _reject(self, ball, now):
        """Reason this ball cannot be real, or None to accept it."""
        c = self.cfg
        if not _finite(ball):
            return "not_finite"
        m = c.margin_mm
        if abs(ball.y) > c.table_half_width_mm + m or ball.x < -m or (
                c.table_length_mm is not None and ball.x > c.table_length_mm + m):
            return "off_table"
        if math.hypot(ball.vx, ball.vy) > c.max_ball_speed_mm_s:
            return "too_fast"
        if now - ball.t > c.max_age_s:
            return "stale"
        if self._prev is not None:
            px, py, pt = self._prev
            dt = ball.t - pt
            if 0.0 < dt < 0.5 and (math.hypot(ball.x - px, ball.y - py)
                                   > c.max_ball_speed_mm_s * dt + c.jump_margin_mm):
                # Too far from the last good ball. Believe it only once it has stayed
                # put for jump_confirm frames (a real ball found again somewhere else).
                n = 1
                if self._jump is not None:
                    jx, jy, jt, jn = self._jump
                    if math.hypot(ball.x - jx, ball.y - jy) <= (
                            c.max_ball_speed_mm_s * max(ball.t - jt, 0.0) + c.jump_margin_mm):
                        n = jn + 1
                self._jump = (ball.x, ball.y, ball.t, n)
                if n < c.jump_confirm:
                    return "jump"
        self._prev, self._jump = (ball.x, ball.y, ball.t), None
        return None

    def _intercept(self, ball):
        """y where the ball crosses the goalie rod, or None if this is not a shot at us."""
        c = self.cfg
        if c.goalie_x_mm is None or not ball.confident:
            return None
        if ball.vx > -c.min_approach_mm_s:        # moving away, or too slow to call a shot
            return None
        dx = ball.x - c.goalie_x_mm
        if dx <= 0:                               # level with the rod or already behind it
            return None
        t_hit = dx / -ball.vx
        if t_hit > c.max_predict_s:
            return None
        y, bounces = fold(ball.y + ball.vy * t_hit, c.table_half_width_mm - c.ball_radius_mm)
        return None if bounces > c.max_bounces else y

    def _lost(self, now, dt):
        c = self.cfg
        if self._last_seen is not None and now - self._last_seen <= c.lost_hold_s:
            self.mode = "HOLD"
            return self.target
        self.mode = "HOME"
        home = self._clamp(c.home_mm)
        step = c.return_speed_mm_s * dt
        gap = home - self.target
        self.target = home if abs(gap) <= step else self.target + math.copysign(step, gap)
        return self.target


# ============================================================================
# Tracker -> strategy frame
# ============================================================================

@dataclass
class TableFrame:
    """How table_setup's table coordinates map to the strategy frame."""
    length_mm: float            # table size along table X (the image's horizontal axis)
    width_mm: float             # table size along table Y
    units_to_mm: float = 1.0    # 25.4 when table_setup works in inches
    goal_at_x0: bool = True     # True: our goal is at the table X = 0 end (the clicked origin)
    y_sign: float = 1.0         # +1: strategy +y = table +Y (up in the image). -1 flips it.

    @classmethod
    def from_setup(cls, setup, units_to_mm=IN_TO_MM, goal_at_x0=True, y_sign=1.0):
        w, h = setup.size
        return cls(w * units_to_mm, h * units_to_mm, units_to_mm, goal_at_x0, y_sign)

    def point(self, tx, ty):
        """Table units -> (x, y) in the strategy frame."""
        x, y = tx * self.units_to_mm, ty * self.units_to_mm
        return (x if self.goal_at_x0 else self.length_mm - x,
                self.y_sign * (y - self.width_mm / 2.0))

    def velocity(self, tvx, tvy):
        k = self.units_to_mm
        return (tvx * k if self.goal_at_x0 else -tvx * k), self.y_sign * tvy * k


def ball_from_track(state, setup, frame):
    """
    tracker.TrackState -> Ball, or None when there is nothing to act on:
    no track (SEARCH), an unconfirmed one (TENTATIVE), or table corners not set.
    Whether the numbers can be a real ball is checked later, in Strategy.update().
    A ball parked under a rod (HIDDEN) is passed on as a low-confidence guess.
    """
    if state is None or state.x is None or state.state in ("SEARCH", "TENTATIVE"):
        return None
    p = setup.to_table(state.x, state.y)
    v = setup.vel_to_table(state.vx, state.vy)
    if p is None or v is None:
        return None
    x, y = frame.point(*p)
    vx, vy = frame.velocity(*v)
    return Ball(x, y, vx, vy, state.stamp, confident=state.state != "HIDDEN")


def goalie_x_from_masks(setup, frame):
    """
    Goalie rod x in the strategy frame, taken from the rod masks drawn on the
    web page: the mask closest to our goal line. None if there are no masks or
    the corners are not set. Only right if our goalie rod has a mask.
    """
    xs = []
    for x0, x1 in list(setup.spans):
        p = setup.to_table((x0 + x1) / 2.0 - setup.g.x0, 0.0)
        if p is None:
            return None
        xs.append(frame.point(p[0], 0.0)[0])
    xs = [x for x in xs if x > 0]
    return min(xs) if xs else None
