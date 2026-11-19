"""
test_strategy.py: laptop tests for strategy.py (no camera, no FPGA).

    python3 test_strategy.py        # or: pytest test_strategy.py
"""

import math
import time
import types

from strategy import (Ball, Strategy, StrategyConfig, TableFrame, ball_from_track,
                      fold, goalie_x_from_masks)

ROD = 80.0     # goalie rod x used in these tests, mm


def shot(x=580.0, y=0.0, vx=-5000.0, vy=0.0, t=0.0, **kw):
    return Ball(x, y, vx, vy, t, **kw)


def test_no_ball_on_first_call_gives_home():
    assert Strategy().update(None, now=0.0) == 0.0


def test_follow_when_rod_position_unknown():
    s = Strategy()                                   # goalie_x_mm = None
    assert s.update(shot(y=30.0, vy=0.0), now=0.0) == 30.0
    assert s.mode == "FOLLOW"


def test_clamp_both_sides():
    assert Strategy().update(Ball(500, 200), now=0.0) == 65.0
    assert Strategy().update(Ball(500, -200), now=0.0) == -65.0


def test_output_is_finite_for_bad_input():
    s = Strategy()
    for bad in (Ball(float("nan"), 0), Ball(0, float("inf")), Ball(0, 0, vy=float("nan"))):
        assert math.isfinite(s.update(bad, now=0.0))


def test_intercept_straight_shot():
    s = Strategy(StrategyConfig(goalie_x_mm=ROD, intercept_confirm=1))
    # 500 mm away at 5 m/s = 0.1 s; drifts 0.1 s * 500 mm/s = 50 mm
    assert abs(s.update(shot(vy=500.0), now=0.0) - 50.0) < 1e-6
    assert s.mode == "INTERCEPT"


def test_fold_one_bounce():
    y, n = fold(400.0, 325.4)
    assert abs(y - 250.8) < 1e-6 and n == 1
    y, n = fold(-400.0, 325.4)
    assert abs(y + 250.8) < 1e-6 and n == 1
    assert fold(100.0, 325.4) == (100.0, 0)


def test_intercept_with_wall_bounce():
    s = Strategy(StrategyConfig(goalie_x_mm=ROD, clamp_mm=400.0, intercept_confirm=1))
    # raw crossing 300 + 0.1 * 1000 = 400; wall for the ball center at 342.9 - 17.5 = 325.4
    assert abs(s.update(shot(y=300.0, vy=1000.0), now=0.0) - 250.8) < 1e-6


def test_follow_cases_that_are_not_shots():
    cfg = StrategyConfig(goalie_x_mm=ROD, intercept_confirm=1)
    cases = {
        "moving away": shot(y=20.0, vx=5000.0, vy=500.0),
        "too slow": shot(y=20.0, vx=-100.0, vy=500.0),
        "behind the rod": shot(x=40.0, y=20.0, vy=500.0),
        "too far in time": shot(x=1100.0, y=20.0, vx=-400.0, vy=10.0),
        "too many bounces": shot(y=20.0, vx=-400.0, vy=3000.0),
        "hidden guess": shot(y=20.0, vy=500.0, confident=False),
    }
    for name, ball in cases.items():
        s = Strategy(cfg)
        assert s.update(ball, now=0.0) == 20.0, name
        assert s.mode == "FOLLOW", name


def test_follow_leads_by_frame_age_up_to_the_cap():
    s = Strategy()
    assert abs(s.update(Ball(500, 0, vy=1000.0, t=0.0), now=0.010) - 10.0) < 1e-6
    s = Strategy()
    assert abs(s.update(Ball(500, 0, vy=1000.0, t=0.0), now=0.08) - 50.0) < 1e-6  # cap 0.05 s


def test_deadband():
    s = Strategy()
    assert s.update(Ball(500, 30.0), now=0.0) == 30.0
    assert s.update(Ball(500, 31.0, t=0.01), now=0.01) == 30.0      # 1 mm < 2 mm
    assert s.update(Ball(500, 33.0, t=0.02), now=0.02) == 33.0


def test_ball_lost_holds_then_returns_home():
    s = Strategy()
    assert s.update(Ball(500, 50.0), now=0.0) == 50.0
    assert s.update(None, now=0.4) == 50.0 and s.mode == "HOLD"
    assert abs(s.update(None, now=0.6) - 10.0) < 1e-6 and s.mode == "HOME"   # 0.2 s at 200 mm/s
    assert s.update(None, now=0.7) == 0.0        # 10 mm left, one 20 mm step: stops at home
    assert s.update(None, now=5.0) == 0.0


class FakeSetup:
    """table_setup.TableSetup stand-in: 0.1 in per px, origin at px (0, 270), y up."""
    size = (47.0, 27.0)
    g = types.SimpleNamespace(x0=0, y0=0)
    spans = [(20, 40), (200, 220), (430, 450)]

    def to_table(self, x, y):
        return 0.1 * x, 0.1 * (270.0 - y)

    def vel_to_table(self, vx, vy):
        return 0.1 * vx, -0.1 * vy


def track(state="TRACK", x=100.0, y=135.0, vx=-1000.0, vy=0.0):
    return types.SimpleNamespace(stamp=1.5, state=state, x=x, y=y, vx=vx, vy=vy)


def test_adapter_units_origin_and_sides():
    setup = FakeSetup()
    near = TableFrame.from_setup(setup)                      # our goal at table X = 0
    b = ball_from_track(track(), setup, near)
    assert abs(b.x - 254.0) < 1e-6 and abs(b.y) < 1e-6       # 10 in, table center line
    assert abs(b.vx + 2540.0) < 1e-6 and b.t == 1.5 and b.confident

    far = TableFrame.from_setup(setup, goal_at_x0=False, y_sign=-1.0)
    b = ball_from_track(track(y=35.0), setup, far)
    assert abs(b.x - (47.0 - 10.0) * 25.4) < 1e-6            # measured from the other end
    assert abs(b.vx - 2540.0) < 1e-6                         # now moving away from us
    assert abs(b.y + 10.0 * 25.4) < 1e-6                     # 10 in above center, sign flipped


def test_adapter_states():
    setup, frame = FakeSetup(), TableFrame.from_setup(FakeSetup())
    assert ball_from_track(None, setup, frame) is None
    assert ball_from_track(track("SEARCH"), setup, frame) is None
    assert ball_from_track(track("TENTATIVE"), setup, frame) is None
    assert ball_from_track(track("COAST"), setup, frame).confident
    assert not ball_from_track(track("HIDDEN"), setup, frame).confident


def test_impossible_balls_are_rejected_and_the_target_holds():
    cases = {
        "off_table": Ball(-2009.7, 1070.0, -1000, 500, t=0.01),      # the 2026-11-16 log glitch
        "too_fast": Ball(500, 20, -19370, 7444, t=0.01),
        "stale": Ball(500, 20, t=-0.5),
        "not_finite": Ball(500, float("nan"), t=0.01),
    }
    for reason, bad in cases.items():
        s = Strategy(StrategyConfig(table_length_mm=1193.8))
        assert s.update(Ball(500, 30.0, t=0.0), now=0.0) == 30.0
        assert s.update(bad, now=0.01) == 30.0, reason               # held, not followed
        assert s.reject == reason and s.rejects == {reason: 1} and s.mode == "HOLD", reason
    s = Strategy(StrategyConfig(table_length_mm=1193.8))
    assert s.update(Ball(1193.8 + 40, 343 + 40, t=0.0), now=0.0) == 65.0     # inside the margin
    assert s.reject is None


def test_a_jump_is_ignored_until_it_persists():
    s = Strategy()
    assert s.update(Ball(500, 30.0, t=0.000), now=0.000) == 30.0
    # 7 ms later the "ball" is 400 mm away: max 15 m/s * 7 ms + 30 mm = 135 mm allowed
    assert s.update(Ball(900, -40.0, t=0.007), now=0.007) == 30.0 and s.reject == "jump"
    assert s.update(Ball(900, -40.0, t=0.014), now=0.014) == 30.0 and s.reject == "jump"
    assert s.update(Ball(900, -40.0, t=0.021), now=0.021) == -40.0 and s.reject is None   # 3rd frame
    assert s.rejects == {"jump": 2} and s.frames == 4


def test_a_single_frame_jump_does_not_move_the_goalie():
    s = Strategy()
    s.update(Ball(500, 30.0, t=0.000), now=0.000)
    assert s.update(Ball(900, -40.0, t=0.007), now=0.007) == 30.0
    assert s.update(Ball(501, 33.0, t=0.014), now=0.014) == 33.0 and s.reject is None     # back to normal


def test_intercept_needs_two_frames_in_a_row():
    s = Strategy(StrategyConfig(goalie_x_mm=ROD))
    first = s.update(shot(x=580.0, vy=500.0, t=0.000), now=0.000)
    assert s.mode == "FOLLOW" and abs(first) < 1e-6                  # not yet believed: follows y
    second = s.update(shot(x=545.0, y=3.5, vy=500.0, t=0.007), now=0.007)
    assert s.mode == "INTERCEPT" and abs(second - 50.0) < 1e-6


def test_goalie_x_from_masks_takes_the_mask_nearest_our_goal():
    setup = FakeSetup()
    assert abs(goalie_x_from_masks(setup, TableFrame.from_setup(setup)) - 3.0 * 25.4) < 1e-6
    far = TableFrame.from_setup(setup, goal_at_x0=False)
    assert abs(goalie_x_from_masks(setup, far) - (47.0 - 44.0) * 25.4) < 1e-6


def time_update(n=20000):
    s = Strategy(StrategyConfig(goalie_x_mm=ROD))
    b = shot(vy=500.0)
    t0 = time.perf_counter()
    for i in range(n):
        s.update(b, now=i * 0.007)
    return (time.perf_counter() - t0) / n * 1e6


if __name__ == "__main__":
    tests = [(k, v) for k, v in sorted(globals().items()) if k.startswith("test_")]
    for name, fn in tests:
        fn()
        print("ok  ", name)
    print(f"{len(tests)} tests passed; update() takes about {time_update():.1f} us per call here")
