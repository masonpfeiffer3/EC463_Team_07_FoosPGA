"""
test_goalie.py: laptop tests for goalie.py and goalie_loop.py (SimRegs, no hardware).

    python3 test_goalie.py        # or: pytest test_goalie.py
"""

import time
import types

import goalie as G
from goalie import Goalie, GoalieConfig, SimRegs
from goalie_loop import GoalieLink
from strategy import Strategy, StrategyConfig, TableFrame
from test_strategy import FakeSetup, track

AX = G.AXIS_BASE            # axis 0


def make(**kw):
    regs = SimRegs()
    g = Goalie(regs, GoalieConfig(usteps_per_mm=80.0, max_speed_mm_s=500.0,
                                  accel_mm_s2=5000.0, **kw))
    return regs, g


def test_check_passes_on_sim_and_fails_on_dead_bus():
    _, g = make()
    assert g.check()[0]
    dead = types.SimpleNamespace(read=lambda off: 0, write=lambda off, v: None)
    assert not Goalie(dead, g.cfg).check()[0]


def test_check_writes_nothing_to_an_unknown_design():
    seen = []
    other = types.SimpleNamespace(read=lambda off: 0x12345678, write=lambda off, v: seen.append(off))
    _, g = make()
    assert not Goalie(other, g.cfg).check()[0] and seen == []


def test_wordmap_is_whole_32_bit_words():
    import mmap
    m = G.WordMap(mmap.mmap(-1, 4096))           # anonymous memory stands in for /dev/mem
    m.write(0x40, -800)                          # negative -> two's complement
    assert m.read(0x40) == 0xFFFFFCE0 and G._signed(m.read(0x40)) == -800
    assert m.mem[0x40:0x44] == bytes([0xE0, 0xFC, 0xFF, 0xFF])      # little-endian on the bus
    m.write(0x44, 0xDEADBEEF)
    assert m.read(0x44) == 0xDEADBEEF and m.read(0x40) == 0xFFFFFCE0
    for bad in (0x41, 0x42):
        try:
            m.write(bad, 1)
        except ValueError:
            continue
        raise AssertionError("unaligned offset accepted")


def test_goalie_on_a_wordmap():
    import mmap
    m = G.WordMap(mmap.mmap(-1, 4096))
    m.write(G.VERSION, (G.VERSION_MAGIC << 16) | 0x0102)
    g = Goalie(m, GoalieConfig(usteps_per_mm=80.0, max_speed_mm_s=500.0, accel_mm_s2=5000.0,
                               require_homed=False))
    assert g.check() == (True, "motor_control_core v1.2")
    g.enable()
    g.set_target_mm(-10.0)
    assert G._signed(m.read(AX + G.TARGET_POSITION)) == -800
    assert m.read(AX + G.COMMAND) == G.CMD_GO


def test_devmem_refuses_an_unaligned_base():
    try:
        G.DevMem(0xB0000004)
    except ValueError:
        return
    raise AssertionError("unaligned base accepted")


def test_enable_writes_limits_speed_and_watchdog():
    regs, g = make(center_usteps=1000)
    g.enable()
    assert G._signed(regs.mem[AX + G.SOFT_LIMIT_MIN]) == 1000 - 65 * 80
    assert regs.mem[AX + G.SOFT_LIMIT_MAX] == 1000 + 65 * 80
    assert regs.mem[AX + G.MAX_VELOCITY] == 500 * 80
    assert regs.mem[AX + G.ACCELERATION] == 5000 * 80
    assert regs.mem[G.WDOG_PERIOD] == 100000
    assert regs.mem[G.GLOBAL_CTRL] == G.CTRL_SER_ENABLE | G.CTRL_WDOG_ENABLE | G.CTRL_MASTER_ENABLE
    assert g.ready()


def test_target_is_microsteps_then_go_then_kick():
    regs, g = make()
    g.enable()
    regs.writes.clear()
    g.set_target_mm(10.0)
    assert list(regs.writes) == [(AX + G.TARGET_POSITION, 800), (AX + G.COMMAND, G.CMD_GO),
                                 (G.WDOG_KICK, 1)]
    assert g.position_mm() == 10.0


def test_negative_target_is_a_32_bit_twos_complement_write():
    regs, g = make()
    g.enable()
    g.set_target_mm(-10.0)
    assert regs.mem[AX + G.TARGET_POSITION] == 0xFFFFFFFF - 800 + 1
    assert g.position_mm() == -10.0


def test_target_is_clamped_to_travel():
    regs, g = make()
    g.enable()
    g.set_target_mm(500.0)
    assert g.position_mm() == 65.0


def test_negative_scale_flips_direction():
    regs = SimRegs()
    g = Goalie(regs, GoalieConfig(usteps_per_mm=-80.0, max_speed_mm_s=500.0, accel_mm_s2=5000.0))
    g.enable()
    g.set_target_mm(10.0)
    assert G._signed(regs.mem[AX + G.TARGET_POSITION]) == -800
    assert g.position_mm() == 10.0
    assert G._signed(regs.mem[AX + G.SOFT_LIMIT_MIN]) < regs.mem[AX + G.SOFT_LIMIT_MAX]


def test_non_finite_target_is_refused():
    _, g = make()
    for bad in (float("nan"), float("inf"), None):
        try:
            g.set_target_mm(bad)
        except ValueError:
            continue
        raise AssertionError(f"accepted {bad!r}")


def test_not_homed_sends_no_move_but_still_kicks():
    regs, g = make()
    g.enable()
    regs.read = lambda off: 0            # axis STATUS without HOMED
    regs.writes.clear()
    g.set_target_mm(10.0)
    assert list(regs.writes) == [(G.WDOG_KICK, 1)]


def test_link_runs_tracker_state_to_register_write():
    setup = FakeSetup()
    frame = TableFrame.from_setup(setup)
    regs, g = make()
    g.enable()
    link = GoalieLink(Strategy(StrategyConfig()), g, setup, frame)
    state = track(y=125.0, vx=0.0)               # 10 px above the center line = 1 in = 25.4 mm
    state.stamp = time.monotonic()               # a fresh frame; an old stamp is rejected as stale
    link.step(state)
    assert abs(g.position_mm() - 25.4) < 0.01
    assert link.strategy.cfg.goalie_x_mm == 3.0 * 25.4      # from the nearest rod mask
    regs.writes.clear()
    link.step(None)                              # no frame: still a target and a kick
    assert (G.WDOG_KICK, 1) in regs.writes and link.error is None


def test_link_survives_a_failing_goalie():
    setup = FakeSetup()
    bad = types.SimpleNamespace(set_target_mm=lambda y: 1 / 0)
    link = GoalieLink(Strategy(), bad, setup, TableFrame.from_setup(setup))
    link.step(track())
    assert "ZeroDivisionError" in link.error


if __name__ == "__main__":
    tests = [(k, v) for k, v in sorted(globals().items()) if k.startswith("test_")]
    for name, fn in tests:
        fn()
        print("ok  ", name)
    print(f"{len(tests)} tests passed")
