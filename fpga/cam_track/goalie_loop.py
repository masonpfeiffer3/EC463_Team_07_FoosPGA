"""
goalie_loop.py

Joins the pieces, once per camera frame:

    tracker.TrackState -> strategy.Ball -> Strategy.update() -> Goalie.set_target_mm()

GoalieLink.step() is called from the tracking thread (kv260_view_tracker.
tracking_loop) for every frame, and with None on a pass that had no frame, so
the FPGA watchdog always gets its target. add_args() / build_link() give
Arducam_frames.py its --goalie options:

    python3 Arducam_frames.py --goalie sim                      # no hardware
    sudo python3 Arducam_frames.py --goalie fpga --goalie-base 0x... \
        --usteps-per-mm ... --max-speed-mm-s ... --accel-mm-s2 ...
"""

from goalie import DevMem, Goalie, GoalieConfig, SimRegs
from strategy import (IN_TO_MM, Strategy, StrategyConfig, TableFrame, ball_from_track,
                      goalie_x_from_masks)

UNITS_TO_MM = {"in": IN_TO_MM, "inch": IN_TO_MM, "mm": 1.0, "cm": 10.0}


class GoalieLink:
    def __init__(self, strategy, goalie, setup, frame, auto_x=True, log_path=None):
        self.strategy, self.goalie, self.setup, self.frame = strategy, goalie, setup, frame
        self.auto_x = auto_x          # take the goalie rod's x from the rod masks
        self.error = None             # last failure, printed once
        self.target = None            # last target sent, mm
        self.closed = False
        self.log = open(log_path, "w", buffering=1) if log_path else None
        if self.log:
            self.log.write("stamp,mode,ball_x,ball_y,ball_vx,ball_vy,target_mm,reject\n")

    def step(self, state):
        """state: tracker.TrackState, or None when there was no new frame."""
        if self.closed:               # shutting down: the tracking thread may run one more pass
            return
        try:
            if self.auto_x:
                self.strategy.cfg.goalie_x_mm = goalie_x_from_masks(self.setup, self.frame)
            ball = ball_from_track(state, self.setup, self.frame)
            y = self.strategy.update(ball)
            self.goalie.set_target_mm(y)
            self.target = y
            if self.log and state is not None:
                b = ball
                cols = (f"{b.x:.1f},{b.y:.1f},{b.vx:.0f},{b.vy:.0f}" if b else ",,,")
                self.log.write(f"{state.stamp:.6f},{self.strategy.mode},{cols},{y:.2f},"
                               f"{self.strategy.reject or ''}\n")
            self.error = None
        except Exception as e:      # never kill the tracking thread; no target = watchdog stop
            msg = f"{type(e).__name__}: {e}"
            if msg != self.error:
                self.error = msg
                print("goalie link:", msg)

    def pause(self):
        """Before something slow in the tracking thread (learning the background)."""
        self.goalie.disable()

    def resume(self):
        self.goalie.enable()

    def close(self):
        self.closed = True
        self.goalie.disable()
        s = self.strategy
        bad = sum(s.rejects.values())
        pct = 100.0 * bad / s.frames if s.frames else 0.0
        print(f"goalie: {s.frames} ball frames, {bad} rejected ({pct:.1f}%) {s.rejects or ''}")
        if self.log:
            self.log.close()


def add_args(ap):
    g = ap.add_argument_group("goalie")
    g.add_argument("--goalie", choices=["off", "sim", "fpga"], default="off",
                   help="sim = run the strategy with no hardware; fpga = write the motor registers")
    g.add_argument("--goalie-base", type=lambda s: int(s, 0),
                   help="motor_control_core base address (Vivado Address Editor)")
    g.add_argument("--usteps-per-mm", type=float, help="microsteps per mm; negative flips +y")
    g.add_argument("--center-usteps", type=int, default=0,
                   help="position counter value with the figure at the goal center")
    g.add_argument("--max-speed-mm-s", type=float)
    g.add_argument("--accel-mm-s2", type=float)
    g.add_argument("--clamp-mm", type=float, default=65.0,
                   help="goalie figure center is held to +/- this (placeholder)")
    g.add_argument("--goalie-x-mm", type=float,
                   help="goalie rod distance from our goal line; default: the nearest rod mask")
    g.add_argument("--goal-far", action="store_true",
                   help="our goal is at the far-corner end of the table, not the (0,0) end")
    g.add_argument("--flip-y", action="store_true", help="reverse the +y direction")
    g.add_argument("--ball-radius-mm", type=float, default=17.5)
    g.add_argument("--goalie-log",
                   help="CSV: stamp,mode,ball x/y/vx/vy (mm),target_mm,reject reason")


def build_link(args, setup):
    """GoalieLink from the command line, or None with --goalie off."""
    if args.goalie == "off":
        return None
    if args.units not in UNITS_TO_MM:
        raise SystemExit(f"--goalie needs --units in, mm or cm (got '{args.units}')")
    frame = TableFrame.from_setup(setup, UNITS_TO_MM[args.units],
                                  goal_at_x0=not args.goal_far,
                                  y_sign=-1.0 if args.flip_y else 1.0)
    strat = Strategy(StrategyConfig(clamp_mm=args.clamp_mm, goalie_x_mm=args.goalie_x_mm,
                                    table_half_width_mm=frame.width_mm / 2.0,
                                    table_length_mm=frame.length_mm,
                                    ball_radius_mm=args.ball_radius_mm))
    if args.goalie == "sim":
        regs = SimRegs()
        cfg = GoalieConfig(args.usteps_per_mm or 100.0, args.max_speed_mm_s or 1000.0,
                           args.accel_mm_s2 or 10000.0, args.center_usteps, args.clamp_mm)
    else:
        missing = [n for n, v in (("--goalie-base", args.goalie_base),
                                  ("--usteps-per-mm", args.usteps_per_mm),
                                  ("--max-speed-mm-s", args.max_speed_mm_s),
                                  ("--accel-mm-s2", args.accel_mm_s2)) if v is None]
        if missing:
            raise SystemExit("--goalie fpga also needs: " + ", ".join(missing))
        regs = DevMem(args.goalie_base)
        cfg = GoalieConfig(args.usteps_per_mm, args.max_speed_mm_s, args.accel_mm_s2,
                           args.center_usteps, args.clamp_mm)
    goalie = Goalie(regs, cfg)
    ok, msg = goalie.check()
    print(f"goalie ({args.goalie}): {msg}")
    if not ok:
        raise SystemExit("goalie: register check failed, not enabling motion")
    goalie.enable()
    return GoalieLink(strat, goalie, setup, frame, auto_x=args.goalie_x_mm is None,
                      log_path=args.goalie_log)
