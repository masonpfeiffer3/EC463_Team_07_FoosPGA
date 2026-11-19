"""
goalie.py

Goalie linear axis: a target in mm -> register writes to motor_control_core.
This is the last Python step before the FPGA:

    y = strategy.update(ball)        # strategy.py
    goalie.set_target_mm(y)          # here: mm -> microsteps -> AXI registers

NOT YET RUN ON HARDWARE. The register offsets below are copied from
fpga/docs/register_map.md, which is marked draft (2026-09-11) and has NOT been
checked against the RTL. The RTL (or the motor-side owner) is the only
reliable source: confirm every constant in the block below before --goalie
fpga. check() refuses to enable motion if the VERSION magic is not there.

`regs` is anything with read(offset) and write(offset, value) on 32-bit words:
    DevMem(base)     /dev/mem + numpy (needs sudo)
    SimRegs()        no hardware: for a laptop, or the Kria without a bitstream
    pynq.MMIO(base, 0x1000) has the same two methods and works as well.

Not done here, and needed before the goalie moves: homing and AXIS_CONFIG
(step/dir polarity, step pulse width). Until the axis reports HOMED,
set_target_mm() keeps the watchdog alive but sends no move.
"""

import collections
import math
import mmap
import os
from dataclasses import dataclass

# ---- register map (register_map.md) ------------------------------------------
VERSION, GLOBAL_CTRL, GLOBAL_STATUS = 0x000, 0x004, 0x008
WDOG_PERIOD, WDOG_KICK, SCRATCH = 0x010, 0x014, 0x038
VERSION_MAGIC = 0xF005                      # VERSION[31:16]
CTRL_MASTER_ENABLE, CTRL_CLEAR_ALL_FAULTS = 1 << 0, 1 << 1
CTRL_WDOG_ENABLE, CTRL_SER_ENABLE = 1 << 2, 1 << 3
GS_ENABLED, GS_WDOG_TRIPPED, GS_ESTOP_OK = 1 << 0, 1 << 3, 1 << 4

AXIS_BASE, AXIS_STRIDE = 0x040, 0x40        # axis N at AXIS_BASE + N * AXIS_STRIDE
TARGET_POSITION, MAX_VELOCITY, ACCELERATION = 0x00, 0x04, 0x08
COMMAND, STATUS, CURRENT_POSITION = 0x10, 0x14, 0x18
SOFT_LIMIT_MIN, SOFT_LIMIT_MAX = 0x24, 0x28
CMD_GO = 1 << 0
AX_HOMED, AX_FAULT = 1 << 4, 1 << 8
# ------------------------------------------------------------------------------


def _signed(v):
    return v - (1 << 32) if v & 0x80000000 else v


FPGA_STATE = "/sys/class/fpga_manager/fpga0/state"


def pl_state():
    """'operating' once a bitstream is loaded (xmutil loadapp), or None if it can't be read."""
    try:
        with open(FPGA_STATE) as f:
            return f.read().strip()
    except OSError:
        return None


class WordMap:
    """
    32-bit register access on a mapped buffer. Each read or write is ONE
    aligned 32-bit access, through a uint32 view (the method PYNQ's MMIO
    uses). struct.pack_into() is not used on purpose: it can reach the bus as
    several partial writes, which would fire a write-1-to-pulse register
    (COMMAND, WDOG_KICK) more than once or show the FPGA a half-written target.
    """

    def __init__(self, mem):
        import numpy as np
        self.mem = mem
        self.words = np.frombuffer(mem, np.uint32)
        self._u32 = np.uint32

    def read(self, off):
        if off & 3:
            raise ValueError(f"register offset 0x{off:X} is not 32-bit aligned")
        return int(self.words[off >> 2])

    def write(self, off, value):
        if off & 3:
            raise ValueError(f"register offset 0x{off:X} is not 32-bit aligned")
        self.words[off >> 2] = self._u32(value & 0xFFFFFFFF)


class DevMem(WordMap):
    """
    A block of FPGA registers mapped through /dev/mem (needs sudo).

    On the KV260 the PL is reached through the PS AXI master ports, so `base`
    is whatever the Vivado Address Editor assigned (the team's camera design
    sits at 0xB000_0000). The bitstream must be loaded first, and its overlay
    must turn on the PL clock: touching a PL address with no clocked slave
    behind it can hang the whole board, not just this script.
    """

    def __init__(self, base, size=0x1000):
        if base % mmap.PAGESIZE:
            raise ValueError(f"base 0x{base:X} is not page-aligned ({mmap.PAGESIZE} bytes)")
        state = pl_state()
        if state is not None and state != "operating":
            raise RuntimeError(f"FPGA state is '{state}': load the bitstream first "
                               f"(sudo xmutil loadapp <app>)")
        fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
        try:
            mem = mmap.mmap(fd, size, mmap.MAP_SHARED,
                            mmap.PROT_READ | mmap.PROT_WRITE, offset=base)
        finally:
            os.close(fd)
        super().__init__(mem)


class SimRegs:
    """
    No hardware. Remembers writes; a GO while enabled moves CURRENT_POSITION
    straight to the target. Always homed, E-stop OK, no faults.
    """

    def __init__(self):
        self.mem = {VERSION: (VERSION_MAGIC << 16) | 0x0001}
        self.writes = collections.deque(maxlen=1000)      # (offset, value), newest last

    def read(self, off):
        if off == GLOBAL_STATUS:
            return (GS_ENABLED if self.mem.get(GLOBAL_CTRL, 0) & CTRL_MASTER_ENABLE else 0) | GS_ESTOP_OK
        if off >= AXIS_BASE and (off - AXIS_BASE) % AXIS_STRIDE == STATUS:
            return AX_HOMED
        return self.mem.get(off, 0)

    def write(self, off, value):
        value &= 0xFFFFFFFF
        self.writes.append((off, value))
        if off == WDOG_KICK:
            return
        if off >= AXIS_BASE and (off - AXIS_BASE) % AXIS_STRIDE == COMMAND:
            ax = off - COMMAND
            if value & CMD_GO and self.mem.get(GLOBAL_CTRL, 0) & CTRL_MASTER_ENABLE:
                self.mem[ax + CURRENT_POSITION] = self.mem.get(ax + TARGET_POSITION, 0)
            return
        self.mem[off] = value


@dataclass
class GoalieConfig:
    usteps_per_mm: float          # microsteps per mm of rod travel. Negative flips +y.
    max_speed_mm_s: float
    accel_mm_s2: float
    center_usteps: int = 0        # position counter value with the figure at the goal center
    travel_mm: float = 65.0       # targets and soft limits are held to +/- this
    axis: int = 0                 # 0 = goalie linear
    wdog_ms: float = 100.0        # no target for this long -> the FPGA stops the axis
    require_homed: bool = True    # send no move until the axis reports HOMED


class Goalie:
    def __init__(self, regs, cfg):
        c = cfg
        if not (math.isfinite(c.usteps_per_mm) and c.usteps_per_mm != 0):
            raise ValueError("usteps_per_mm must be a non-zero number")
        if not (c.max_speed_mm_s > 0 and c.accel_mm_s2 > 0 and c.travel_mm >= 0):
            raise ValueError("max_speed_mm_s and accel_mm_s2 must be > 0, travel_mm >= 0")
        self.r, self.cfg = regs, cfg
        self.ax = AXIS_BASE + c.axis * AXIS_STRIDE

    # -- units -------------------------------------------------------------------

    def to_usteps(self, mm):
        return self.cfg.center_usteps + int(round(mm * self.cfg.usteps_per_mm))

    def to_mm(self, usteps):
        return (usteps - self.cfg.center_usteps) / self.cfg.usteps_per_mm

    # -- setup -------------------------------------------------------------------

    def check(self):
        """
        Is this the block the constants above describe? Returns (ok, message).
        Reads VERSION first and writes nothing unless its magic matches, so a
        different design at this address is never written to.
        """
        v = self.r.read(VERSION)
        if v >> 16 != VERSION_MAGIC:
            return False, (f"VERSION = 0x{v:08X}, expected magic 0x{VERSION_MAGIC:04X} in the top "
                           f"half: wrong base address, or the RTL does not follow this register map")
        self.r.write(SCRATCH, 0xDEADBEEF)
        if self.r.read(SCRATCH) != 0xDEADBEEF:
            return False, "SCRATCH did not read back"
        return True, f"motor_control_core v{(v >> 8) & 0xFF}.{v & 0xFF}"

    def enable(self):
        """Write limits, speed, acceleration and the watchdog, then permit motion."""
        c = self.cfg
        k = abs(c.usteps_per_mm)
        lo, hi = sorted((self.to_usteps(-c.travel_mm), self.to_usteps(c.travel_mm)))
        self.r.write(self.ax + SOFT_LIMIT_MIN, lo & 0xFFFFFFFF)     # signed -> 32-bit word
        self.r.write(self.ax + SOFT_LIMIT_MAX, hi & 0xFFFFFFFF)
        self.r.write(self.ax + MAX_VELOCITY, int(round(c.max_speed_mm_s * k)))
        self.r.write(self.ax + ACCELERATION, int(round(c.accel_mm_s2 * k)))
        self.r.write(WDOG_PERIOD, int(round(c.wdog_ms * 1000.0)))
        self.r.write(WDOG_KICK, 1)
        self.r.write(GLOBAL_CTRL, CTRL_SER_ENABLE | CTRL_WDOG_ENABLE | CTRL_MASTER_ENABLE)

    def disable(self):
        """Motion off and watchdog off (table setup, shutdown). Outputs go to their safe state."""
        self.r.write(GLOBAL_CTRL, CTRL_SER_ENABLE)

    def clear_faults(self):
        """Clear latched faults (watchdog, limits, E-stop). Call on purpose, never in the loop."""
        self.r.write(GLOBAL_CTRL, self.r.read(GLOBAL_CTRL) | CTRL_CLEAR_ALL_FAULTS)

    # -- every frame ---------------------------------------------------------------

    def set_target_mm(self, y):
        """Absolute target for the goalie figure's center, mm from the goal center."""
        if not (isinstance(y, (int, float)) and math.isfinite(y)):
            raise ValueError(f"goalie target must be a finite number, got {y!r}")
        c = self.cfg
        y = min(max(float(y), -c.travel_mm), c.travel_mm)
        if not c.require_homed or self.r.read(self.ax + STATUS) & AX_HOMED:
            self.r.write(self.ax + TARGET_POSITION, self.to_usteps(y) & 0xFFFFFFFF)
            self.r.write(self.ax + COMMAND, CMD_GO)
        self.r.write(WDOG_KICK, 1)          # same function as the command, on purpose

    def position_mm(self):
        """Where the goalie has been COMMANDED to (pulse count), not a measurement."""
        return self.to_mm(_signed(self.r.read(self.ax + CURRENT_POSITION)))

    def ready(self):
        """True when enabled, E-stop OK, watchdog not tripped, axis homed and fault-free."""
        g = self.r.read(GLOBAL_STATUS)
        a = self.r.read(self.ax + STATUS)
        return bool(g & GS_ENABLED and g & GS_ESTOP_OK and not g & GS_WDOG_TRIPPED
                    and a & AX_HOMED and not a & AX_FAULT)
