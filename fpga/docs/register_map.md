# Register Map — motor_control_core

**Status:** Draft / tentative
**Last updated:** 2026-09-11
**Applies to:** AXI4-Lite (KV260), SPI (Tiny Tapeout), and the ESP32 fallback
protocol. Same offsets and bit positions in all three.

---

## 1. Conventions

- All registers are **32 bits**, naturally aligned, byte offsets in hex.
- Access: **RW** read/write, **RO** read-only, **WO** write-only (reads
  return 0), **W1C** write-1-to-clear, **W1P** write-1-to-pulse
  (self-clearing, reads return 0).
- Reserved bits: write 0, ignore on read. Do not read-modify-write a W1P
  register.
- Units, fixed project-wide:

| Quantity | Unit | Type |
|---|---|---|
| Position | microsteps | signed 32-bit |
| Velocity | microsteps/s | signed 32-bit (sign = direction) |
| Acceleration | microsteps/s² | unsigned 32-bit |
| Timestamp | microseconds | unsigned, 64-bit as two registers |
| Angle (encoder) | encoder counts | unsigned, per-device width |

Microsteps, not steps, not millimetres. The conversion to physical units lives
in software, in exactly one place, and never in RTL.

---

## 2. Address layout

| Range | Block |
|---|---|
| `0x000 – 0x03F` | Global control and status |
| `0x040 – 0x07F` | Axis 0 (goalie linear) |
| `0x080 – 0x0BF` | Axis 1 (goalie rotary) |
| `0x0C0 – 0x0FF` | Axis 2 (defense linear) |
| `0x100 – 0x13F` | Axis 3 (defense rotary) |
| `0x140 – 0x17F` | Axis 4 (midfield linear) |
| `0x180 – 0x1BF` | Axis 5 (midfield rotary) |
| `0x1C0 – 0x1FF` | Axis 6 (attack linear) |
| `0x200 – 0x23F` | Axis 7 (attack rotary) |

**Axis base address = `0x040 + (N × 0x40)`**, N = 0..7. Total 576 bytes,
decoded as a 1 KB window.

The 0x40 stride is deliberately larger than the 0x38 currently used — it keeps
address decode to a simple bit-slice and leaves room to add per-axis
registers without moving anything.

---

## 3. Global block

| Offset | Name | Access | Reset | Description |
|---|---|---|---|---|
| `0x000` | `VERSION` | RO | — | `[31:16]` magic `0xF005`, `[15:8]` major, `[7:0]` minor |
| `0x004` | `GLOBAL_CTRL` | RW | `0x0000_0000` | See §3.1 |
| `0x008` | `GLOBAL_STATUS` | RO | — | See §3.2 |
| `0x00C` | `FAULT_SUMMARY` | RO | `0` | `[7:0]` one bit per axis, set if that axis has any latched fault |
| `0x010` | `WDOG_PERIOD` | RW | `0` | Watchdog timeout in µs. `0` = watchdog disabled |
| `0x014` | `WDOG_KICK` | W1P | — | Any write resets the watchdog timer |
| `0x018` | `TIMESTAMP_LO` | RO | — | Free-running µs counter, low 32 bits. **Read this first** |
| `0x01C` | `TIMESTAMP_HI` | RO | — | High 32 bits, latched when `TIMESTAMP_LO` is read |
| `0x020` | `SYNC_START` | W1P | — | `[7:0]` axis bitmask; starts all selected axes on the same clock edge |
| `0x024` | `SYNC_ABORT` | W1P | — | `[7:0]` axis bitmask; controlled decel to stop |
| `0x028` | `INPUT_SHADOW` | RO | — | Raw deserialised input word, see `interfaces.md` §5 |
| `0x02C` | `OUTPUT_FORCE` | RW | `0` | Debug only. `[31]` enable force, `[23:0]` forced output word |
| `0x030` | `SER_CTRL` | RW | `0x0000_0004` | `[7:0]` shift clock divider (core clk ÷ 2×(div+1)). Default = 20 MHz from 100 MHz |
| `0x034` | `SER_STATUS` | RO | — | `[15:0]` measured serializer update period in core clocks |
| `0x038` | `SCRATCH` | RW | `0` | Software use. Useful as a first bring-up bus test |
| `0x03C` | — | — | — | Reserved |

### 3.1 `GLOBAL_CTRL` (0x004)

| Bit | Name | Description |
|---|---|---|
| 0 | `MASTER_ENABLE` | `1` permits motion. `0` forces all outputs safe immediately. Cleared by watchdog, E-stop, or reset |
| 1 | `CLEAR_ALL_FAULTS` | W1P within this register. Clears all latched faults on all axes |
| 2 | `WDOG_ENABLE` | Enable watchdog. Ignored if `WDOG_PERIOD` is 0 |
| 3 | `SER_ENABLE` | Enable serializer output. Must be 1 for any output to reach the PCB |
| 4 | `ESTOP_MASK` | **Debug only.** Ignore E-stop input. Should be permanently 0 in any build that drives real motors |
| 31:5 | — | Reserved |

> `ESTOP_MASK` exists so the board can be bring-up tested on a bench with no
> E-stop wired. It must be removed or hard-tied to 0 before the demo build.
> Note this in the bring-up checklist, not just here.

### 3.2 `GLOBAL_STATUS` (0x008)

| Bit | Name | Description |
|---|---|---|
| 0 | `ENABLED` | Master enable is actually asserted |
| 1 | `ANY_BUSY` | At least one axis is in motion |
| 2 | `ANY_FAULT` | At least one axis has a latched fault |
| 3 | `WDOG_TRIPPED` | Watchdog expired. Latched until faults cleared |
| 4 | `ESTOP_OK` | E-stop loop is intact (live input, not latched) |
| 5 | `MOTOR_PWR_OK` | Motor power rail present |
| 6 | `SER_RUNNING` | Serializer is cycling |
| 15:8 | `BUSY_MASK` | Per-axis busy |
| 31:16 | — | Reserved |

---

## 4. Per-axis block

Offsets below are relative to the axis base (`0x040 + N×0x40`).

| Offset | Name | Access | Reset | Description |
|---|---|---|---|---|
| `+0x00` | `TARGET_POSITION` | RW | `0` | Signed microsteps, absolute |
| `+0x04` | `MAX_VELOCITY` | RW | `0` | Unsigned microsteps/s. Magnitude only; direction comes from the move |
| `+0x08` | `ACCELERATION` | RW | `0` | Unsigned microsteps/s² |
| `+0x0C` | `DECELERATION` | RW | `0` | Unsigned microsteps/s². If `0`, `ACCELERATION` is used |
| `+0x10` | `COMMAND` | W1P | — | See §4.1 |
| `+0x14` | `STATUS` | RO | — | See §4.2 |
| `+0x18` | `CURRENT_POSITION` | RO | `0` | Signed microsteps, live |
| `+0x1C` | `CURRENT_VELOCITY` | RO | `0` | Signed microsteps/s, live |
| `+0x20` | `FAULT` | W1C | `0` | See §4.3 |
| `+0x24` | `SOFT_LIMIT_MIN` | RW | `0x8000_0000` | Signed. Motion below this is refused/stopped |
| `+0x28` | `SOFT_LIMIT_MAX` | RW | `0x7FFF_FFFF` | Signed |
| `+0x2C` | `AXIS_CONFIG` | RW | `0` | See §4.4 |
| `+0x30` | `EXEC_TIMESTAMP` | RW | `0` | µs value at which a `GO_AT` command executes |
| `+0x34` | `ENCODER_POSITION` | RO | `0` | Raw encoder counts. Rotary axes only; `0` on linear axes |
| `+0x38` | `FOLLOWING_ERROR` | RO | `0` | Signed, commanded − encoder, in microsteps. Rotary only |
| `+0x3C` | `HOME_OFFSET` | RW | `0` | Position value loaded into the counter at home switch capture |

### 4.1 `COMMAND` (+0x10), write-1-to-pulse

| Bit | Name | Description |
|---|---|---|
| 0 | `GO` | Begin move to `TARGET_POSITION` immediately |
| 1 | `GO_AT` | Begin move when the timestamp counter reaches `EXEC_TIMESTAMP` |
| 2 | `ABORT` | Controlled deceleration to a stop, hold position |
| 3 | `ESTOP_AXIS` | Immediate step-train halt, no deceleration. Will lose position on an open-loop axis |
| 4 | `ZERO` | Set `CURRENT_POSITION` to `HOME_OFFSET`. Refused while busy |
| 5 | `CLEAR_FAULT` | Clear this axis's latched faults |
| 6 | `JOG_POS` | Continuous motion in + direction at `MAX_VELOCITY` until `ABORT` or limit |
| 7 | `JOG_NEG` | Continuous motion in − direction |
| 31:8 | — | Reserved |

Writing two mutually exclusive bits in one write is undefined. Don't.

### 4.2 `STATUS` (+0x14)

| Bit | Name | Description |
|---|---|---|
| 0 | `BUSY` | Move in progress |
| 1 | `DONE` | Last move completed normally. Cleared on next `GO` |
| 2 | `AT_TARGET` | `CURRENT_POSITION == TARGET_POSITION` |
| 3 | `DIR` | Current commanded direction, `1` = positive |
| 4 | `HOMED` | Axis has been homed since reset |
| 5 | `LIMIT_MIN` | Min limit input asserted (live) |
| 6 | `LIMIT_MAX` | Max limit input asserted (live) |
| 7 | `HOME_SW` | Home switch input asserted (live) |
| 8 | `FAULT` | Any latched fault on this axis |
| 9 | `PENDING` | A `GO_AT` is armed and waiting for its timestamp |
| 10 | `ACCEL` | Currently accelerating |
| 11 | `DECEL` | Currently decelerating |
| 31:12 | — | Reserved |

### 4.3 `FAULT` (+0x20), write-1-to-clear

| Bit | Name | Description |
|---|---|---|
| 0 | `DRIVER_FAULT` | External driver asserted its fault output |
| 1 | `LIMIT_HIT` | Hardware limit reached during motion |
| 2 | `SOFT_LIMIT` | Move would exceed a soft limit |
| 3 | `FOLLOWING_ERR` | Encoder disagreement exceeded threshold (rotary) |
| 4 | `WDOG` | Axis stopped by global watchdog |
| 5 | `ESTOP` | Axis stopped by E-stop |
| 6 | `CMD_INVALID` | Illegal command, e.g. `GO` with `MAX_VELOCITY` = 0 |
| 7 | `ENCODER_LOST` | Encoder read failed / no response |
| 31:8 | — | Reserved |

A latched fault holds the axis disabled until cleared. Faults latch even if
the condition goes away — that is the point of them.

### 4.4 `AXIS_CONFIG` (+0x2C)

| Bit | Name | Description |
|---|---|---|
| 0 | `INVERT_DIR` | Flip DIR output polarity |
| 1 | `INVERT_STEP` | Flip STEP output polarity |
| 2 | `INVERT_ENABLE` | Flip ENABLE output polarity (driver-dependent) |
| 3 | `ENABLE_HOLD` | Keep driver enabled when idle. `0` = release holding torque |
| 4 | `LIMITS_ACTIVE_LOW` | Interpretation of limit/home inputs |
| 5 | `ENC_PRESENT` | This axis has a rod-angle encoder |
| 6 | `ENC_CHECK_EN` | Enable following-error checking |
| 15:8 | `STEP_HIGH_CYCLES` | Minimum STEP high time in serializer periods, 1–255 |
| 31:16 | `FOLLOW_ERR_LIMIT` | Following-error fault threshold, microsteps |

`INVERT_*` bits exist so that wiring mistakes become a software fix instead of
a rework. Set them once during bring-up and record the values in the bring-up
log.

---

## 5. Typical software sequences

**Bring-up sanity check**

```
write SCRATCH = 0xDEADBEEF; read back; compare      ← bus works
read VERSION; expect magic 0xF005                   ← right bitstream
```

**Enable the system**

```
for each axis: write AXIS_CONFIG, SOFT_LIMIT_MIN/MAX
write WDOG_PERIOD = 5000            (5 ms)
write GLOBAL_CTRL = SER_ENABLE | WDOG_ENABLE | MASTER_ENABLE
```

**A single move**

```
write TARGET_POSITION, MAX_VELOCITY, ACCELERATION
write COMMAND = GO
poll STATUS until DONE, or wait
```

**Coordinated move on 4 rods** — the reason `SYNC_START` exists:

```
for axis in {0,2,4,6}:
    write TARGET_POSITION, MAX_VELOCITY, ACCELERATION
write SYNC_START = 0b01010101
```

**Control loop, per vision frame**

```
read TIMESTAMP_LO                    ← for latency accounting
compute targets from prediction
write per-axis TARGET_POSITION / MAX_VELOCITY
write SYNC_START = active_mask
write WDOG_KICK                      ← every loop, no exceptions
```

The watchdog kick belongs in the same function as the command writes. If it
lives in a separate thread, a hung control loop will keep kicking a dead
horse and the watchdog protects nothing.

---

## 6. Open questions

1. **Register width for velocity/acceleration.** 32-bit unsigned
   microsteps/s² overflows conceptually long before it overflows numerically
   — the useful range is maybe 0–10⁷. Consider narrowing to 24 bits to save
   ASIC area, at the cost of a less uniform map. Currently: keep 32, revisit
   at TT wrapper time.
2. **`GO_AT` timestamp width.** Currently compares against
   `TIMESTAMP_LO` only, so scheduling wraps every ~71 minutes. Fine for
   gameplay, ugly in a log. Decide whether to widen the comparison.
3. **Whether `ENCODER_POSITION` should be scaled to microsteps in RTL** or
   left raw with conversion in software. Leaning raw — keeps the divide out
   of fabric.
4. **`ESTOP_AXIS` (COMMAND bit 3) semantics on open-loop axes.** It
   deliberately loses position. Should it auto-set `HOMED = 0`? Probably yes.
   Not yet implemented.
5. Whether to add a per-axis `MOVE_COUNT` / diagnostic counter block for
   endurance testing. Cheap in fabric, useful in Phase 6.
