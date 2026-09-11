# Interfaces — KV260 ↔ Motor Interface PCB

**Status:** Draft / tentative. **Freeze before PCB schematic capture.**
**Last updated:** 2026-09-11

---

## 1. Scope

This document defines the physical and logical interface between the
controller (KV260 Pmod, or the Tiny Tapeout ASIC, or the ESP32 fallback) and
the motor interface PCB. It is the contract that lets the PCB and the RTL be
designed in parallel by different people.

**Nothing in this document may change after the Rev A schematic is sent for
review without an explicit changelog entry at the bottom.**

---

## 2. Physical interface — Pmod connector

The KV260 exposes a single 12-pin Pmod (2×6): 8 signal pins, 2 × GND,
2 × 3.3 V. All 8 signals are used. There is no spare.

| Pmod pin | Signal | Dir (controller view) | Description |
|---|---|---|---|
| 1 | `SHIFT_CLK` | out | Serial clock, common to output and input chains |
| 2 | `OUT_DATA` | out | Serial data to the output shift-register chain |
| 3 | `OUT_LATCH` | out | Rising edge transfers shift register → output pins |
| 4 | `IN_LOAD` | out | Falling edge captures parallel inputs into input chain |
| 5 | GND | — | |
| 6 | 3.3 V | — | **Reference only. Do not draw board power from this.** |
| 7 | `IN_DATA` | in | Serial data from the input shift-register chain |
| 8 | `ENC_CLK` | out | SPI clock for the rod-angle encoder chain |
| 9 | `ENC_MOSI` | out | SPI data to encoders |
| 10 | `ENC_MISO` | in | SPI data from encoders |
| 11 | GND | — | |
| 12 | 3.3 V | — | Reference only |

Signalling: **3.3 V LVCMOS**, both directions. The PCB level-shifts to 5 V
locally where the driver opto-inputs require it.

Encoder chip-selects are **not** Pmod pins. They are bits in the output word
(§4), which is why the encoder interface is 3 wires here rather than 3 + 4.

> **Pin assignment is not final.** Confirm against the KV260 carrier schematic
> and the actual Pmod-to-PL pin constraints before layout, and record the
> `.xdc` package pins here once known.

---

## 3. Alternate controller header

The PCB carries a second header exposing the same 8 signals plus GND and a
3.3 V input, for the Tiny Tapeout board or an ESP32-S3. **Only one controller
header may be populated/connected at a time**; there is no arbitration and no
bus contention protection.

Pinout: identical signal order to §2, 0.1" 2×6, keyed.

---

## 4. Output word — 24 bits

Three cascaded 8-bit output shift registers (74AHCT595 / 74LVC595 family).

**Bit 23 is shifted out first** and ends up in the last device of the chain.
MSB-first throughout.

| Bit | Signal | Safe state | Notes |
|---|---|---|---|
| 23 | `SPARE_OUT1` | 0 | |
| 22 | `STATUS_LED` | 0 | Heartbeat, driven by fabric |
| 21 | `ENC_CS_N[3]` | 1 | Attack rod encoder |
| 20 | `ENC_CS_N[2]` | 1 | Midfield rod encoder |
| 19 | `ENC_CS_N[1]` | 1 | Defense rod encoder |
| 18 | `ENC_CS_N[0]` | 1 | Goalie rod encoder |
| 17 | `CAM_TRIGGER` | 0 | External camera trigger |
| 16 | `MOTOR_EN_N` | 1 | **Global** driver enable, active low |
| 15 | `DIR[7]` | 0 | Attack rotary |
| 14 | `DIR[6]` | 0 | Attack linear |
| 13 | `DIR[5]` | 0 | Midfield rotary |
| 12 | `DIR[4]` | 0 | Midfield linear |
| 11 | `DIR[3]` | 0 | Defense rotary |
| 10 | `DIR[2]` | 0 | Defense linear |
| 9 | `DIR[1]` | 0 | Goalie rotary |
| 8 | `DIR[0]` | 0 | Goalie linear |
| 7 | `STEP[7]` | 0 | Attack rotary |
| 6 | `STEP[6]` | 0 | Attack linear |
| 5 | `STEP[5]` | 0 | Midfield rotary |
| 4 | `STEP[4]` | 0 | Midfield linear |
| 3 | `STEP[3]` | 0 | Defense rotary |
| 2 | `STEP[2]` | 0 | Defense linear |
| 1 | `STEP[1]` | 0 | Goalie rotary |
| 0 | `STEP[0]` | 0 | Goalie linear |

**Safe state.** The 595 chain's `OE_N` is pulled to its disabled state by a
resistor, and the PCB applies the safe pattern through pull-up/pull-down on
the buffer outputs. With the KV260 unconfigured, unpowered, or held in reset,
`MOTOR_EN_N` must read high (drivers disabled) and all `STEP` must be low.
This is PCB-2 and FPG-12 in `requirements.md`, and it is the single most
important electrical property of the board.

**Per-axis enable is deliberately absent.** One global enable saves 8 bits
and 8 signals, and no gameplay case wants one axis enabled while another is
not. If per-axis enable turns out to be needed for thermal management, it
costs a fourth output register.

---

## 5. Input word — 24 bits

Three cascaded parallel-in/serial-out registers (74HC165 / 74LVC165 family).

| Bit | Signal | Description |
|---|---|---|
| 23:18 | `SPARE_IN[5:0]` | Reserved |
| 17 | `MOTOR_PWR_OK` | Motor supply present (divider + comparator) |
| 16 | `ESTOP_OK` | E-stop loop intact. **Status only — not the safety path** |
| 15:8 | `DRIVER_FAULT[7:0]` | Per-axis external driver fault output |
| 7:4 | `LIMIT_FAR[3:0]` | Far travel limit, linear axes 0/2/4/6 |
| 3:0 | `HOME_SW[3:0]` | Home switch, linear axes 0/2/4/6 |

Note the asymmetry: only the 4 linear axes have limit and home switches.
Rotary axes rotate continuously and are bounded by nothing — their position
reference comes from the absolute encoder (§7), not a switch.

Input polarity is configured per axis by `LIMITS_ACTIVE_LOW` in
`AXIS_CONFIG`. Wire switches **normally-closed** so that a broken wire reads
as "limit hit" rather than "all clear."

---

## 6. Serial timing — and the main open risk

### 6.1 Nominal

| Parameter | Value | Note |
|---|---|---|
| Core clock | 100 MHz | |
| `SHIFT_CLK` | 20 MHz | Divider default in `SER_CTRL` |
| Word length | 24 bits | |
| Shift time | 1.2 µs | 24 ÷ 20 MHz |
| Latch + reload overhead | ~0.2 µs | |
| **Update period** | **~1.4 µs** | ≈ **714 kHz** full-state refresh |

Output and input chains share `SHIFT_CLK` and shift concurrently — the 165
chain loads on `IN_LOAD` and shifts out on the same clock the 595 chain shifts
in. One 24-cycle burst services both directions.

### 6.2 The problem

STEP edges can only change at update boundaries. At a 200 kHz step rate the
period is 5 µs, so a pulse is quantised to ~1.4 µs granularity — roughly
**28 % of the step period**. That is severe timing jitter, and stepper drivers
and motor resonance do not love it.

| Step rate | Period | Updates per period | Quantisation |
|---|---|---|---|
| 50 kHz | 20 µs | 14 | 7 % — fine |
| 100 kHz | 10 µs | 7 | 14 % — acceptable |
| 150 kHz | 6.7 µs | 4.8 | 21 % — marginal |
| 200 kHz | 5 µs | 3.6 | 28 % — **problem** |

### 6.3 Candidate mitigations

| Option | Effect | Cost |
|---|---|---|
| Raise `SHIFT_CLK` to 40 MHz | Update period → 0.7 µs, 14 % at 200 kHz | Signal integrity on ribbon cable; may need series termination |
| Split into a 16-bit fast word (STEP[7:0] + 8 spare) and a slow word (DIR, enables, CS) refreshed at 10 kHz | 0.4 µs update at 40 MHz → 8 % | More complex serializer, two latch signals — **needs a 9th Pmod pin, which does not exist** |
| Reduce microstepping so 200 kHz is never needed | Eliminates the problem | Coarser motion, more audible/resonant |
| Accept 150 kHz as the real ceiling | Honest | Restates FPG-2 |

**Recommendation for Rev A:** design for 40 MHz `SHIFT_CLK` (proper series
termination, short cable, ground returns between signals) and treat 200 kHz
as a stretch. Measure actual achievable step rate in Phase 7 and then decide
whether FPG-2 needs restating.

This is the highest-value thing to simulate before the PCB is ordered.

### 6.4 Signal integrity

- Controller-to-PCB cable ≤ **150 mm**.
- Ribbon or twisted pair with a ground return adjacent to every signal.
- Series termination (~33 Ω) at the source on `SHIFT_CLK`, `OUT_DATA`,
  `OUT_LATCH`, `IN_LOAD`, `ENC_CLK`, `ENC_MOSI`.
- `SHIFT_CLK` is the fastest and most critical net. Route it first.

---

## 7. Encoder interface

Standard SPI mode 1 (CPOL=0, CPHA=1) — confirm against the chosen part.

- 4 devices, individually selected by `ENC_CS_N[3:0]` from the output word.
- Because chip-select comes from the output chain, **a CS change costs one
  full serializer update** (~1.4 µs). Encoder reads are therefore not
  free-running; budget ~5 µs per device and poll at the motion update rate
  (10 kHz), not per step.
- Read all 4 in round-robin: one device per motion update, so each rod's angle
  refreshes at 2.5 kHz. This is ample — rod dynamics are far slower.

**Part not selected.** Requirements: absolute (no homing cycle), ≥12-bit,
SPI, tolerant of a magnet on a rotating rod end. AS5047P / AS5048A class is
the obvious starting point. Pick before schematic capture — the CS timing and
supply voltage are the only things that can surprise the layout.

---

## 8. Power

| Rail | Source | Used by |
|---|---|---|
| 5 V | External regulated supply, barrel or terminal block | Driver opto-input side, 595 output buffers |
| 3.3 V | On-board LDO from 5 V | Shift registers, Schmitt triggers, encoders |
| 24–48 V | Separate supply direct to drivers | **Never touches this PCB** |

The Pmod 3.3 V pins are a **reference only**. The board must operate with the
Pmod 3.3 V pins unloaded. Drawing board power from the Pmod risks the KV260's
regulator and creates a ground-loop path through the connector.

Protection chain on the 5 V input: reverse-polarity FET → polyfuse → TVS →
bulk cap → LDO.

---

## 9. Signal conditioning chains

**Input path**, every input without exception:

```
switch/sensor ─► series R ─► TVS ─► RC filter ─► Schmitt trigger ─► '165
                             (~1 µs time constant)      74LVC14
```

**Output path**, per motor driver signal:

```
'595 ─► buffer / open-drain stage ─► current-limit R ─► opto input on driver
```

Check the specific driver's opto forward current (typically 8–16 mA at 5 V)
and size the series resistor per channel. DM542/DM420-class drivers are the
assumed target; confirm the datasheet before choosing the buffer family.

---

## 10. ESP32-S3 fallback protocol

Used only if the PCB slips. Link: SPI at 10–20 MHz, or UART at 1 Mbps.

Fixed 16-byte command frame:

| Offset | Field | Size | Notes |
|---|---|---|---|
| 0 | `SYNC` | 1 | `0xA5` |
| 1 | `AXIS` | 1 | 0–7, or `0xFF` for broadcast |
| 2 | `OPCODE` | 1 | Mirrors `COMMAND` bits in the register map |
| 3 | `FLAGS` | 1 | |
| 4 | `TARGET_POSITION` | 4 | signed, microsteps |
| 8 | `MAX_VELOCITY` | 4 | microsteps/s |
| 12 | `ACCELERATION` | 2 | microsteps/s² ÷ 256 |
| 14 | `CRC16` | 2 | CCITT over bytes 0–13 |

Estimated transfer: ~0.16 ms at 1 Mbps UART, ~10 µs at 20 MHz SPI. Both are
comfortably inside the 0.5 ms command budget (LAT-6).

**Never stream individual STEP edges across this link.** The ESP32 runs
profiling and pulse generation locally using hardware timers / RMT
peripherals, not delay loops. Benchmark 8 simultaneous axes at 100–150 kHz
before relying on this path.

---

## 11. Verification checklist

Before the schematic is considered done:

- [ ] Every bit in §4 and §5 traced to a physical connector pin
- [ ] Safe state verified with the controller unplugged
- [ ] Safe state verified during KV260 configuration (bitstream loading)
- [ ] Opto drive current calculated from the actual driver datasheet
- [ ] `SHIFT_CLK` at 40 MHz simulated / signal-integrity checked
- [ ] Serializer minimum STEP high/low time verified against driver spec
- [ ] Encoder part selected and its CS timing confirmed
- [ ] Pmod package pins recorded from the real `.xdc`

---

## 12. Changelog

| Date | Change | Reason |
|---|---|---|
| 2026-09-11 | Initial draft | — |
