# Interfaces — Controller ↔ Motor Interface PCB

**Status:** Draft rev B. **Freeze before Rev A schematic is sent for review.**
**Last updated:** 2026-09-12

---

## 1. Scope

This document defines the physical and logical interface between the
controller (KV260 Pmod, or the ESP32-S3 fallback)
and the motor interface PCB. It is the contract that lets the PCB and the RTL
be designed in parallel by different people.

**Nothing here may change after the Rev A schematic goes out for review
without a changelog entry in §13.**

---

## 2. Controller I/O — what the KV260 actually provides

Confirmed against UG1089 v1.4, the K26 SOM datasheet, and the Kria-PYNQ
`base.xdc`. These facts constrain everything below.

| Property | Value | Source |
|---|---|---|
| Connector | J2, Digilent Pmod 2×6, 12-pin | UG1089 |
| Supply pin | 3.3 V, **100 mA** | UG1089 |
| Signalling | LVCMOS33, direct HDIO connection, no translator on the carrier | Kria-PYNQ `base.xdc` |
| Bank | 45 (HDA), VCCO supplied by the carrier card | K26 SOM datasheet |
| Max HDIO data rate | **250 Mb/s** | K26 SOM datasheet |
| Signal pins available | 8, all used | — |

Two consequences that drive the design:

1. **40 MHz `SCLK` is well inside spec.** The earlier concern about an
   auto-direction level translator throttling the link was unfounded — the
   Pmod pins land straight on HDIO. The limit is cable signal integrity, not
   the SOM.
2. **The Pmod pins float for the entire boot.** The carrier only supplies PL
   VCCO after the SOM asserts `VCCOEN_PL_M2C`, and the pins stay
   unconfigured until the bitstream loads — tens of seconds after 12 V is
   applied on a Linux boot. Every controller-driven net on the PCB needs a
   resistor that defines its state without the FPGA. See §6.

### 2.1 Pin assignment

| Pmod pin | Signal | Dir (controller view) | Package pin | Description |
|---|---|---|---|---|
| 1 | `SCLK` | out | H12 | Serial clock, shared by output and input chains |
| 2 | `SDO` | out | E10 | Serial data to the '595 output chain |
| 3 | `SDI` | in | D10 | Serial data from the '165 input chain |
| 4 | `STROBE` | out | C11 | Combined '165 load and '595 latch, see §5.2 |
| 5 | GND | — | — | |
| 6 | 3.3 V | — | — | Powers the return-path buffer only, see §9 |
| 7 | `OUT_EN` | out | B10 | Global output enable, high = outputs driven |
| 8 | `ENC_CLK` | out | E12 | SPI clock, rod-angle encoder chain |
| 9 | `ENC_MOSI` | out | D11 | SPI data to encoders |
| 10 | `ENC_MISO` | in | B11 | SPI data from encoders |
| 11 | GND | — | — | |
| 12 | 3.3 V | — | — | Tied to pin 6 |

Package pins are from the Kria-PYNQ `base.xdc` `pmod[7:0]` bus, all
`IOSTANDARD LVCMOS33`, bank 45. `pmod[0]` is Pmod connector pin 1, net
`HDA11`, connector position `som240_1_a17`.

> **Verify before layout:** the `pmod[4:7]` → connector pin 7–10 mapping
> follows the standard Digilent row-2 convention but has not been checked
> against the KV260 carrier schematic. Also check for series resistors or ESD
> arrays inline on the eight nets; if the carrier already has 33–100 Ω in
> series, don't stack more on top (§5.4).

Two changes from draft rev A, both worth understanding:

- `OUT_LATCH` and `IN_LOAD` are now one pin (`STROBE`). The '165 loads on a
  low level and the '595 latches on a rising edge, so a single low-going
  pulse serves both. This frees a pin.
- The freed pin became `OUT_EN`, driving the '595 `/OE` through an inverter.
  One line disables every output regardless of shift register contents. This
  is the watchdog's hardware lever and the power-up default.

---

## 3. Alternate controller header

A second 2×6 0.1" keyed header exposes the same 8 signals plus GND and a
3.3 V input, for an ESP32-S3.

**Only one controller header may be populated at a time.** There is no
arbitration and no bus contention protection. Mark this on the silkscreen.

The 3.3 V pin on this header is an **input** — the alternate controller
supplies the rail that powers the return-path buffer (§9), exactly as the
KV260 does through the Pmod.

---

## 4. Logic family

| Function | Part | Rail | Why |
|---|---|---|---|
| Output chain | 4 × 74AHCT595 | 5 V | AHCT input thresholds (V_IH = 2.0 V) accept 3.3 V drive directly. AHCT is fast enough for 40 MHz; plain HCT tops out near 25 MHz. |
| Input chain | 3 × 74AHCT165 | 5 V | Same threshold and speed reasoning |
| Input conditioning | 74HC14 | 5 V | Schmitt trigger, one per input |
| Driver output stage | 3 × 74LVC07A | 3.3 V | Open-drain, 5.5 V tolerant outputs, sinks the opto LEDs |
| Encoder CS level shift | 1 × 74LVC125 | 3.3 V | LVC inputs are 5.5 V tolerant, so it steps the 5 V '595 outputs down |
| Return path to controller | 1 × 74LVC2G17 | **Pmod 3.3 V** | See §9 |

Confirm AHCT595 and AHCT165 are JLCPCB Basic parts before committing. If
AHCT165 is not stocked, HCT165 is acceptable only if `SCLK` drops to 20 MHz,
which costs you the frame timing in §5.3 — check stock early.

---

## 5. Serial frame

### 5.1 Structure

- Output chain: 4 cascaded 74AHCT595 = **32 bits**
- Input chain: 3 cascaded 74AHCT165 = **24 bits**
- One frame = **32 `SCLK` cycles**; input bits 24–31 are don't-care
- **Bit 0 is shifted out first** and ends up in the last device of the chain

### 5.2 Strobe sequence

`STROBE` idles high. One frame is:

1. Drive `STROBE` low, hold ≥ 100 ns. The '165 chain captures its parallel
   inputs asynchronously. The '595 sees a falling edge on `RCLK`, which does
   nothing.
2. Release `STROBE` high. The rising edge latches the '595 shift register
   contents — the word shifted in during the *previous* frame — to the
   output pins. The '165 returns to shift mode.
3. Issue 32 `SCLK` cycles.
   - Controller updates `SDO` on the **falling** edge; the '595 samples it on
     the rising edge.
   - `SDI` bit *k* is valid during `SCLK` low phase *k*; the controller
     samples it there. Rising edge *k* advances the '165 to bit *k*+1.
   - Note bit 0 of the input word is valid immediately after step 2, before
     the first rising edge.
4. Repeat.

**Output data is latched one frame after it is shifted.** The RTL must
account for this single-frame pipeline delay when scheduling step edges.

Tie '595 `SRCLR` high (or to a synchronous reset) and '165 `CLK INH` low.

### 5.3 Timing

| Parameter | Value | Note |
|---|---|---|
| Serializer clock domain | 200 MHz | MMCM output, separate from the 100 MHz motor core |
| `SCLK` | 40 MHz | 200 ÷ 5 |
| Frame | 32 bits | |
| Shift time | 0.80 µs | |
| Strobe + turnaround | ~0.15 µs | |
| **Frame period** | **~0.95 µs** | ≈ **1.05 MHz** full-state update |

> The serializer needs its own clock domain. 40 MHz is not an integer divide
> of 100 MHz — 100 ÷ 5 = 20 and 100 ÷ 2 = 50, nothing in between. Generate
> 200 MHz from an MMCM and divide by 5. Cross into the motor core's 100 MHz
> domain with a proper CDC, not an ad-hoc handshake.

### 5.4 Step edge quantisation

STEP edges can only move at frame boundaries, so every step is quantised to
~0.95 µs.

| Step rate | Period | Frames/period | Quantisation |
|---|---|---|---|
| 64 kHz | 15.6 µs | 16.4 | 6 % |
| 100 kHz | 10 µs | 10.5 | 9 % |
| 150 kHz | 6.7 µs | 7.0 | 14 % |
| 200 kHz | 5 µs | 5.3 | 19 % |

**This is less alarming than it looks, provided the step generator is built
as a phase accumulator.** Accumulate a fixed velocity increment each frame
and emit a step when the accumulator rolls over. The instantaneous period
error is then bounded at ±1 frame and **does not accumulate** — the average
step rate is exact and position never drifts. Rotor inertia integrates the
rest. What you must not do is round the commanded period to whole frames,
which does accumulate error.

**Reality check on the step rate target.** A NEMA 23 at 1.8°, 16×
microstepping is 3200 steps/rev. At 20 rev/s that's 64 kHz. The 200 kHz
figure in `requirements.md` (FPG-2) is roughly 3× what the mechanics will
ever ask for. Treat 150 kHz as the design point and 200 kHz as headroom.

### 5.5 Minimum pulse width — check this against the real driver

DM542-class drivers commonly specify a **minimum STEP pulse width around
2.5 µs**. If that holds for the part you buy, the driver caps you at 200 kHz
on its own, independent of anything in this document, and it means STEP must
be driven at roughly 50 % duty rather than as a narrow pulse.

At a 0.95 µs frame the shortest representable pulse is one frame. To meet a
2.5 µs minimum you need 3 frames high and 3 frames low, which caps the step
rate at ~175 kHz.

**Verify the actual minimum pulse width, minimum low time, and maximum input
frequency from the datasheet of the driver you buy, and record them here.**
This is the single number most likely to invalidate the timing above.

### 5.6 Signal integrity

- Controller-to-PCB cable ≤ **150 mm**.
- Ribbon or twisted pair with a ground return adjacent to every signal.
- 33 Ω series termination at the source on `SCLK`, `SDO`, `STROBE`,
  `ENC_CLK`, `ENC_MOSI` — unless the carrier already has series resistors on
  those nets, in which case subtract what's there.
- `SCLK` is the fastest and most critical net. Route it first.
- Provide a DNP footprint for a second resistor in series with `SCLK` so the
  edge rate can be slowed during bring-up without a respin.

---

## 6. Output word — 32 bits

| Bit | Signal | Safe state | Notes |
|---|---|---|---|
| 0 | `STEP[0]` | 0 | Goalie linear |
| 1 | `STEP[1]` | 0 | Goalie rotary |
| 2 | `STEP[2]` | 0 | Defense linear |
| 3 | `STEP[3]` | 0 | Defense rotary |
| 4 | `STEP[4]` | 0 | Midfield linear |
| 5 | `STEP[5]` | 0 | Midfield rotary |
| 6 | `STEP[6]` | 0 | Attack linear |
| 7 | `STEP[7]` | 0 | Attack rotary |
| 8–15 | `DIR[7:0]` | 0 | Same axis order |
| 16–23 | `ENA_N[7:0]` | **1** | **1 = driver disabled**, see below |
| 24 | `CAM_TRIGGER` | 0 | External camera trigger |
| 25 | `ENC_CS_N[0]` | 1 | Goalie rod encoder |
| 26 | `ENC_CS_N[1]` | 1 | Defense rod encoder |
| 27 | `ENC_CS_N[2]` | 1 | Midfield rod encoder |
| 28 | `ENC_CS_N[3]` | 1 | Attack rod encoder |
| 29 | `STATUS_LED` | 0 | Heartbeat, driven by fabric |
| 30–31 | `SPARE_OUT[1:0]` | 0 | Bring to test points |

### 6.1 Why enable is active-low, and why it is per-axis

DM542-class drivers **disable** when the ENA opto is energised and **enable**
when it is left alone. The naive mapping therefore leaves all eight motors at
full holding current for the entire ~30 s boot. Inverting the sense in
hardware — pull-up on the ENA net, so Hi-Z means "opto energised" means
"disabled" — makes the power-up default genuinely off.

Draft rev A specified a single global enable to save 8 bits. That is reversed
here. Per-axis enable costs 8 bits (0.2 µs of frame time, ~4 % more
quantisation) and buys the ability to bring up and debug one axis at a time
with the other seven electrically dead, which during Phase 5–7 is worth far
more than the timing. It also allows thermal backoff on idle rods.

### 6.2 Safe state implementation

With `OUT_EN` low the '595 outputs are Hi-Z, so resistors alone define what
the drivers see during boot, during reconfiguration, and with the controller
unplugged:

| Net group | Resistor | Resulting state |
|---|---|---|
| `STEP[7:0]`, `DIR[7:0]` | 10 kΩ **pull-down** | Opto off, no pulses |
| `ENA_N[7:0]` | 10 kΩ **pull-up** | Opto energised, driver disabled |
| `ENC_CS_N[3:0]` | 10 kΩ **pull-up** | Encoders deselected |
| `CAM_TRIGGER`, `STATUS_LED`, spares | 10 kΩ pull-down | Inactive |
| `SCLK`, `SDO`, `STROBE`, `OUT_EN` | 10 kΩ pull-down on the PCB side | Defined AHCT inputs while the Pmod floats |

The pull-downs on the four controller-driven lines are not optional. AHCT
inputs left floating draw shoot-through current, and bank 45 has no VCCO at
all until `VCCOEN_PL_M2C` asserts.

This is PCB-2 and FPG-12 in `requirements.md` and it is the single most
important electrical property of the board.

---

## 7. Input word — 24 bits

| Bit | Signal | Description |
|---|---|---|
| 0–3 | `HOME_SW[3:0]` | Home switch, linear axes (0, 2, 4, 6) |
| 4–7 | `LIMIT_FAR[3:0]` | Far travel limit, same axes |
| 8–15 | `DRIVER_FAULT[7:0]` | Per-axis external driver fault output |
| 16 | `ESTOP_OK` | E-stop loop intact. **Status only — not the safety path** |
| 17 | `MOTOR_PWR_OK` | Motor supply present (divider + comparator) |
| 18–23 | `SPARE_IN[5:0]` | Reserved, brought to a header |

Only the 4 linear axes have limit and home switches. Rotary axes turn
continuously and are bounded by nothing; their position reference is the
absolute encoder (§8), not a switch.

Wire all switches **normally closed** so a broken wire reads as "limit hit"
rather than "all clear." Input polarity is configured per axis by
`LIMITS_ACTIVE_LOW` in `AXIS_CONFIG`.

> **Check before committing bits 8–15.** The DM542T has an alarm output;
> several cheaper DM542 clones do not. If the driver you buy has no fault
> output, leave these as spare inputs with pull-ups and say so here.

---

## 8. Encoder interface

SPI mode 1 (CPOL=0, CPHA=1) — confirm against the chosen part.

- 4 devices, individually selected by `ENC_CS_N[3:0]` from the output word.
- `ENC_CLK` and `ENC_MOSI` come straight from the Pmod at 3.3 V, buffered
  locally to drive 4 loads.
- `ENC_CS_N` originates in the 5 V '595 chain and is stepped down by the
  74LVC125 at 3.3 V. Encoders in this class are **not 5 V tolerant** — do not
  connect '595 outputs to them directly.
- Because chip-select comes from the output chain, **a CS change costs one
  full frame** (~0.95 µs). Encoder reads are not free-running; budget ~4 µs
  per device and poll at the motion update rate (10 kHz), not per step.
- Read round-robin, one device per motion update, so each rod's angle
  refreshes at 2.5 kHz. Rod dynamics are far slower than that.

**Part not selected.** Requirements: absolute (no homing cycle), ≥12-bit,
SPI, tolerant of a magnet on a rotating rod end. AS5047P / AS5048A class is
the obvious starting point. If you choose a daisy-chainable part, bits 25–28
free up and the CS latency disappears. Pick before schematic capture — CS
timing and supply voltage are the only things that can surprise the layout.

---

## 9. Power

| Rail | Source | Used by |
|---|---|---|
| 5 V | External regulated supply, barrel or terminal block | '595, '165, '14, driver opto-input side |
| 3.3 V | On-board LDO from 5 V | '07A, '125, encoders |
| **Pmod 3.3 V** | Controller | **74LVC2G17 only** |
| 24–48 V | Separate supply direct to the drivers | **Never touches this PCB** |

### 9.1 The return-path rule

Exactly two nets on this board drive *into* the controller: `SDI` and
`ENC_MISO`. Both pass through a single 74LVC2G17 dual Schmitt buffer, and
that part is powered **from the Pmod 3.3 V pin**, not from the local LDO.

The reason: when the controller is off, its rail is gone, the buffer is
unpowered, and its outputs are Hi-Z. The board can never inject current into
an unpowered FPGA pin. Any other arrangement means a board powered before the
KV260 is forward-biasing ESD diodes on bank 45.

The LVC2G17's inputs are 5.5 V tolerant, which is what lets it take the 5 V
'165 output on one channel while running at 3.3 V.

Budget: one dual gate switching at 40 MHz, well under 10 mA against the
100 mA Pmod allowance. Everything else on the board is on the local rails.
Measure the actual draw during bring-up and record it here.

### 9.2 Protection

5 V input chain: reverse-polarity P-FET → polyfuse → TVS → bulk cap → LDO.

---

## 10. Signal conditioning chains

**Input path**, every input without exception:

```
switch/sensor ─► series R ─► TVS ─► RC filter ─► Schmitt trigger ─► '165
                                    (~1 µs τ)      74HC14
```

**Output path**, per motor driver signal:

```
'595 (5 V) ─► 74LVC07A open drain ─► opto cathode on driver
                                     driver opto anode tied to board 5 V
```

Common-anode wiring: all `PUL+`, `DIR+`, `ENA+` tie to the board's 5 V rail;
the '07A sinks `PUL−`, `DIR−`, `ENA−`. DM542-class drivers have internal
current limiting for 5 V input, so no external series resistor is needed —
**confirm this on the datasheet of the driver actually purchased** and check
the '07A package total sink current against 8 channels of opto drive.

---

## 11. ESP32-S3 fallback protocol

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

## 12. Verification checklist

Before the schematic is considered done:

- [ ] Every bit in §6 and §7 traced to a physical connector pin
- [ ] Safe state verified with the controller unplugged
- [ ] Safe state verified during KV260 configuration (bitstream loading)
- [ ] Board powered with KV260 off — confirm zero current into any Pmod pin
- [ ] Driver minimum STEP pulse width, minimum low time, and max input
      frequency read off the real datasheet and recorded in §5.5
- [ ] Driver opto forward current calculated from the real datasheet
- [ ] Driver fault output confirmed present, or bits 8–15 demoted to spares
- [ ] `SCLK` at 40 MHz simulated and checked over 150 mm of the actual cable
- [ ] AHCT595 / AHCT165 confirmed as JLCPCB Basic parts
- [ ] Encoder part selected, 5 V tolerance confirmed (assume not), CS timing
      confirmed
- [ ] Pmod connector pin 7–10 mapping confirmed against the carrier schematic
- [ ] Carrier-side series resistors / ESD arrays on the Pmod nets identified
- [ ] Shared-clock single-strobe frame proven on a breadboard before layout

---

## 13. Changelog

| Date | Change | Reason |
|---|---|---|
| 2026-09-11 | Initial draft (rev A) | — |
| 2026-09-12 | `OUT_LATCH` + `IN_LOAD` merged into `STROBE` | '165 loads on level, '595 latches on edge; one pin serves both |
| 2026-09-12 | Freed pin assigned to `OUT_EN` | Hardware global disable independent of shift register contents |
| 2026-09-12 | Output word 24 → 32 bits, per-axis `ENA_N` replaces global `MOTOR_EN_N` | Per-axis electrical isolation during Phase 5–7 bring-up outweighs 0.2 µs of frame time |
| 2026-09-12 | Enable sense inverted, pull-up specified | DM542-class drivers enable when ENA is unconnected; naive default leaves all motors energised through boot |
| 2026-09-12 | `SCLK` 20 → 40 MHz, serializer moved to a 200 MHz domain | HDIO confirmed at 250 Mb/s; 40 MHz is not an integer divide of 100 MHz |
| 2026-09-12 | Quantisation framed as bounded, non-accumulating | Phase-accumulator step generation makes the average rate exact |
| 2026-09-12 | Pmod 3.3 V promoted from "reference only" to the return-buffer rail | Guarantees Hi-Z into an unpowered controller; the ground-loop concern in rev A was the wrong trade |
| 2026-09-12 | Logic family fixed to AHCT (was AHCT/LVC "family") | Plain HCT cannot do 40 MHz; AHCT thresholds accept 3.3 V drive without a translator |
| 2026-09-12 | Encoder CS level-shift added | '595 runs at 5 V; AS5047-class encoders are not 5 V tolerant |
| 2026-09-12 | Package pins and bank recorded from `base.xdc` | Was an open item in rev A |