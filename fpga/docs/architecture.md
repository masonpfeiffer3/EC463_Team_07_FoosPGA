# Architecture — Autonomous Foosball Opponent

**Status:** Draft / tentative
**Last updated:** 2026-09-11

---

## 1. The one rule

> **The motor control core is vendor-neutral RTL. Everything AMD-specific
> lives in a wrapper above it.**

This single constraint is what makes the ESP32 fallback a wiring change rather than a redesign,
and what keeps the core simulatable in Verilator without a Vivado licence. If
a decision anywhere in this document conflicts with it, this rule wins.

Practical consequences:

- No `BUFG`, `IBUF`, `OBUFT`, `MMCME*`, `XPM_*`, or inferred Xilinx block RAM
  inside `rtl/motor/`, `rtl/safety/`, or `rtl/io/`.
- No AXI in the core. The core exposes a plain synchronous register-file
  interface; the AXI4-Lite bridge sits in the wrapper.
- No `initial` blocks for reset. Use an explicit synchronous or asynchronous
  reset port — ASIC flows will not infer FPGA power-on state.
- Stay inside the synthesisable SystemVerilog subset that Verilator, Vivado,
  and Yosys all accept. This is not a stylistic preference; it is what lets
  the *same* testbench run against RTL, FPGA implementation, and ASIC
  gate-level netlist.

---

## 2. System block diagram

```
                        ┌──────────────────────┐
                        │  High-speed camera   │
                        │  global shutter      │
                        │  ≥200 FPS, ≤500 µs   │
                        └──────────┬───────────┘
                                   │ USB3 / MIPI CSI-2
                                   ▼
   ┌───────────────────────────────────────────────────────────────┐
   │                      AMD Kria KV260 (K26)                     │
   │                                                               │
   │  ┌─────────────────────────┐   ┌──────────────────────────┐   │
   │  │   PS — ARM Cortex-A53   │   │   PL — FPGA fabric       │   │
   │  │   PetaLinux / Ubuntu    │   │   100 MHz                │   │
   │  │                         │   │                          │   │
   │  │  camera capture         │   │  ┌────────────────────┐  │   │
   │  │  calibration/homography │   │  │ AXI wrapper        │  │   │
   │  │  ball detection         │AXI│  │ (AMD-specific)     │  │   │
   │  │  velocity estimation    │◄──┼─►├────────────────────┤  │   │
   │  │  trajectory prediction  │   │  │ motor_control_core │  │   │
   │  │  strategy               │   │  │ PORTABLE RTL       │  │   │
   │  │  logging                │   │  │  ×8 axis_ctrl      │  │   │
   │  │                         │   │  │  scheduler         │  │   │
   │  │                         │   │  │  watchdog          │  │   │
   │  │                         │   │  │  fault_manager     │  │   │
   │  │                         │   │  │  serializer/deser  │  │   │
   │  └─────────────────────────┘   │  └─────────┬──────────┘  │   │
   │                                └────────────┼─────────────┘   │
   └─────────────────────────────────────────────┼─────────────────┘
                                                 │ Pmod, 8 signals
                                                 ▼
                        ┌────────────────────────────────────┐
                        │   Motor interface PCB (Rev A)      │
                        │   shift registers, buffers,        │
                        │   protection, connectors           │
                        │   NO motor phase current           │
                        └───┬──────────────┬──────────────┬──┘
                            │              │              │
              STEP/DIR/EN   │      limits/ │       SPI    │
                            ▼      faults  │              ▼
                 ┌──────────────────┐      │     ┌────────────────┐
                 │ 8 commercial     │      └─────┤ 4 rod-angle    │
                 │ stepper drivers  │            │ encoders       │
                 │ 24–48 V          │            └────────────────┘
                 └────────┬─────────┘
                          ▼
                 ┌──────────────────┐         ┌──────────────────┐
                 │ 4 × NEMA 23 lin  │         │  E-STOP          │
                 │ 4 × NEMA 17 rot  │◄────────┤  contactor,      │
                 └──────────────────┘  power  │  hardware only   │
                                              └──────────────────┘
```

---

## 3. Why the work is split this way

**Linux/PS gets everything irregular.** Vision, calibration, prediction, and
strategy are algorithmically messy, benefit from OpenCV and NumPy, change
often during development, and tolerate millisecond-scale jitter because
timestamps let you compensate. Putting them in fabric early would cost weeks
and buy nothing.

**Fabric gets everything rhythmic.** Step pulse generation at 200 kHz across
8 axes is trivial arithmetic that must never hiccup. Linux cannot do this —
not because it is slow, but because it is not deterministic. A 200 µs
scheduling delay is invisible to a video pipeline and catastrophic to a step
train.

**The dividing line is the register file.** Software writes target position,
velocity, acceleration. Fabric figures out the pulses. Software never sees an
individual step edge. This is also exactly the interface the ESP32 fallback
and the ASIC present, which is why the fallback is cheap.

---

## 4. Module hierarchy

```
foosball_top.sv                    ← AMD-specific, NOT portable
├── zynq_ps_wrapper                  block design, AXI, clocking
├── axi4lite_to_regfile.sv           bus bridge
└── motor_control_core.sv          ← PORTABLE. Reused verbatim for ASIC.
    ├── register_file.sv             §16 register map
    ├── command_scheduler.sv         timestamped / synchronised starts
    ├── timestamp_counter.sv         free-running µs counter
    ├── watchdog.sv                  heartbeat → global disable
    ├── fault_manager.sv             aggregates + latches faults
    ├── motor_serializer.sv          24-bit out, shift-register chain
    ├── input_deserializer.sv        24-bit in, shift-register chain
    └── axis_controller.sv  ×8
        ├── motion_profile.sv        trapezoidal ramp → velocity
        ├── step_generator.sv        velocity → STEP/DIR pulses
        └── position_counter.sv      signed 32-bit microsteps
```

Directory mapping:

| Path | Contents | Portable? |
|---|---|---|
| `rtl/motor/` | axis_controller, motion_profile, step_generator, position_counter, motor_control_core | **yes** |
| `rtl/safety/` | watchdog, fault_manager | **yes** |
| `rtl/io/` | motor_serializer, input_deserializer, register_file | **yes** |
| `rtl/common/` | shared packages, reset synchroniser, CDC primitives | **yes** |
| `rtl/wrapper_kv260/` | foosball_top, AXI bridge, Pmod constraints glue | no |
| `rtl/wrapper_tt/` | Tiny Tapeout top, SPI bridge, pin mapping | no |

---

## 5. Axis numbering

Fixed, referenced by the register map, the serial protocol, and the PCB
silkscreen. Do not renumber after the PCB is fabricated.

| Axis | Rod | Type | Motor |
|---|---|---|---|
| 0 | Goalie | Linear | NEMA 23 |
| 1 | Goalie | Rotary | NEMA 17 |
| 2 | Defense | Linear | NEMA 23 |
| 3 | Defense | Rotary | NEMA 17 |
| 4 | Midfield | Linear | NEMA 23 |
| 5 | Midfield | Rotary | NEMA 17 |
| 6 | Attack | Linear | NEMA 23 |
| 7 | Attack | Rotary | NEMA 17 |

So `axis = rod × 2 + (0 for linear, 1 for rotary)`, with rods numbered from
the automated goal outward. Even axes are linear, odd axes are rotary — which
makes "all linear axes" and "all rotary axes" single-bit-mask operations.

---

## 6. Data flow and latency budget

```
photon
  │  exposure                                    ≤ 0.5 ms
  ▼
sensor readout + transport + driver + userspace  ≤ 5–8 ms   ← dominant term
  │
  ▼
ball detection (threshold, moments, centroid)    ≤ 1–2 ms
  │
  ▼
state update: undistort, homography, velocity    ≤ 0.5 ms
  │
  ▼
trajectory prediction incl. wall bounces         ≤ 1 ms
  │
  ▼
strategy: choose rod targets                     ≤ 0.5 ms
  │
  ▼
register writes over AXI                         ≤ 0.1 ms
  │
  ▼
fabric: profile update → step train              ≤ 0.1 ms
  │
  ▼
serializer → PCB → driver → motor motion begins  ≤ 0.1 ms
  │
  ▼                                              ───────────
first mechanical motion                          ≈ 9–14 ms
```

Budget target ≤ 15 ms (LAT-1), stretch ≤ 10 ms (LAT-2).

Two observations that should drive effort allocation:

1. **Camera transport dominates by an order of magnitude.** Optimising the
   FPGA motor path buys nothing. This is why FPGA vision acceleration is
   Phase 10 and not Phase 2.
2. **Mechanical settling is not in this budget.** Carriage acceleration and
   rod inertia add to it and are expected to be the real limiter. Measure
   them in Phase 5 before optimising anything upstream.

---

## 7. Clock domains

| Domain | Rate | Scope |
|---|---|---|
| `clk_ps` | PS-derived | AXI bridge only, in the wrapper |
| `clk_core` | 100 MHz | Everything inside `motor_control_core` |
| `clk_shift` | 20–40 MHz | Serializer output clock, derived by integer division from `clk_core` |

`clk_shift` is generated by **counter-based enable** inside the core, not by a
second clock source and not by an MMCM. The serializer runs on `clk_core` with
a divided enable. This keeps the whole core single-clock, which removes every
CDC concern from the portable RTL and is the single biggest favour you can do
the eventual ASIC hardening flow.

The only real CDC in the design is AXI ↔ core, and it lives in the wrapper
where it is allowed to use vendor primitives.

---

## 8. Reset strategy

- One asynchronous-assert, synchronous-deassert reset per clock domain.
- Active-low `rst_n` throughout, for ASIC-flow compatibility.
- **All motor outputs are safe when `rst_n` is low.** STEP low, DIR
  don't-care, ENABLE in the disabled polarity, serializer latch held so the
  PCB's shift registers present their power-on safe state.
- Software must explicitly clear a global enable bit after reset before any
  axis will move. There is no "moves on boot" path.

---

## 9. Fallback and stretch paths

Both reuse `motor_control_core`'s *interface*, not its implementation, which
is why neither is a redesign.

**ESP32-S3 fallback** (if the PCB slips):

```
KV260 ──SPI/UART, high-level commands──► ESP32-S3 ──STEP/DIR──► 8 drivers
```

Commands carry axis, target position, velocity, acceleration, optional
execution timestamp — the same fields as the register map. The ESP32 runs
profiling and pulse generation locally using hardware timers/RMT. **Never**
stream individual step edges across this link. Benchmark 8 axes at 100–150 kHz
before trusting it.

**Tiny Tapeout ASIC** (stretch):

```
KV260 ──SPI──► TT motor ASIC ──serial──► same interface PCB ──► same drivers
```

`motor_control_core` is reused unchanged. Only `rtl/wrapper_tt/` is new.
Estimated 20–50 additional person-hours *if* portability was maintained from
day one; 50–100+ if vendor IP crept in. That difference is the entire
justification for §1.

---

## 10. Known architectural risks

| Risk | Impact | Mitigation |
|---|---|---|
| Serialised STEP quantises pulse timing to the serializer period (~1.2 µs at 20 MHz / 24 bits) | Step jitter at high rates; possible resonance | Raise shift clock to 40 MHz; consider a short fast word carrying only STEP. See `interfaces.md` §6. |
| KV260 single Pmod is a single point of failure for all motor I/O | Total loss of actuation | ESP32 fallback; alternate controller header on the PCB |
| Camera latency unmeasured in its true exposure-to-detection sense | Whole latency budget is unvalidated | LED transition test, Phase 1, before any mechanical work |
| Mechanical acceleration is the expected real bottleneck | Play quality ceiling independent of electronics | Phase 5 single-axis characterisation before committing to 8 |
| Vendor IP creeping into the core | ASIC path cost triples | Verilator in CI — it will simply fail to compile on Xilinx primitives |

---

## 11. Open questions

1. Does `command_scheduler` belong inside the portable core, or is
   timestamp-scheduled execution a wrapper concern? Currently inside, on the
   grounds that the ASIC would want it too.
2. Encoder SPI: bit-banged from the core's serializer block, or a separate
   portable SPI master? Separate master is cleaner but adds pins.
3. Whether the fault manager should stop only the faulted axis or all axes.
   Currently: driver faults and limits stop the faulted axis; watchdog and
   E-stop stop everything.
4. Where homing lives — fabric state machine or software sequence. Software
   is simpler and homing is not latency-critical. Leaning software.
