# Requirements — Autonomous Foosball Opponent

**Status:** Draft / tentative
**Last updated:** 2026-09-11
**Owner:** TBD

---

## 0. How to read this document

Each requirement has an ID (`SYS-1`, `VIS-3`, ...), a priority, and a
verification method. Priorities:

| Priority | Meaning |
|---|---|
| **M** | Mandatory. The project fails its own definition without it. |
| **S** | Should. Expected in the final demo, but the robot works without it. |
| **C** | Could. Stretch goal. Never a dependency for anything marked M. |

Verification methods: **T** = test on hardware, **S** = simulation,
**A** = analysis/calculation, **I** = inspection/demonstration.

A requirement without a number in it is usually a wish, not a requirement.
If you find yourself writing "fast" or "accurate," replace it with a figure
or move it to §9.

---

## 1. System

| ID | Priority | Requirement | Verify |
|---|---|---|---|
| SYS-1 | M | The system shall autonomously play one side of an unmodified-geometry standard foosball table against a single human opponent. | I |
| SYS-2 | M | The automated side shall control 4 rods (goalie, defense, midfield, attack). | I |
| SYS-3 | M | Each automated rod shall have 1 linear translation axis and 1 rotational axis, for 8 independently controlled axes total. | I |
| SYS-4 | M | The human side shall remain fully manual and unmodified. | I |
| SYS-5 | M | The system shall track the ball in real time and estimate its position, velocity, and predicted trajectory including wall bounces. | T |
| SYS-6 | M | The system shall select and execute rod actions autonomously with no human input during play. | I |
| SYS-7 | M | The robot shall function with no dependency on the Tiny Tapeout ASIC (§8). | I |
| SYS-8 | S | Total parts cost shall not exceed **$1,500**, excluding donated/sponsored parts and lab machining time. | A |
| SYS-9 | S | The system shall block a majority of shots on goal from a casual human player, measured over ≥50 shot attempts. | T |
| SYS-10 | C | The system shall score against a casual human player. | T |

**Open:** SYS-9 is the only requirement that states a play-quality target, and
"casual" is not yet defined. Decide before Phase 9 whether this is measured
against a named person, a team average, or a fixed shot rig.

---

## 2. Vision

| ID | Priority | Requirement | Verify |
|---|---|---|---|
| VIS-1 | M | The camera shall be global shutter. Rolling shutter is disqualifying at the target ball speeds. | I |
| VIS-2 | M | Sustained capture rate shall be ≥ **200 FPS** at the operating resolution. | T |
| VIS-3 | M | Exposure time shall be ≤ **500 µs**; target **100–250 µs** with bright diffuse lighting. | T |
| VIS-4 | M | The ball shall subtend ≥ **12 px** diameter in the captured image; ≥ 15 px preferred. | A, T |
| VIS-5 | M | Every frame shall carry a timestamp usable for latency compensation and prediction. | T |
| VIS-6 | M | The system shall report a per-detection confidence value and shall detect loss of track and reacquire. | T |
| VIS-7 | M | The camera shall be calibrated to table coordinates via homography; reported ball position shall be in millimetres on the table plane. | T |
| VIS-8 | S | Position error shall be ≤ **5 mm RMS** across the playing surface after calibration. | T |
| VIS-9 | S | Dropped-frame rate shall be < **0.1 %** over a 60 s continuous run. | T |
| VIS-10 | S | Inter-frame jitter shall be < **1 ms** (95th percentile). | T |
| VIS-11 | S | Raw frame datasets of real gameplay shall be recorded and stored for offline algorithm development and as FPGA test vectors. | I |
| VIS-12 | C | Capture rate ≥ **300–400 FPS**, pursued only if measurement shows the 200 FPS path limits play quality. | T |

**Design basis.** Worst-case ball speed assumed **10 m/s**. Ball travel
between samples: 50 mm @ 200 FPS, 33 mm @ 300 FPS, 25 mm @ 400 FPS. Motion
blur: 10 mm @ 1 ms exposure, 2.5 mm @ 250 µs, 1 mm @ 100 µs.

**Open:** VIS-4 depends on final camera mount height and lens FOV; run this
calculation before the mount is machined, not after.

---

## 3. Latency

| ID | Priority | Requirement | Verify |
|---|---|---|---|
| LAT-1 | M | End-to-end latency from photon capture to a valid motor command shall be ≤ **15 ms**. | T |
| LAT-2 | S | End-to-end latency shall be ≤ **10 ms**. | T |
| LAT-3 | M | End-to-end latency **jitter** shall be ≤ **2 ms** (95th percentile). | T |
| LAT-4 | S | Camera exposure to usable frame in software shall be ≤ **5–8 ms**. | T |
| LAT-5 | S | State/trajectory update shall complete in ≤ **2 ms**. | T |
| LAT-6 | S | Command transfer and scheduling shall complete in ≤ **0.5 ms**. | T |
| LAT-7 | S | FPGA image processing, if implemented, shall complete in < **1 ms**. | T |

**Rationale for LAT-3.** A consistent delay can be compensated with
timestamps and forward prediction. Jitter cannot. Prioritise reducing the
spread over reducing the mean.

**Open:** LAT-1 must be measured as true exposure-to-detection latency using
a timestamped LED transition, not inferred from frame-to-frame timing. The
5 ms measured on the initial PC test is unverified in this respect.

---

## 4. Linear axes (4)

| ID | Priority | Requirement | Verify |
|---|---|---|---|
| LIN-1 | M | Each linear axis shall be belt-driven from a NEMA 23-class stepper. | I |
| LIN-2 | M | Continuous torque at the motor shall be ≥ **2.0 N·m**; 2–3 N·m target. | A, T |
| LIN-3 | M | Travel shall cover the full legal rod translation range of the table. | T |
| LIN-4 | M | Each axis shall have a home switch, a far travel limit, and a physical hard stop at each end. | I |
| LIN-5 | M | The hard stops shall arrest the carriage without damage at maximum commanded velocity. | T |
| LIN-6 | S | Each axis shall traverse its full travel in ≤ **150 ms**. | T |
| LIN-7 | S | Open-loop operation shall be acceptable; position error after a 10-minute play session shall be ≤ **2 mm** without re-homing. | T |
| LIN-8 | C | Closed-loop steppers, only if LIN-7 fails on hardware. | T |

**Open:** LIN-6 is a guess. It should be derived from the ball-crossing time
across a rod's coverage at 10 m/s, not chosen. Do that calculation and
replace this number.

---

## 5. Rotary axes (4)

| ID | Priority | Requirement | Verify |
|---|---|---|---|
| ROT-1 | M | Each rotary axis shall be driven from a NEMA 17-class stepper through a belt reduction, initial ratio **2:1**. | I |
| ROT-2 | M | Continuous torque at the motor shall be ≥ **0.4 N·m**; 0.4–0.6 N·m target. | A, T |
| ROT-3 | M | The axis shall perform a full kick stroke and return without losing synchronism. | T |
| ROT-4 | M | Rotary motor mass shall be minimised where the motor rides on a moving carriage. | A |
| ROT-5 | S | Direct **rod-angle** feedback (not motor-shaft) shall be provided on each rotary axis. | T |
| ROT-6 | S | Rod angle shall be recoverable after belt slip, coupling slip, or impact-induced movement, without a homing cycle. | T |
| ROT-7 | S | Backlash at the rod, measured at the player figure, shall be ≤ **2°**. | T |

**Rationale for ROT-5.** A motor-shaft encoder verifies the motor did what it
was told. It cannot see belt slip, coupling slip, backlash, or a rod knocked
by the ball. Rod-angle sensing is the higher-value measurement and is the
reason encoder chip-selects appear in the PCB I/O list.

---

## 6. FPGA / motor control

| ID | Priority | Requirement | Verify |
|---|---|---|---|
| FPG-1 | M | The design shall implement **8 simultaneous** independent axis controllers. | S, T |
| FPG-2 | M | Maximum STEP rate shall be ≥ **150 kHz** per axis; target **200 kHz**. | S, T |
| FPG-3 | M | All 8 axes shall sustain the maximum STEP rate concurrently. | S, T |
| FPG-4 | M | Motion setpoint update rate shall be ≥ **10 kHz**. | S |
| FPG-5 | M | Each axis shall implement trapezoidal velocity profiling with independent acceleration and deceleration. | S |
| FPG-6 | M | Commanded velocity shall never exceed the programmed maximum. | S |
| FPG-7 | M | Step pulse timing shall respect the driver's minimum STEP high time, minimum low time, and DIR setup/hold. | S, T |
| FPG-8 | M | Position shall be maintained as **signed 32-bit microsteps** and shall be exact — no accumulated rounding. | S |
| FPG-9 | M | A hardware watchdog shall disable all motor outputs if not serviced within a programmable interval. | S, T |
| FPG-10 | M | Limit inputs and driver fault inputs shall stop the affected axis in hardware, without software intervention. | S, T |
| FPG-11 | M | A global disable shall force all STEP/DIR/ENABLE outputs to their safe state in one action. | S, T |
| FPG-12 | M | Outputs shall be in the safe state from configuration/reset until explicitly enabled by software. | T |
| FPG-13 | M | The motor control core shall be written in portable, synthesisable SystemVerilog with **no vendor-specific primitives or IP**. All AMD-specific logic shall live in a separate wrapper. | I |
| FPG-14 | S | A free-running timestamp counter shall be readable by software and shall be usable to schedule command execution. | S, T |
| FPG-15 | S | Simultaneous multi-axis motion start shall be supported via a single synchronised trigger. | S |
| FPG-16 | C | Vision pipeline blocks in fabric — only after software profiling proves a need. | T |

**Assumptions.** Fabric clock **100 MHz**. Serializer clock **20–40 MHz**.

---

## 7. Motor interface PCB

| ID | Priority | Requirement | Verify |
|---|---|---|---|
| PCB-1 | M | The board shall expand the KV260's single Pmod into the full motor and sensor I/O set via serialised shift-register chains. | T |
| PCB-2 | M | The board shall carry **no motor phase current**. All high-current switching stays in commercial external drivers. | I |
| PCB-3 | M | The board shall contain no MCU, CPLD, or firmware in Rev A. | I |
| PCB-4 | M | All inputs shall be protected and conditioned (filter + Schmitt trigger) before reaching a shift register. | I |
| PCB-5 | M | Outputs shall drive opto-isolated motor driver inputs at their specified current. | A, T |
| PCB-6 | M | Outputs shall default to the safe (motors disabled) state when unpowered, during KV260 configuration, and on loss of the serial clock. | T |
| PCB-7 | M | The board shall monitor E-stop status and motor-power status as inputs. | T |
| PCB-8 | M | The board shall not be powered from the Pmod rail. An external regulated 5 V supply shall feed local regulation. | I |
| PCB-9 | M | The board shall accept the KV260 Pmod **or** an alternate Tiny Tapeout controller header on the same serial interface. | I |
| PCB-10 | S | All components shall be JLCPCB/LCSC stocked and assembly-compatible. Large connectors may be hand-soldered. | I |
| PCB-11 | S | Rev A assembled cost shall be ≤ **$100** for the first batch. | A |
| PCB-12 | S | Test points and status LEDs shall be provided on every rail and every serial signal. | I |
| PCB-13 | S | Reverse-polarity protection, polyfuse, and TVS/ESD protection on all off-board signals. | I |

---

## 8. Safety

| ID | Priority | Requirement | Verify |
|---|---|---|---|
| SAF-1 | M | A physical E-stop shall remove or disable motor power through a path that does not depend on software, the FPGA, or the interface PCB. | T |
| SAF-2 | M | E-stop actuation shall bring all axes to rest and shall latch until manually reset. | T |
| SAF-3 | M | Every axis shall have physical hard stops that bound travel independently of software limits. | I |
| SAF-4 | M | Software position limits shall be enforced in hardware by the FPGA, in addition to the physical limits. | S, T |
| SAF-5 | M | Loss of the software heartbeat shall disable motor outputs (see FPG-9). | T |
| SAF-6 | M | Guarding shall prevent contact with moving mechanism during public demonstration. | I |
| SAF-7 | S | Motor power shall be fused and shall use a contactor that drops out on E-stop. | I |

**Non-negotiable.** SAF-1 is not satisfied by a GPIO read, an interrupt
handler, or an FPGA input. It is satisfied by a switch in the motor power
path.

---

## 9. Explicitly out of scope

Listed so they don't quietly become requirements:

- Two-sided automation (machine vs. machine).
- Ball serving / automatic ball return.
- Score detection.
- Learned strategy requiring training infrastructure beyond the capstone term.
- Replacing the Kria with a custom ASIC.
- Any requirement that the Tiny Tapeout ASIC exist, arrive, or work.

---

## 10. Open questions

1. **Board choice.** KV260 vs KR260 is not frozen. Current lean is KV260
   (~70/30) on the assumption the interface PCB lands. If the PCB schedule
   slips past Phase 4, this decision should be revisited, not defended.
2. **LIN-6 and ROT-3 timing targets** are placeholders pending a real
   reachability calculation from ball speed and rod coverage.
3. **STEP jitter budget** — the serialised output path quantises step timing
   to the serializer update period. See `interfaces.md` §6; this is the
   largest known technical risk in the PCB approach and FPG-2/FPG-7 may need
   to be restated in terms of achievable jitter.
4. **Rod-angle sensor selection** (ROT-5) — absolute magnetic vs. incremental
   with index. Absolute avoids a homing cycle per rod and is preferred, but
   the part is not chosen.
5. **Who owns each requirement section** once the team is formed.
