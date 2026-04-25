# Humanoid Robot — Coding Standard & Project Layout
**Platform:** ESP32-S3 · PlatformIO · Arduino Framework
**Author:** Hinz
**Version:** 2.2
**Last Revised:** 2026-04-25

---

## 0. How to Read This Document

This standard is structured in two tiers:

- **Principles** — the *why* behind each rule. These are stable and should
  not need to change as the project evolves.
- **Conventions** — the *current how*. These are the defaults that implement
  each principle right now. They can be changed when a better option exists,
  as long as the principle is still satisfied and the change is documented here.

When a convention no longer serves its principle, update the convention.
The principle is the constraint; the convention is the solution.

> **On project-specific values:** This document contains no hardware constants,
> loop rates, thresholds, or pin numbers. Those live exclusively in the codebase
> (`project_wide_defs.h`, `joint_config.h`, `robot_geometry.h`). If you need a
> value, read it from there — not from documentation.

---

## 1. Core Philosophy

> Write code that a future-you, sleep-deprived and debugging at midnight, can
> read and trust without re-learning the whole system.

Three reasons this standard exists:

1. **Blast radius control** — a bug in the IMU filter should never require
   touching servo logic.
2. **Incremental progress** — every session should leave the codebase strictly
   better, never laterally reshuffled.
3. **Visible failure** — broken things shout; they never hide.

---

## 2. Project Layout

### Principle
Each module does exactly one job. When a bug is "in the controller," you should
be able to find it in one place without reading unrelated code.

### Current Convention

The project uses a **flat layout**: all headers in `include/`, all
implementations in `src/`. Layering is enforced through include discipline
and API boundaries — not directory hierarchy.

```
humanoid-robot/
│
├── platformio.ini
│
├── include/                        # All headers. One per module.
│   ├── project_wide_defs.h         # System-wide: pins, loop rates, safety limits, gain ceilings, gait params.
│   ├── robot_geometry.h            # Physical geometry constants (leg lengths, ankle linkage, etc.).
│   ├── gait_types.h                # Shared structs: FootTarget, BalanceCorrection (no logic; avoids circular includes).
│   ├── imu.h                       # Layer 1 — raw IMU data struct + init/read API
│   ├── state_estimator.h           # Layer 2 — complementary filter + IMUState (configurable via FilterConfig)
│   ├── balance_controller.h        # Layer 3 — PD control, BalanceConfig, BalanceState
│   ├── weight_shift.h              # Layer 3.5 — lateral CoM + phase-driven gait (evolving toward GaitController)
│   ├── motion_manager.h            # Layer 3.5 — joint authority arbitration
│   ├── leg_ik.h                    # Layer 3.5 — closed-form 2-link IK; FootTarget → joint angles
│   ├── servo_driver.h              # Layer 4a — PCA9685 hardware write (pulse only)
│   ├── joint_model.h               # Layer 4b — joint math, limits, smooth stepping, per-channel deadband
│   ├── joint_config.h              # Layer 4c — single source of truth for joint calibration
│   ├── servo_control.h             # Layer 4d — motion coordinator (public API)
│   ├── oe_control.h                # Layer 5 — output enable / e-stop pin management
│   └── webcomm.h                   # Layer 6 — WebSocket server + telemetry broadcast
│
├── src/                            # All implementations. Mirrors include/ by name.
│   ├── main.cpp                    # setup() + loop() only. Wiring, no logic.
│   ├── imu.cpp
│   ├── state_estimator.cpp
│   ├── balance_controller.cpp
│   ├── weight_shift.cpp
│   ├── leg_ik.cpp
│   ├── motion_manager.cpp
│   ├── servo_driver.cpp
│   ├── joint_model.cpp
│   ├── joint_config.cpp
│   ├── servo_control.cpp
│   └── WebComm.cpp                 # Note: PascalCase — matches the class name.
│
├── data/                           # LittleFS assets served over HTTP
│   └── index.html                  # Browser UI (HTML + inline JS + CSS)
│
└── docs/
    ├── CODING_STANDARD.md          # This file.
    ├── ARCHITECTURE.md             # System diagram + design rationale.
    ├── gait_development_integration.md  # Gait plan evaluation + integration strategy.
    └── TUNING_LOG.md               # Dated records of gain changes and results.
```

### Notes on Layout Evolution

**`gait_types.h`** is a shared struct file, not a module. It defines `FootTarget`
and `BalanceCorrection` without logic so that `weight_shift.h` and `leg_ik.h`
can both include it without circular dependencies. It is not a layer; it is a
vocabulary file.

**`weight_shift.h/cpp`** is the current home of lateral CoM injection and is
actively evolving into a phase-driven GaitController (see §3). The file will be
renamed when the module is structurally stable. Do not create a separate
`gait_controller.h` prematurely — that adds a layer of indirection before the
design is settled.

**`leg_ik.h/cpp`** implements the closed-form 2-link IK solver. Its geometry
constants come from `robot_geometry.h`. It accepts `FootTarget` and returns
joint angles in degrees for `MotionManager.submit()`.

### Invariants That Don't Change

- **`main.cpp` is a wiring file.** No logic. If it grows past ~80 lines,
  logic has leaked into it.
- **One header = one module responsibility.** If a header does two jobs,
  split it.

Document any structural change in `ARCHITECTURE.md` before implementing it.

---

## 3. Layer Boundaries

### Principle
Data flows in one direction through the stack. No layer reaches back up.
This is the single rule that makes the system debuggable in isolation.

### Current Layer Stack

```
imu.h/cpp               → Layer 1:   raw BMI160 I/O → RawIMUData
state_estimator.h/cpp   → Layer 2:   complementary filter → IMUState
balance_controller.h/cpp→ Layer 3:   PD law + fall detection → SOURCE_BALANCE joint commands
weight_shift.h/cpp      → Layer 3.5: phase-driven CoM + foot trajectories → SOURCE_GAIT
leg_ik.h/cpp            → Layer 3.5: FootTarget → joint angles (geometry transform only)
motion_manager.h/cpp    → Layer 3.5: priority arbitration — flush dispatches joint winners
servo_control.h/cpp     → Layer 4:   motion API (delegates to JointModel + ServoDriver)
oe_control.h (main.cpp) → Layer 5:   output enable pin, boot hold, e-stop gate
webcomm.h/cpp           → Layer 6:   WebSocket server, telemetry broadcast, command parsing
```

**`main.cpp`** is the only file that wires layers together. It passes data
between them by calling their public APIs and passing return values downward.

A lower-layer file is not allowed to `#include` a header from a higher layer.
If you find yourself wanting to do this, a shared struct in the lower-level
header (or `gait_types.h`) is the correct solution.

### MotionManager — Joint Authority Arbitration

`MotionManager` sits between the controllers (Layer 3/3.5) and servo hardware
(Layer 4). It resolves conflicts when multiple sources want to command the same
joint in the same tick:

```
SOURCE_UI      (priority 0) — WebSocket slider commands
SOURCE_GAIT    (priority 1) — WeightShift / GaitController (IK output)
SOURCE_BALANCE (priority 2) — BalanceController (always wins shared joints)
```

Every controller calls `motionManager.submit(SOURCE_*, channel, angleDeg)`.
`main.cpp` calls `motionManager.flush()` once per tick after all submissions.
Nothing bypasses this path — including debug code.

### Per-Tick Ordering (main.cpp)

The ordering of calls in `main.cpp` is a known architectural constraint — it
must remain:

```
1. imu.read()
2. stateEstimator.update()
3. weightShift.update()       ← injects roll setpoint into BalanceConfig before balance runs
4. balanceController.update()
5. motionManager.flush()
```

Do not reorder these without a design review. `weightShift.update()` before
`balanceController.update()` is intentional: the injected setpoint must be
present when the PD law computes.

### Gait Integration — Phase 4 Merge Step

In Phase 4 of gait development, a merge step is inserted between steps 3 and 4:

```
3b. BalanceCorrection bc = balanceController.getCorrection();
3c. weightShift.applyCorrection(bc);   ← applies task-space trim to FootTargets before IK
```

This is deferred until Phase 1–3 are validated. Do not implement it early.

### Adapting This

As the project grows (CPG, foot FSR, additional IMUs), the layer map will evolve.
Update `ARCHITECTURE.md` first, then implement. The rule — **unidirectional
data flow** — stays permanent.

---

## 4. Naming

### Principle
A name should communicate what something is and what unit it carries, without
requiring the reader to look at surrounding context.

### Current Conventions

| Construct | Convention | Example |
|-----------|-----------|---------|
| Classes | `PascalCase` | `BalanceController` |
| Methods | `camelCase` | `computeOutput()` |
| Member variables | `_camelCase` | `_servo`, `_cfg`, `_pitch` |
| Constants / macros | `UPPER_SNAKE` | `BALANCE_KP_MAX` |
| Struct fields, local variables | `camelCase` | `state.pitchRate`, `float deltaTime` |
| Joint index enum prefix | `BJI_` | `BJI_ANKLE_L_PITCH` |
| Hardware channel constant prefix | `IDX_` | `IDX_ANKLE_L_PITCH` |

> **On member variable prefix:** The codebase uses `_camelCase` (single
> underscore) — not the `m_camelCase` pattern. These are equivalent in purpose;
> the underscore form is what the existing code uses. Be consistent within
> any file you edit.

**Unit suffixes are mandatory for all physical quantities:**

```cpp
// BAD — unit unknown from the name alone
float angle;
float vel;

// GOOD — unambiguous
float pitchRad;
float pitchRateDegPerSec;
uint32_t loopPeriodUs;
float stepHeightMm;
```

Every physical quantity must either carry a unit suffix (`Rad`, `Deg`, `Mm`,
`Ms`, `Us`, `Hz`, `G`) or have a `// unit: radians` comment on the same line.
This is the non-negotiable part of the naming principle.

### Adapting This

The invariant is: **unit must be unambiguous from name or comment alone**.
That part does not change.

---

## 5. File Headers

### Principle
Opening a file should immediately tell you its purpose, what it consumes,
what it produces, and what changed last — without reading the implementation.

### Current Convention

```cpp
// =============================================================================
// FILE:    state_estimator.h
// MODULE:  estimation
// LAYER:   2 — State Estimation
//
// PURPOSE:
//   Consumes raw IMU samples and produces a filtered IMUState (pitch, roll,
//   rates). Uses a complementary filter. See ARCHITECTURE.md §3 for filter
//   choice rationale and upgrade path.
//
// INPUTS:  RawIMUData    (from imu.h)
// OUTPUTS: IMUState      (consumed by balance_controller)
//
// DEPENDENCIES:
//   imu.h — RawIMUData
//
// LAST CHANGED: 2026-04-25  |  Hinz  |  <what changed and why>
// =============================================================================
```

### Adapting This

The invariant is: every file must document its **purpose**, **inputs/outputs**,
and **last significant change with reason**. If a field is genuinely irrelevant,
omit it — don't fill it with "N/A."

---

## 6. Function Design

### Principle
A function does one thing, is describable in a single sentence without "and,"
and makes its inputs and outputs explicit through its signature.

### Current Conventions

**Soft length limit: 40 lines.** This is a signal to review, not a hard error.

**No side effects through globals:**
```cpp
// BAD — invisible side effect through global
void updateAnkle(float cmd) { g_ankleCmd = cmd; }

// GOOD — explicit output
JointCmd computeAnkleCmd(float controlOutput, float neutralAngleDeg);
```

**Comment the why, not the what.**
```cpp
// BAD
pitch = alpha * pitch + (1 - alpha) * accelPitch; // blend pitch

// GOOD
// Complementary blend: gyro integral dominates short-term (low noise, high
// bandwidth); accel corrects long-term drift. Alpha is rate-dependent —
// recalculate if loop rate changes: alpha ≈ 1 - (1 / (tau_sec * hz)).
pitch = alpha * pitch + (1 - alpha) * accelPitch;
```

---

## 7. Constants and Configuration

### Principle
A value that appears in more than one place is a bug waiting to happen.
Every tunable parameter, threshold, and hardware assignment has exactly one
home. Changing it there propagates everywhere automatically.

### Current Convention

| File | Contains |
|---|---|
| `include/project_wide_defs.h` | Pin assignments, loop rates, system-wide safety limits, gain ceilings, **gait phase and trajectory parameters** |
| `include/robot_geometry.h` | Physical geometry: leg segment lengths, ankle linkage dimensions, squat configuration |
| `include/joint_config.h` + `src/joint_config.cpp` | Per-joint calibration: neutral angle, direction, limits, servo range, no-load speed |
| `include/gait_types.h` | Shared struct definitions: `FootTarget`, `BalanceCorrection` (no constants) |

**Gait parameters live in a clearly delimited `// GAIT` block inside
`project_wide_defs.h`.** Do not create a separate `gait_config.h` unless the
gait parameter count becomes large enough to pollute the main file. The block
delimiter provides sufficient separation.

No numeric literal that represents a tunable value, threshold, or hardware
parameter belongs in a `.cpp` file. Acceptable bare literals: `0`, `1`, `-1`,
and mathematical constants (`M_PI`, `1e-6f`).

### Adapting This

If a new module has a large, self-contained parameter set, create a dedicated
config header (e.g., `cpg_config.h`) following the same single-source pattern.
The invariant: **one definition, one file, one place to change it.**

---

## 8. Error Handling

### Principle
Failures are explicit, named, and acted on immediately. A function that can
fail must communicate that failure to its caller. A caller that receives a
failure must handle or propagate it — never silently continue.

### Current Convention

Hardware-facing functions return a struct with a `valid` field:
```cpp
IMUState result;
result.valid = false;
if (!readSucceeded) return result; // explicit invalid, no garbage data
```

Layer boundaries check validity before proceeding:
```cpp
IMUState state = estimator.update(raw, dt);
if (!state.valid) {
    // Don't call oe_estop() here — let the balance controller's fall
    // detection handle persistent invalidity. Just skip this tick.
    return;
}
```

Errors are always: **named** (string reason passed to Serial or telemetry),
**visible** (surfaced in UI log panel or Serial monitor), and **acted on**
(servo disable via `oe_estop()` or tick skip). Never silently swallow a failure.

### Adapting This

The `valid` bool in structs is the current pattern. If the project grows to
require richer error types (error codes, severity levels), replace it — as
long as: failure is explicit, failure is named, and failure is never silent.

---

## 9. WebSocket Protocol

### Principle
The UI and firmware must never silently diverge. The protocol must be
human-readable enough to debug with a browser console.

### Current Convention

The protocol is **lightweight string-based** — not JSON. Messages use a
`TYPE:payload` prefix format with comma-separated key=value fields for
structured data. There is no version field; breaking changes are handled
by updating both `WebComm.cpp` and `index.html` together in the same commit.

**Outbound (ESP32 → browser):**
```
STATE:<ch>=<pulse_us>:<deg_abs>,...;ESTOP=0|1
IMU:ax=X,ay=Y,az=Z,gx=X,gy=Y,gz=Z
ESTIMATE:pitch=X,roll=X,pitchRate=X,rollRate=X
CALIB:state=collecting|done|waiting,progress=0.75,remaining=1.2
BALANCE:p_en=1,r_en=0,p_err=0.031,p_u=1.5,p_ank=1.5,...
ESTOP:ACTIVE  |  ESTOP:CLEAR
SPEED:<ch0_degS>,<ch1_degS>,...
GAIT:phase=0.42,lStance=1,liftL=0.0,liftR=14.3,latShift=32.1
```

**Inbound (browser → ESP32):**
```
<channel>:<absoluteDeg>              # slider command (e.g., "1:138.5")
CMD:RESETP
CMD:PITCH_ON  |  CMD:PITCH_OFF
CMD:ROLL_ON   |  CMD:ROLL_OFF
CMD:BALANCE_TUNE:Kp=10.0,Kd=0.3,setpoint=0.0,ankle=1.0,hip=0.0,...
CMD:WEIGHT_SHIFT:left|right|center
CMD:WEIGHT_SHIFT_TUNE:setpoint=0.05,ankle=5.0,ramp=500,...
CMD:GAIT_START  |  CMD:GAIT_STOP
CMD:GAIT_TUNE:phaseRate=0.4,stepHeight=20.0,stanceWidth=40.0,...
CMD:SPEED:<channel>:<degPerSec>
CMD:ESTOP
CMD:CLEAR_ESTOP
SAVE:<pose_name>
```

When adding new message types: add the outbound format to `WebComm.cpp`
broadcastX() methods and the inbound handler to `handleWebSocketMessage()`.
Update `index.html` JS to match. Both changes go in the same commit.

---

## 10. Git Commit Convention

### Principle
Every commit is a recoverable checkpoint. The message identifies what changed,
where, and why — so bisecting a regression is fast.

### Current Convention

```
<module>: <what changed> — <why>

Examples:
  estimation: fix gyro axis remapping — Z was mapped to Y causing 90° error
  control: add output clamp — prevent servo damage on gain overshoot
  safety: lower estop threshold — hardware test showed higher value causes impact
  actuation: add ankle pitch rate limiter — suppresses step inputs that excite resonance
  gait: add phase counter to WeightShift — first step toward GaitController
  gait: add sinusoidal foot lift trajectory — Phase 1 stepping in place
```

- Commit after every working checkpoint.
- Never commit broken code to `main`.
- Never bundle two unrelated changes in one commit — rollback precision
  depends on separation.

---

## 11. Test Stubs

### Principle
Every module should have at least one test that runs without hardware.
Tests are executable documentation of what correct behavior means.

### Current Convention

Every new `.cpp` module gets a companion `_test.cpp` with compile-time guards:

```cpp
#ifdef UNIT_TEST
#include "state_estimator.h"
#include <cassert>

void test_stationaryReturnsNearZeroPitch() {
    // simulate stationary input, verify pitch ≈ 0
}
#endif
```

An incomplete stub that documents intended behavior is better than nothing.
Write the test before the implementation when feasible.

---

## 12. The Minimal Rewrite Checklist

Before making any change, answer these four questions:

1. **What is the exact problem?** (One sentence. If you can't write it,
   you don't understand it yet.)
2. **What is the minimum change that fixes it?** (Can this be done in ≤5 lines?
   If not, why not?)
3. **What can break?** (List downstream modules. Check their interfaces.)
4. **How will I verify it works?** (Serial log, UI panel, or test stub?)

If you catch yourself rewriting a whole file when only one function is wrong,
stop. Protect the parts that already work.

---

## 13. Anti-Patterns

| Anti-pattern | Why it's dangerous |
|---|---|
| Logic in `main.cpp` | Turns the wiring file into a monolith |
| Magic numbers in `.cpp` files | Tuning requires a code search; values diverge |
| `#include` crossing layers upward | Creates hidden coupling |
| Silent fallback on sensor failure | Conceals the bug; servo still moves |
| Committing dead code | Future-you won't know if it's intentional |
| Two unrelated changes in one commit | Rollback precision lost |
| Global mutable state | Race conditions; invisible side effects |
| Values duplicated in docs and code | Docs go stale; code wins silently |
| Writing hardware directly from a controller, bypassing MotionManager | Priority arbitration breaks; last-write wins on shared joints |
| Calling `oe_estop()` from application code (other than BalanceController) | E-stop state must be managed through the defined safety path |
| Hardcoding `dt = 0.01f` in gait code | Silently breaks if `IMU_LOOP_HZ` changes; derive `dt` from `project_wide_defs.h` |
| Defining `FootTarget` in `weight_shift.h` | Circular include — `FootTarget` belongs in `gait_types.h` |
| Adding CPG before Phase 3 is validated | CPG self-oscillates; need a known-good trajectory baseline first |
| Tuning `phaseRate` and `stepHeight` simultaneously | Tune `stepHeight` to clean lift/land first, then increase `phaseRate` |
| Lifting swing foot before weight shift threshold is met | Skip the `leftMayLift` / `rightMayLift` guard and the robot falls |
| Calling `oe_estop()` from GaitController on gait error | Gait should pause phase; e-stop is BalanceController's domain |
| Two StateEstimator instances without separate `FilterConfig` | IMU configuration for torso vs foot must be independently tunable |

---

## 14. Common Edits — Quick Reference

**Change a joint's neutral angle →** `src/joint_config.cpp`. One entry in the
config array. Done. Touch nothing else.

**Add a telemetry field →** Add a `snprintf` field to the relevant broadcast
method in `src/WebComm.cpp`. Add the corresponding key parse to `index.html`
JS. Only add to a module header struct if it's a new physical quantity crossing
module boundaries.

**Tune gains without reflashing →** Send `CMD:BALANCE_TUNE:Kp=X,Kd=Y,...` via
the WebSocket UI. Log the session result in `docs/TUNING_LOG.md`.

**Tune gait parameters without reflashing →** Send `CMD:GAIT_TUNE:phaseRate=X,
stepHeight=Y,...` via WebSocket. Tune `stepHeight` first (verify clean
lift/land), then `phaseRate`. One parameter at a time.

**Add a new safety condition →** Add the check inside `BalanceController::update()`
(for balance-related conditions) or in the `oe_loop()`/`oe_estop()` path in
`main.cpp` (for hardware-level conditions). Named string reason. Add display to
the UI log panel. Do not scatter safety logic across multiple modules.

**Change a loop rate or threshold →** `include/project_wide_defs.h`. One line.
Then verify any downstream calculations that depend on the value — document
their recalculation formula in a comment next to the constant, not in docs.

**Add a new gait parameter →** Add it to the `// GAIT` block in
`project_wide_defs.h`. Wire it to `CMD:GAIT_TUNE` parsing in `WebComm.cpp` so
it can be tuned at runtime without reflashing.

**Add a new joint command source →** Add an entry to the `MotionSource` enum in
`motion_manager.h` with the correct priority integer. Add a dispatch case in
`MotionManager::_applyCommand()`. Call `submit(SOURCE_NEW, ch, deg)` from the
new controller. No other files need to change.

**Add a foot IMU (LSM6DSV16X) →** Instantiate a second `StateEstimator` with a
foot-specific `FilterConfig`. Mount on SPI to avoid I2C address collision with
the torso BMI160. Do not share filter state between torso and foot estimators.
Do this after first stable steps are confirmed — do not conflate IMU driver work
with gait debugging.

---

## 15. Open Known Issues

These are confirmed architectural gaps or unverified assumptions. Each has a
planned resolution path. Do not paper over them with workarounds.

| Issue | Status | Resolution Path |
|---|---|---|
| **BalanceConfig concurrency** | Open | WiFi task writes `BalanceConfig`; main task reads. No mutex. Medium priority: resolve via command-staging pattern (WebComm queues config update; main task applies at tick boundary). |
| **Torso pitch servo direction** | Unverified | Active balance on torso pitch not yet enabled. Verify direction before enabling: tilt robot forward and confirm correction opposes the lean. Use `BALANCE_TUNE:sign=1/-1` to correct without reflashing. |
| **Foot IMU integration** | Planned post-first-steps | Two LSM6DSV16X IMUs (SPI) on hand. Sequence after first stable steps to avoid conflating driver bugs with gait issues. |

---

## 16. Deviations Log

When you intentionally deviate from a convention, record it here so the next
reader knows the choice was deliberate:

| Date | File | Convention Deviated | Reason |
|------|------|---------------------|--------|
| — | `src/WebComm.cpp` | File named `WebComm.cpp` not `web_comm.cpp` | Matches the class name `WebComm`; changed before this convention was written |
| — | `weight_shift.h/cpp` | Module doing gait phase work named "WeightShift" | Historical name preserved during active gait integration; will be renamed to `GaitController` when module is structurally stable |

---

*End of CODING_STANDARD.md*
