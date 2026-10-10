# Humanoid Robot — Coding Standard & Project Layout
**Platform:** ESP32-S3 · PlatformIO · Arduino Framework
**Owner:** Hinz
**Version:** 2.3
**Last Revised:** 2026-05-14
**Goal of this file:** Encode *invariants and conventions*, not current values
or feature status. For live state, read the code or `CURRENT_FOCUS.md`.

---

## 0. How to Read This Document

Two tiers:

- **Principles** — the *why*. Stable across project evolution.
- **Conventions** — the *current how*. May change when a better option exists,
  provided the principle still holds and the change is documented here.

> **No hardware constants, loop rates, thresholds, or pins live in this file.**
> Read them from `project_wide_defs.h`, `joint_config.h`, `robot_geometry.h`.
> If documentation and code disagree, the code wins. Open an issue in
> `CURRENT_FOCUS.md` or fix the doc.

For module ownership, layer map, and per-tick ordering, see `ARCHITECTURE.md`.

---

## 1. Core Philosophy

> Write code a future-you, sleep-deprived and debugging at midnight, can read
> and trust without re-learning the whole system.

Three reasons:

1. **Blast-radius control** — a bug in the IMU filter must not require touching
   servo logic.
2. **Incremental progress** — every session should leave the codebase strictly
   better, not laterally reshuffled.
3. **Visible failure** — broken things shout; they never hide.

---

## 2. Project Layout

### Principle
Each module does exactly one job. When a bug is "in the controller," you find
it in one place without reading unrelated code.

### Current Convention

Flat layout: all headers in `include/`, all implementations in `src/`. Layering
is enforced through include discipline and API boundaries — not directory
hierarchy.

```
humanoid-robot/
├── platformio.ini
├── include/                    # one header per module
│   ├── project_wide_defs.h     # pins, loop rates, safety limits, gait params
│   ├── robot_geometry.h        # leg lengths, ankle linkage, CoM height
│   ├── gait_types.h            # FootTarget, BalanceCorrection (no logic)
│   ├── imu.h                   # L1 — BMI160 raw read (torso)
│   ├── lsm6dsv_driver.h        # L1 — LSM6DSV raw read (foot, per-instance)
│   ├── state_estimator.h       # L2 — complementary filter → IMUState
│   ├── balance_controller.h    # L3 — PD + task-space dx/dy + recovery detect
│   ├── weight_shift.h          # L3.5 — CoM shift author (task-space + bias)
│   ├── foot_contact.h          # L3.5 — pure predicates (isFootFlat / Loaded)
│   ├── gait_controller.h       # L3.5 — phase-driven stepping + recovery FSM
│   ├── leg_ik.h                # L3.5 — closed-form 2-link IK
│   ├── motion_manager.h        # L3.5 — joint authority arbitration
│   ├── servo_driver.h          # L4 — PCA9685 pulse writer
│   ├── joint_model.h           # L4 — joint math + smooth stepping
│   ├── joint_config.h          # L4 — single source of per-joint calibration
│   ├── servo_control.h         # L4 — motion coordinator (public API)
│   ├── oe_control.h            # L5 — OE pin / e-stop gate
│   └── webcomm.h               # L6 — WebSocket server + telemetry
├── src/                        # mirrors include/ by name
│   ├── main.cpp                # setup() + loop() only — wiring, no logic
│   ├── *_test.cpp              # UNIT_TEST-guarded test stubs
│   └── ...
├── data/index.html             # browser UI served from LittleFS
└── docs/                       # this dir
    ├── CODING_STANDARD.md      # invariants & conventions (this file)
    ├── ARCHITECTURE.md         # layer map + data flow + per-tick ordering
    ├── CURRENT_FOCUS.md        # live milestone state + open issues
    ├── GAIT_DESIGN.md          # gait FSM + capture-point + recovery
    └── TUNING_LOG.md           # dated gain change records
```

### Notes on Naming History

- `gait_types.h` is a vocabulary file (no logic). `FootTarget` and
  `BalanceCorrection` live here so `weight_shift`, `gait_controller`,
  `leg_ik`, and `balance_controller` can share types without circular
  includes.
- `WebComm.cpp` (PascalCase) matches its class name; predates this convention.
  See Deviations Log.
- `weight_shift` historically also drove the gait phase. That role moved into
  `gait_controller`; `weight_shift` is now the CoM-shift authoring module
  (lateral + forward task-space offsets and ankle-roll bias injection).

### Invariants

- **`main.cpp` is a wiring file.** No logic. If it grows past ~150 lines
  excluding comments, logic has leaked.
- **One header = one module responsibility.** Split if it does two jobs.

Document structural changes in `ARCHITECTURE.md` before implementing.

---

## 3. Layer Boundaries

### Principle
Data flows downward. No layer reaches back up. This is the single rule that
makes the system debuggable in isolation.

The full layer stack, per-tick ordering, and arbitration rules live in
`ARCHITECTURE.md`. This file only states the invariants:

1. A lower-layer header must never `#include` a higher-layer header.
   If you want to, put the shared type in `gait_types.h` (or a similar
   vocabulary file).
2. All servo writes go through `MotionManager.submit()` → `flush()`. Nothing
   bypasses this — including debug code.
3. The per-tick call order in `main.cpp` is a design invariant. Reordering
   requires updating `ARCHITECTURE.md` first.

### Joint authority arbitration

```
SOURCE_UI      (priority 0) — slider commands
SOURCE_GAIT    (priority 1) — WeightShift / GaitController / IK panel
SOURCE_BALANCE (priority 2) — BalanceController (always wins on shared joints)
```

In task-space mode (`BalanceConfig::task_space_output = true`),
BalanceController does **not** submit leg joints; it returns a
`BalanceCorrection` that GaitController merges into `FootTarget` before
LegIK. Torso pitch/roll still submit at `SOURCE_BALANCE` independently
(gated by `torso_balance_enabled`).

---

## 4. Naming

### Principle
A name communicates what something is and what unit it carries, without forcing
the reader to look at surrounding context.

### Current Conventions

| Construct | Convention | Example |
|---|---|---|
| Classes | `PascalCase` | `BalanceController` |
| Methods | `camelCase` | `computeCorrection()` |
| Member variables | `_camelCase` | `_pitchRateFiltered` |
| Constants / macros | `UPPER_SNAKE` | `GAIT_PHASE_RATE_HZ` |
| Struct fields, locals | `camelCase` | `state.pitchRate`, `dt_s` |
| Joint index enum prefix | `BJI_` | `BJI_ANKLE_L_PITCH` |
| Hardware channel prefix | `IDX_` | `IDX_L_ANKLE_PITCH` |

**Unit suffixes are mandatory** for any physical quantity. Suffixes in current
use: `Rad`, `Deg`, `Mm`, `M`, `Ms`, `Us`, `S`, `Hz`, `G`, `Lsb`. If a name
cannot carry one (e.g. struct field naming convention), put a `// unit: ...`
comment on the same line.

```cpp
// BAD
float angle;
// GOOD
float pitchRad;
float pitchRateDegPerSec;
uint32_t loopPeriodUs;
```

---

## 5. File Headers

### Principle
Opening a file should tell you its purpose, IO, and last meaningful change
without reading the implementation.

### Current Convention

```cpp
// =============================================================================
// FILE:    <name>.h
// MODULE:  <area>
// LAYER:   <N> — <layer name>
//
// PURPOSE: <1-3 sentences>
// INPUTS:  <struct types or 'sensor'>
// OUTPUTS: <struct types or 'commands via MotionManager'>
//
// DEPENDENCIES: <other headers consumed; one line each>
//
// LAST CHANGED: YYYY-MM-DD | author | what changed and why
// =============================================================================
```

Omit fields that are genuinely irrelevant — don't fill them with "N/A."

---

## 6. Function Design

### Principle
A function does one thing, describable in a sentence without "and," with
explicit inputs and outputs in the signature.

### Conventions

- **Soft length limit: 40 lines** — review signal, not error.
- No side effects through globals. Pass and return.
- Comment the **why**, not the **what**.
- Hardware-facing functions never silently signal failure; return a struct
  with a `valid` field or a status enum.

```cpp
// BAD — invisible global side effect
void updateAnkle(float cmd) { g_ankleCmd = cmd; }

// GOOD — explicit
JointCmd computeAnkleCmd(float controlOutput, float neutralAngleDeg);
```

---

## 7. Constants and Configuration

### Principle
Any tunable parameter, threshold, or hardware assignment has exactly one home.
Changing it there propagates everywhere automatically.

### Current Convention

| File | Contains |
|---|---|
| `project_wide_defs.h` | Pin assignments, loop rates, safety limits, gain ceilings, gait params, capture-point params, recovery params, foot IMU constants |
| `robot_geometry.h` | Physical geometry: leg segments, ankle linkage, chain delta, CoM height |
| `joint_config.{h,cpp}` | Per-joint calibration: neutral, direction, limits, range, no-load speed |
| `gait_types.h` | Shared struct definitions only (no constants) |

Acceptable bare literals in `.cpp` files: `0`, `1`, `-1`, mathematical
constants (`M_PI`, `1e-6f`), and trivial bit masks. Everything else lives in
a config header.

---

## 8. Error Handling

### Principle
Failures are explicit, named, and acted on immediately. A caller receiving a
failure either handles or propagates it — never silently continues.

### Current Convention

- Hardware-facing functions return a struct with a `valid` field
  (`RawIMUData`, `IMUState`) or a status enum (`IKStatus`).
- Layer boundaries check validity before proceeding:
  ```cpp
  IMUState state = estimator.update(raw, dt);
  if (!state.valid) return;   // skip tick — don't escalate to e-stop here
  ```
- Errors are **named** (string reason to Serial or telemetry), **visible**
  (UI log panel or serial), and **acted on** (e-stop, tick skip, or
  graceful refusal). Never swallow.

---

## 9. WebSocket Protocol

### Principle
UI and firmware must never silently diverge. The protocol must be human-
readable enough to debug from a browser console.

### Current Convention

Lightweight string-based, not JSON. `TYPE:payload` prefix; comma-separated
`key=value` for structured payloads. No version field — UI and firmware
change together in the same commit.

The current packet inventory is the authoritative one in `WebComm.cpp` and
the `onmessage` handler in `data/index.html`. **Do not maintain a duplicate
list in docs** — it goes stale. To see what packets exist, search:

```
grep -E '^[A-Z_]+:' src/WebComm.cpp
grep "startsWith\(" data/index.html
```

When adding a new packet, update both files in the same commit.

---

## 10. Git Commit Convention

```
<module>: <what changed> — <why>

Examples:
  gait: add capture-point step placement — sagittal LIP correction Stage 2.2
  foot-contact: add isFootFlat/isFootLoaded predicates — Phase F2 telemetry
  balance: clear roll_setpoint when entering task-space — eliminates dy_mm vs setpoint fight
```

- Commit after every working checkpoint.
- Never commit broken code to `main`.
- Never bundle two unrelated changes in one commit.

---

## 11. Test Stubs

### Principle
Every module should have at least one test that runs without hardware.

### Convention

`#ifdef UNIT_TEST` guards on a companion `<module>_test.cpp`. Enable by adding
`-DUNIT_TEST` to `build_flags` in `platformio.ini` and calling the test
runner from `setup()`.

Existing test runners: `runLegIKTests()` (`leg_ik_test.cpp`),
`runGaitControllerTests()` (`gait_controller_test.cpp`).

A stub that documents intended behavior beats nothing.

---

## 12. The Minimal Rewrite Checklist

Before any change:

1. **What is the exact problem?** One sentence.
2. **What is the minimum change that fixes it?** ≤5 lines if possible.
3. **What can break?** List downstream modules.
4. **How will I verify it works?** Serial log, UI panel, or test stub.

If you catch yourself rewriting a whole file when one function is wrong:
stop. Protect what works.

---

## 13. Anti-Patterns

| Anti-pattern | Why dangerous |
|---|---|
| Logic in `main.cpp` | Wiring file becomes monolith |
| Magic numbers in `.cpp` | Tuning requires search; values diverge |
| `#include` crossing layers upward | Hidden coupling |
| Silent fallback on sensor failure | Conceals the bug; servo still moves |
| Committing dead code | Future-you can't tell if it's intentional |
| Two unrelated changes in one commit | Rollback precision lost |
| Global mutable state | Race conditions; invisible side effects |
| Values duplicated in docs and code | Docs go stale; code wins silently |
| Writing servos directly, bypassing MotionManager | Priority arbitration breaks |
| Calling `oe_estop()` outside BalanceController / user CMD | E-stop must be a single safety path |
| Hardcoding `dt = 0.0025f` in gait code | Use `IMU_LOOP_HZ` derivation; otherwise silently breaks |
| `FootTarget` defined in `weight_shift.h` | Circular include — belongs in `gait_types.h` |
| Tuning Kp and Kd simultaneously | Root cause untraceable |
| Tuning `phaseRate` and `stepHeight` simultaneously | They interact |
| Adding a foot consumer that ignores `IMUState::valid` | Predicates may fire on calibrating estimator |
| Two `StateEstimator` instances sharing `FilterConfig` | Foot and torso scales differ; bias is per-instance |

---

## 14. Common Edits — Quick Reference

| Goal | Where |
|---|---|
| Change a joint's neutral / direction / limit | `src/joint_config.cpp` |
| Add a new telemetry field | `WebComm.cpp` broadcast methods + `index.html` parser, same commit |
| Tune balance / gait / recovery / foot-contact at runtime | WebSocket commands; see Section 9 |
| Add a new safety condition | `BalanceController::update()` for balance-related; `oe_loop()`/`oe_estop()` for hardware |
| Change a loop rate or threshold | `project_wide_defs.h`; verify dependent recalc formulas in adjacent comments |
| Add a new joint command source | New entry in `MotionSource` enum + case in `MotionManager::_applyCommand()`; call `submit(SOURCE_NEW, ch, deg)` |
| Add or modify a foot IMU | Instantiate a `StateEstimator` per foot with a foot-specific `FilterConfig`. Foot IMUs sit on Wire1 (SDA=11, SCL=12 @ 200 kHz), distinct from the BMI160's Wire0 bus. Each breakout's SA0 is hardwired to select its 7-bit address (0x6A or 0x6B); the driver does not drive GPIO. Do not raise the bus clock without re-running F1's loop-budget telemetry. |
| Add a new gait tuning parameter | `GaitConfig` field + `WebComm.cpp::GAIT_TUNE` parser + UI slider |
| Verify IK output for a specific FootTarget | UI IK Test panel (`IK_VALIDATE`, `IK_SOLVE`) — runs `LegIK::validateRoundtrip` and broadcasts result |

---

## 15. Open Architectural Issues

For *live* status (in-progress milestones, in-flight bugs), see
`CURRENT_FOCUS.md`. Persistent architectural concerns that don't fit a
milestone:

| Issue | Status | Resolution path |
|---|---|---|
| `BalanceConfig` concurrency | Open | WiFi task writes some fields (`setConfig()`), main task reads. Targeted setters (`setAnkleRollBias`, `setRollSetpointRad`) sidestep this for the WeightShift hot path; full-struct setters from the WS parser are still racy. Resolve via command-staging: queue config update in WS handler, apply at tick boundary. |
| Foot IMU bus margin at high speed | Open | Wire1 verified at 200 kHz with ~40 cm flex. Do not raise without re-running the F1 loop-budget telemetry (see `lsm6dsv_driver.h` notes). |
| `IMU_LOOP_HZ` not centralised | Open | The 400 Hz tick rate is encoded as the literal `2500` µs gate in `main.cpp` and as a comment-only constant elsewhere. Promote to a named constant in `project_wide_defs.h` and derive `dt` from it everywhere. |

---

## 16. Deviations Log

| Date | File | Convention deviated | Reason |
|---|---|---|---|
| — | `src/WebComm.cpp` | PascalCase, not snake_case | Matches class name; predates convention |
| — | `weight_shift.h/cpp` | Module renamed historical "gait" responsibility moved out | Kept name during incremental refactor; can be renamed to `com_shift` after gait stabilises |

---

*End of CODING_STANDARD.md*
