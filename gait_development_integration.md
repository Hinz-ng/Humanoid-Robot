# Gait Development Integration Guide

> **Document scope:** Evaluation of `Humanoid Walking Gait Development Plan` + integration
> strategy into the existing ESP32-S3 codebase. Written for a solo developer who already has
> the layer stack running (IMU → StateEstimator → BalanceController → WeightShift →
> MotionManager → ServoControl). Read this once before touching any file.

---

## Plan Evaluation Summary

### What the Plan Gets Right

The core insight — *walking is timed CoM coordination, not position control* — is correct and
is the right frame to build the entire gait system around. The phase-based spine, the
stance/swing split, and the task-space balance correction direction are all architecturally
sound for a first real walking gait on this hardware class. The phased roadmap (Phase 1 → 5)
also matches how real locomotion systems are validated: smooth stepping before forward motion,
forward motion before balance integration.

### Critical Gap: WeightShift Already Exists

The plan does not acknowledge that `weight_shift.h/cpp` (Layer 3.5) already implements
lateral CoM injection and swing leg lift — the exact behavior targeted in Plan Phase 2. This
is the most important discrepancy between the plan and the codebase.

**Decision required before writing code:**

| Option | Description | Risk |
|--------|-------------|------|
| **A — Extend WeightShift** | Add phase variable and trajectory functions into WeightShift; rename conceptually to GaitController | Low disruption; reuses existing MotionManager wiring |
| **B — Replace WeightShift** | Write GaitController as a new module; delete or stub WeightShift | Cleaner long-term; more upfront refactor |
| **C — Promote WeightShift** | Keep WeightShift as the phase driver; GaitController wraps it for forward walking | Over-layering; avoid |

**Recommendation: Option A.** Extend WeightShift into the phase-driven GaitController
rather than duplicating the MotionManager wiring and BalanceConfig injection logic. Rename
the file when the module is structurally stable. The plan's "GaitController" maps to an
evolved WeightShift.

### Secondary Gap: Balance Output Refactor Is Non-Trivial

The plan frames BalanceController's move to task-space output as a moderate change. In the
actual codebase, `BalanceController` submits joint-angle corrections at `SOURCE_BALANCE`
priority (priority 2 — always wins). Changing its output to a `BalanceCorrection` struct
applied before IK means:

- `BalanceController::update()` no longer calls `motionManager.submit(SOURCE_BALANCE, ...)` for walking joints
- A new merge step (FootTarget + BalanceCorrection → IK → submit) replaces direct submission
- The `SOURCE_BALANCE` path still needs to exist for non-walking joints (torso, neck, etc.)

This is a medium-complexity refactor, not a small one. It also touches the MotionManager
flush ordering gotcha and the WeightShift → BalanceConfig coupling. Plan for it in Phase 4,
not Phase 1.

### Minor Concerns

- **`dt` source:** The plan's phase update (`_phase += dt * _phaseRate`) assumes `dt` is
  available. Confirm `dt` is derived from `IMU_LOOP_HZ` (in `project_wide_defs.h`), not a
  free-running timer. Do not introduce a second timing source.
- **IK module status:** The plan treats `LegIK` as existing and stable. Verify `leg_ik.h/cpp`
  exists and its input interface before designing the FootTarget → IK boundary.
- **`FootTarget` struct definition:** Needs a single canonical home. Recommend
  `include/gait_types.h` to avoid circular includes between GaitController and IK.

---

## High-Level Architecture

### Current Layer Stack (Annotated)

```
Layer 1   imu.h/cpp              BMI160 raw read → RawIMUData
Layer 2   state_estimator.h/cpp  Complementary filter → IMUState
Layer 3   balance_controller.h/cpp  Pitch + roll PD → SOURCE_BALANCE joint commands
Layer 3.5 weight_shift.h/cpp     Lateral CoM + swing lift → SOURCE_GAIT joint commands
          motion_manager.h/cpp   Priority arbitration (flush once per tick)
Layer 4   servo_control.h/cpp    JointModel + ServoDriver
Layer 5   oe_control (main.cpp)  OE gate, e-stop
Layer 6   webcomm.h/cpp          WebSocket + telemetry
```

### Target Layer Stack (Post-Integration)

```
Layer 1   imu.h/cpp              (unchanged)
Layer 2   state_estimator.h/cpp  (unchanged)
Layer 3   balance_controller.h/cpp  → BalanceCorrection (task-space, Phase 4+)
          gait_types.h           FootTarget + BalanceCorrection structs (new)
Layer 3.5 weight_shift.h/cpp     → GaitController (phase-driven, extended)
          leg_ik.h/cpp           FootTarget → joint angles (verify exists)
          motion_manager.h/cpp   (unchanged — IK output submits joints)
Layer 4   servo_control.h/cpp    (unchanged)
Layer 5   oe_control (main.cpp)  (unchanged)
Layer 6   webcomm.h/cpp          (add phase telemetry, optionally)
```

### Data Flow Diagram

```
                   IMU
                    │
             StateEstimator
                    │
              IMUState (pitch, roll, rates)
                    │
          ┌─────────┴──────────┐
          │                    │
   BalanceController      GaitController  ◄── Phase (0→1), phaseRate
          │                    │
   BalanceCorrection      FootTarget[L/R]
   (dx,dy,dPitch,dRoll)        │
          └─────────┬──────────┘
                    │
           Merge: FootTarget + BalanceCorrection
           (Phase 4+ only; Phase 1–3: GaitController only)
                    │
                   IK
                    │
             joint angles
                    │
          MotionManager.submit(SOURCE_GAIT)
                    │
          MotionManager.flush()  ◄── called once per tick in main.cpp
                    │
             ServoControl
                    │
                  Servos
```

> **Phase 1–3 simplification:** Skip the merge step entirely. GaitController outputs foot
> targets → IK → MotionManager. BalanceController continues submitting joint corrections at
> SOURCE_BALANCE as today. This means balance and gait will fight slightly — acceptable for
> early validation, not for final gait.

---

## Implementation Plan

### Phase 1 — Smooth Stepping In Place

**Goal:** Replace waypoint system with continuous phase-driven foot trajectories. No forward
motion yet. Validate symmetry and smoothness.

**Files to touch:**
- `weight_shift.h` / `weight_shift.cpp` — add phase variable and trajectory functions
- `include/project_wide_defs.h` — add `GAIT_PHASE_RATE_HZ`, `GAIT_STEP_HEIGHT_MM`,
  `GAIT_STANCE_WIDTH_MM`
- `include/gait_types.h` — create; define `FootTarget` struct
- `main.cpp` — update call site if WeightShift interface changes

**Key additions to WeightShift:**

```cpp
// weight_shift.h additions
float _phase;       // [0.0 → 1.0), cycles per PHASE_RATE
float _phaseRate;   // cycles/sec — from project_wide_defs.h

// Stance/swing derived per tick
bool _leftIsStance;  // phase < 0.5

void updatePhase(float dt_sec);
FootTarget computeFootTarget(bool isLeft, float phase);
```

**Phase update (use fixed dt from loop rate, not millis()):**

```cpp
// dt must come from project_wide_defs.h loop period, not a new timer
constexpr float DT_SEC = 1.0f / IMU_LOOP_HZ;

void WeightShift::updatePhase() {
    _phase += DT_SEC * _phaseRate;
    if (_phase >= 1.0f) _phase -= 1.0f;
    _leftIsStance = (_phase < 0.5f);
}
```

**Trajectory functions (start with sinusoids — do not optimize yet):**

```cpp
// Lateral CoM shift — weight fully over stance side at phase 0.25, 0.75
float lateralShift_mm = GAIT_STANCE_WIDTH_MM * sinf(TWO_PI * _phase);

// Swing leg vertical lift — only when this leg is swing
float swingPhase = _leftIsStance ? (_phase - 0.5f) * 2.0f : _phase * 2.0f;
float liftHeight_mm = 0.0f;
if (isSwingLeg) {
    liftHeight_mm = GAIT_STEP_HEIGHT_MM * sinf(PI * swingPhase);
}
```

**Test criteria:**
- Serial print: `phase, leftIsStance, liftHeight_L, liftHeight_R` each tick
- Visually: symmetric step rhythm, no jerky transitions, no phase jump at 1.0 → 0.0 wrap
- Confirm: foot never lifts before CoM has shifted (verify via serial log timing)

**Estimated effort:** 1–2 sessions.

---

### Phase 2 — Validated Single-Leg Support

**Goal:** Confirm the lateral CoM shift is sufficient for brief single-leg standing. This is
the physical gate before adding forward steps.

**Key constraint:** The robot must be able to stand on one leg for ≥ 0.5 seconds without
the stance-side ankle corrections saturating. If it cannot, the step height or stance width
needs to change — not the phase rate.

**Files to touch:**
- `project_wide_defs.h` — add `GAIT_WEIGHT_SHIFT_ADVANCE_DEG` (how far weight shifts before
  foot lifts; start at 80% of stance width)
- `weight_shift.cpp` — add lift inhibit: foot only clears ground once IMU roll confirms weight
  on stance side

**Lift inhibit logic (minimal):**

```cpp
// Delay swing leg lift until lateral shift is sufficiently advanced
// Use roll from IMUState as proxy for CoM position
bool weightOnLeft  = (imuState.rollRad >  WEIGHT_SHIFT_THRESHOLD_RAD);
bool weightOnRight = (imuState.rollRad < -WEIGHT_SHIFT_THRESHOLD_RAD);

bool leftMayLift  = weightOnRight;  // CoM over right = left can lift
bool rightMayLift = weightOnLeft;
```

> `WEIGHT_SHIFT_THRESHOLD_RAD` goes in `project_wide_defs.h`. Start conservatively (~5–8°).

**Test criteria:**
- Can the robot hold balance on one leg for one full swing phase without falling?
- Does SOURCE_BALANCE (BalanceController) saturate during single-leg stance?
- Serial log: `roll_rad, leftMayLift, rightMayLift` each tick

**Estimated effort:** 1 session + tuning.

---

### Phase 3 — Forward Walking

**Goal:** Add X-axis foot placement offset. Produce 2–5 steps forward.

**Key insight from the plan (correct):** Forward walking = controlled falling forward + foot
catching. The X offset of each step creates the lean; the swing foot placement stops the fall.

**Files to touch:**
- `weight_shift.cpp` — add `x_mm` component to FootTarget
- `project_wide_defs.h` — add `GAIT_STEP_LENGTH_MM`
- `gait_types.h` — ensure FootTarget carries `x_mm`, `y_mm`, `h_mm`

**X trajectory (start linear, not sinusoidal):**

```cpp
// Swing phase goes 0 → 1 during swing
// Foot starts behind CoM, lands ahead of CoM
float swingX_mm = GAIT_STEP_LENGTH_MM * (swingPhase - 0.5f);
// Stance foot: stationary or slight pushback (optional — skip for Phase 3)
float stanceX_mm = 0.0f;
```

**Integration with IK:**

```cpp
FootTarget ft;
ft.x_mm = swingX_mm;
ft.y_mm = lateralShift_mm;    // from Phase 1
ft.h_mm = liftHeight_mm;      // from Phase 1
// Pass ft to IK; IK returns joint angles; submit via SOURCE_GAIT
```

> If `leg_ik.h/cpp` does not exist yet, Phase 3 is blocked until IK is implemented.
> Do not approximate IK with manual joint angle offsets — this creates a debt that
> breaks every downstream phase.

**Test criteria:**
- Robot takes ≥ 2 steps before falling
- No foot drag (h_mm clearance sufficient)
- Phase rate ≤ 0.5 Hz for initial tests — slow enough to observe and stop

**Estimated effort:** 1–3 sessions depending on IK status.

---

### Phase 4 — Balance Integration (Task-Space)

**Goal:** Move BalanceController output from joint-space overrides to task-space corrections
applied before IK. This is the architectural refactor deferred from Phase 1.

> This phase has the highest complexity and the highest risk of breaking existing behavior.
> Only begin after Phase 3 produces consistent forward walking.

**Files to touch:**
- `balance_controller.h` — add `BalanceCorrection getCorrection()` output path
- `balance_controller.cpp` — compute `dx_mm`, `dy_mm`, `dPitch_deg`, `dRoll_deg` instead of
  (or in addition to) joint commands
- `gait_types.h` — add `BalanceCorrection` struct
- `main.cpp` — update merge step in walk update path
- `weight_shift.cpp` (GaitController) — accept BalanceCorrection, apply before IK call

**BalanceCorrection struct:**

```cpp
// gait_types.h
struct BalanceCorrection {
    float dx_mm;       // forward/back body shift
    float dy_mm;       // lateral body shift
    float dPitch_deg;  // trunk pitch trim
    float dRoll_deg;   // trunk roll trim
    bool  valid;       // false → do not apply this tick
};
```

**Migration strategy (do not remove joint-space path immediately):**

Consider keeping the existing `SOURCE_BALANCE` joint submission path active with a feature
flag during transition. This lets you compare walking behavior with and without task-space
corrections without a hard cutover:

```cpp
// project_wide_defs.h
#define BALANCE_OUTPUT_TASK_SPACE 0  // 0 = legacy joint-space, 1 = task-space
```

**Safety audit required** for any change to BalanceController output path:
- E-stop threshold and fall detection must remain on the joint-space path (or equivalent)
- `SOURCE_BALANCE` for torso/neck joints (non-walking joints) should remain unchanged
- Confirm `motionManager.flush()` ordering is preserved after refactor

**Test criteria:**
- Walking gait survives a small lateral push
- Oscillation reduced vs Phase 3 baseline
- E-stop still triggers on fall — do not break this

**Estimated effort:** 2–4 sessions.

---

### Phase 5 — Refinement (Optional)

Consider only after Phase 4 is stable on hardware.

| Refinement | Benefit | Risk |
|------------|---------|------|
| Smoothstep instead of sinusoid | Less peak acceleration | Introduces new trajectory shape to re-tune |
| Phase correction (sensor feedback) | Adapts to terrain | Adds control loop complexity |
| Velocity-based step length | Natural speed control | Requires velocity estimate (IMU integration drift) |
| Foot FSR / contact detection | Removes lift inhibit heuristic | New hardware dependency |
| CPG oscillator | Biologically-motivated coordination | Reduces debuggability; skip unless Phase 4 fails |

Do not pursue Phase 5 items if Phase 4 is not stable. Complexity added before a solid foundation creates debt, not capability.

---

## Execution Guide

### Before Writing Any Code

1. Verify `leg_ik.h/cpp` exists in the project. If it does not, IK implementation is Phase 0
   and must precede everything else.
2. Read `project_wide_defs.h` for current loop rate (`IMU_LOOP_HZ`) — this determines `dt`.
3. Read `weight_shift.h/cpp` for current function signatures and MotionManager call sites.
4. Read `robot_geometry.h` for stance width and leg geometry constants — do not hardcode these.
5. Add `gait_types.h` as the first new file (FootTarget + BalanceCorrection structs). All
   other modules include this; nothing else should define these structs.

### New File: `include/gait_types.h`

```cpp
// Module:      gait_types
// Layer:       3.5 (shared type definitions)
// Purpose:     Canonical structs for GaitController ↔ IK ↔ BalanceController boundary
// Inputs:      (none — type definitions only)
// Outputs:     FootTarget, BalanceCorrection
// Dependencies: (none)

#pragma once

struct FootTarget {
    float x_mm;    // forward/back foot position relative to hip
    float y_mm;    // lateral foot position relative to hip
    float h_mm;    // foot height (0 = ground contact)
    bool  valid;   // false → do not command this leg this tick
};

struct BalanceCorrection {
    float dx_mm;       // forward/back body correction
    float dy_mm;       // lateral body correction
    float dPitch_deg;  // pitch trim applied to foot targets
    float dRoll_deg;   // roll trim applied to foot targets
    bool  valid;
};
```

### Constants to Add to `project_wide_defs.h`

Add these in a clearly delimited `// GAIT` block. Do not mix with existing balance or IMU constants.

```cpp
// ============================================================
// GAIT PARAMETERS — GaitController (weight_shift evolution)
// ============================================================
constexpr float GAIT_PHASE_RATE_HZ          = 0.4f;   // start slow; cycles/sec
constexpr float GAIT_STEP_HEIGHT_MM         = 20.0f;  // swing foot clearance
constexpr float GAIT_STANCE_WIDTH_MM        = 40.0f;  // lateral CoM amplitude
constexpr float GAIT_STEP_LENGTH_MM         = 30.0f;  // Phase 3+; forward stride
constexpr float GAIT_WEIGHT_SHIFT_THRESHOLD_RAD = 0.10f; // ~5.7° — min roll before lift
```

All values are starting points for physical tuning. Do not treat them as final.

### main.cpp Integration (Walk Update Path)

The per-tick ordering in `main.cpp` must remain:

```
1. imu.read()
2. stateEstimator.update()
3. weightShift.update()     ← GaitController; injects roll setpoint into BalanceConfig
4. balanceController.update()
5. motionManager.flush()
```

This ordering is a known codebase constraint — WeightShift must run before
`balanceController.update()` so the injected setpoint is used in the same tick.

For Phase 4, the merge step inserts between step 3 and step 4:

```
3b. BalanceCorrection bc = balanceController.getCorrection();
3c. weightShift.applyCorrection(bc);  ← modifies FootTargets before IK
```

Do not restructure the tick ordering beyond this insertion.

### Telemetry Additions (WebSocket)

Consider adding these to the periodic telemetry broadcast for debugging:

```json
{
  "phase": 0.42,
  "leftStance": true,
  "liftL_mm": 0.0,
  "liftR_mm": 14.3,
  "lateralShift_mm": 32.1
}
```

These can be added to `webcomm.cpp`'s existing broadcast function without structural changes.

---

## Considerations & Caveats

### Caveats from Plan Evaluation

**1. WeightShift coupling to BalanceConfig**
WeightShift calls `_bal->setConfig()` every tick to inject `roll_setpoint_rad`. Once
GaitController is running, the balance setpoint must track the instantaneous CoM target, not
a static value. Consider whether this injection should become dynamic (phase-driven) in Phase 2.

**2. SOURCE_BALANCE always wins**
During Phase 1–3, BalanceController (SOURCE_BALANCE, priority 2) will override GaitController
(SOURCE_GAIT, priority 1) on shared joints. This is acceptable for early validation but means
balance corrections may partially cancel gait commands. Do not assume the gait is executing
cleanly until Phase 4's task-space merge is in place.

**3. Sinusoidal trajectory assumptions**
The plan recommends sin-based trajectories. These produce non-zero velocity at the start and
end of each step (the derivative of sin at 0 and π is not zero for the full arc). For Phase 1
this is fine. For Phase 3 and beyond, consider smoothstep or raised cosine for the vertical
trajectory to produce zero-velocity foot touchdown. Flag this if jerk at landing is observed.

**4. Phase wrap at 1.0 → 0.0**
The modulo wrap creates a discontinuity if any downstream consumer integrates phase over time.
Prefer wrapping early (before trajectory computation) and confirm that no function receives
a phase value near 1.0 and 0.0 in adjacent ticks without the wrap being handled.

**5. IMU noise during single-leg stance**
The lift inhibit (Phase 2) uses `imuState.rollRad` as a proxy for CoM position. IMU roll
integrates foot load, body lean, and servo compliance noise. If the threshold is too tight,
the foot will never lift. If too loose, the robot lifts too early and falls. Physical tuning
is unavoidable here; treat `GAIT_WEIGHT_SHIFT_THRESHOLD_RAD` as the primary Phase 2 tuning
knob.

### Design Trade-offs

| Decision | Chosen Direction | Alternative | Reason |
|----------|-----------------|-------------|--------|
| Extend WeightShift vs new GaitController file | Extend WeightShift | New file | Avoids duplicating MotionManager wiring |
| Phase-based vs waypoint trajectories | Phase-based | Waypoints | Waypoints have no coordination primitive |
| Task-space balance (Phase 4) vs joint-space | Task-space (deferred) | Keep joint-space | Joint-space breaks IK consistency at scale |
| CPG oscillator | Defer to Phase 5+ | Implement now | Reduces debuggability during early gait |
| Foot FSR sensors | Defer | Implement now | Removes hardware dependency; heuristic is sufficient for Phase 1–3 |

### Performance Constraints

- **Loop budget:** All gait computation runs in the IMU loop tick. IK must be fast and
  deterministic. Profile if loop overruns appear (`IMU_LOOP_HZ` slip). Do not use
  `Serial.println()` in the hot path during final gait testing.
- **Float math on ESP32-S3:** The S3 has a hardware FPU. `sinf()`, `cosf()` are acceptable.
  `atan2f()` inside IK is fine. Avoid `sqrt()` if a magnitude check suffices.
- **Memory:** Adding GaitController state (phase, trajectories, FootTarget buffers) is a small
  heap increment. Not a concern unless LittleFS flash usage is already near capacity.

### When to Stop and Reassess

- If the robot falls within the first 10ms of Phase 1 (stepping in place): the issue is
  MotionManager priority conflict, not gait math. Debug with `motionManager.flush()` logging.
- If Phase 2 never achieves single-leg balance: the mechanical compliance or servo torque is
  the bottleneck. No amount of software tuning compensates for hardware limits.
- If Phase 3 produces consistent forward motion but BalanceController is saturating: do not
  move to Phase 4 immediately. First confirm Phase 3 is stable without balance active, then
  re-enable balance.

---

## Anti-Patterns to Avoid (Codebase-Specific)

| Anti-Pattern | Why It Fails Here |
|---|---|
| Hardcoding `dt = 0.01` | Will silently break if `IMU_LOOP_HZ` changes — read from `project_wide_defs.h` |
| Reintroducing joint overrides in GaitController | SOURCE_GAIT (priority 1) will be silently dropped by SOURCE_BALANCE (priority 2) on shared joints |
| Lifting foot before weight shift | Validated by `leftMayLift` flag — do not remove this check for "smoother" behavior |
| Calling `oe_estop()` from GaitController | E-stop is BalanceController's domain; GaitController should pause phase, not trigger estop |
| Defining FootTarget in weight_shift.h | Circular include risk — keep in `gait_types.h` |
| Adding CPG before Phase 3 is stable | CPG self-oscillates; debugging requires knowing ground truth trajectory first |
| Tuning phaseRate and stepHeight simultaneously | Tune stepHeight to zero-crossing first (verify lift/land), then increase phaseRate |

---

## Roadmap Milestone Tracking

| Milestone | Condition | Gate |
|-----------|-----------|------|
| **M1** Phase system live | Serial log shows `phase` ticking 0→1 continuously | Verify wrap is clean |
| **M2** Symmetric stepping in place | Left/right lift symmetric; no joint spike on phase boundary | Log `liftL_mm`, `liftR_mm` |
| **M3** Single-leg stance ≥ 0.5s | Robot holds stance leg without ankle saturation | Log `roll_rad` vs threshold |
| **M4** 2+ forward steps | Robot steps forward twice before falling | Video record; note fall direction |
| **M5** 5 consistent forward steps | Repeatable; no emergency balance saturation | Balance log: is `SOURCE_BALANCE` fighting gait? |
| **M6** Task-space balance integrated | Same 5-step test with Phase 4 corrections active; oscillation ≤ Phase 3 | Confirm e-stop still works |

Do not declare a milestone complete until the physical test passes — simulation or serial-only
validation is not sufficient.

---

*Document generated from gait plan + codebase architecture analysis.*
*Source of truth for all specific values: `project_wide_defs.h`, `robot_geometry.h`, `joint_config.h`.*
*This document is a guide — developer judgment supersedes any recommendation here when hardware behavior conflicts.*
