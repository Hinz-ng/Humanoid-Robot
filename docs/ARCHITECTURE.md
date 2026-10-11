# Architecture

**Owner:** Hinz  ·  **Last revised:** 2026-07-07 (V1-closure audit B7 sync)
**Scope:** Layer map, data flow, per-tick ordering, arbitration rules.
**Out of scope:** Specific values (read `project_wide_defs.h`) and live milestone status.

> If this document and the code disagree, the code wins. Open an issue or fix
> the doc. Module roles below name files, not implementations — for current
> behaviour, read the source.

---

## 1. Layer Stack

```
L1  imu.h/cpp                 raw BMI160 read           → RawIMUData (torso)
L1  lsm6dsv_driver.h/cpp      raw LSM6DSV read (×2)     → RawIMUData (per foot)

L2  state_estimator.h/cpp     complementary filter      → IMUState (×3 instances)
                                                          (pitch, roll, rates,
                                                           CoM velocity)

L3  balance_controller.h/cpp  PD + boost + clamp        → BalanceCorrection (task-space
                                                          dx/dy) — leg authority is
                                                          task-space ONLY (submits no
                                                          leg joints; the old ankle-roll
                                                          bias submission was retired in
                                                          the consolidation refactor)
L3  torso_stabilizer.h/cpp    joint-space torso PD      → torso pitch/roll joint cmds
                                                          (own PD on torso IMUState;
                                                           independent of balance)

L3.5 weight_shift.h/cpp       CoM-shift author          → FootTarget x/y offsets;
                                                          ankle-roll CoM-shift tilt is
                                                          READ by GaitController's single
                                                          composition site (_ankleRollDeg);
                                                          also submits torso roll at
                                                          SOURCE_GAIT (torso_shift_deg,
                                                          live at −20° default — A-11)

L3.5 foot_contact.h/cpp       pure predicates           → isFootFlat, isFootLoaded
                                                          (F3 SHIPPED: isFootFlat of the
                                                           stance foot is the DEFAULT
                                                           lift gate — LiftGateMode::
                                                           STANCE_FLAT; snapshot built
                                                           per tick in main.cpp)

L3.5 foot_impact.h/cpp        stateful impact detector  → per-foot touchdown telemetry
                                                          (telemetry only at L0)

L3.5 gait_controller.h/cpp    phase-driven stepping FSM → FootTarget per leg per tick
                                                          + recovery sub-FSM

L3.5 steering_controller.h/cpp hip-yaw turning law      → ch4/ch7 yaw at SOURCE_GAIT
                                                          (Stage 1; default inert)

L3.5 leg_ik.h/cpp             closed-form 2-link IK     → joint angles

L3.5 motion_manager.h/cpp     joint authority arbiter   → resolved per-joint command

L4  servo_control.h/cpp       motion coordinator
L4  joint_model.h/cpp         joint math + smooth-step
L4  joint_config.{h,cpp}      per-joint calibration
L4  servo_driver.h/cpp        PCA9685 pulse writer

L5  oe_control.h / main.cpp   OE pin / boot hold / e-stop gate
L6  webcomm.h/cpp             WebSocket + telemetry + command parsing
```

**Vocabulary file (no logic):**
- `gait_types.h` — `FootTarget`, `BalanceCorrection`, `mergeBalanceCorrection()`.

---

## 2. Module Roles (One Line Each)

| Module | Role |
|---|---|
| `imu` | BMI160 I²C burst read → `RawIMUData`. |
| `lsm6dsv_driver` | One LSM6DSV (foot) over I²C; `begin()` configures ±2g/±2000 dps/480Hz HP. |
| `state_estimator` | Complementary filter + still-window calibration + IIR accel LPF + accel sanity weight. |
| `balance_controller` | Pitch + roll PD with deadband, nonlinear gain boost, IIR + delta-clamp shaping, Smith predictor lag compensation. Outputs task-space `BalanceCorrection.dx/dy` only — it submits NO joints (torso is TorsoStabilizer's; ankle roll is composed by GaitController at SOURCE_GAIT). Owns fall detection and Stage 0 catch-step *detection*. The `ankle_roll_bias_*` / `swing_leveler_roll_bias_*` config fields and `_shapeOutput` are vestigial post-consolidation (audit A-24) — do not re-wire them. |
| `torso_stabilizer` | Independent joint-space torso pitch/roll PD. Reads the torso IMUState directly, runs its own PD + output shaping, and submits torso joints at `SOURCE_BALANCE`. No coupling to BalanceController. |
| `weight_shift` | Authors CoM lateral & forward offsets as task-space contributions to `FootTarget`. Ankle-roll CoM-shift tilt (`ankle_shift_deg`) is read by GaitController's composition site each tick. Submits torso roll (`torso_shift_deg`, shipped −20°, live) at SOURCE_GAIT during shifts. Per-ankle phased ramp; swing-leg mask published by gait. |
| `foot_contact` | Pure stateless predicates over foot `IMUState`. **F3 shipped and default-on:** `main.cpp` builds a `FootContactSnapshot` per tick and `GaitConfig::liftGateMode = STANCE_FLAT` makes the stance foot's `isFootFlat` a hard lift gate. NOTE: the control-path snapshot uses ZERO neutrals (untared) — a mounting offset beyond the flat band permanently closes the gate; fail-safe is the `waitShiftMaxMs` abort to IDLE. |
| `gait_controller` | Modes: IDLE / RUNNING / SINGLE_STEP / WSHIFT_ONLY / POSE_STEP / RECOVERY_STEP. Half-cycle FSM: WAIT_SHIFT → SWINGING → STABILIZING → STOPPING. Owns capture-point step placement and catch-step *execution*. Pushes (active, swingSide, halfPhase) to `steering_controller`. |
| `steering_controller` | Stage-1 open-loop hip-yaw turning. Slews a `turnCmd` setpoint and submits an anatomical yaw to BOTH hip-yaw joints (ch4/ch7) at `SOURCE_GAIT` only while gait is RUNNING. `halfSign·(s−0.5)` law (zero yaw-rate at touchdown). Default inert (`enabled=false` → no submissions). Cross-task config via targeted atomic setters. |
| `leg_ik` | Sagittal: 2-link law of cosines + flat-foot ankle. Frontal: rigid-link `atan2`. Always submits identical anatomical angles per leg — JointConfig `direction` handles left/right mirroring. |
| `motion_manager` | Per-tick slot table; `submit()` keeps highest-priority command; `flush()` dispatches and clears. Drains all slots during e-stop. |
| `servo_control`/`joint_model`/`servo_driver` | Joint-angle ↔ pulse conversion, per-joint speed-limited smooth stepping, PCA9685 writes. |
| `oe_control` | Active-low OE pin, boot hold, software e-stop. Single safety gate; only `BalanceController` (fall detect) and user `CMD:ESTOP` trigger it. |
| `webcomm` | Async WebSocket server. String protocol (`TYPE:payload`). All UI sliders & commands enter through here. |

---

## 3. Per-Tick Ordering (main.cpp)

The 400 Hz gate runs the following sequence. **Reordering requires a design
review and an update to this file before the diff lands.**

```
0.  webComm.applyStagedTuning()     → applies WiFi-task staged config updates,
                                      then drains the staged command queue
                                      (audit B1/B1.2 — MUST stay first: it
                                      precedes every config consumer)

1.  imu.update()                    → RawIMUData (torso)
2.  stateEstimator.update(raw, dt)  → IMUState (torso)
3.  leftFootIMU.read() / remap()    → RawIMUData (left foot)
4.  rightFootIMU.read() / remap()   → RawIMUData (right foot)
5.  footImpactDetector.update(...)  → per-foot impact snapshot
6.  leftFootEstimator.update(...)   → IMUState (left foot)
7.  rightFootEstimator.update(...)  → IMUState (right foot)
7a. foot boot-neutral publish       → once per foot, on the rising edge of
                                      CALIB_DONE: neutral pitch/roll →
                                      webComm.setFootNeutral (telemetry/tare)
7b. FootContactSnapshot build       → isFootFlat/isFootLoaded per foot from
                                      this tick's IMUStates (ZERO neutrals on
                                      the control path); same snapshot feeds
                                      the F3 lift gate AND the UI telemetry

8.  weightShift.update(dt)          ← reads swing-leg mask published by
                                      GaitController on the previous tick
                                      (1-tick stale; deliberate)
                                    → submits torso roll at SOURCE_GAIT when
                                      |torso_shift_deg| > 0 and shifting

9.  balState = balanceController.update(state)
        → fills BalanceController._lastCorrection (dx/dy) + PD + telemetry
        → submits NOTHING (task-space only, post-consolidation)
10. torsoStabilizer.update(state)
        → runs its own pitch/roll PD on the torso IMUState (independent of
          balance); submits torso pitch/roll at SOURCE_BALANCE (joint-space)
11. bc = balanceController.getLastCorrection()
12. gaitController.update(dt, state, &bc, &balState, footSnapPtr)
        → merges bc into stance FootTargets before LegIK
        → submits leg joint angles at SOURCE_GAIT (single ankle-roll
          composition site _ankleRollDeg)
        → pushes (active, swingSide, halfPhase) to SteeringController

12a. steeringController.setYawRates(torso, lf, rf)
        → Stage 1.1 characterization inputs (gated on estimator validity);
          must precede steeringController.update()
13. steeringController.update(dt)   → hip-yaw turning law (Stage 1)
        → submits the two hip-yaw joints (ch4/ch7) at SOURCE_GAIT
          (uncontested slot); default inert (enabled=false → no submissions)

14. motionManager.flush()           → resolve per-joint winner, dispatch
14a. walkingInstr.update(...)       → per-step diagnostics aggregator; pure
                                      reader, submits nothing (runs AFTER
                                      flush by design — reads this tick's
                                      final state)
15. webComm.broadcast*(...)         → IMU, foot IMU, ESTIMATE, FOOT_CONTACT,
                                      FOOT_IMPACT, WALKING, BALANCE, TORSO,
                                      GAIT, STEER, CALIB (rate-limited)
```

Outside the 400 Hz gate (every loop iteration):
- `oe_loop()` (boot hold release)
- `servoController.update()` (smooth stepper, internal 50 Hz limit)
- Periodic `STATE:` broadcast at 10 Hz

---

## 4. Joint Authority Arbitration

`MotionManager` holds one slot per channel per tick. `submit(src, ch, deg)`:

- Empty slot → always accepted.
- Incoming priority ≥ existing → overwrites.
- Lower priority → silently dropped.

Priorities:

| Source | Value | Frame | Method called by `flush()` |
|---|---|---|---|
| `SOURCE_UI` | 0 | absolute servo deg (0–270) | `ServoControl::setTargetAngle` (smooth-stepped) |
| `SOURCE_GAIT` | 1 | joint-relative deg | `ServoControl::setJointAngleSmooth` |
| `SOURCE_BALANCE` | 2 | joint-relative deg | `ServoControl::setJointAngleDirect` (immediate) |

**During E-stop, `flush()` drains and discards all slots.** Commands queued
while estopped are contextually stale and never reach hardware after OE re-enables.

**Leg vs torso authority:** BalanceController submits *nothing* — it returns
a `BalanceCorrection` that GaitController folds into the FootTarget before
IK. `SOURCE_BALANCE` carries exactly two channels: `IDX_TORSO_PITCH` and
`IDX_TORSO_ROLL`, both from `TorsoStabilizer` (audit-verified — nothing else
submits at SOURCE_BALANCE; in particular no ANKLE_PITCH, which keeps the
post-contact ankle-release design gate open). `IDX_TORSO_ROLL` has a second,
lower-priority writer: WeightShift's `torso_shift_deg` term at SOURCE_GAIT —
live at the shipped −20° whenever a shift is in progress, and the ONLY
writer when torso balance is disarmed (the default).

---

## 5. Data Flow Around the Foot IMUs

Phase F1 wiring is complete. **F3 is shipped and default-on** (audit A-07
corrected this section): the per-tick `FootContactSnapshot` drives both the
UI telemetry and the gait lift gate (`LiftGateMode::STANCE_FLAT` default).

```
LSM6DSV (L)  ─┐
              ├─►  Wire1 (SDA=11, SCL=12 @ 200 kHz)  ─► lsm6dsv_driver.read()
LSM6DSV (R)  ─┘                                         │
                                                        ▼ RawIMUData (per foot)
                                          _remapFootRawToRobotFrame() in main.cpp
                                          (NOT identity — a +90° Z-rotation:
                                           robot = (−chip_y, +chip_x, +chip_z).
                                           Audit A-12 fixed this line; the code
                                           is the source of truth — edit the
                                           function if mounting changes)
                                                        │
                                                        ▼
                                          {left,right}FootEstimator.update()
                                          (own FilterConfig, own gyro bias calibration)
                                                        │
                                                        ▼ IMUState (per foot)
                              ┌─────────────────────────┴───────────────────┐
                              ▼                                              ▼
                isFootFlat / isFootLoaded                    broadcastFoot{IMU,Estimate,Contact}
                (FootContactSnapshot in main.cpp —          (rate-limited telemetry to UI)
                 *stance-foot isFootFlat IS the              + FOOT_BUDGET (loop-time
                 default lift gate — F3 shipped*)             instrumentation)
```

**Bus topology rationale:** I²C on a dedicated bus (Wire1), not SPI as
earlier plans assumed. Failing-loudly (NACK on bad address) was the deciding
factor over SPI's silent corruption mode during F0/F1 bring-up. See history
block in `lsm6dsv_driver.h`.

---

## 6. Filter Architecture

`StateEstimator` is instantiated **three** times: torso (BMI160), left foot
(LSM6DSV), right foot (LSM6DSV). Each has its own `FilterConfig` and runs
its own boot-time gyro-bias calibration (waits for stillness, accumulates
samples, then goes live).

Per-instance configuration (`FOOT_IMU_*` constants in `project_wide_defs.h`):
- `accel_scale` / `gyro_scale` — chip-specific (BMI160 vs LSM6DSV differ).
- `accel_lpf_beta` — foot environment is more vibration-prone than torso.
- `accel_sanity_margin` — foot transients (footstrike) are larger.
- `alpha` (complementary filter blend) — tuned per environment.
- `pitch_gyro_sign` / `roll_gyro_sign` — defaults `+1`; flip per instance
  only if a sign-verification test against torso fails.

**CoM velocity estimate** (`IMUState::vx_CoM_mm_s`, `vy_CoM_mm_s`) uses an
inverted-pendulum approximation: `v = H_COM_MM × angularRate`. Valid during
rotation about the stance ankle. Degrades during double support and
footstrike — acceptable for first-pass capture-point control.

---

## 7. Control Sub-Architectures

### 7.1 Balance Controller

```
IMUState → derivative LPF → Smith predictor (servo lag tau)
         → deadband → nonlinear gain boost (effective_kp)
         → PD: u = sign × (Kp_eff × err + Kd × rateFiltered)
         → clamp ± max_correction_deg

  → convert clamped u → dx_mm / dy_mm via pitch/roll_to_dx/dy_per_deg
  → IIR + outer ±25 mm clamp
  → emit as BalanceCorrection (consumed by GaitController)
```

The clamped PD output is cached only for `BalanceState` telemetry (`p_u`).
Torso pitch/roll are owned entirely by the independent `TorsoStabilizer`
(see §7.4) — BalanceController no longer exposes its PD to it. Post-
consolidation, BalanceController submits NO joints at all; ankle roll is
composed by GaitController at SOURCE_GAIT (audit A-24: the leftover
`ankle_roll_bias_*` fields/setters are vestigial).

`BalanceController` also runs **Stage 0 catch-step detection**: persistent
saturation OR persistent tilt over a debounce window sets
`BalanceState::recovery_flag`. Detection is gated by `recovery_enabled` and
suppressed during gait if `recovery_suppress_during_gait` is set.

### 7.2 Gait Controller

Half-cycle FSM (per leg):

```
WAIT_SHIFT  → wait until |wsProgress| ≥ liftGateThreshold
              (RUNNING/SINGLE_STEP also require IMU roll gate if useImuGate)
SWINGING    → advance _halfPhase ∈ [0, 1); foot follows sin(localPhase × π)
              vertical + linear stride x (with optional capture offset)
STABILIZING → hold pose for GAIT_STABILIZE_MS between halves
STOPPING    → ramp swing fraction to zero over GAIT_STOP_RAMP_MS, then IDLE
```

Stage 3 — both legs receive cycle-role-aware FootTargets each tick
(`_submitCycleStanceBoth`). Stride applies to **both** legs:
swing leg starts at −L/2, lands at +L/2; stance leg retreats from +L/2 to −L/2.

**Capture-point** (sagittal only, Stage 2.2 — disabled by default):
swing leg's stride x is augmented by
`clamp(vx_CoM × INV_OMEGA0 × captureGain, ±captureOffsetMaxMm, ±stepLengthMm)`.

**Catch-step recovery** is a separate top-level mode (`RECOVERY_STEP`) with
its own sub-FSM (TRIGGERED → PRE_LIFT → LIFTING → LANDING → DAMP_HOLD →
SOFT_RETURN → COMPLETE). 2D directional; magnitude scaled from disturbance
tilt; chain detection triggers a second step if residual tilt is significant
at COMPLETE; refractory lockout otherwise.

**Post-contact rollover** (telemetry only): `_updateRolloverTelemetry` opens an
observation window at the touchdown latch and streams the just-landed foot's
roll-to-flat behaviour on the `GAIT:` packet. Submits nothing. The **post-contact
ankle YIELD** (additive dorsiflexion bias on the just-planted leg, post-IK in
`_submitFootTargetIK`, armed only at the touchdown latch, no fallback) **is
implemented AND ships active** — enabled=true with `yieldDorsiflexDeg=2.0`
(audit A-07/A-09 corrected this paragraph; the old "specified but not yet
wired" status was stale, and the shipped-active default is pending a human
decision). It is a SOURCE_GAIT-only term on ankle pitch; the prerequisite —
nothing submits `ANKLE_PITCH` at SOURCE_BALANCE — was audit-verified (Phase A
headline table).

### 7.3 WeightShift

Authoring module — does NOT own gait phase. Each tick:

1. Per-ankle progress ramps (with phase delay between swing and stance) toward
   `±1` set by `trigger()`.
2. The ankle-roll CoM-shift tilt is no longer pushed into BalanceConfig —
   GaitController's `_comShiftTilt()`/`_ankleRollDeg()` read
   `WeightShiftConfig::ankle_shift_deg` and the per-ankle progress each tick
   and compose the tilt at SOURCE_GAIT (single composition site).
3. Emits task-space stance shift (`forward_lean_mm`, `lateral_shift_mm`)
   via `getStanceShift()` — consumed by `GaitController::_buildFootTarget`.
4. Submits torso roll (`progress × torso_shift_deg`) at SOURCE_GAIT whenever
   a shift is active — LIVE at the shipped −20° (audit A-11; direction
   unverified, pending human confirmation).

The swing-leg mask is published *to* WeightShift by GaitController each
tick via `setSwingLegMask()`.

### 7.4 Torso Stabilizer

Fully independent joint-space torso pitch/roll PD (the torso joints are not in
the IK chain). It shares **no** state with `BalanceController`.

`TorsoStabilizer::update(state)` reads the torso `IMUState` directly and runs its
own per-axis PD (`TorsoConfig`: `Kp`/`Kd`, setpoint, deadband, derivative-LPF,
`correction_sign`, max-correction clamp) — **lean PD, no gain-boost and no
servo-lag predictor**. PID was rejected: servo backlash + an upright setpoint
with no persistent external torque means an integral term only winds up and
limit-cycles. The clamped PD output runs through an independent output-shaping
pipeline (zero-bypass → delta clamp → IIR → velocity damp → final clamp) and is
submitted to `IDX_TORSO_PITCH` / `IDX_TORSO_ROLL` at `SOURCE_BALANCE`. Each axis
is gated by its own `pitch_enabled` / `roll_enabled`, so torso balance runs
regardless of whether leg balance or gait are active. Tuned via `TORSO_TUNE`;
telemetry on the `TORSO:` packet.

(The legacy joint-space *leg* balance path and the `task_space_output` runtime
fork were removed in the consolidation refactor — task-space is now the only
leg-balance architecture.)

---

## 8. Calibration / Persistence

| Item | Where | Lifetime |
|---|---|---|
| Joint calibration (neutral, direction, limits) | `joint_config.cpp` | Source-controlled |
| IMU bias (torso + both feet) | Each `StateEstimator` boot calibration | Per-boot, not persisted |
| Balance / gait / weight-shift / recovery / foot-contact tuning | Live config structs | Per-boot. Tune at runtime via WebSocket; reflash to persist |
| Saved poses | LittleFS | Persisted |

If a foot's neutral pitch under load is captured (planned F4), it will be
written via `WebComm::setFootNeutralPitch` and consumed by `isFootLoaded`.

---

## 9. What This Document Does *Not* Specify

- Specific loop rates, thresholds, gains, or pin numbers — read the code.
- Gait FSM transition details and capture-point math derivations — see
  `GAIT_DESIGN.md` and the comment blocks in `gait_controller.cpp`.
- The complete WebSocket packet inventory — grep `WebComm.cpp` and `index.html`.

---

*End of ARCHITECTURE.md*
