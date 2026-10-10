# Current Focus

**Owner:** Hinz · **Updated:** 2026-06-06
**Place this file in:** project knowledge **and** Claude Code repo root.
**Purpose:** Live state of the project — what's in progress, what's gated,
what's open. Update *when* state changes, not on a schedule.

> For invariants and conventions, see `CODING_STANDARD.md`.
> For module map and per-tick ordering, see `ARCHITECTURE.md`.
> For gait FSM, capture-point math, and recovery details, see `GAIT_DESIGN.md`.

---

## 1. Active Workstreams (this month)

### A. Capture-point walking improvement
**Status:** implemented (Stage 2.2), **disabled by default**, awaiting hardware validation.

- Sagittal-only capture-point step placement in `GaitController::_captureOffsetMm`.
- LIP model: `offset = clamp(vx_CoM × INV_OMEGA0 × captureGain, ±captureOffsetMaxMm, ±stepLengthMm)`.
- CoM velocity from inverted-pendulum approximation: `IMUState::vx_CoM_mm_s = H_COM_MM × pitchRate`.
- All defaults in `project_wide_defs.h` (`GAIT_CAPTURE_*`).
- Runtime toggle: UI "Capture" button (`CMD:GAIT_TUNE:captureEnabled=1`).

**Next:** smooth-floor validation at low `phaseRate` and short `stepLength`,
then step-by-step lift of `captureGain` toward 1.0.

**Reference plan:** `Walking_Improvement_Plan.md` (Tier 2).
**Open question (live):** does this robot routinely produce `|vx_CoM| > 0.05 m/s`
at walking cadence? If not, capture offset will stay small and the feature
mostly correct for disturbances rather than drive forward speed.

---

### B. Foot IMU integration — Phase F2

**Status:** predicates implemented (`foot_contact.h/cpp`), wired to telemetry
**only**. Not yet consumed by any controller.

What's done:
- Both LSM6DSV chips on Wire1 (I²C @ 200 kHz, addresses 0x6A/0x6B via hardwired SA0).
- Per-foot `StateEstimator` instance with foot-specific `FilterConfig`.
- Raw frame remap is a +90° Z-rotation — robot = (−chip_y, +chip_x, +chip_z),
  NOT identity (audit A-12 corrected this line; the code in
  `_remapFootRawToRobotFrame` in `main.cpp` is the source of truth).
- Pure predicates `isFootFlat()` / `isFootLoaded()`.
- WebSocket telemetry: `FOOT_IMU`, `FOOT_ESTIMATE`, `FOOT_CONTACT`, `FOOT_BUDGET`.
- WebSocket tuning: `CMD:FOOT_CONTACT_TUNE` for thresholds and load sign.
- Loop-budget instrumentation (5 s window) — proves no tick budget regression.

**F3 STATUS CORRECTION (audit A-07, 2026-07):** F3 is **implemented AND
default-on** — `main.cpp` builds a `FootContactSnapshot` per tick and the
shipped `GaitConfig::liftGateMode = LiftGateMode::STANCE_FLAT` makes the
stance foot's `isFootFlat` a hard lift gate. The old "telemetry only / do
not start F3" text below every doc was stale. Facts to know:
- The control-path predicate uses **zero neutrals** (untared); the tared
  version is telemetry preview only. A mounting offset beyond the flat band
  (0.035/0.05 rad) would permanently close the gate → the `waitShiftMaxMs`
  (1500 ms) timeout aborts to IDLE cleanly (fail-safe, verified in audit).
- A dead/uncalibrated foot IMU fails the same safe way (snapshot invalid →
  gate closed → timeout abort).

What remains:
- **Retro-validation** — the 60 s stepping-run predicate-stability gate was
  never formally run even though the gate shipped; run it to confirm.
- **F4** — voltage-drift / true-neutral calibration tool. WebSocket command
  that captures unloaded foot pitch per leg and feeds it as
  `setFootNeutralPitch(...)` so `isFootLoaded` (and eventually the zero-
  neutral control-path gate) uses session-correct deltas.

**Reference plan:** `FOOT_IMU_PHASE2_PLAN.md`.

---

### C. Sagittal recovery hardening (prerequisite for forward walking)

**Status:** Stage 2 implemented (2D directional, magnitude-scaled, chain detection).
Lateral catch verified anecdotally; sagittal historically weaker.

What's done:
- 2D `triggerRecovery(pitch, roll)` with magnitude scaling
  (`RECOVERY_MAGNITUDE_GAIN_MM_PER_RAD = 150.0f`).
- Sub-FSM: PRE_LIFT → LIFTING → LANDING → DAMP_HOLD → SOFT_RETURN → COMPLETE.
- Chain detection at COMPLETE — re-triggers if residual tilt exceeds
  `RECOVERY_CHAIN_TILT_THRESHOLD_RAD`.
- Refractory lockout (`RECOVERY_LOCKOUT_MS`) after a clean step.
- Recovery flag suppressed during RUNNING gait (`recovery_suppress_during_gait`).
- Manual test buttons: `GAIT_TEST_RECOVERY_F/B/L/R` (no detection threshold).

**Open gap:** the published prerequisite for forward walking is "sagittal
catch at ±0.20 rad lands the foot *ahead* of the falling CoM at least 4/5
attempts." Validate this empirically before raising `captureGain` or
extending `stepLengthMm`.

**Reference plan:** `FORWARD_WALKING_PLAN.md` (Stage 0).

---

### D. Support-transfer dwell + timeout
**Status:** implemented, awaiting hardware validation.

- `WAIT_SHIFT` now requires a continuous `liftGateOK` hold for `liftDwellMs`
  before entering `SWINGING`.
- `waitShiftMaxMs` aborts a stalled support transfer via `stop()` so the gait
  ramps cleanly back to IDLE instead of waiting indefinitely.
- Telemetry now exposes per-episode `waitShiftElapsedMs`, `gateDwellMs`, and a
  one-tick `waitShiftTimedOut` edge, plus config echoes for reconnect sync.

**Next:** run the regression/dwell/timeout validation sequence before marking
this workstream complete or locking in nonzero dwell defaults.

---

### E. Lateral CoM observation gate (Stage 3)
**Status:** implemented, awaiting hardware validation. Default `liftGateMode`
remains `STANCE_FLAT` until `COM_LATERAL` is validated.

- `LiftGateMode::COM_LATERAL` gates on neutral-corrected observed lateral CoM
  displacement from torso IMU roll.
- `CMD:TORSO_TARE` captures the current torso pitch/roll as gait-local
  neutrals, and `GAIT_TUNE` can override the neutrals and threshold manually.
- `GAIT:` telemetry now exposes `comLatMm`, `gateComLat`, the neutral echoes,
  and the configured lateral threshold for tuning against observed behavior.

**Next:** run the tare, sign/magnitude, low-threshold wiring, threshold-sweep,
and deliberate-mistune validation sequence before promoting `COM_LATERAL`.

---

### F. Touchdown-impulse swing shaping (W1 / W2 / W4)
**Status:** all three implemented, default ON, awaiting hardware validation.

Workstream targeting touchdown / support-transfer impulse — see
`GAIT_DESIGN.md` §3.6–3.8 for the design detail.

- **W1 — asymmetric swing apex.** `_swingFrac()` time-warps `localPhase` so
  the vertical lift apex lands at `swingApexFrac`; below 0.5 = slower descent
  into touchdown. Validated, meaningful improvement.
- **W2 — continuous lateral CoM trim.** `_comLateralTrimMm()` adds observed
  lateral CoM displacement to both feet's `y_mm` every tick. Validated,
  meaningful improvement.
- **W4 — touchdown compliance.** The early-contact / swing-truncation event
  also fires a brief stance-height yield (`−complianceDepthMm` on both legs'
  `h_sagittal_mm`, smoothstep decay over `complianceDecayMs`). Transient and
  self-clearing — no persistent posture bias, no force loop, no new sensing.
  Defaults in `project_wide_defs.h` (`GAIT_TD_COMPLIANCE_*`). Runtime knobs:
  UI "Compliance" toggle + depth/decay sliders (`CMD:COMPLIANCE_TUNE`).
  Telemetry: `GAIT:` fields `cmpOff` / `cmpAct`.

**W3 (foot-loaded-gated compliance) was dropped** — `isFootLoaded` is
unreliable and unused; W4 reuses the already-validated early-contact event
instead.

- **Horizontal stride front-load (analog of W1 in x).** `_warpedStrideFrac()`
  compresses the SWING leg's horizontal stride smoothstep into
  `[0, strideCompleteFrac]` then holds at +L/2, so the foot reaches its
  landing position with near-zero horizontal velocity before touchdown.
  Stance retreat is **not** warped (timing left alone to keep the experiment
  isolated from a separate "shift CoM forward earlier" change). Default
  `strideCompleteFrac = 1.0` is bit-identical to the legacy stride. New
  telemetry `touchdownSwingVelX_mm_s` reports the swing foot's horizontal
  velocity at the touchdown-latch fire moment. Tuned via WS key
  `strideComplete`. Awaiting hardware validation — sweep `1.0 → 0.7 → 0.6
  → 0.5` at 0.6 Hz / 30 mm / 10 mm and watch `tdSwingVx` fall and mode-B
  (rock/rebound) events drop.

**Next:** A/B run with W4 toggled off vs on at baseline settings (0.55 Hz,
35 mm stride); confirm touchdown-pitch transient drops without new oscillation,
then tune `complianceDepthMm` upward from the conservative 4 mm default.

---

### H. Inter-step hold sweep + lateral-blend probe

**Status:** implemented, awaiting hardware validation.

Two `GaitConfig` fields added to diagnose mode-B (backward rebound) and test
foot-to-foot lateral CoM blending without a full architecture change:

- **`stabilizeMs`** (default 400 = `GAIT_STABILIZE_MS`, WS: `stabilizeMs`):
  Promotes the compile-time STABILIZING hold to runtime. Sweep down from
  400 → 250 → 150 → 80 → 40 → 0. At 0 the sub-state collapses to ~1 tick;
  floor risk is prior-half IIR / mechanical transient superposition
  (empirical knee ~80–150 ms expected).
- **`skipStabilizeNone`** (default false, WS: `skipNone`): Step-4 cheap probe.
  Skips `trigger(NONE)` at `_advanceHalf()` and immediately arms the next
  swing direction. CoM target flips foot-to-foot without returning to center.
  Pair with low `stabilizeMs`. Revert OFF if lateral oscillation appears.

Both are bit-identical to prior behavior at default values.

**Sweep plan:** fix `phaseRate=0.65`, `liftGate=0.7`; step `stabilizeMs`
400→250→150→80→40→0; observe mode-B rate at each level. Then test
`skipNone=1` at each level.

**Reference:** `GAIT_DESIGN.md` §2 (STABILIZING row), §8 (touch table).

---

### G. Balance/gait architecture consolidation refactor
**Status:** implemented (Phases 1–3 complete, builds clean), **awaiting
hardware validation** before flashing.

Consolidation only — no capability change; intent is to preserve current
behaviour while collapsing duplicated authority:
- Legacy joint-space leg balance and the `task_space_output` runtime fork
  removed — task-space (`BalanceCorrection` dx/dy) is the only leg-balance path.
- Torso pitch/roll balance extracted into a new joint-space `TorsoStabilizer`
  (`torso_stabilizer.h/cpp`, `TorsoConfig`).
- **Torso decoupling (follow-up refactor):** `TorsoStabilizer` is now a fully
  independent subsystem — its own pitch/roll PD on the torso IMUState (lean PD,
  no gain-boost / no servo-lag predictor; PID rejected for backlash windup),
  own enables/gains/shaping/telemetry, own `TORSO_TUNE` commands and `TORSO:`
  packet, own "Torso Balance" UI panel. `BalancePDOutput` / `getPDOutput()` /
  `_pdValid` and the torso sliders/toggle/telemetry in the Balance panel were
  removed. No shared state with BalanceController remains.
- Dead fields/UI removed: `BalanceConfig` ankle/hip ratios, `BalanceCorrection`
  `dPitch_deg`/`dRoll_deg`, the UI ratio sliders + task-space toggle + formula
  display, the WeightShift roll-setpoint injection.

**Gate (Phase 0 — must clear before flashing):** task-space walking must be
confirmed on par with the pre-refactor legacy default on hardware — static
balance + 60 s in-place stepping, balance PD both enabled and disabled. The
shipped default was legacy; consolidation makes task-space the only path.

**Torso direction gate:** torso PD defaults to disabled with zero gains. Before
raising gains on hardware, confirm `correction_sign` per axis (torso joints have
no end-stops; software limits are safety-critical).

**W2/W5 lateral-trim ↔ roll-PD unification was deliberately deferred** to a
separate refactor — see the reference plan §6.

**Reference plan:** `.claude/plans/review-the-current-balance-keen-stroustrup.md`.

---

### I. Walking instrumentation panel — read-only telemetry (NEW)
**Status:** live (builds clean). Measurement tool for the forward-walking
bottleneck A/B work. Instrumentation only — no gait/control change.

`walking_instrumentation` module → `WALKING:` packet → collapsible
"🔬 Walking Instrumentation" panel. 17 metrics aggregated once per completed
step into a fixed 10-step ring buffer; each shows Current + Average, expand for
the last-10-steps history. Groups: Touchdown (peak accel, impulse proxy [g·s],
swing-speed-at-contact, double-touchdown + interval), Swing Energy (peak 3D /
horizontal / vertical speed, ½·v² energy proxy), CoM / Body Response, Reach
(peak reach %, flat-foot guard activations). Controls: Enable toggle + Reset
history (`CMD:WALKING_INSTR:enable=0|1` / `:reset=1`). Read-only consumer; no
control coupling; `reset()` also runs on e-stop via `oe_clear()`.

**Caveats (read before trusting):**
- **CoM bob / vertical CoM velocity are COMMANDED proxies** (stance hip height
  `max(fhL,fhR)`), not measured — the firmware has no vertical-CoM estimate.
- **Double-touchdown is PROVISIONAL.** `|a|` sits near 1 g both in contact and
  airborne, so two spikes ≠ a bounce. The flag requires a gyro-confirmed
  lift-off (`bounceGyroThreshDps`, sustained `releaseDwellMs`) between contacts.
  Validate against logged accel/gyro traces and tune before relying on it; the
  raw onset count is surfaced for audit.
- For the **swing-leg-momentum hypothesis**, correlate the swing-energy proxy /
  touchdown swing speed against the **measured** body-response peaks (lateral /
  sagittal CoM speed, torso roll rate, torso roll @ touchdown).

---

### J. Sagittal rollover instrumentation + post-contact ankle release (NEW)
**Status (corrected by the V1-closure audit, 2026-07):** instrumentation
**live** (telemetry-only); ankle release (yield) **IMPLEMENTED and SHIPS
ACTIVE** — `GAIT_YIELD_ENABLED_DEFAULT=true` with `yieldDorsiflexDeg=2.0`,
engage 10 ms + release 400 ms. The "specified, not implemented" status below
was stale, and because the yield is live the "control bit-identical" claim
for the instrumentation A/B no longer holds. **PENDING HUMAN DECISION
(audit A-09): bless the active default or restore 0.0.**

Targets the diagnosed sagittal bottleneck: the foot lands toe-first ahead of
the CoM and cannot roll (pinned ankle on non-backdrivable servos), so the
pinned sole scrubs as the body vaults → double-touchdown → forward topple
(Mode A) or backward shove (Mode B). Full causal model + design record:
`POST_CONTACT_ANKLE_RELEASE_CONTEXT.md` and
`SAGITTAL_GAIT_DIAGNOSIS_CONTEXT.md`.

**Rollover instrumentation (implemented).** `GaitController::_updateRolloverTelemetry`
opens a post-contact observation window at the touchdown latch, survives the
`_advanceHalf` role swap, auto-closes after `rolloverWindowMs`. `GAIT:` fields:
`rolloverFootPitchRad` (just-landed foot's neutral-corrected pitch — sweeps to 0
on a clean roll, stays pitched when pinned), `rolloverDurationMs` (time-to-flat;
0 after window close = never flattened), `ankleCmdVsFootPitchDivergenceDeg`
(cross-frame — read the CURVE not the value), `postContactCoMVelSign` (coarse
fwd/back sign; `vx_CoM` degrades at footstrike), `postContactBodyPitchPeak`
(vault signature; most trustworthy "did it help" scalar). See the context doc
§2 for the three-way diagnosis (engaged+helped / engaged+didn't-help /
didn't-engage → mechanical).

**Review flags — CLOSED by the V1-closure audit (Phase B, 2026-07):**
1. ~~index agreement~~ **PROVEN** — `sideIdx(s)` is `static_cast<size_t>(s)`
   by definition (`side.h` static_asserts LEFT=0/RIGHT=1), and B3 added an
   anchoring `static_assert` at the `_rolloverSide` write site.
2. ~~neutral correction~~ **ALREADY IMPLEMENTED** — `pitchCorr = torso.pitch
   − torsoPitchNeutralRad` at accumulation (gait_controller.cpp; audit A-23).
3. Divergence is relative-only (cross-frame); CoM-vel sign is coarse-only.
   (Still true — inherent to the metric, not a defect.)

**Ankle release (IMPLEMENTED — ships active, see status header).** Post-contact
stance-leg dorsiflexion that *relaxes the held-flat ankle command* so the
planted foot can roll. Single gate (touchdown latch only — no eligibility
window, no pre-arm, no `_advanceHalf` fallback ⇒ 1:1 with detected
touchdowns); smoothstep engage→decay; sign negative (dorsiflex) baked in;
additive **post-IK** in `_submitFootTargetIK` (cannot trip the flat-foot
guard); side-locked; survives the role swap. Shipped `yieldDorsiflexDeg=2.0`
(the design's inert-0 default was overridden in tuning). Cadence caveat:
engage+release = 410 ms ≈ a half-cycle at 0.7 Hz — at higher cadence alternate
steps silently get no yield (`yieldSkipCount` counts these).

**Strong prior (not validated):** firmware release is likely **too late** —
servo slew, not trigger timing, is the binding constraint (~40–60 ms
contact-to-dorsiflexion vs a 50–150 ms vault; the existence of
`swingPreloadCompleteFrac` proves the ankle can't hit attitude over hundreds of
ms of swing). The **lateness falsifier** (compare `yieldBiasDeg` vs
`rolloverFootPitchRad`, both timestamped from `_touchdownMs`) is the headline
readout and decides firmware-vs-mechanical. **Recommendation:** prototype a
rockered / compliant-TPU sole in parallel — zero detection latency, zero slew.

**Next:** decide the shipped-active default (A-09 human decision); then run
the deciding A/B (release off vs on, **h-restore held fixed**, **W4 off**,
fixed moderate operating point). Prerequisites already closed by the audit:
flag #1 proven + static_assert anchored (B3); `ANKLE_PITCH` at
`SOURCE_BALANCE` verified impossible (SOURCE_BALANCE carries exactly the two
torso channels).

---

### K. Hip-yaw steering — Stage 1 + 1.1 (NEW)

**Status:** Stage 1 + Stage 1.1 implemented, **ships inert**
(`GAIT_STEER_ENABLED_DEFAULT=false` → `SteeringController::update()` submits
nothing → bit-identical firmware).

**Stage 1.1 (characterization, instrumentation only — law/timing/ownership
unchanged):**
- **Sign baked:** `STEERING_TURN_SIGN = -1` (hardware-verified). Runtime sign
  control removed (UI button, `STEER_TUNE:turnSign`, `setTurnSign()`); the field
  + `yawLawDeg()` parameter stay.
- **Measured yaw rate surfaced:** `IMUState::yawRate` = debiased, **pre-deadband**
  gyro-Z (rad/s). The gz deadband was removed in `_scaleRaw` (gz had no consumer;
  the ~0.57°/s deadband would bias slow-turn integrals low). All three estimators
  (torso + 2 feet) now carry it.
- **Per-step yaw budget** (`SteeringYawBudget`): integrated over each footfall —
  `cmd` (DERIVED, summed submitted joint deltas) vs `body`/`stance`/`swing`
  (MEASURED gyro-Z integrals), pre/post-touchdown split,
  `realizationRatio = body/cmd` (guarded), 6-step rolling means. Latched on the
  swing-side flip. Inputs: `setYawRates()` (main, foot-validity-gated) +
  `touchdownLocked` added to the gait push. `STEER:` packet + UI panel extended;
  live torso/L/R yaw rates shown for the gyro-sign gate.
- Tests added: cmd consistency, mid-half slew exactness, role-swap, post-TD
  partition, disabled-zero. (Also fixed the pre-existing `leg_ik_test.cpp`
  aggregate-init blocker so the `-DUNIT_TEST` build links.)
- **Next (hardware):** STEERING_PLAN §9 — drift baseline (turnCmd=0), gyro-sign
  confirm (`STEERING_YAW_RATE_SIGN`), reproduce the floor observations
  (realizationRatio across phaseRate / stepHeight / stepLength).

**Stage 1 (open-loop turning):**

What's done:
- New L3.5 `steering_controller.h/cpp` (analogue of `WeightShift`): slews a
  `turnCmd` setpoint into `_turnEff` and submits an anatomical hip-yaw command
  to BOTH legs (ch4/ch7) at `SOURCE_GAIT` during `GaitMode::RUNNING`. Law:
  `turnEff·yawStepDeg·turnSign·halfSign·(s−0.5)`, clamped to `±GAIT_STEER_MAX_YAW_DEG`.
- `GaitController` pushes `(active, swingSide, halfPhase)` each tick (one pointer +
  one call, after `_publishSwingLegMaskToWS()`). It never authors yaw itself.
- `main.cpp`: `steeringController.update(dt)` runs between gait and `flush()`.
- WebComm transport split: `CMD:STEER:<turn>` (hot, turnCmd only) and
  `CMD:STEER_TUNE:enable=,yawStep=,turnSign=,slew=` (cold). Both write the one
  `SteeringConfig` via targeted **atomic setters** (cross-task RMW-safe). `STEER:`
  telemetry packet broadcast at 20 Hz.
- Safing: WebComm forces `turnCmd=0` on WebSocket disconnect and on `CMD:ESTOP`;
  the controller also parks the yaw joints whenever estopped / not RUNNING.
- UI: virtual joystick in the gait panel (X→turnCmd, Y ignored; spring-to-center
  on release) + live `STEER:` readout + bench `STEER_TUNE` widgets.
- Unit tests `runSteeringControllerTests()`: disabled-no-op, turn-zero,
  accumulation+halfSign, boundary continuity, slew bound, double-support hold, clamp.

**Next (hardware):** STEERING_PLAN §9 gates — direction confirm at `yawStepDeg≈1°`,
`turnCmd=0` regression parity, authority ramp, joystick + disconnect safing.

**Reference plan:** `STEERING_PLAN.md`.

---

## 2. What's Working (Stable Baseline)

Treat as load-bearing — touch only with regression risk in mind.

- Static balance: pitch + roll PD with nonlinear gain boost, deadband, IIR + delta-clamp output shaping, Smith predictor.
- WeightShift with per-ankle phased ramps and swing-leg bias masking.
- Closed-form 2-link IK with FK roundtrip validation (`LegIK::validateRoundtrip`).
- MotionManager priority arbitration (UI < GAIT < BALANCE), with e-stop drain.
- Gait FSM modes: IDLE / RUNNING / SINGLE_STEP / WSHIFT_ONLY / POSE_STEP — verified for in-place stepping.
- Task-space balance output: `BalanceCorrection` (dx/dy) merged into FootTarget before IK — the only leg-balance path after the consolidation refactor.
- Joint-space torso balance via `TorsoStabilizer` (separate from leg balance).
- Boot calibration with still-window gate for all three IMU instances (torso + 2 feet).
- WebSocket telemetry / tuning round-trip for balance, weight shift, gait, recovery, foot contact.

---

## 3. Open Issues

### Architectural

- **RMW race — FIXED (V1-closure audit B1/B1.2, 2026-07).** WS parsers now
  stage config edits into `StagedConfig<T>` seqlock slots and FSM commands
  into an SPSC ring; the main task applies both at the top of the 400 Hz
  gate (`webComm.applyStagedTuning()`, tick step 0). Covers Balance, Torso,
  Gait, WeightShift, FootImpact, SwingFootLeveler configs + gait/WS/balance
  commands. The "don't tune while armed" workaround is retired. Residuals
  (documented in `docs/AUDIT_FINDINGS.md`): TORSO_TARE's two direct float
  writes (needs sync reply), ServoControl pose/speed/trim calls, e-stop
  paths (deliberately immediate, A-03).
- ~~`IMU_LOOP_HZ` not a named constant~~ — **DONE (audit B2):** promoted to
  `project_wide_defs.h`; gate period + calib-dt derive from it.

### Validation gates (must clear before next milestone)

| Gate | Required by | Status |
|---|---|---|
| Stage 0 sagittal recovery: ≥4/5 catches at ±0.20 rad both directions | Capture-point full enable, forward walking >5 cycles | open |
| F2 60 s stepping run with stable predicates (no spurious flicker) | retro-validates the ALREADY-SHIPPED F3 gate (audit A-07) | open — needs deliberate session |
| Sign verification for `isFootLoaded` `loaded_pitch_sign` per foot under known load | STANCE_LOADED gate mode | open — needs single-leg-stance bench session |
| Zero-neutral flat-band check: tared foot neutrals within the 0.035/0.05 rad flat band | trusting the shipped STANCE_FLAT gate after any chassis/mount change (control path is untared — audit A-07) | open — read FOOT_CONTACT telemetry |
| ~~Rollover instrumentation flag #1~~ | trusting `ankleCmdVsFootPitchDivergenceDeg` | **CLOSED — proven + static_assert (audit B3)** |
| ~~Confirm balance does not submit `ANKLE_PITCH` at `SOURCE_BALANCE`~~ | post-contact ankle release | **CLOSED — verified; SOURCE_BALANCE = torso pitch/roll only (audit)** |
| A-09 human decision: yield ships active (2°) — bless or zero | any yield/rollover A/B run | **open — operator decision** |
| A-05 human decision: `pitch_setpoint_rad = 10.0` rad default | arming pitch balance safely | **open — operator decision** |
| A-11 human decision: WeightShift `torso_shift_deg = -20` live | torso-roll behavior during shifts | **open — operator decision** |
| B6 frame-rate ladder (50 → 250/300 Hz) per `SERVO_FRAME_HZ` block | raising servo frame rate | open — bench session |

### Hardware

- Voltage drift between power cycles affects servo neutrals. Mitigated by per-session UI re-zero. Permanent fix is F4 (voltage-drift calibration tool).
- Foot IMU cable length ~40 cm on Wire1 at 200 kHz. Verified stable; do **not** raise clock without re-running F1 loop-budget telemetry.

---

## 4. Deliberately Deferred

These have a clear case but are **not** the bottleneck right now:

- Centralised `IMU_LOOP_HZ` constant (cleanup, no behaviour change).
- Foot IMU upgrade for torso (BMI160 → LSM6DSV) — only after walking is robust on flat floor.
- CPG-driven gait — UVC reference uses fixed-cycle phase; match that until walking works.
- ZMP / full LIP closed-loop — hardware tier (hobby digital servos) doesn't repay the complexity yet.
- Quaternion attitude estimator — gimbal-lock band is well outside the operating envelope.
- Per-foot independent `FootContactConfig` — split only if telemetry shows asymmetric droop.

---

## 5. Quick Pointers

| For | Read |
|---|---|
| Layer map / per-tick ordering | `ARCHITECTURE.md` |
| Style, naming, anti-patterns | `CODING_STANDARD.md` |
| Gait FSM + capture-point + recovery details | `GAIT_DESIGN.md` |
| Capture-point implementation rationale | `Walking_Improvement_Plan.md` |
| Sagittal-recovery prerequisite for walking | `FORWARD_WALKING_PLAN.md` (Stage 0) |
| Foot IMU phases F0–F5 | `FOOT_IMU_INTEGRATION_PLAN.md`, `FOOT_IMU_PHASE2_PLAN.md` |
| Sagittal causal model + ankle-release design | `SAGITTAL_GAIT_DIAGNOSIS_CONTEXT.md`, `POST_CONTACT_ANKLE_RELEASE_CONTEXT.md` |
| Live constants & defaults | `include/project_wide_defs.h`, `joint_config.cpp`, `robot_geometry.h` |

---

*End of CURRENT_FOCUS.md*
