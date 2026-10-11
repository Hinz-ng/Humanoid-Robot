# Gait Design

**Owner:** Hinz · **Updated:** 2026-06-06
**Scope:** Gait FSM, capture-point step placement, catch-step recovery,
WeightShift coupling.
**Out of scope:** Specific numeric values (read `project_wide_defs.h`),
balance controller internals (see `balance_controller.h/cpp`).

> Source of truth: `gait_controller.h/cpp`, `weight_shift.h/cpp`,
> `project_wide_defs.h` (GAIT and RECOVERY blocks). This file explains *why*
> the FSM looks the way it does; the code is *what* it does.

---

## 1. Top-Level Modes

`GaitController::start(mode)` enters one of:

| Mode | Purpose |
|---|---|
| `IDLE` | No motion. WeightShift centered. Holds stance pose if balance correction or weight shift is active. |
| `RUNNING` | Continuous stepping. Half-cycle FSM cycles indefinitely. |
| `SINGLE_STEP` | Exactly one full cycle (both legs swing once), then IDLE. |
| `WSHIFT_ONLY` | CoM ramps left↔right with no foot lift — used for ankle/CoM verification. |
| `POSE_STEP` | Pose-by-pose: pauses at gate-open / peak swing / landing. `nextPose()` advances. Used for visual verification of each FSM transition. |
| `RECOVERY_STEP` | Open-loop 2D catch step. Triggered automatically by `BalanceState::recovery_flag` (gated; see §4) or manually via WebSocket. |

---

## 2. Half-Cycle FSM (Stepping Modes)

Inside RUNNING / SINGLE_STEP / WSHIFT_ONLY / POSE_STEP, each half-cycle
runs one of these sub-states:

```
WAIT_SHIFT ──► SWINGING ──► (foot landed) ──► STABILIZING ──► WAIT_SHIFT (next half)
                                                                  │
                                                                  └─ stop() ──► STOPPING ──► IDLE
```

| Sub-state | Behaviour |
|---|---|
| `WAIT_SHIFT` | `_halfPhase` held at 0; no foot lift. Exits when `liftGateOK`. |
| `SWINGING`   | `_halfPhase` advances at `2 × phaseRateHz` cycles/s. Swing foot follows an asymmetric raised cosine vertically (`_swingFrac`, apex at `swingApexFrac`) and a smoothstep stride horizontally (`(_warpedStrideFrac(localPhase) − 0.5) × stepLengthMm`). `_warpedStrideFrac` compresses the swing-leg stride into `[0, strideCompleteFrac]` (then holds 1.0) so the foot reaches `+L/2` with zero horizontal velocity before touchdown; 1.0 reduces this to the legacy unwarped smoothstep bit-identically, but the SHIPPED default is 0.2 (strongly front-loaded — audit A-08 doc sync). Stance leg retreats with the opposite **unwarped** smoothstep formula — stance timing drives body translation and is intentionally left untouched. Both envelopes have zero velocity at the role-swap boundary. |
| `STABILIZING`| Inserted by `_advanceHalf()` between halves. Pose held for `GaitConfig::stabilizeMs` (runtime; shipped `GAIT_STABILIZE_MS` = 180 ms — audit A-08 doc sync). Default path: WeightShift centered via `trigger(NONE)` so IIR transients damp before the next direction is armed. `skipStabilizeNone=true` (step-4 probe, default OFF): skips the center trigger and immediately arms the next direction — CoM target flips foot-to-foot without passing through center. At low `stabilizeMs` the target reversal catches the ramp mid-decay, providing partial lateral blending. |
| `STOPPING`   | Swing fraction ramps to 0 over `GAIT_STOP_RAMP_MS`. Stride scale fades to zero in lockstep so accumulated body displacement decays cleanly. WeightShift centered at half-elapsed. Transitions to IDLE when ramp completes and WeightShift settles. |

**Lift gate logic** (`GaitController::update` — CORRECTED by the V1-closure
audit A-07; the previous pseudocode here predated F3 and showed only
progressOK + imuOK):
```
progressOK = |wsProgress| in the correct signed direction ≥ liftGateThreshold
             (always required except in LiftGateMode::OFF)
extraOK    = ONE additional predicate selected by GaitConfig::liftGateMode:
    OFF                     — none (kinematic only, diagnostic)
    IMU_ROLL                — torso roll past wsThresholdRad, correct direction
    STANCE_FLAT             — isFootFlat(stance foot)   ← SHIPPED DEFAULT (F3)
    STANCE_LOADED           — isFootLoaded(stance foot) (needs F4 calibration)
    COM_LATERAL             — neutral-corrected lateral CoM obs ≥ comLateralGateMm
    STANCE_FLAT_COM_LATERAL — both
liftGateOK = progressOK && extraOK
```
The shipped STANCE_FLAT predicate on the control path uses **zero neutrals**
(untared — the tared version is telemetry preview only). A dead foot IMU or a
mounting offset beyond the flat band closes the gate permanently; the
`waitShiftMaxMs` timeout then aborts to IDLE (fail-safe, audit-verified).

IMU_ROLL remains selectable but is only meaningful in joint-space sessions;
in task-space mode CoM shift is driven by foot placement, not ankle tilt, so
IMU roll is no longer the right signal for gate opening.

**Support-transfer dwell + timeout**: `WAIT_SHIFT` now requires the composed
`liftGateOK` signal to stay continuously true for `liftDwellMs` before
transitioning to `SWINGING`; any false tick resets the dwell counter. In the
same sub-state, `waitShiftMaxMs` bounds how long the controller will wait for
a confirmed transfer before calling `stop()` and ramping cleanly back to IDLE.
Defaults and tuned values remain in `project_wide_defs.h`.

---

## 3. Capture-Point Step Placement

Stage 2.2 — sagittal only, off by default.

### 3.1 Model

Linear inverted pendulum at CoM height `h`:
```
ω₀          = sqrt(g / h)
INV_OMEGA0  = 1 / ω₀
captureRaw  = vx_CoM × INV_OMEGA0 × captureGain
```

`INV_OMEGA0` is precomputed in `project_wide_defs.h` as
`GAIT_CAPTURE_INV_OMEGA0_S`. Recompute if `H_COM_MM` in
`robot_geometry.h` changes.

### 3.2 Capping (composite, conservative)

```
byField = clamp(captureRaw, ±captureOffsetMaxMm)        // operator hard cap
byStep  = clamp(byField,    ±stepLengthMm)              // never exceed nominal
```

`byStep` is the second cap: a noisy CoM-velocity spike cannot double the
stride in one tick. IK reach is enforced separately at submit time —
this function is intentionally pure-math.

### 3.3 Application

Applied to swing leg only, inside `_buildFootTarget()`:
```
strideRaw = (localPhase - 0.5) × stepLengthMm + captureRaw_clamped
```

Stance leg is **not** affected: capture point is a foot-placement correction,
not a body-translation correction.

### 3.4 Telemetry

- `GaitState::captureOffsetMm` — applied offset this tick. Zero when capture
  disabled or `vx_CoM ≈ 0`.
- UI Gait panel shows live offset and a Capture ON/OFF toggle.

### 3.5 Gating

`_captureOffsetMm()` returns 0 when:
- `captureEnabled == false`
- `IMUState::valid == false`
- `oe_is_estopped()`
- `stepLengthMm ≤ 0` (capture is meaningless without an open-loop nominal stride)

---

## 3.6 Lateral CoM Trim (W2)

UVC-style continuous lateral foot-placement correction. Where the
`COM_LATERAL` lift gate is a one-shot permission check, this trim runs every
tick: `_comLateralTrimMm()` observes lateral CoM displacement
(`H_COM_MM × (imuRoll − torsoRollNeutralRad)`), scales it by `comLatTrimGain`,
caps it at `±comLatTrimMaxMm`, and `_buildFootTarget()` adds it to **both**
feet's `y_mm` (sign per leg mirrors `mergeBalanceCorrection`'s `dy`: CoM-right
→ right foot abducts, left foot adducts → body tracks over the feet).

Applied on every `_buildFootTarget()` path including IDLE stance refresh.
The touchdown latch freezes only swing-leg `x`/`h`, never `y`, so the trim
keeps tracking after touchdown. `comLatTrimEnabled` defaults ON; toggle OFF
(UI "Lat Trim" button) for A/B baseline runs. Returns 0 when disabled,
IMU invalid, or estopped. Telemetry: `GaitState::comLatTrimMm`.

**Touchdown trim suppression.** The foot-strike at touchdown injects a short
torso-roll transient into the IMU that is not a real CoM excursion. When
`touchdownTrimSuppressEnabled` is set (default ON), `_updateTrimSuppression()`
gates the trim while `_touchdownLocked` is held and for `touchdownTrimHoldMs`
after it clears. In the window the trim is either **frozen** at its last clean
(pre-touchdown) value or **forced to 0** — selected by `touchdownTrimFreezeHold`
(default freeze; UI "Mode" button flips it live). The decision is made once per
tick (`_comLateralTrimMm()` is const and runs several times per tick); the raw
computation lives in `_comLateralTrimRawMm()`. Telemetry:
`GaitState::trimSuppressActive`. Tuned via `CMD:TOUCHDOWN_TUNE`.

## 3.7 Swing-Envelope Apex (W1)

`_swingFrac()` is an asymmetric raised cosine. `localPhase` is time-warped so
the lift apex lands at `swingApexFrac`. 0.5 = symmetric (legacy raised
cosine). Below 0.5 = faster lift, slower descent — lower vertical speed into
touchdown. Zero vertical velocity at lift-off / apex / touchdown holds for any
apex value. Tunable via UI; clamped `[0.05, 0.95]`.

**Default 0.2** (lowered from 0.5 on 2026-05-20): hardware testing found a
strongly front-loaded apex clearly better than symmetric — the long, slow
descent visibly reduces touchdown disturbance.

## 3.8 Touchdown Compliance (W4)

Event-triggered geometric yield that softens the landing impulse. It reuses
the **existing** early-contact / swing-truncation event verbatim — the same
`if (imp.detected)` branch in `update()` that sets the touchdown latch also
fires a compliance event. There is no separate detection path and no
force/impedance loop.

On the event, `_buildFootTarget()` adds a small negative offset
(`−complianceDepthMm`, default 4 mm) to **both** legs' `h_sagittal_mm`, so the
body yields evenly downward — a small symmetric squat, no lateral asymmetry.
`_updateCompliance()` then decays the offset back to zero on a smoothstep
profile over `complianceDecayMs` (default 180 ms), smearing the impulse across
tens of ms instead of one tick. The offset always returns to zero, so there is
no persistent posture bias.

The compliance timer is **independent of the touchdown latch** (own start
timestamp, not cleared by `_clearTouchdownLatch()`), so the yield decays
smoothly across the half-cycle boundary instead of snapping when the next
swing begins. It is reset only on mode entry (`start`), `init`, and recovery
entry. Recovery itself is unaffected — `_runRecoveryStep()` sets
`h_sagittal_mm` directly and never calls `_buildFootTarget()`.

Composition: the compliance offset is a fixed additive term on `h` applied
before `mergeBalanceCorrection()`. It is open-loop (time-based) where the
balance correction is closed-loop; the two simply sum on `h`, with no shared
state or feedback coupling.

`complianceEnabled` defaults ON; toggle OFF (UI "Compliance" button) for A/B
baseline runs. Tunables: `complianceDepthMm`, `complianceDecayMs`
(`CMD:COMPLIANCE_TUNE`). Telemetry: `GaitState::complianceOffsetMm` /
`complianceActive`.

---

## 3.9 Post-Contact Ankle Release + Rollover Instrumentation

Full design record and Claude Code handoff prompt:
`POST_CONTACT_ANKLE_RELEASE_CONTEXT.md`. Authoritative empirical causal model:
`SAGITTAL_GAIT_DIAGNOSIS_CONTEXT.md`. This section is the gait-level summary.

**Premise.** The diagnosed sagittal bottleneck is *post-contact*: the swing foot
lands toe-first ahead of the CoM and cannot roll (pinned ankle on a
non-backdrivable position servo). Flat-foot IK keeps commanding the sole flat
while the body vaults, so the servo fights the rollover; the pinned sole scrubs
→ double-touchdown → forward topple (Mode A) or backward shove (Mode B). The
binding constraint is *the foot cannot roll*, not anything before contact (every
pre-contact correction tried — capture-point, pitch-PD, pitch-placement — was
null; every contact-energy/geometry change moved stability).

**Rollover instrumentation (IMPLEMENTED — telemetry only, control bit-identical).**
`_updateRolloverTelemetry` opens an observation window at the touchdown latch
(`_rolloverStartMs = _touchdownMs`), services it each tick, survives the
`_advanceHalf` role swap (the landed foot becomes stance while still rolling),
and auto-closes after `rolloverWindowMs`. `GAIT:` fields:
- `rolloverFootPitchRad` — just-landed foot's neutral-corrected pitch; sweeps to
  0 on a clean roll, stays pitched when pinned. The "did it engage" ground truth.
- `rolloverDurationMs` — time-to-flat (`|pitch| ≤ rolloverFlatBandRad`); 0 after
  window close = never flattened.
- `ankleCmdVsFootPitchDivergenceDeg` — commanded ankle pitch − measured foot
  pitch. **Cross-frame** (different zeros): read the *curve through the window*,
  not the absolute value.
- `postContactCoMVelSign` — sagittal CoM-velocity sign at peak |·| (coarse only;
  `vx_CoM` degrades at footstrike — cross-check against the pitch peak).
- `postContactBodyPitchPeak` — peak-|·| signed torso pitch; the vault signature
  and the most trustworthy "did it help" scalar.

Three-way diagnosis: engaged+helped (rollover was binding; release works) /
engaged+didn't-help (rollover not binding) / didn't-engage (servos too stiff →
go mechanical). Open review flags (fix before trusting data): confirm `sideIdx()`
matches `static_cast<uint8_t>(Side)` so the divergence indexes the right leg
(must-fix); neutral-correct `postContactBodyPitchPeak`.

**Ankle release (SPECIFIED — not yet implemented).** A post-contact stance-leg
dorsiflexion that *relaxes the held-flat ankle command* so the planted foot can
roll — framed as **release, not drive**: a late relax degrades gracefully where
a late impulse (W4) misses entirely. Structural sibling of the heel-strike
*release* half, with two deliberate departures:
- **Single gate.** Armed only at the touchdown latch (which already gates
  `_halfPhase > touchdownGatePhase`). No eligibility window, no pre-arm.
- **No fallback.** Unlike heel-strike, no `_advanceHalf` arm ⇒ engagement is 1:1
  with *detected* touchdowns; a missed detection produces zero release.

Profile: smoothstep engage over `yieldEngageMs` → smoothstep decay over
`yieldReleaseMs` (≤ `rolloverWindowMs`), timer-based, side-locked, survives the
role swap. Sign is **negative (dorsiflexion), baked in**, post-contact only —
pre-contact dorsiflexion shortens the catch lever (the falsified heel-strike
failure); plantarflexion post-contact is push-off → Mode B. Applied additively
**post-IK** in `_submitFootTargetIK`
(`ankle_pitch_cmd = r.ankle_pitch_deg + pitch_bias + heel_bias + yield_bias`), so
it never enters the IK target and cannot trip the flat-foot guard.
**SHIPS ACTIVE (audit A-09, 2026-07):** `yieldEnabled=true` AND
`yieldDorsiflexDeg=2.0` — the design's inert-0 default was overridden in
tuning (commit "Ankle yield"). Shipped engage 10 ms + release 400 ms = 410 ms
≈ a full half-cycle at 0.7 Hz, so at higher cadence alternate steps can find
the previous release still active and get NO yield (`yieldSkipCount` counts
these). Pending human decision: bless the active default or restore 0. Until
then any A/B that assumes "release off by default" is confounded.

**Strong prior (unvalidated): firmware release is likely too late.** The binding
constraint is servo *slew*, not trigger timing (~40–60 ms contact-to-dorsiflexion
vs a 50–150 ms vault; `swingPreloadCompleteFrac` exists precisely because the
ankle can't hit attitude over hundreds of ms of swing). The **lateness falsifier**
— compare `yieldBiasDeg` against `rolloverFootPitchRad`, both timestamped from
`_touchdownMs` — is the headline readout and decides firmware-vs-mechanical. A
rockered / compliant-TPU sole (zero latency, zero slew) is the recommended
parallel track. Co-test the release A/B with **h-restore held fixed** (same
leg/window — re-extension fights the roll) and **W4 off**.

---

## 4. Catch-Step Recovery

A separate top-level mode with its own sub-FSM. Triggered by:

1. Automatic — `BalanceState::recovery_flag` rises and:
   - `_state.mode != RECOVERY_STEP` (no re-trigger mid-step)
   - `_state.mode != RUNNING` (suppressed during walking — F6 mitigation;
     keep gait debugging and recovery debugging separate)
2. Manual — `CMD:GAIT_TEST_RECOVERY_{F,B,L,R}` from the UI (no detection
   threshold; useful for tuning the FSM independently of `BalanceController`).

### 4.1 Trigger

`triggerRecovery(pitch, roll)`:
- Refractory lockout: skipped if `millis() < _rcvLockoutUntilMs`.
- Computes signed `_rcvStepX_mm` and `_rcvStepY_mm` from tilt × gain,
  clamped to `RECOVERY_STEP_{FORWARD,LATERAL}_MAX_MM`, with a floor of
  `RECOVERY_STEP_MIN_MAGNITUDE_MM` on the dominant axis.
- Picks swing leg = side body is falling toward (`_rcvSwingIsRight = roll ≥ 0`).
- Clears any active gait state; enters `RECOVERY_STEP` mode at sub-state `TRIGGERED`.

### 4.2 Sub-FSM

```
TRIGGERED   → PRE_LIFT   (RECOVERY_PRE_LIFT_MS)
            → LIFTING    (RECOVERY_LIFT_MS, smoothstep ascent)
            → LANDING    (RECOVERY_LAND_MS, smoothstep descent)
            → DAMP_HOLD  (RECOVERY_DAMP_HOLD_MS, hold landing pose)
            → SOFT_RETURN(RECOVERY_SOFT_RETURN_MS, smoothstep decay to centered)
            → COMPLETE   → (chain detect: residual tilt > threshold ⇒ re-trigger)
                        → IDLE + refractory lockout
```

Both legs receive a `FootTarget` every tick during recovery:
- Swing leg: forward x = `_rcvStepX_mm`, lateral y = `±|_rcvStepY_mm|` (sign
  expressed by swing-leg selection).
- Stance leg: counter-shift `−swingX × RECOVERY_STANCE_COUNTERSHIFT_FRAC` and
  abduction widening (`+|swingY| × frac` or `+0.20 × |swingX|`).

**Smoothstep** (zero velocity at both ends, peak in the middle) is used for
both vertical and horizontal trajectories during LIFTING and LANDING. The
earlier `sin(π·t/2)` variant had peak velocity at touchdown — the
literal cause of the high-speed tipping observed during early development.

### 4.3 Chain Detection

At `COMPLETE`, if `|roll| > RECOVERY_CHAIN_TILT_THRESHOLD_RAD` or
`|pitch| > RECOVERY_CHAIN_TILT_THRESHOLD_RAD`, immediately re-trigger
with the current tilt. Otherwise enter the refractory lockout.

This catches the case where a single step is insufficient for a large
disturbance — taking a second step while time remains is better than
falling.

---

## 5. WeightShift Coupling

WeightShift is the CoM-shift *author*. It does **not** drive gait phase;
GaitController does. Coupling points:

### 5.1 WeightShift → GaitController (read at FootTarget build time)

`getStanceShift(isRightLeg, x_offset_mm, y_offset_mm)` returns this tick's
task-space contributions:
- `x_offset_mm` = `−smoothedForwardLean_mm` (both legs, IIR-smoothed by
  `shift_smooth_alpha`)
- `y_offset_mm` = signed by leg side and `progress × lateral_shift_mm`

Sign convention for `lateral_shift_mm` (current default negative — adduction
strategy): swing-side foot adducts inward toward midline while stance-side
foot abducts. Net effect: foot tracks under CoM.

### 5.2 WeightShift → BalanceController (per-tick, via dedicated setters)

- `setAnkleRollBias(left, right)` — additive ankle-roll bias applied
  *after* the balance IIR (so it doesn't perturb the smoothed correction
  state). Swing-side bias is scaled by `(1 − liftFrac)` so airborne foot
  returns to IK-computed flat.

WeightShift no longer injects a roll setpoint: lateral CoM control flows
through `FootTarget.y_mm` (task-space `dy_mm`). The legacy joint-space leg
balance path and the `setRollSetpointRad` injection were removed in the
balance/gait consolidation refactor.

### 5.3 GaitController → WeightShift (published once per tick)

`setSwingLegMask(swing, liftFrac)` — published by
`GaitController::_publishSwingLegMaskToWS()` at the top of every
`update()` call (so it always runs, even on early-return paths).

Staleness: WeightShift reads on its next tick (1 tick ≈ 2.5 ms).
Acceptable for the bias-scaling use.

---

## 6. Stride Math Continuity

Every active half-cycle, both legs receive cycle-role-aware FootTargets via
`_submitCycleStanceBoth(localPhase)`. The role-swap at half boundary is
continuous:

```
end of half N:    swing leg x = +L/2, stance leg x = −L/2
start of half N+1: roles swap; new swing starts at −L/2, new stance at +L/2
                  → positions match across the boundary (no jump)
```

Unit-tested in `gait_controller_test.cpp::test_stage3_continuity` and
`test_stage3_midstance_zero`.

---

## 7. Telemetry to UI

`broadcastGaitState()` packs `GaitState` into a single WS packet (`GAIT:`).
Fields most relevant to gait debugging:

- `mode`, `sub` — top-level mode and half-cycle sub-state.
- `phase` — composite display phase `[0, 1)`.
- `liftL_deg`, `liftR_deg` — hip-pitch IK output on the swing leg.
- `wsProgress`, `gate` — weight-shift progress and lift-gate status.
- `lastIKStatus{L,R}`, `lastReachPct{L,R}`, `lastFlatFootErr{L,R}` — per-leg IK diagnostics.
- `captureOffsetMm` — applied capture-point offset this tick.
- `cycleXDispl_mm` — estimated body translation this half-cycle.
- `rolloverFootPitchRad`, `rolloverDurationMs`, `ankleCmdVsFootPitchDivergenceDeg`, `postContactCoMVelSign`, `postContactBodyPitchPeak` — post-contact rollover instrumentation (telemetry only; §3.9). Valid through the window opened at the touchdown latch.
- `rcvSub`, `rcvDir{X,Y}`, `rcvElapsedMs`, `rcvStep{X,Y}_mm`, `rcvMagnitude`, `rcvStanceY_mm` — recovery telemetry (valid only when `mode == RECOVERY_STEP`).

Per-step timing fields — captured once per footfall before `_advanceHalf()`:

- `stepPeriodMs` — time between consecutive footfalls (ms); 0 until second footfall.
- `waitMs` — duration of the WAIT_SHIFT sub-state preceding this swing.
- `swingMs` — lift-off commit → footfall (this half-cycle only).
- `stabilizeMsMeas` — measured STABILIZING duration preceding this WAIT_SHIFT; 0 on first swing.
- `postContactMs` — time from `_touchdownLocked` latch until footfall advance; 0 if latch did not fire.
- `lastStepSide` — which side just completed swing: 0=LEFT, 1=RIGHT.
- `cadenceHz` — instantaneous cadence derived from `stepPeriodMs` (steps/s per leg, full cycles = cadenceHz/2).
- `cadenceStepsMin` — same, expressed as steps/min.
- `meanStepPeriodMs`, `meanCadenceHz` — rolling mean over last 6 footfalls.
- `meanStepPeriodL`, `meanStepPeriodR` — per-side rolling mean step period (asymmetry diagnosis).
- `meanSwingMsL`, `meanSwingMsR` — per-side rolling mean swing duration.

The UI Gait Diagnostics panel renders all of the above under the "Step timing"
section. No heap allocation; ring buffer is fixed at 6 entries in `GaitController`.
Timing fields reset to 0 on `start()` / `stop()`. POSE_STEP footfalls are excluded
(operator-controlled timing is not meaningful).

The UI Gait panel renders all of the above and exposes runtime tuning of
every parameter via `CMD:GAIT_TUNE`.

---

## 8. What to Touch When You Want To…

| Goal | Touch |
|---|---|
| Lower lift-gate threshold | `GAIT_TUNE:liftGate=...` (runtime) or `GaitConfig::liftGateThreshold` default |
| Try capture-point on flat floor | UI "Capture" button + `GAIT_TUNE:captureGain=...` |
| Make recovery steps softer | Increase `RECOVERY_SOFT_RETURN_MS`, decrease `RECOVERY_MAGNITUDE_GAIN_MM_PER_RAD` |
| Increase stride length | `GAIT_TUNE:stepLength_mm=...` — capped at `stanceHeightMm × 0.20` |
| Confirm IK at edge of capture envelope | UI "IK Test" panel — `IK_VALIDATE` for math, `IK_SOLVE` for motion |
| Add a new gait sub-state | Extend `StepSubState` enum, branch in `update()`, mirror into telemetry struct + UI |
| Decouple stride from capture | Set `captureEnabled=false` (UI toggle) |
| Sweep inter-step hold (mode-B diagnosis) | `GAIT_TUNE:stabilizeMs=...` — sweep 400→250→150→80→40→0; watch for backward rebound drop |
| Probe foot-to-foot lateral blending | `GAIT_TUNE:skipNone=1` with low `stabilizeMs`; revert if lateral oscillation appears |
| Diagnose post-contact rollover / scrub | Watch `GAIT:` rollover fields (§3.9); A/B the ankle release with h-restore fixed + W4 off |

---

*End of GAIT_DESIGN.md*
