// =============================================================================
// FILE:    gait_controller.cpp
// MODULE:  gait
// LAYER:   3.5 — Gait / Motion
// LAST CHANGED: 2026-04-25 | Hinz | Phase 1: pose-by-pose mode, leg inversion
//
// IMPLEMENTATION NOTES
// ────────────────────
// Each stepping half-cycle has two sub-states:
//
//   WAIT_SHIFT — WeightShift is ramping; phase holds; no foot lift.
//                Gate: wsProgress in the correct signed direction >= liftGateThreshold.
//                This prevents the swing foot from leaving the ground before the
//                stance leg is loaded — the root cause of the previous "shuffle".
//
//   SWINGING   — Gate is open; _halfPhase advances [0→1); foot follows
//                sin(_halfPhase × π): 0 at start, peak at midpoint, 0 at end.
//                Once _halfPhase >= 1.0, _advanceHalf() flips the half-cycle
//                and re-enters WAIT_SHIFT for the next leg.
//
// Weight shift direction (hardware-verified):
//   _currentHalf == 0 → LEFT shift (body over left leg, LEFT foot swings)
//   _currentHalf == 1 → RIGHT shift (body over right leg, RIGHT foot swings)
//
// POSE_STEP mode pause points:
//   1 = gate-open   : weight shifted, foot still on ground (verify CoM)
//   2 = peak swing  : foot at max height (verify lift height & direction)
//   3 = foot landed : before advancing half (verify stable landing)
//   Call nextPose() to advance through each hold point.
//
// Priority note: SOURCE_GAIT (priority 1) < SOURCE_BALANCE (priority 2).
// BalanceController will partially oppose the hip/knee commands on shared
// joints. This is expected and acceptable in Phase 1.
// =============================================================================

#include "gait_controller.h"
#include "balance_controller.h"  // BalanceState — needed for recovery flag observation
#include "weight_shift.h"
#include "oe_control.h"
#include "leg_ik.h"
#include "joint_config.h"
#include "motion_manager.h"
#include <math.h>

// ---------------------------------------------------------------------------
void GaitController::init(MotionManager* mm, WeightShift* ws, BalanceController* bal) {
    if (mm == nullptr) {
        Serial.println("[GaitCtrl] ERROR: MotionManager null — update() will no-op.");
    }
    _mm               = mm;
    _ws               = ws;
    _bal              = bal;
    _state            = {};
    _currentHalf      = 0;
    _halfPhase        = 0.0f;
    _subState         = StepSubState::WAIT_SHIFT;
    _lastShiftDir     = 0;
    _poseHoldPt       = 0;
    _stabilizeStartMs = 0;
    _stopStartMs      = 0;
    _stopSwingFrac    = 0.0f;
    _stopHalf         = 0;
    Serial.println("[GaitCtrl] Initialized.");
}

// ---------------------------------------------------------------------------
void GaitController::start(GaitMode mode) {
    // NOTE: lateral_shift_mm sign is no longer rejected here. Negative is the
    // intended default (adduction strategy). If a future gait phase requires
    // the translation-led convention (positive), gate it inside the relevant
    // mode handler rather than at start().
    _state.mode    = mode;
    _state.running = (mode != GaitMode::IDLE);

    if (mode == GaitMode::IDLE) {
        _state.phase      = 0.0f;
        _state.liftL_deg  = 0.0f;
        _state.liftR_deg  = 0.0f;
        _state.wsProgress = 0.0f;
        _state.liftGateOK = false;
        _state.poseHold   = 0;
        _currentHalf      = 0;
        _halfPhase        = 0.0f;
        _subState         = StepSubState::WAIT_SHIFT;
        _lastShiftDir     = 0;
        _poseHoldPt       = 0;
        _stabilizeStartMs = 0;
        _stopStartMs      = 0;
        _stopSwingFrac    = 0.0f;
        _stopHalf         = 0;
        // Stage 1+: clear recovery state so stale telemetry is not broadcast.
        _rcvSub              = RecoverySubState::TRIGGERED;
        _rcvDirX             = 0;
        _rcvDirY             = 0;
        _rcvSubStartMs       = 0;
        _rcvSwingIsRight     = false;
        _state.recoverySub       = RecoverySubState::TRIGGERED;
        _state.recoveryDirX      = 0;
        _state.recoveryDirY      = 0;
        _state.recoveryElapsedMs = 0;
        _state.rcvStepX_mm   = 0.0f;
        _state.rcvStepY_mm   = 0.0f;
        _state.rcvMagnitude  = 0.0f;
        _state.cycleXDispl_mm = 0.0f;
        if (_ws) _ws->trigger(ShiftDirection::NONE);
        Serial.println("[GaitCtrl] IDLE — weight shift centered.");
        return;
    }

    // RUNNING, SINGLE_STEP, WSHIFT_ONLY, POSE_STEP all start from half=0.
    _currentHalf      = 0;
    _halfPhase        = 0.0f;
    _subState         = StepSubState::WAIT_SHIFT;
    _lastShiftDir     = 0;
    _poseHoldPt       = 0;
    _stabilizeStartMs = 0;
    _stopStartMs      = 0;
    _stopSwingFrac    = 0.0f;
    _stopHalf         = 0;
    _state.poseHold   = 0;
    _triggerShiftForHalf(0);  // begin LEFT shift immediately

    Serial.printf("[GaitCtrl] start mode=%d\n", static_cast<int>(mode));
}

void GaitController::stop() {
    if (_state.mode == GaitMode::IDLE) return;
    if (_subState == StepSubState::STOPPING) return;

    _stopSwingFrac = (_subState == StepSubState::SWINGING)
                     ? _swingFrac(_halfPhase)
                     : 0.0f;
    _stopHalf       = _currentHalf;
    _stopStartMs    = millis();
    _subState       = StepSubState::STOPPING;
    _state.subState = _subState;
    _poseHoldPt     = 0;
    _state.poseHold = 0;
    Serial.printf("[GaitCtrl] STOP requested — ramping swing %.2f→0 over %ums\n",
                  _stopSwingFrac, (unsigned)GAIT_STOP_RAMP_MS);
}

void GaitController::singleStep() { start(GaitMode::SINGLE_STEP); }

// ---------------------------------------------------------------------------
//  nextPose  —  advance one step in POSE_STEP mode.
//  At hold 1 (gate-open) and hold 2 (peak): clear the pause, resume freely.
//  At hold 3 (landing): clear pause AND call _advanceHalf() to flip half-cycle.
// ---------------------------------------------------------------------------
void GaitController::nextPose() {
    if (_state.mode != GaitMode::POSE_STEP || _poseHoldPt == 0) return;

    const uint8_t hold = _poseHoldPt;
    _poseHoldPt       = 0;
    _state.poseHold   = 0;

    if (hold == 3) {
        // At landing: transition to next half now.
        _halfPhase = 0.0f;
        _advanceHalf();
    }
    // For holds 1 and 2: clear pause; update() will resume from current phase.
    Serial.printf("[GaitCtrl] POSE_STEP: nextPose (was hold=%d)\n", hold);
}

// ---------------------------------------------------------------------------
//  _triggerShiftForHalf  —  send the correct WeightShift direction for the
//  given half-cycle. Idempotent: skipped if direction has not changed.
// ---------------------------------------------------------------------------
void GaitController::_triggerShiftForHalf(uint8_t half) {
    if (!_ws) return;
    // half=0 → body weight over RIGHT leg (right=stance, left=swing) → trigger LEFT
    // half=1 → body weight over LEFT  leg (left=stance,  right=swing) → trigger RIGHT
    // (Enum values reversed; see weight_shift.h convention block.)
    const int8_t wantDir = (half == 0) ? +1 : -1;
    if (wantDir == _lastShiftDir) return;
    _lastShiftDir = wantDir;
    _ws->trigger(wantDir > 0 ? ShiftDirection::LEFT : ShiftDirection::RIGHT);
    Serial.printf("[GaitCtrl] half=%d  trigger=%s  CoM→%s\n",
                  half,
                  wantDir > 0 ? "LEFT"  : "RIGHT",
                  wantDir > 0 ? "right" : "left");
}

// ---------------------------------------------------------------------------
//  _swingFrac  —  sin-shaped lift fraction.
//  localPhase ∈ [0, 1) → [0, 1]: 0 at start, 1.0 at midpoint, 0 at end.
// ---------------------------------------------------------------------------
float GaitController::_swingFrac(float localPhase) const {
    return sinf(localPhase * static_cast<float>(M_PI));
}

// ---------------------------------------------------------------------------
//  _captureOffsetMm — sagittal capture-point step-placement correction.
//
//  Returns the mm offset to add to the swing leg's stride. Zero when
//  capture is disabled, IMU invalid, or estopped. Sign convention:
//    vx_CoM > 0 (body advancing forward) → positive offset
//                                          (foot lands further forward)
//
//  Capping logic (composite, conservative):
//    raw      = vx_CoM × INV_OMEGA0 × captureGain
//    by_field = clamp(raw, ±captureOffsetMaxMm)
//    by_step  = clamp(by_field, ±stepLengthMm)
//
//  by_step is the second cap: the capture offset alone never exceeds the
//  open-loop nominal stride. Without this, a noisy IMU spike could double
//  the stride in one tick.
//
//  IK reach is checked separately at submit time via the existing
//  IK status path — this function is intentionally pure-math.
// ---------------------------------------------------------------------------
float GaitController::_captureOffsetMm() const {
    if (!_cfg.captureEnabled)        return 0.0f;
    if (!_lastImuState.valid)        return 0.0f;
    if (oe_is_estopped())            return 0.0f;
    if (_cfg.stepLengthMm <= 0.01f)  return 0.0f;  // capture meaningless without nominal stride

    const float raw = _lastImuState.vx_CoM_mm_s
                    * GAIT_CAPTURE_INV_OMEGA0_S
                    * _cfg.captureGain;

    const float byField = constrain(raw,
                                    -_cfg.captureOffsetMaxMm,
                                    +_cfg.captureOffsetMaxMm);
    const float byStep  = constrain(byField,
                                    -_cfg.stepLengthMm,
                                    +_cfg.stepLengthMm);
    return byStep;
}

// ---------------------------------------------------------------------------
//  _buildFootTarget — Stage 1: vertical lift only.
//  Stance leg returns the neutral pose (x=0, y=0, h=stanceHeight).
//  Swing  leg returns same x,y with h reduced by swingFrac × stepHeight.
//  No lateral or fore/aft motion in Stage 1 — those land in Stage 2.
// ---------------------------------------------------------------------------
FootTarget GaitController::_buildFootTarget(bool isRight,
                                            float swingFrac,
                                            bool isSwing,
                                            float localPhase,
                                            const BalanceCorrection* bc,
                                            float strideScale) const {
    FootTarget t;
    t.isRightLeg   = isRight;
    t.x_mm         = GAIT_STANCE_X_OFFSET_MM;
    t.y_mm         = 0.0f;
    t.h_frontal_mm = 0.0f;

    const float liftMm = isSwing ? (swingFrac * _cfg.stepHeightMm) : 0.0f;
    t.h_sagittal_mm = _cfg.stanceHeightMm - liftMm;

    // Stage 3: linear stride applied to BOTH legs.
    //   Swing  leg: x = (localPhase - 0.5) × L  → starts at -L/2, lands at +L/2
    //   Stance leg: x = (0.5 - localPhase) × L  → starts at +L/2, retreats to -L/2
    // Net effect: body advances ≈ L per half-cycle. Foot stays planted while the
    // hip translates forward in body frame (foot retreats relative to hip).
    if (_cfg.stepLengthMm > 0.0f && strideScale > 0.0f) {
        const float captureRaw = isSwing ? _captureOffsetMm() : 0.0f;
        const float strideRaw  = isSwing
            ? (localPhase - 0.5f) * _cfg.stepLengthMm + captureRaw
            : (0.5f - localPhase) * _cfg.stepLengthMm;
        t.x_mm += strideRaw * strideScale;
    }

    // Stage 2: WeightShift task-space contribution (forward lean + lateral shift).
    if (_ws != nullptr) {
        float wsX_mm = 0.0f, wsY_mm = 0.0f;
        _ws->getStanceShift(isRight, wsX_mm, wsY_mm);
        t.x_mm += wsX_mm;
        t.y_mm += wsY_mm;
    }

    // Stage 3: balance correction merges onto stance leg only.
    if (bc != nullptr && !isSwing) {
        mergeBalanceCorrection(t, *bc);
    }

    t.valid = (t.h_sagittal_mm >= 20.0f);
    return t;
}

// ---------------------------------------------------------------------------
//  _submitFootTargetIK — Resolve FootTarget through LegIK and submit angles.
//
//  Joint indices come from joint_config.h. Submission goes through SOURCE_GAIT
//  so MotionManager priority arbitration is preserved. JointModel direction
//  fields handle left/right mirroring — submit the SAME anatomical angle to
//  both legs (do NOT negate for left here).
//
//  On DOMAIN_ERROR (h too small, NaN guard): no submission. Logs at 1 Hz to
//  avoid serial flood. UNREACHABLE / LIMIT_CLAMPED still submit the (clamped)
//  solution; status is recorded for the UI.
// ---------------------------------------------------------------------------
LegIKResult GaitController::_submitFootTargetIK(bool isRight,
                                                 const FootTarget& t) {
    LegIKResult r;
    if (_mm == nullptr) { r.status = IKStatus::DOMAIN_ERROR; return r; }
    if (!t.valid)       { r.status = IKStatus::DOMAIN_ERROR; return r; }

    r = LegIK::solve(t);

    if (r.status == IKStatus::DOMAIN_ERROR) {
        static uint32_t _lastLogMs = 0;
        const uint32_t now = millis();
        if (now - _lastLogMs > 1000) {
            _lastLogMs = now;
            Serial.printf("[GaitCtrl] IK DOMAIN_ERROR  side=%s  x=%.1f h=%.1f y=%.1f\n",
                          isRight ? "R" : "L",
                          t.x_mm, t.h_sagittal_mm, t.y_mm);
        }
        return r;
    }

    // Flat-foot guard: a clamped ankle pitch indicates the IK ran out of
    // soft-limit headroom (typically when reach is too close to maxReach
    // and the pose can't keep the sole flat). Treat |error| > 2 deg as a tuning
    // red flag - log and skip submission so the previous tick's pose holds.
    // Rationale: gait_development_integration.md section 6.5 - abort step on
    // flat_foot_error > ~2 deg.
    if (fabsf(r.flat_foot_error_deg) > 2.0f) {
        static uint32_t _lastFFLogMs = 0;
        const uint32_t now = millis();
        if (now - _lastFFLogMs > 1000) {
            _lastFFLogMs = now;
            Serial.printf("[GaitCtrl] FLAT-FOOT GUARD  side=%s  err=%.2f deg  "
                          "x=%.1f h=%.1f reach=%.1f%% - pose held.\n",
                          isRight ? "R" : "L", r.flat_foot_error_deg,
                          t.x_mm, t.h_sagittal_mm, r.sagittal_reach_pct);
        }
        return r;   // status fields populated; caller updates telemetry
    }

    const uint8_t hipPitch   = isRight ? IDX_R_HIP_PITCH   : IDX_L_HIP_PITCH;
    const uint8_t kneePitch  = isRight ? IDX_R_KNEE_PITCH  : IDX_L_KNEE_PITCH;
    const uint8_t anklePitch = isRight ? IDX_R_ANKLE_PITCH : IDX_L_ANKLE_PITCH;
    const uint8_t hipRoll    = isRight ? IDX_R_HIP_ROLL    : IDX_L_HIP_ROLL;
    const uint8_t ankleRoll  = isRight ? IDX_R_ANKLE_ROLL  : IDX_L_ANKLE_ROLL;

    // SOURCE_GAIT (priority 1). SOURCE_BALANCE (priority 2) on shared joints
    // will still override these. Expected for Stage 1 — see refactor §3.
    _mm->submit(SOURCE_GAIT, hipPitch,   r.hip_pitch_deg);
    _mm->submit(SOURCE_GAIT, kneePitch,  r.knee_pitch_deg);
    _mm->submit(SOURCE_GAIT, anklePitch, r.ankle_pitch_deg);
    _mm->submit(SOURCE_GAIT, hipRoll,    r.hip_roll_deg);
    _mm->submit(SOURCE_GAIT, ankleRoll,  r.ankle_roll_deg);

    return r;
}

// ---------------------------------------------------------------------------
void GaitController::_submitStanceBothLegs(const BalanceCorrection* bc) {
    // strideScale=0: stride and capture suppressed. Yields x=GAIT_STANCE_X_OFFSET_MM,
    // y=0 (plus WS task-space and balance correction). Used as the neutral
    // baseline whenever no leg is in an active swing role.
    const FootTarget tL = _buildFootTarget(false, 0.0f, false, 0.0f, bc, 0.0f);
    const FootTarget tR = _buildFootTarget(true,  0.0f, false, 0.0f, bc, 0.0f);

    const LegIKResult rL = _submitFootTargetIK(false, tL);
    const LegIKResult rR = _submitFootTargetIK(true,  tR);

    _state.lastIKStatusL = static_cast<uint8_t>(rL.status);
    _state.lastIKStatusR = static_cast<uint8_t>(rR.status);
    _state.lastReachPctL = rL.sagittal_reach_pct;
    _state.lastReachPctR = rR.sagittal_reach_pct;
    _state.lastFlatFootErrL_deg = rL.flat_foot_error_deg;
    _state.lastFlatFootErrR_deg = rR.flat_foot_error_deg;
}

// ---------------------------------------------------------------------------
//  _submitCycleStanceBoth — Stage 3: cycle-role aware submission.
//
//  At any localPhase, exactly one leg is in the swing role (foot on or near
//  the planting trajectory) and one is in the stance role (foot retreating
//  through body frame). This helper submits both legs with the correct role
//  flag so _buildFootTarget applies the appropriate stride formula.
//
//  Use during WAIT_SHIFT and STABILIZING active sub-states. SWINGING uses its
//  own explicit submission (with non-zero swingFrac for the lift). IDLE and
//  STOPPING continue to use _submitStanceBothLegs (no stride applied).
//
//  Note: liftFrac=0 here. The lift trajectory is the responsibility of the
//  SWINGING branch, which overrides whatever this helper submitted.
// ---------------------------------------------------------------------------
void GaitController::_submitCycleStanceBoth(float localPhase,
                                              const BalanceCorrection* bc) {
    const bool leftIsSwing = (_currentHalf == 0);
    const FootTarget tL = _buildFootTarget(/*isRight=*/false, /*liftFrac=*/0.0f,
                                            /*isSwing=*/  leftIsSwing,
                                            localPhase, bc);
    const FootTarget tR = _buildFootTarget(/*isRight=*/true,  /*liftFrac=*/0.0f,
                                            /*isSwing=*/ !leftIsSwing,
                                            localPhase, bc);

    const LegIKResult rL = _submitFootTargetIK(false, tL);
    const LegIKResult rR = _submitFootTargetIK(true,  tR);

    _state.lastIKStatusL = static_cast<uint8_t>(rL.status);
    _state.lastIKStatusR = static_cast<uint8_t>(rR.status);
    _state.lastReachPctL = rL.sagittal_reach_pct;
    _state.lastReachPctR = rR.sagittal_reach_pct;
    _state.lastFlatFootErrL_deg = rL.flat_foot_error_deg;
    _state.lastFlatFootErrR_deg = rR.flat_foot_error_deg;
}

// ---------------------------------------------------------------------------
//  _advanceHalf  —  called when _halfPhase >= 1.0 (foot back on ground).
//  Flips the half-cycle and re-enters WAIT_SHIFT for the next leg.
//  SINGLE_STEP stops after completing half=1 (both legs have taken one step).
// ---------------------------------------------------------------------------
void GaitController::_advanceHalf() {
    if (_state.mode == GaitMode::SINGLE_STEP && _currentHalf == 1) {
        stop();
        Serial.println("[GaitCtrl] SINGLE_STEP complete — IDLE.");
        return;
    }

    _currentHalf      = 1 - _currentHalf;   // 0→1 or 1→0
    _halfPhase        = 0.0f;
    _state.cycleXDispl_mm = 0.0f;
    _subState         = StepSubState::STABILIZING;
    _stabilizeStartMs = millis();
    // Center the weight shift; the structural/IIR transients from the half just
    // completed damp during the hold. _triggerShiftForHalf() for the new half
    // runs after STABILIZE expires (see update() STABILIZING branch).
    if (_ws) _ws->trigger(ShiftDirection::NONE);
    _lastShiftDir = 0;
    Serial.printf("[GaitCtrl] STABILIZE — half=%d, %ums hold\n",
                  _currentHalf, (unsigned)GAIT_STABILIZE_MS);
}

// ---------------------------------------------------------------------------
//  update  —  main control loop, every tick from main.cpp.
// ---------------------------------------------------------------------------
void GaitController::update(float dt_s, const IMUState& imuState,
                            const BalanceCorrection* bc,
                            const BalanceState* balState,
                            const FootContactSnapshot* footSnap) {
    _lastImuState = imuState;

    // Publish swing-leg hint to WeightShift FIRST. Reflects last tick's
    // substate; consumed on next tick's WS update (2.5 ms staleness).
    // Placing this at the top guarantees it runs on every early-return path.
    _publishSwingLegMaskToWS();

    if (_mm == nullptr)   return;
    if (oe_is_estopped()) return;

    // Clear per-tick lift outputs (re-set below when swinging).
    _state.liftL_deg  = 0.0f;
    _state.liftR_deg  = 0.0f;
    _state.liftGateOK = false;
    _state.gateImuRollOK    = false;
    _state.gateStanceFlat   = false;
    _state.gateStanceLoaded = false;
    _state.gateStanceIsLeft = false;

    // Stage 1: observe recovery flag from BalanceController.
    // Suppressed when already in RECOVERY_STEP (no re-trigger mid-step) and
    // during RUNNING (F6 mitigation — false positives during walking).
    // balState is the previous-tick BalanceState in legacy mode (2.5 ms stale),
    // which is within the recovery_confirm_ticks debounce window.
    if (balState != nullptr
        && balState->recovery_flag
        && _state.mode != GaitMode::RECOVERY_STEP
        && _state.mode != GaitMode::RUNNING) {
        triggerRecovery(imuState.pitch, imuState.roll);
    }

    if (_state.mode == GaitMode::RECOVERY_STEP) {
        _runRecoveryStep(dt_s, imuState);
        return;
    }

    const WeightShiftState wsState = _ws ? _ws->getState() : WeightShiftState{};
    const bool wsActive = (_ws != nullptr) &&
                          (fabsf(wsState.progress) > 0.005f || wsState.ramping);
    const bool gaitActive = (_state.mode != GaitMode::IDLE);

    if (!wsActive && !gaitActive) {
        _state.phase      = 0.0f;
        _state.wsProgress = 0.0f;
        _state.poseHold   = 0;
        // Task-space mode: BalanceCorrection reaches servos only through IK.
        // GaitController is the sole consumer of bc, so we must not skip it
        // here — the robot must maintain its pose under balance correction even
        // when standing still with no gait or weight-shift active.
        if (bc != nullptr && bc->valid) {
            _submitStanceBothLegs(bc);
        }
        return;
    }

    // ── Weight shift state ────────────────────────────────────────────────────
    const float wsProgress = _ws ? wsState.progress : 1.0f;
    _state.wsProgress = wsProgress;

    // -- Phase F3: composable lift gate ------------------------------------
    //
    // Layer 1 (always required, except when explicitly bypassed):
    //   progressOK - kinematic check: WeightShift has ramped into the
    //                correct signed direction with sufficient magnitude.
    //
    // Layer 2 (extra verification, mode-dependent):
    //   imuRollOK    - torso IMU roll past wsThresholdRad in correct direction
    //   stanceFlat   - stance foot's pitch and roll within the flat-threshold
    //   stanceLoaded - stance foot's pitch shows the loaded-deviation signature
    //
    // POSE_STEP unconditionally bypasses Layer 2 - the operator is in the loop.
    //
    // half=0 -> LEFT foot swings -> RIGHT foot is stance (rollSign = +1)
    // half=1 -> RIGHT foot swings -> LEFT  foot is stance (rollSign = -1)
    const bool poseMode     = (_state.mode == GaitMode::POSE_STEP);
    const bool isLeftStance = (_currentHalf == 1);
    const float rollSign    = (_currentHalf == 0) ? +1.0f : -1.0f;

    const bool progressOK = (wsProgress * rollSign >= _cfg.liftGateThreshold);

    // Always evaluate every sub-gate for telemetry consistency. The mode
    // dispatch below picks which ones actually compose into liftGateOK.
    const bool imuRollOK = (imuState.roll * rollSign >= _cfg.wsThresholdRad);

    bool stanceFlat   = false;
    bool stanceLoaded = false;
    if (footSnap != nullptr) {
        if (isLeftStance) {
            stanceFlat   = footSnap->leftValid && footSnap->leftFlat;
            stanceLoaded = footSnap->leftValid && footSnap->leftLoaded;
        } else {
            stanceFlat   = footSnap->rightValid && footSnap->rightFlat;
            stanceLoaded = footSnap->rightValid && footSnap->rightLoaded;
        }
    }

    bool extraGate = true;
    if (!poseMode) {
        switch (_cfg.liftGateMode) {
            case LiftGateMode::OFF:
                extraGate = true;
                break;
            case LiftGateMode::IMU_ROLL:
                extraGate = imuRollOK;
                break;
            case LiftGateMode::STANCE_FLAT:
                extraGate = stanceFlat;
                break;
            case LiftGateMode::STANCE_LOADED:
                // STANCE_LOADED implicitly requires the foot to be flat — an
                // unloaded foot that happens to be flat is not "loaded", and
                // a loaded-but-tilted foot indicates the stance leg is on the
                // edge. Compose flat AND loaded so the mode means what its name
                // suggests. This is the one place we DO compose, because the
                // two predicates describe the same physical event.
                extraGate = stanceFlat && stanceLoaded;
                break;
        }
    }

    _state.liftGateOK = progressOK && extraGate;
    _state.gateImuRollOK    = imuRollOK;
    _state.gateStanceFlat   = stanceFlat;
    _state.gateStanceLoaded = stanceLoaded;
    _state.gateStanceIsLeft = isLeftStance;

    _state.subState = _subState;
    _state.phase    = _currentHalf * 0.5f + _halfPhase * 0.5f;
    _state.poseHold = _poseHoldPt;

    // Keep the cycle baseline alive on every active tick. Later branches may
    // overwrite one leg with a swing target. Stage 3: legs reflect their gait-
    // cycle role even when not actively lifting, so accumulated body displacement
    // is preserved through WAIT_SHIFT and STABILIZING.
    _submitCycleStanceBoth(_halfPhase, bc);

    // BUG FIX (weight-shift runaway): when gait mode is IDLE but WeightShift is
    // active (manual WEIGHT_SHIFT:left/right from UI), only the stance IK should
    // run. The swing FSM below MUST NOT execute — otherwise the gate-open path
    // lifts the foot and _advanceHalf() flips _currentHalf, retriggering an
    // opposite-direction WeightShift. Result is endless L↔R oscillation that
    // only the Center button stops. Pinning substate to WAIT_SHIFT and zeroing
    // poseHold keeps state coherent for the next GAIT_START.
    if (!gaitActive) {
        _subState         = StepSubState::WAIT_SHIFT;
        _state.subState   = _subState;
        _halfPhase        = 0.0f;
        _state.poseHold   = 0;
        _state.liftL_deg  = 0.0f;
        _state.liftR_deg  = 0.0f;
        return;
    }

    if (_subState == StepSubState::STOPPING) {
        const uint32_t elapsed = millis() - _stopStartMs;
        const float liftRamp = (elapsed >= GAIT_STOP_RAMP_MS)
                             ? 0.0f
                             : (1.0f - (float)elapsed / (float)GAIT_STOP_RAMP_MS);
        const float stopFrac    = _stopSwingFrac * liftRamp;
        const float strideScale = liftRamp;   // identical time profile

        const bool leftIsSwing = (_stopHalf == 0);
        const FootTarget tL = _buildFootTarget(false, stopFrac,  leftIsSwing,
                                               _halfPhase, bc, strideScale);
        const FootTarget tR = _buildFootTarget(true,  stopFrac, !leftIsSwing,
                                               _halfPhase, bc, strideScale);
        const LegIKResult rL = _submitFootTargetIK(false, tL);
        const LegIKResult rR = _submitFootTargetIK(true,  tR);

        _state.lastIKStatusL = static_cast<uint8_t>(rL.status);
        _state.lastIKStatusR = static_cast<uint8_t>(rR.status);
        _state.lastReachPctL = rL.sagittal_reach_pct;
        _state.lastReachPctR = rR.sagittal_reach_pct;
        _state.lastFlatFootErrL_deg = rL.flat_foot_error_deg;
        _state.lastFlatFootErrR_deg = rR.flat_foot_error_deg;
        _state.liftL_deg = leftIsSwing  ? rL.hip_pitch_deg : 0.0f;
        _state.liftR_deg = !leftIsSwing ? rR.hip_pitch_deg : 0.0f;

        // Telemetry: applied capture also fades — surface the actual value.
        _state.captureOffsetMm = _captureOffsetMm() * strideScale;
        _state.cycleXDispl_mm  = _halfPhase * _cfg.stepLengthMm * strideScale;

        if (elapsed >= GAIT_STOP_RAMP_MS / 2 && _ws && _lastShiftDir != 0) {
            _ws->trigger(ShiftDirection::NONE);
            _lastShiftDir = 0;
        }

        const bool wsSettled = (!_ws) ||
                               (fabsf(_ws->getState().progress) < 0.01f
                                && !_ws->getState().ramping);
        if (elapsed >= GAIT_STOP_RAMP_MS && wsSettled) {
            Serial.println("[GaitCtrl] STOP complete — entering IDLE.");
            _state.mode      = GaitMode::IDLE;
            _state.running   = false;
            _subState        = StepSubState::WAIT_SHIFT;
            _state.subState  = _subState;
            _currentHalf     = 0;
            _halfPhase       = 0.0f;
            _stopSwingFrac   = 0.0f;
            _stopStartMs     = 0;
            _stopHalf        = 0;
            _state.phase     = 0.0f;
            _state.wsProgress = wsSettled ? 0.0f : _state.wsProgress;
            _state.poseHold  = 0;
            _state.captureOffsetMm = 0.0f;   // ADD
            _state.cycleXDispl_mm  = 0.0f;   // ADD
        }
        return;
    }

    // ── POSE_STEP: if currently paused, re-submit lift to hold pose ───────────
    if (_state.mode == GaitMode::POSE_STEP && _poseHoldPt > 0) {
        if (_subState == StepSubState::SWINGING) {
            const float liftFrac = _swingFrac(_halfPhase);
            // _currentHalf == 0 → LEFT foot swings (hardware-verified, see top comment).
            const bool leftIsSwing = (_currentHalf == 0);

            const FootTarget tL = _buildFootTarget(/*isRight=*/false, liftFrac,
                                                    /*isSwing=*/  leftIsSwing,
                                                    _halfPhase, bc);
            const FootTarget tR = _buildFootTarget(/*isRight=*/true,  liftFrac,
                                                    /*isSwing=*/ !leftIsSwing,
                                                    _halfPhase, bc);

            const LegIKResult rL = _submitFootTargetIK(false, tL);
            const LegIKResult rR = _submitFootTargetIK(true,  tR);

            _state.lastIKStatusL = static_cast<uint8_t>(rL.status);
            _state.lastIKStatusR = static_cast<uint8_t>(rR.status);
            _state.lastReachPctL = rL.sagittal_reach_pct;
            _state.lastReachPctR = rR.sagittal_reach_pct;
            _state.lastFlatFootErrL_deg = rL.flat_foot_error_deg;
            _state.lastFlatFootErrR_deg = rR.flat_foot_error_deg;

            // Repurposed UI fields: hip pitch produced by IK on the swing leg.
            // (Renaming to liftL_hipDeg in Stage 2 alongside the UI.)
            _state.liftL_deg = leftIsSwing  ? rL.hip_pitch_deg : 0.0f;
            _state.liftR_deg = !leftIsSwing ? rR.hip_pitch_deg : 0.0f;
        }
        return;
    }

    // ── WSHIFT_ONLY mode ──────────────────────────────────────────────────────
    if (_state.mode == GaitMode::WSHIFT_ONLY) {
        if (_cfg.phaseRateHz > 0.0f) {
            const float halfPeriod_s = 1.0f / (2.0f * _cfg.phaseRateHz);
            _halfPhase += dt_s / halfPeriod_s;
            if (_halfPhase >= 1.0f) {
                _halfPhase   = 0.0f;
                _currentHalf = 1 - _currentHalf;
                _triggerShiftForHalf(_currentHalf);
            }
        }
        return;
    }

    // ── RUNNING / SINGLE_STEP / POSE_STEP: weight-shift-gated stepping ────────

    // STABILIZING is entered by _advanceHalf() between half-cycles. Hold pose
    // (no submits, no phase advance) until GAIT_STABILIZE_MS has elapsed, then
    // arm WAIT_SHIFT and trigger the new half's shift. This separates the prior
    // shift's mechanical settling from the next shift's command edge so the two
    // do not superimpose into a destabilising transient.
    if (_subState == StepSubState::STABILIZING) {
        if (millis() - _stabilizeStartMs >= GAIT_STABILIZE_MS) {
            _subState       = StepSubState::WAIT_SHIFT;
            _state.subState = _subState;
            _triggerShiftForHalf(_currentHalf);
            Serial.printf("[GaitCtrl] STABILIZE done → WAIT_SHIFT half=%d\n",
                          _currentHalf);
        }
        return;
    }

    if (_subState == StepSubState::WAIT_SHIFT) {
        if (!_state.liftGateOK) return;  // still waiting for CoM to shift
        // Gate opened — begin the swing arc from zero for a clean sin profile.
        _subState  = StepSubState::SWINGING;
        _halfPhase = 0.0f;
        _state.subState = _subState;
        Serial.printf("[GaitCtrl] Gate open  half=%d  ws=%.3f  roll=%.4f rad\n",
                      _currentHalf, wsProgress, imuState.roll);

        // POSE_STEP: pause here so user can verify weight shift before lift.
        if (_state.mode == GaitMode::POSE_STEP) {
            _poseHoldPt     = 1;
            _state.poseHold = 1;
            Serial.println("[GaitCtrl] POSE_STEP: hold=1 gate-open");
            return;
        }
    }

    // SWINGING: compute lift, submit, advance phase.
    const float liftFrac = _swingFrac(_halfPhase);
    // _currentHalf == 0 → LEFT foot swings (hardware-verified, see top comment).
    const bool leftIsSwing = (_currentHalf == 0);

    // Build BOTH foot targets every tick. Stance leg also goes through IK
    // so it holds its commanded pose against priority arbitration.
    const FootTarget tL = _buildFootTarget(/*isRight=*/false, liftFrac,
                                            /*isSwing=*/  leftIsSwing,
                                            _halfPhase, bc);
    const FootTarget tR = _buildFootTarget(/*isRight=*/true,  liftFrac,
                                            /*isSwing=*/ !leftIsSwing,
                                            _halfPhase, bc);

    const LegIKResult rL = _submitFootTargetIK(false, tL);
    const LegIKResult rR = _submitFootTargetIK(true,  tR);

    _state.lastIKStatusL = static_cast<uint8_t>(rL.status);
    _state.lastIKStatusR = static_cast<uint8_t>(rR.status);
    _state.lastReachPctL = rL.sagittal_reach_pct;
    _state.lastReachPctR = rR.sagittal_reach_pct;
    _state.lastFlatFootErrL_deg = rL.flat_foot_error_deg;
    _state.lastFlatFootErrR_deg = rR.flat_foot_error_deg;

    // Stage 3 telemetry: estimated body translation this half-cycle.
    // _halfPhase × stepLengthMm. Resets to 0 in _advanceHalf().
    _state.cycleXDispl_mm = _halfPhase * _cfg.stepLengthMm;
    // Stage 2.2 — broadcast the capture offset that was applied this tick.
    // Reads as 0 when capture disabled or vx_CoM ≈ 0.
    _state.captureOffsetMm = _captureOffsetMm();

    // Repurposed UI fields: hip pitch produced by IK on the swing leg.
    _state.liftL_deg = leftIsSwing  ? rL.hip_pitch_deg : 0.0f;
    _state.liftR_deg = !leftIsSwing ? rR.hip_pitch_deg : 0.0f;

    if (_cfg.phaseRateHz > 0.0f) {
        const float halfPeriod_s = 1.0f / (2.0f * _cfg.phaseRateHz);
        const float newPhase     = _halfPhase + dt_s / halfPeriod_s;

        // POSE_STEP: pause at peak (halfPhase first crosses 0.5).
        if (_state.mode == GaitMode::POSE_STEP &&
            _halfPhase < 0.5f && newPhase >= 0.5f) {
            _halfPhase      = 0.5f;
            _poseHoldPt     = 2;
            _state.poseHold = 2;
            _state.phase    = _currentHalf * 0.5f + _halfPhase * 0.5f;
            Serial.println("[GaitCtrl] POSE_STEP: hold=2 peak");
            return;
        }

        _halfPhase = newPhase;
    }

    if (_halfPhase >= 1.0f) {
        // POSE_STEP: pause at landing before advancing to next half-cycle.
        if (_state.mode == GaitMode::POSE_STEP) {
            _halfPhase      = 0.999f;  // hold near end; nextPose() will advance
            _poseHoldPt     = 3;
            _state.poseHold = 3;
            _state.phase    = _currentHalf * 0.5f + _halfPhase * 0.5f;
            Serial.println("[GaitCtrl] POSE_STEP: hold=3 landing");
            return;
        }
        _halfPhase = 0.0f;
        _advanceHalf();
    }
}

// ---------------------------------------------------------------------------
//  triggerRecovery  —  arm a 2D directional catch step (Stage 2).
// ---------------------------------------------------------------------------
void GaitController::triggerRecovery(float tilt_pitch_rad, float tilt_roll_rad) {
    // Refractory lockout: ignore triggers shortly after the previous step completed.
    if (millis() < _rcvLockoutUntilMs) {
        Serial.printf("[GaitCtrl] RECOVERY blocked — lockout %u ms remaining\n",
                      (unsigned)(_rcvLockoutUntilMs - millis()));
        return;
    }
    if (oe_is_estopped())                        return;
    if (_state.mode == GaitMode::RECOVERY_STEP)  return;  // no re-trigger mid-step

    // Combined disturbance magnitude (rad).
    const float tilt_mag = sqrtf(tilt_pitch_rad * tilt_pitch_rad
                                + tilt_roll_rad  * tilt_roll_rad);

    // Direction unit vector (±1 for telemetry).
    _rcvDirX = (tilt_pitch_rad > 0.0f) ? +1 : (tilt_pitch_rad < 0.0f ? -1 : 0);
    _rcvDirY = (tilt_roll_rad  > 0.0f) ? +1 : (tilt_roll_rad  < 0.0f ? -1 : 0);

    // Swing leg = leg on the side body is falling toward.
    _rcvSwingIsRight = (tilt_roll_rad >= 0.0f);

    // Step magnitudes proportional to per-axis tilt component, capped.
    // Sign of lateral recovery is expressed by the swing leg selection, not
    // by the y_mm magnitude. Use absolute lateral magnitude so left & right
    // outward steps are both positive in the swing leg frontal frame.
    const float gain = RECOVERY_MAGNITUDE_GAIN_MM_PER_RAD;
    _rcvStepX_mm = constrain(tilt_pitch_rad * gain,
                              -RECOVERY_STEP_FORWARD_MAX_MM,
                              +RECOVERY_STEP_FORWARD_MAX_MM);
    _rcvStepY_mm = constrain(fabsf(tilt_roll_rad) * gain,
                              -RECOVERY_STEP_LATERAL_MAX_MM,
                              +RECOVERY_STEP_LATERAL_MAX_MM);

    // Apply minimum magnitude floor to dominant axis only.
    if (fabsf(_rcvStepX_mm) > fabsf(_rcvStepY_mm)) {
        if (fabsf(_rcvStepX_mm) < RECOVERY_STEP_MIN_MAGNITUDE_MM) {
            _rcvStepX_mm = copysignf(RECOVERY_STEP_MIN_MAGNITUDE_MM, _rcvStepX_mm);
        }
    } else {
        if (fabsf(_rcvStepY_mm) < RECOVERY_STEP_MIN_MAGNITUDE_MM) {
            _rcvStepY_mm = copysignf(RECOVERY_STEP_MIN_MAGNITUDE_MM, _rcvStepY_mm);
        }
    }

    // Mirror into telemetry state.
    _state.rcvStepX_mm  = _rcvStepX_mm;
    _state.rcvStepY_mm  = _rcvStepY_mm;
    _state.rcvMagnitude = sqrtf(_rcvStepX_mm * _rcvStepX_mm
                               + _rcvStepY_mm * _rcvStepY_mm);

    // Clear any residual gait or weight-shift activity.
    if (_ws) _ws->trigger(ShiftDirection::NONE);
    _lastShiftDir    = 0;
    _halfPhase       = 0.0f;
    _currentHalf     = 0;
    _poseHoldPt      = 0;
    _state.poseHold  = 0;

    _state.mode    = GaitMode::RECOVERY_STEP;
    _state.running = true;
    _enterRcvSub(RecoverySubState::TRIGGERED);

    Serial.printf("[GaitCtrl] RECOVERY triggered  pitch=%.3f roll=%.3f mag=%.3f  "
                  "step=(%.1f,%.1f)mm  swing=%s\n",
                  tilt_pitch_rad, tilt_roll_rad, tilt_mag,
                  _rcvStepX_mm, _rcvStepY_mm,
                  _rcvSwingIsRight ? "R" : "L");
}

// ---------------------------------------------------------------------------
void GaitController::_enterRcvSub(RecoverySubState s) {
    _rcvSub              = s;
    _state.recoverySub   = s;
    _rcvSubStartMs       = millis();
}

// ---------------------------------------------------------------------------
//  _runRecoveryStep  —  open-loop catch-step state machine (Stage 2).
//
//  Sub-state sequence:
//    TRIGGERED → LIFTING → LANDING → DAMP_HOLD → SOFT_RETURN → COMPLETE
//  Each timed stage submits FootTargets through _submitFootTargetIK() so
//  MotionManager priority arbitration is respected. Balance controller's
//  SOURCE_BALANCE submissions still override SOURCE_GAIT on shared joints.
//
//  swingTargetY_mm = _rcvStepY_mm (signed per-trigger). T2.1 on hardware must
//  verify all four directions; fix any sign errors in triggerRecovery().
// ---------------------------------------------------------------------------
void GaitController::_runRecoveryStep(float dt_s, const IMUState& imuState) {
    const uint32_t elapsed = millis() - _rcvSubStartMs;
    _state.recoveryElapsedMs = elapsed;
    _state.recoveryDirX      = _rcvDirX;
    _state.recoveryDirY      = _rcvDirY;

    // Step targets set at trigger time. y_mm sign convention: positive = outward
    // in the swing leg's frontal frame (same as Stage 1 +30 mm). For right-swing
    // this is rightward; for left-swing leftward. The IK and JointConfig direction
    // fields handle the per-leg mapping. T2.1 must verify all four directions on
    // hardware; if a sign is wrong, fix it in triggerRecovery(), not here.
    const float swingTargetX_mm = _rcvStepX_mm;               // sagittal (signed: + = forward)
    const float swingTargetY_mm = _rcvStepY_mm;               // lateral  (signed: + = outward for swing leg)
    const bool  sagittalDominant = fabsf(_rcvStepX_mm) > fabsf(_rcvStepY_mm);
    const float effectiveStepHeight_mm = sagittalDominant
        ? RECOVERY_STEP_HEIGHT_SAGITTAL_MM
        : RECOVERY_STEP_HEIGHT_LATERAL_MM;

    switch (_rcvSub) {

        case RecoverySubState::TRIGGERED:
            _enterRcvSub(RecoverySubState::PRE_LIFT);
            break;

        case RecoverySubState::PRE_LIFT: {
            const float t        = 1.0f;
            const float liftFrac = 0.0f;

            FootTarget tSwing;
            tSwing.isRightLeg    = _rcvSwingIsRight;
            tSwing.x_mm          = GAIT_STANCE_X_OFFSET_MM;
            tSwing.y_mm          = 0.0f;
            tSwing.h_sagittal_mm = _cfg.stanceHeightMm;
            tSwing.h_frontal_mm  = 0.0f;
            tSwing.valid         = true;

            FootTarget tStance;
            tStance.isRightLeg    = !_rcvSwingIsRight;
            tStance.x_mm          = GAIT_STANCE_X_OFFSET_MM
                                   - swingTargetX_mm * RECOVERY_STANCE_COUNTERSHIFT_FRAC * t;
            // Positive y_mm = outward in the stance leg's own frontal frame.
            // Stance hip abducts AWAY from the fall, widening the base of support.
            const float baseWidenMm = fabsf(swingTargetX_mm) * 0.20f * t;
            tStance.y_mm          = fmaxf(+fabsf(swingTargetY_mm) * RECOVERY_STANCE_COUNTERSHIFT_FRAC * t,
                                         baseWidenMm);
            _state.rcvStanceY_mm  = tStance.y_mm;
            tStance.h_sagittal_mm = _cfg.stanceHeightMm;
            tStance.h_frontal_mm  = 0.0f;
            tStance.valid         = true;

            const LegIKResult rSwing  = _submitFootTargetIK(_rcvSwingIsRight,  tSwing);
            const LegIKResult rStance = _submitFootTargetIK(!_rcvSwingIsRight, tStance);
            _state.lastIKStatusL = static_cast<uint8_t>(_rcvSwingIsRight ? rStance.status : rSwing.status);
            _state.lastIKStatusR = static_cast<uint8_t>(_rcvSwingIsRight ? rSwing.status  : rStance.status);
            _state.lastReachPctL = _rcvSwingIsRight ? rStance.sagittal_reach_pct : rSwing.sagittal_reach_pct;
            _state.lastReachPctR = _rcvSwingIsRight ? rSwing.sagittal_reach_pct  : rStance.sagittal_reach_pct;
            _state.lastFlatFootErrL_deg = _rcvSwingIsRight ? rStance.flat_foot_error_deg : rSwing.flat_foot_error_deg;
            _state.lastFlatFootErrR_deg = _rcvSwingIsRight ? rSwing.flat_foot_error_deg  : rStance.flat_foot_error_deg;

            if (elapsed >= RECOVERY_PRE_LIFT_MS) {
                _enterRcvSub(RecoverySubState::LIFTING);
            }
            break;
        }

        case RecoverySubState::LIFTING: {
            const float t        = fminf(static_cast<float>(elapsed)
                                         / static_cast<float>(RECOVERY_LIFT_MS), 1.0f);
            // Smoothstep ascent: zero foot velocity at lift-off (t=0) AND at peak (t=1).
            // sin(t·π/2) had peak velocity at t=0, which slammed the foot off the ground —
            // only invisible at low servo speeds because the smooth-stepper clipped it.
            const float liftFrac = 3.0f * t * t - 2.0f * t * t * t;

            FootTarget tSwing;
            tSwing.isRightLeg    = _rcvSwingIsRight;
            // Smoothstep on horizontal motion: zero velocity at t=0 (foot still grounded)
            // and t=1 (foot landed). Vertical lift uses sin*π/2 which has FULL velocity at
            // t=0 — together this means the foot rises before it moves forward, eliminating
            // the scrub mechanism. Cosmetic improvement at minimum, mechanical at best.
            const float t_xy = 3.0f * t * t - 2.0f * t * t * t;
            tSwing.x_mm = swingTargetX_mm * t_xy + GAIT_STANCE_X_OFFSET_MM;
            tSwing.y_mm = swingTargetY_mm * t_xy;
            tSwing.h_sagittal_mm = _cfg.stanceHeightMm - liftFrac * effectiveStepHeight_mm;
            tSwing.h_frontal_mm  = 0.0f;
            tSwing.valid         = (tSwing.h_sagittal_mm >= 20.0f);

            FootTarget tStance;
            tStance.isRightLeg    = !_rcvSwingIsRight;
            // Counter-shift is fully established by end of PRE_LIFT. Holding it constant
            // here prevents the swing hip from being dragged backward during the lift,
            // which manifests as ground-scrubbing (the "shuffle"). The * t ramp was a
            // silent bug: it threw away PRE_LIFT's load transfer at every sub-state edge.
            tStance.x_mm          = GAIT_STANCE_X_OFFSET_MM
                                   - swingTargetX_mm * RECOVERY_STANCE_COUNTERSHIFT_FRAC;
            // Positive y_mm = outward in the stance leg's own frontal frame.
            // Stance hip abducts AWAY from the fall, widening the base of support.
            // Was: -swingTargetY * frac (pushed stance toward the fall — wrong).
            const float baseWidenMm = fabsf(swingTargetX_mm) * 0.20f;
            tStance.y_mm          = fmaxf(+fabsf(swingTargetY_mm) * RECOVERY_STANCE_COUNTERSHIFT_FRAC,
                                         baseWidenMm);
            _state.rcvStanceY_mm  = tStance.y_mm;
            tStance.h_sagittal_mm = _cfg.stanceHeightMm;
            tStance.h_frontal_mm  = 0.0f;
            tStance.valid         = true;

            const LegIKResult rSwing  = _submitFootTargetIK(_rcvSwingIsRight,  tSwing);
            const LegIKResult rStance = _submitFootTargetIK(!_rcvSwingIsRight, tStance);
            _state.lastIKStatusL = static_cast<uint8_t>(_rcvSwingIsRight ? rStance.status : rSwing.status);
            _state.lastIKStatusR = static_cast<uint8_t>(_rcvSwingIsRight ? rSwing.status  : rStance.status);
            _state.lastReachPctL = _rcvSwingIsRight ? rStance.sagittal_reach_pct : rSwing.sagittal_reach_pct;
            _state.lastReachPctR = _rcvSwingIsRight ? rSwing.sagittal_reach_pct  : rStance.sagittal_reach_pct;
            _state.lastFlatFootErrL_deg = _rcvSwingIsRight ? rStance.flat_foot_error_deg : rSwing.flat_foot_error_deg;
            _state.lastFlatFootErrR_deg = _rcvSwingIsRight ? rSwing.flat_foot_error_deg  : rStance.flat_foot_error_deg;

            if (elapsed >= RECOVERY_LIFT_MS) {
                _enterRcvSub(RecoverySubState::LANDING);
            }
            break;
        }

        case RecoverySubState::LANDING: {
            const float t        = fminf(static_cast<float>(elapsed)
                                         / static_cast<float>(RECOVERY_LAND_MS), 1.0f);
            // Smoothstep descent: zero foot velocity at peak (t=0) AND at touchdown (t=1).
            // cos(t·π/2) had peak velocity at touchdown — the literal cause of the high-
            // speed tipping. The body was experiencing a 250 mm/s vertical foot impulse
            // at ground contact instead of a smoothed deceleration.
            const float liftFrac = 1.0f - (3.0f * t * t - 2.0f * t * t * t);

            FootTarget tSwing;
            tSwing.isRightLeg    = _rcvSwingIsRight;
            tSwing.x_mm          = swingTargetX_mm + GAIT_STANCE_X_OFFSET_MM;
            tSwing.y_mm          = swingTargetY_mm;
            tSwing.h_sagittal_mm = _cfg.stanceHeightMm - liftFrac * effectiveStepHeight_mm;
            tSwing.h_frontal_mm  = 0.0f;
            tSwing.valid         = (tSwing.h_sagittal_mm >= 20.0f);

            FootTarget tStance;
            tStance.isRightLeg    = !_rcvSwingIsRight;
            // Counter-shift is fully established by end of PRE_LIFT. Holding it constant
            // here prevents the swing hip from being dragged backward during the lift,
            // which manifests as ground-scrubbing (the "shuffle"). The * t ramp was a
            // silent bug: it threw away PRE_LIFT's load transfer at every sub-state edge.
            tStance.x_mm          = GAIT_STANCE_X_OFFSET_MM
                                   - swingTargetX_mm * RECOVERY_STANCE_COUNTERSHIFT_FRAC;
            const float baseWidenMm = fabsf(swingTargetX_mm) * 0.20f;
            tStance.y_mm          = fmaxf(+fabsf(swingTargetY_mm) * RECOVERY_STANCE_COUNTERSHIFT_FRAC,
                                         baseWidenMm);
            _state.rcvStanceY_mm  = tStance.y_mm;
            tStance.h_sagittal_mm = _cfg.stanceHeightMm;
            tStance.h_frontal_mm  = 0.0f;
            tStance.valid         = true;

            const LegIKResult rSwing  = _submitFootTargetIK(_rcvSwingIsRight,  tSwing);
            const LegIKResult rStance = _submitFootTargetIK(!_rcvSwingIsRight, tStance);
            _state.lastIKStatusL = static_cast<uint8_t>(_rcvSwingIsRight ? rStance.status : rSwing.status);
            _state.lastIKStatusR = static_cast<uint8_t>(_rcvSwingIsRight ? rSwing.status  : rStance.status);
            _state.lastReachPctL = _rcvSwingIsRight ? rStance.sagittal_reach_pct : rSwing.sagittal_reach_pct;
            _state.lastReachPctR = _rcvSwingIsRight ? rSwing.sagittal_reach_pct  : rStance.sagittal_reach_pct;
            _state.lastFlatFootErrL_deg = _rcvSwingIsRight ? rStance.flat_foot_error_deg : rSwing.flat_foot_error_deg;
            _state.lastFlatFootErrR_deg = _rcvSwingIsRight ? rSwing.flat_foot_error_deg  : rStance.flat_foot_error_deg;

            if (elapsed >= RECOVERY_LAND_MS) {
                _enterRcvSub(RecoverySubState::DAMP_HOLD);
            }
            break;
        }

        case RecoverySubState::DAMP_HOLD: {
            // Hold both feet at planted positions briefly so structural transients
            // damp before the foot starts returning to centered stance.
            FootTarget tSwing;
            tSwing.isRightLeg    = _rcvSwingIsRight;
            tSwing.x_mm          = swingTargetX_mm + GAIT_STANCE_X_OFFSET_MM;
            tSwing.y_mm          = swingTargetY_mm;
            tSwing.h_sagittal_mm = _cfg.stanceHeightMm;
            tSwing.h_frontal_mm  = 0.0f;
            tSwing.valid         = true;

            FootTarget tStance;
            tStance.isRightLeg    = !_rcvSwingIsRight;
            tStance.x_mm          = GAIT_STANCE_X_OFFSET_MM
                                   - swingTargetX_mm * RECOVERY_STANCE_COUNTERSHIFT_FRAC;
            const float baseWidenMm = fabsf(swingTargetX_mm) * 0.20f;
            tStance.y_mm          = fmaxf(+fabsf(swingTargetY_mm) * RECOVERY_STANCE_COUNTERSHIFT_FRAC,
                                         baseWidenMm);
            _state.rcvStanceY_mm  = tStance.y_mm;
            tStance.h_sagittal_mm = _cfg.stanceHeightMm;
            tStance.h_frontal_mm  = 0.0f;
            tStance.valid         = true;

            _submitFootTargetIK(_rcvSwingIsRight,  tSwing);
            _submitFootTargetIK(!_rcvSwingIsRight, tStance);

            if (elapsed >= RECOVERY_DAMP_HOLD_MS) {
                _enterRcvSub(RecoverySubState::SOFT_RETURN);
            }
            break;
        }

        case RecoverySubState::SOFT_RETURN: {
            // Smoothstep decay: foot offset decays 1.0 → 0.0 over RECOVERY_SOFT_RETURN_MS.
            const float t     = fminf(static_cast<float>(elapsed)
                                      / static_cast<float>(RECOVERY_SOFT_RETURN_MS), 1.0f);
            const float decay = 1.0f - (3.0f * t * t - 2.0f * t * t * t);

            FootTarget tSwing;
            tSwing.isRightLeg    = _rcvSwingIsRight;
            tSwing.x_mm          = swingTargetX_mm * decay + GAIT_STANCE_X_OFFSET_MM;
            tSwing.y_mm          = swingTargetY_mm * decay;
            tSwing.h_sagittal_mm = _cfg.stanceHeightMm;
            tSwing.h_frontal_mm  = 0.0f;
            tSwing.valid         = true;

            FootTarget tStance;
            tStance.isRightLeg    = !_rcvSwingIsRight;
            tStance.x_mm          = GAIT_STANCE_X_OFFSET_MM
                                   - swingTargetX_mm * RECOVERY_STANCE_COUNTERSHIFT_FRAC * decay;
            const float baseWidenMm = fabsf(swingTargetX_mm) * 0.20f * decay;
            tStance.y_mm          = fmaxf(+fabsf(swingTargetY_mm) * RECOVERY_STANCE_COUNTERSHIFT_FRAC * decay,
                                         baseWidenMm);
            _state.rcvStanceY_mm  = tStance.y_mm;
            tStance.h_sagittal_mm = _cfg.stanceHeightMm;
            tStance.h_frontal_mm  = 0.0f;
            tStance.valid         = true;

            _submitFootTargetIK(_rcvSwingIsRight,  tSwing);
            _submitFootTargetIK(!_rcvSwingIsRight, tStance);

            if (elapsed >= RECOVERY_SOFT_RETURN_MS) {
                _enterRcvSub(RecoverySubState::COMPLETE);
            }
            break;
        }

        case RecoverySubState::COMPLETE: {
            // Chain detection: if the body is still tilted after landing, a single
            // step was insufficient (large disturbance or marginal catch). Re-trigger
            // immediately rather than entering the refractory lockout, so the robot
            // takes a second step while it still has time to recover.
            //
            // start(IDLE) must come BEFORE triggerRecovery() because triggerRecovery()
            // guards against mode == RECOVERY_STEP. start(IDLE) clears that flag.
            const bool stillTilted =
                (fabsf(imuState.roll)  > RECOVERY_CHAIN_TILT_THRESHOLD_RAD) ||
                (fabsf(imuState.pitch) > RECOVERY_CHAIN_TILT_THRESHOLD_RAD);
            start(GaitMode::IDLE);  // clears recovery state, centers WeightShift
            if (stillTilted) {
                Serial.printf("[GaitCtrl] RECOVERY CHAIN — residual tilt "
                              "roll=%.3f pitch=%.3f — stepping again.\n",
                              imuState.roll, imuState.pitch);
                triggerRecovery(imuState.pitch, imuState.roll);
                // No lockout: triggerRecovery() above is the next step.
            } else {
                _rcvLockoutUntilMs = millis() + RECOVERY_LOCKOUT_MS;
                Serial.printf("[GaitCtrl] RECOVERY complete — lockout %u ms.\n",
                              (unsigned)RECOVERY_LOCKOUT_MS);
            }
            break;
        }
    }
}

// ---------------------------------------------------------------------------
//  _publishSwingLegMaskToWS  -  inform WeightShift which leg is airborne.
//
//  Called once at the start of update(). WeightShift consumes the published
//  hint on its NEXT tick (1 tick / 2.5 ms staleness - same as the previous
//  cross-module mask design). Always publishes - sticky semantics in WS would
//  otherwise leave stale state on early-return paths.
// ---------------------------------------------------------------------------
void GaitController::_publishSwingLegMaskToWS() const {
    if (!_ws) return;

    if (oe_is_estopped()) {
        _ws->setSwingLegMask(SwingLeg::NONE, 0.0f);
        return;
    }

    if (_state.mode == GaitMode::IDLE || _state.mode == GaitMode::WSHIFT_ONLY) {
        _ws->setSwingLegMask(SwingLeg::NONE, 0.0f);
        return;
    }

    if (_state.mode == GaitMode::RECOVERY_STEP) {
        // PRE_LIFT and SOFT_RETURN keep full bias (foot grounded). LIFTING /
        // LANDING / DAMP_HOLD are airborne -> liftFrac = 1.
        const bool airborne = (_rcvSub == RecoverySubState::LIFTING)
                           || (_rcvSub == RecoverySubState::LANDING)
                           || (_rcvSub == RecoverySubState::DAMP_HOLD);
        const SwingLeg swing = _rcvSwingIsRight ? SwingLeg::RIGHT : SwingLeg::LEFT;
        _ws->setSwingLegMask(swing, airborne ? 1.0f : 0.0f);
        return;
    }

    if (_subState == StepSubState::SWINGING) {
        const SwingLeg swing = (_currentHalf == 0) ? SwingLeg::LEFT : SwingLeg::RIGHT;
        _ws->setSwingLegMask(swing, _swingFrac(_halfPhase));
        return;
    }

    if (_subState == StepSubState::STOPPING) {
        const uint32_t elapsed  = millis() - _stopStartMs;
        const float    liftRamp = (elapsed >= GAIT_STOP_RAMP_MS)
                                  ? 0.0f
                                  : 1.0f - (float)elapsed / (float)GAIT_STOP_RAMP_MS;
        const SwingLeg swing = (_stopHalf == 0) ? SwingLeg::LEFT : SwingLeg::RIGHT;
        _ws->setSwingLegMask(swing, _stopSwingFrac * liftRamp);
        return;
    }

    // WAIT_SHIFT, STABILIZING: foot grounded - full bias both legs.
    _ws->setSwingLegMask(SwingLeg::NONE, 0.0f);
}
