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
#include "weight_shift.h"
#include "oe_control.h"
#include "leg_ik.h"
#include "joint_config.h"
#include "motion_manager.h"
#include <math.h>

// ---------------------------------------------------------------------------
void GaitController::init(MotionManager* mm, WeightShift* ws) {
    if (mm == nullptr) {
        Serial.println("[GaitCtrl] ERROR: MotionManager null — update() will no-op.");
    }
    _mm               = mm;
    _ws               = ws;
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
//  _buildFootTarget — Stage 1: vertical lift only.
//  Stance leg returns the neutral pose (x=0, y=0, h=stanceHeight).
//  Swing  leg returns same x,y with h reduced by swingFrac × stepHeight.
//  No lateral or fore/aft motion in Stage 1 — those land in Stage 2.
// ---------------------------------------------------------------------------
FootTarget GaitController::_buildFootTarget(bool isRight,
                                            float swingFrac,
                                            bool isSwing,
                                            const BalanceCorrection* bc) const {
    FootTarget t;
    t.isRightLeg   = isRight;
    t.x_mm         = GAIT_STANCE_X_OFFSET_MM;
    t.y_mm         = 0.0f;
    t.h_frontal_mm = 0.0f;  // 0 = auto-derive from sagittal in LegIK::solve

    const float liftMm = isSwing ? (swingFrac * _cfg.stepHeightMm) : 0.0f;
    t.h_sagittal_mm = _cfg.stanceHeightMm - liftMm;

    // Stage 2: compose WeightShift's task-space contribution. Body forward-lean
    // applies to both legs symmetrically; lateral shift is signed per leg side.
    if (_ws != nullptr) {
        float wsX_mm = 0.0f, wsY_mm = 0.0f;
        _ws->getStanceShift(isRight, wsX_mm, wsY_mm);
        t.x_mm += wsX_mm;
        t.y_mm += wsY_mm;
    }

    // Stage 3: merge balance correction onto stance leg only.
    // Swing leg keeps its planned trajectory unmodified.
    if (bc != nullptr && !isSwing) {
        mergeBalanceCorrection(t, *bc);
    }

    // Defensive guard — LegIK rejects < 10mm; flag earlier here.
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
    const FootTarget tL = _buildFootTarget(/*isRight=*/false, 0.0f, /*isSwing=*/false, bc);
    const FootTarget tR = _buildFootTarget(/*isRight=*/true,  0.0f, /*isSwing=*/false, bc);

    const LegIKResult rL = _submitFootTargetIK(false, tL);
    const LegIKResult rR = _submitFootTargetIK(true,  tR);

    _state.lastIKStatusL = static_cast<uint8_t>(rL.status);
    _state.lastIKStatusR = static_cast<uint8_t>(rR.status);
    _state.lastReachPctL = rL.sagittal_reach_pct;
    _state.lastReachPctR = rR.sagittal_reach_pct;
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
                            const BalanceCorrection* bc) {
    if (_mm == nullptr)   return;
    if (oe_is_estopped()) return;

    // Clear per-tick lift outputs (re-set below when swinging).
    _state.liftL_deg  = 0.0f;
    _state.liftR_deg  = 0.0f;
    _state.liftGateOK = false;

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

    // Gate direction check (signed, not just magnitude):
    //   half=0 (LEFT shift) → wsProgress ramps toward +1 → gate when >= +threshold
    //   half=1 (RIGHT shift) → wsProgress ramps toward -1 → gate when <= -threshold
    if (_currentHalf == 0) {
        _state.liftGateOK = (wsProgress >= _cfg.liftGateThreshold);
    } else {
        _state.liftGateOK = (wsProgress <= -_cfg.liftGateThreshold);
    }

    _state.subState = _subState;
    _state.phase    = _currentHalf * 0.5f + _halfPhase * 0.5f;
    _state.poseHold = _poseHoldPt;

    // Keep the stance baseline alive on every active tick. Later branches may
    // overwrite one leg with a swing target, but stance-only states still need
    // fresh IK submissions each loop.
    _submitStanceBothLegs(bc);

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
        const float stopFrac = (elapsed >= GAIT_STOP_RAMP_MS)
                             ? 0.0f
                             : (_stopSwingFrac
                                * (1.0f - (float)elapsed / (float)GAIT_STOP_RAMP_MS));

        const bool leftIsSwing = (_stopHalf == 0);
        const FootTarget tL = _buildFootTarget(false, stopFrac,  leftIsSwing, bc);
        const FootTarget tR = _buildFootTarget(true,  stopFrac, !leftIsSwing, bc);
        const LegIKResult rL = _submitFootTargetIK(false, tL);
        const LegIKResult rR = _submitFootTargetIK(true,  tR);

        _state.lastIKStatusL = static_cast<uint8_t>(rL.status);
        _state.lastIKStatusR = static_cast<uint8_t>(rR.status);
        _state.lastReachPctL = rL.sagittal_reach_pct;
        _state.lastReachPctR = rR.sagittal_reach_pct;
        _state.liftL_deg = leftIsSwing  ? rL.hip_pitch_deg : 0.0f;
        _state.liftR_deg = !leftIsSwing ? rR.hip_pitch_deg : 0.0f;

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
                                                    /*isSwing=*/  leftIsSwing, bc);
            const FootTarget tR = _buildFootTarget(/*isRight=*/true,  liftFrac,
                                                    /*isSwing=*/ !leftIsSwing, bc);

            const LegIKResult rL = _submitFootTargetIK(false, tL);
            const LegIKResult rR = _submitFootTargetIK(true,  tR);

            _state.lastIKStatusL = static_cast<uint8_t>(rL.status);
            _state.lastIKStatusR = static_cast<uint8_t>(rR.status);
            _state.lastReachPctL = rL.sagittal_reach_pct;
            _state.lastReachPctR = rR.sagittal_reach_pct;

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
                                            /*isSwing=*/  leftIsSwing, bc);
    const FootTarget tR = _buildFootTarget(/*isRight=*/true,  liftFrac,
                                            /*isSwing=*/ !leftIsSwing, bc);

    const LegIKResult rL = _submitFootTargetIK(false, tL);
    const LegIKResult rR = _submitFootTargetIK(true,  tR);

    _state.lastIKStatusL = static_cast<uint8_t>(rL.status);
    _state.lastIKStatusR = static_cast<uint8_t>(rR.status);
    _state.lastReachPctL = rL.sagittal_reach_pct;
    _state.lastReachPctR = rR.sagittal_reach_pct;

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
