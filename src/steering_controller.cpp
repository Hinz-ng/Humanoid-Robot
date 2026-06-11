// =============================================================================
// FILE:    steering_controller.cpp
// LAST CHANGED: 2026-06-11 | Hinz | Stage 1.1 per-step yaw-budget telemetry
// =============================================================================

#include "steering_controller.h"
#include "oe_control.h"   // oe_is_estopped
#include <math.h>

// ---------------------------------------------------------------------------
//  yawLawDeg()  —  the pure per-tick yaw law (no estop, no submit, no state).
//
//  yaw = turnEff * yawStepDeg * turnSign * halfSign * (s - 0.5), clamped.
//
//  s = smoothstep(halfPhase): (s-0.5) is the command shape — it is +0.5 at the
//  half boundary on BOTH sides (continuous across the role swap), 0 at mid-swing,
//  and has zero slope at the ends, so yaw-RATE is zero at touchdown → no
//  rotational scrub of the planted foot.
//
//  halfSign flips the body-effect with the stance leg: +q rotates the body CCW
//  on right-stance (ψ=φ_R+q_R) and CW on left-stance (ψ=φ_L−q_L). Drop it and
//  the law oscillates with zero net turn. Over one swing (s: 0→1) the anatomical
//  Δq is +yawStepDeg on a LEFT swing / −yawStepDeg on a RIGHT swing; combined
//  with the stance relation each footfall yields the same-signed body yaw.
// ---------------------------------------------------------------------------
float SteeringController::yawLawDeg(float turnEff, float yawStepDeg,
                                    float turnSign, float maxYawDeg,
                                    Side swingSide, float halfPhase) {
    const float h        = halfPhase;
    const float s        = h * h * (3.0f - 2.0f * h);
    const float halfSign = (swingSide == Side::LEFT) ? +1.0f : -1.0f;
    const float yaw      = turnEff * yawStepDeg * turnSign * halfSign * (s - 0.5f);
    return constrain(yaw, -maxYawDeg, maxYawDeg);
}

// ---------------------------------------------------------------------------
//  slewToward()  —  one step of `current` toward `target`, capped at ±maxStep.
//  Keeps _turnEff (and hence the yaw output) C¹ regardless of when the operator
//  moves turnCmd.
// ---------------------------------------------------------------------------
float SteeringController::slewToward(float current, float target, float maxStep) {
    const float delta = target - current;
    if (fabsf(delta) <= maxStep) return target;
    return current + (delta > 0.0f ? maxStep : -maxStep);
}

// ---------------------------------------------------------------------------
//  update()  —  slew the effective command, evaluate the yaw law, submit.
//
//  Order matters:
//    1. enabled gate FIRST   → a disabled module is a true no-op (no slew, no
//       submit, zeroed telemetry). The yaw channels stay an uncontested
//       SOURCE_GAIT slot the UI can still drive — bit-identical ship.
//    2. slew _turnEff toward target on EVERY enabled tick (including estopped /
//       inactive) so the command is always continuous; the target collapses to 0
//       whenever estopped or not RUNNING, so the joints park at neutral smoothly.
//    3. submit ONLY when not estopped — mirrors GaitController, which submits
//       nothing while OE is HIGH.
// ---------------------------------------------------------------------------
void SteeringController::update(float dt_s) {
    // (1) Disabled → true no-op. Reset telemetry, effective command, AND all the
    //     characterization state so a later enable starts clean.
    if (!_cfg.enabled) {
        _turnEff         = 0.0f;
        _measYawRateFilt = 0.0f;
        _budget.reset();
        _ringIdx = _ringCount = 0;
        _wasActive = _footfallPending = false;
        _state = SteeringState{};
        return;
    }

    // (2) Slew _turnEff toward the target. Target is the live setpoint only while
    //     turning is permitted (RUNNING and not estopped); otherwise it decays to
    //     0 so the yaw joints return to neutral over ~1 step.
    const bool  estopped = oe_is_estopped();
    const float target   = (estopped || !_in.active) ? 0.0f : _cfg.turnCmd;
    _turnEff = slewToward(_turnEff, target, _cfg.turnSlewPerSec * dt_s);

    // Telemetry common to both the estopped and active paths.
    _state.turnCmd = _cfg.turnCmd;
    _state.turnEff = _turnEff;

    // (3) Never author joints while estopped. Refresh telemetry and bail; the
    //     slewed _turnEff continues to decay toward 0 on subsequent ticks. An
    //     in-flight budget window is dropped (the robot is disabled).
    if (estopped || _mm == nullptr) {
        _state.yawCmdDeg  = 0.0f;
        _state.active     = false;
        _state.yawRateDps = 0.0f;
        if (_wasActive) {
            _budget.reset();
            _clearBudgetTelemetry();
            _wasActive = _footfallPending = false;
        }
        return;
    }

    const float yaw = yawLawDeg(_turnEff, _cfg.yawStepDeg, _cfg.turnSign,
                                _cfg.maxYawDeg, _in.swingSide, _in.halfPhase);

    // Identical anatomical command to BOTH legs — the JointConfig `direction`
    // field (right +1, left -1) handles the physical mirror. No per-leg opposite
    // signs here. SOURCE_GAIT joint-relative degrees (from neutral).
    _mm->submit(SOURCE_GAIT, channelFor(Side::LEFT,  LegJoint::HIP_YAW), yaw);
    _mm->submit(SOURCE_GAIT, channelFor(Side::RIGHT, LegJoint::HIP_YAW), yaw);

    _state.yawCmdDeg = yaw;
    _state.active    = true;

    // ----------------------------------------------------------------------
    // Stage 1.1 — per-step yaw budget (TELEMETRY ONLY; never affects control).
    // Runs after the submit; reads the just-computed `yaw` as the commanded
    // anatomical angle. Live foot/torso rates were injected via setYawRates().
    // ----------------------------------------------------------------------
    // Live displays (always refreshed on the live path, incl. IDLE so the
    // gyro-sign bench gate works without walking).
    _measYawRateFilt = STEERING_YAW_RATE_LPF_ALPHA * _measYawRateFilt
                     + (1.0f - STEERING_YAW_RATE_LPF_ALPHA) * _torsoYawDps;
    _state.yawRateDps      = _measYawRateFilt;
    _state.footLeftYawDps  = _leftFootYawDps;
    _state.footRightYawDps = _rightFootYawDps;

    if (_in.active) {
        // stance = the NON-swing foot; captured per window so attribution holds
        // the same physical foot across the role swap.
        const bool stanceFootIsLeft = (_in.swingSide == Side::RIGHT);

        if (!_wasActive) {
            // Entering RUNNING — open the first window fresh.
            _budget.reset();
            _budget.openWindow(yaw, stanceFootIsLeft);
            _wasActive       = true;
            _footfallPending = false;
        } else if (_footfallPending) {
            // Footfall boundary (swing-side flip): close + publish the completed
            // window, then open the next on the now-current swing side.
            _budget.latch(STEERING_RATIO_MIN_CMD_DEG);
            _publishBudgetStep();
            _budget.openWindow(yaw, stanceFootIsLeft);
            _footfallPending = false;
        }

        const float halfSign  = (_in.swingSide == Side::LEFT) ? +1.0f : -1.0f;
        const float stanceDps = _budget.stanceIsLeft ? _leftFootYawDps  : _rightFootYawDps;
        const float swingDps  = _budget.stanceIsLeft ? _rightFootYawDps : _leftFootYawDps;
        _budget.tick(yaw, halfSign, _torsoYawDps, stanceDps, swingDps,
                     _in.touchdownLocked, dt_s);
    } else if (_wasActive) {
        // RUNNING → not RUNNING: drop the in-flight window and clear step telemetry.
        _budget.reset();
        _clearBudgetTelemetry();
        _wasActive       = false;
        _footfallPending = false;
    }
}

// ---------------------------------------------------------------------------
//  _publishBudgetStep()  —  copy the just-latched window into telemetry, push
//  the 6-step ring, and recompute the rolling means.
// ---------------------------------------------------------------------------
void SteeringController::_publishBudgetStep() {
    _state.cmdYawStepDeg     = _budget.cmdStep;
    _state.bodyYawStepDeg    = _budget.bodyStep;
    _state.stanceFootStepDeg = _budget.stanceStep;
    _state.swingFootStepDeg  = _budget.swingStep;
    _state.cmdYawPostTdDeg   = _budget.cmdPostTdStep;
    _state.bodyYawPostTdDeg  = _budget.bodyPostTdStep;
    _state.realizationRatio  = _budget.ratio;

    _ringCmd[_ringIdx]    = _budget.cmdStep;
    _ringBody[_ringIdx]   = _budget.bodyStep;
    _ringStance[_ringIdx] = _budget.stanceStep;
    _ringSwing[_ringIdx]  = _budget.swingStep;
    _ringRatio[_ringIdx]  = _budget.ratio;
    _ringIdx = (uint8_t)((_ringIdx + 1) % RING);
    if (_ringCount < RING) _ringCount++;

    float c = 0, b = 0, st = 0, sw = 0, r = 0;
    for (uint8_t i = 0; i < _ringCount; ++i) {
        c  += _ringCmd[i];    b  += _ringBody[i];
        st += _ringStance[i]; sw += _ringSwing[i];
        r  += _ringRatio[i];
    }
    const float inv = (_ringCount > 0) ? (1.0f / (float)_ringCount) : 0.0f;
    _state.meanCmdYawStepDeg     = c  * inv;
    _state.meanBodyYawStepDeg    = b  * inv;
    _state.meanStanceFootStepDeg = st * inv;
    _state.meanSwingFootStepDeg  = sw * inv;
    _state.meanRealizationRatio  = r  * inv;
}

// ---------------------------------------------------------------------------
//  _clearBudgetTelemetry()  —  zero the published step fields + ring (used on
//  the active→inactive edge and on e-stop so no stale step survives).
// ---------------------------------------------------------------------------
void SteeringController::_clearBudgetTelemetry() {
    _state.cmdYawStepDeg     = _state.bodyYawStepDeg    = 0.0f;
    _state.stanceFootStepDeg = _state.swingFootStepDeg  = 0.0f;
    _state.cmdYawPostTdDeg   = _state.bodyYawPostTdDeg  = 0.0f;
    _state.realizationRatio  = 0.0f;
    _state.meanCmdYawStepDeg     = _state.meanBodyYawStepDeg    = 0.0f;
    _state.meanStanceFootStepDeg = _state.meanSwingFootStepDeg  = 0.0f;
    _state.meanRealizationRatio  = 0.0f;
    _ringIdx = _ringCount = 0;
}
