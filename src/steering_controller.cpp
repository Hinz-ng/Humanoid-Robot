// =============================================================================
// FILE:    steering_controller.cpp
// LAST CHANGED: 2026-06-09 | Hinz | Stage 1 open-loop hip-yaw steering
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
    // (1) Disabled → true no-op. Reset telemetry and effective command so a
    //     later enable starts from a clean, centered state.
    if (!_cfg.enabled) {
        _turnEff = 0.0f;
        _state   = SteeringState{};
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
    //     slewed _turnEff continues to decay toward 0 on subsequent ticks.
    if (estopped || _mm == nullptr) {
        _state.yawCmdDeg = 0.0f;
        _state.active    = false;
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
}
