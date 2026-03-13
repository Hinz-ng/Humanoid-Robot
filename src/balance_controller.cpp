// =============================================================================
// balance_controller.cpp
// Purpose : PD pitch balance controller implementation.
//
// PIPELINE (one call to update()):
//   IMUState
//     → Gate checks (enabled? valid? estopped? fallen?)
//     → error     = pitch - pitch_setpoint_rad
//     → u_raw     = (Kp * error + Kd * pitchRate) * correction_sign
//     → u_clamped = constrain(u_raw, -max_correction_deg, +max_correction_deg)
//     → ankle_cmd = u_clamped * ankle_ratio
//     → hip_cmd   = u_clamped * hip_ratio
//     → _applyPitchCorrection(ankle_cmd, hip_cmd)  → hardware (immediate)
//     → return BalanceState
//
// SMOOTH STEPPING NOTE:
//   applyBalanceOffset() uses immediate=true, bypassing the JointModel smooth
//   stepping loop. This is intentional — the control loop runs at 400 Hz and
//   cannot afford actuation lag. The smooth stepper still runs for UI/pose
//   commands on the same joints; the balance write simply overrides the hardware
//   target directly each tick.
//
//   Consequence: do NOT command balance-controlled joints via the UI sliders
//   while the balance controller is enabled. They will fight each other.
//   The MotionArbiter (Phase 3) resolves this cleanly. For now, disable the
//   controller before using sliders on ankle/hip pitch joints.
// =============================================================================

#include "balance_controller.h"
#include "oe_control.h"   // oe_is_estopped(), oe_estop()
#include <math.h>         // fabsf

// ---------------------------------------------------------------------------
BalanceController::BalanceController(ServoControl* servo)
    : _servo(servo), _cfg(), _lastState() {}

// ---------------------------------------------------------------------------
void BalanceController::init() {
    _lastState = {};
    // Log the starting config so it's visible in Serial on boot.
    Serial.printf("[BalanceController] Init. enabled=%s  Kp=%.1f  Kd=%.2f  setpoint=%.3f rad\n",
                  _cfg.enabled ? "true" : "false",
                  _cfg.Kp, _cfg.Kd, _cfg.pitch_setpoint_rad);
}

// ---------------------------------------------------------------------------
BalanceState BalanceController::update(const IMUState& state) {
    BalanceState out = {};   // zero-initialized; active=false, fell=false by default

    // --- Gate 1: must be explicitly enabled ---
    // The controller is off by default. Enable via WebSocket CMD:BAL_ENABLE
    // or by setting cfg.enabled = true in code after verifying correction direction.
    if (!_cfg.enabled) {
        _lastState = out;
        return out;
    }

    // --- Gate 2: estimator must be valid (calibration done, no IMU error) ---
    if (!state.valid) {
        _lastState = out;
        return out;
    }

    // --- Gate 3: hardware must not be in an ESTOP state ---
    if (oe_is_estopped()) {
        _lastState = out;
        return out;
    }

    // --- Fall detection ---
    // If pitch exceeds the fall threshold, the robot is on the ground or
    // completely unrecoverable. Fire ESTOP to protect servos from holding
    // max torque against the floor indefinitely.
    if (fabsf(state.pitch) > _cfg.fall_threshold_rad) {
        oe_estop();
        out.fell = true;
        Serial.printf("[BalanceController] FALL DETECTED: |pitch|=%.3f rad > threshold=%.3f → ESTOP\n",
                      state.pitch, _cfg.fall_threshold_rad);
        _lastState = out;
        return out;
    }

    // --- PD computation ---
    //
    // error: deviation from target (rad). Positive = leaning forward.
    //   - When robot leans forward, error > 0.
    //   - Controller should push ankles to correct (bring CoM back over feet).
    //
    // u_raw: raw control output in degrees (before clamping).
    //   - Units: degrees. Passed directly to setJointAngle() as joint-relative degrees.
    //   - Kp term: proportional to position error. Drives correction magnitude.
    //   - Kd term: proportional to pitch rate. Damps oscillation.
    //
    // correction_sign: empirically determined direction multiplier.
    //   +1.0 if positive pitch → positive joint correction restores upright.
    //   -1.0 if the above is backwards on your hardware.
    float error  = state.pitch - _cfg.pitch_setpoint_rad;
    float u_raw  = (_cfg.Kp * error + _cfg.Kd * state.pitchRate) * _cfg.correction_sign;

    // Hard clamp — prevents runaway correction due to bad Kp or sensor spike.
    float u_clamped = constrain(u_raw, -_cfg.max_correction_deg, _cfg.max_correction_deg);

    // --- Distribute to joints ---
    // ankle_cmd: joint-relative degrees sent to both ankle pitch joints.
    // hip_cmd:   joint-relative degrees sent to both hip pitch joints.
    //
    // JointModel::setJointAngle handles the direction field for each joint,
    // so the same signed value correctly mirrors left and right joints.
    float ankle_cmd = u_clamped * _cfg.ankle_ratio;
    float hip_cmd   = u_clamped * _cfg.hip_ratio;

    // --- Write to hardware ---
    _applyPitchCorrection(ankle_cmd, hip_cmd);

    // --- Fill output state for telemetry ---
    out.active        = true;
    out.pitch_error   = error;
    out.u_raw         = u_raw;
    out.u_clamped     = u_clamped;
    out.ankle_cmd_deg = ankle_cmd;
    out.hip_cmd_deg   = hip_cmd;
    out.fell          = false;

    _lastState = out;
    return out;
}

// ---------------------------------------------------------------------------
void BalanceController::_applyPitchCorrection(float ankle_deg, float hip_deg) {
    if (!_servo) return;

    // Ankle pitch joints — primary correction authority.
    // Both sides receive the same joint-relative angle; direction is handled internally.
    _servo->setJointAngleDirect(IDX_R_ANKLE_PITCH, ankle_deg);
    _servo->setJointAngleDirect(IDX_L_ANKLE_PITCH, ankle_deg);
    if (fabsf(hip_deg) > 0.01f) {
        _servo->setJointAngleDirect(IDX_R_HIP_PITCH, hip_deg);
        _servo->setJointAngleDirect(IDX_L_HIP_PITCH, hip_deg);
    }
}