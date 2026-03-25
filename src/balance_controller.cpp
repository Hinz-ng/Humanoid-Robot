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
#include "motion_manager.h"    // MotionSource, MotionManager::submit()

// ---------------------------------------------------------------------------
BalanceController::BalanceController(ServoControl* servo)
    : _servo(servo), _cfg(), _lastState() {}

// ---------------------------------------------------------------------------
void BalanceController::init() {
    _lastState = {};
    // Log the starting config so it's visible in Serial on boot.
    Serial.printf("[BalanceController] Init. pitch_en=%s  roll_en=%s  Kp=%.1f  Kd=%.2f  sp=%.3f rad\n",
                  _cfg.pitch_enabled ? "true" : "false",
                  _cfg.roll_enabled  ? "true" : "false",
                  _cfg.Kp, _cfg.Kd, _cfg.pitch_setpoint_rad);
}

// ---------------------------------------------------------------------------
BalanceState BalanceController::update(const IMUState& state) {
    BalanceState out = {};   // zero-initialized; all fields false/0 by default

    // --- Gate 1: at least one controller must be enabled ---
    // Both pitch and roll are off by default. Enable independently via
    // CMD:PITCH_ON / CMD:ROLL_ON after verifying sign conventions on hardware.
    if (!_cfg.pitch_enabled && !_cfg.roll_enabled) {
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

    // --- Fall detection (pitch OR roll exceeding threshold triggers ESTOP) ---
    // Covers both axes: if the robot falls forward, backward, or laterally,
    // it is past recoverable range. Torque-holding against the floor damages servos.
    if (fabsf(state.pitch) > _cfg.fall_threshold_rad ||
        fabsf(state.roll)  > _cfg.fall_threshold_rad) {
        oe_estop();
        // Reset servo state while OE is HIGH so no stale commands apply on re-enable.
        if (_servo != nullptr) {
            _servo->resetToNeutral();
        }
        out.fell = true;
        Serial.printf("[BalanceController] FALL DETECTED: pitch=%.3f roll=%.3f rad  threshold=%.3f → ESTOP\n",
                      state.pitch, state.roll, _cfg.fall_threshold_rad);
        _lastState = out;
        return out;
    }

    // =========================================================================
    //  PITCH PD (sagittal plane — forward/back)
    //  error: positive = leaning forward.
    //  correction_sign: +1 if positive error → positive ankle command corrects.
    //                   Verify empirically before first enable.
    // =========================================================================
    if (_cfg.pitch_enabled) {
        float error     = state.pitch - _cfg.pitch_setpoint_rad;
        float u_raw     = (_cfg.Kp * error + _cfg.Kd * state.pitchRate) * _cfg.correction_sign;
        float u_clamped = constrain(u_raw, -_cfg.max_correction_deg, _cfg.max_correction_deg);

        float ankle_cmd = u_clamped * _cfg.ankle_ratio;
        float hip_cmd   = u_clamped * _cfg.hip_ratio;
        float torso_cmd = u_clamped * _cfg.torso_ratio;

        _applyPitchCorrection(ankle_cmd, hip_cmd, torso_cmd);

        out.pitch_active  = true;
        out.pitch_error   = error;
        out.u_raw         = u_raw;
        out.u_clamped     = u_clamped;
        out.ankle_cmd_deg = ankle_cmd;
        out.hip_cmd_deg   = hip_cmd;
        out.torso_cmd_deg = torso_cmd;
    }

    // =========================================================================
    //  ROLL PD (frontal plane — left/right)
    //  error: positive = leaning right (verify sign on hardware before enabling).
    //  roll_correction_sign: verify empirically — tilt right, observe roll in
    //  telemetry, confirm ankle roll correction pushes back toward upright.
    //  max_roll_correction_deg: starts at 8° — tighter than pitch (5.6° margin).
    // =========================================================================
    if (_cfg.roll_enabled) {
        float roll_error     = state.roll - _cfg.roll_setpoint_rad;
        float u_roll_raw     = (_cfg.Kp_roll * roll_error + _cfg.Kd_roll * state.rollRate)
                               * _cfg.roll_correction_sign;
        float u_roll_clamped = constrain(u_roll_raw,
                                         -_cfg.max_roll_correction_deg,
                                          _cfg.max_roll_correction_deg);

        float ankle_roll_cmd = u_roll_clamped * _cfg.ankle_roll_ratio;
        float hip_roll_cmd   = u_roll_clamped * _cfg.hip_roll_ratio;
        float torso_roll_cmd = u_roll_clamped * _cfg.torso_roll_ratio;

        _applyRollCorrection(ankle_roll_cmd, hip_roll_cmd, torso_roll_cmd);

        out.roll_active        = true;
        out.roll_error         = roll_error;
        out.u_roll_raw         = u_roll_raw;
        out.u_roll_clamped     = u_roll_clamped;
        out.ankle_roll_cmd_deg = ankle_roll_cmd;
        out.hip_roll_cmd_deg   = hip_roll_cmd;
        out.torso_roll_cmd_deg = torso_roll_cmd;
    }

    // active = true if any controller ran this tick — maintains backward compat
    // for downstream code (telemetry, UI) that checks state.active.
    out.active = out.pitch_active || out.roll_active;
    out.fell   = false;

    _lastState = out;
    return out;
}

// ---------------------------------------------------------------------------
void BalanceController::_applyPitchCorrection(float ankle_deg, float hip_deg, float torso_deg) {
    // ── Path A: MotionManager is wired (normal runtime path) ─────────────────
    if (_motionManager) {
        _motionManager->submit(SOURCE_BALANCE, IDX_R_ANKLE_PITCH, ankle_deg);
        _motionManager->submit(SOURCE_BALANCE, IDX_L_ANKLE_PITCH, ankle_deg);
        if (fabsf(hip_deg) > 0.01f) {
            _motionManager->submit(SOURCE_BALANCE, IDX_R_HIP_PITCH, hip_deg);
            _motionManager->submit(SOURCE_BALANCE, IDX_L_HIP_PITCH, hip_deg);
        }
        // Torso pitch is a single joint (not bilateral like ankle/hip).
        // Sign is negated: the torso pitch servo mechanical direction is opposite
        // to ankle/hip for the same correction intent. correction_sign handles the
        // global ankle/hip direction; the torso requires an additional internal flip.
        // Do NOT remove this negation without re-verifying torso direction on hardware.
        if (fabsf(torso_deg) > 0.01f) {
            _motionManager->submit(SOURCE_BALANCE, IDX_TORSO_PITCH, -torso_deg);
        }
        return;
    }

    // ── Path B: MotionManager absent — direct write fallback ─────────────────
    if (!_servo) return;
    Serial.println("[BalanceController] WARNING: MotionManager not wired — "
                   "using direct servo write. Wire setMotionManager() in setup().");
    _servo->setJointAngleDirect(IDX_R_ANKLE_PITCH, ankle_deg);
    _servo->setJointAngleDirect(IDX_L_ANKLE_PITCH, ankle_deg);
    if (fabsf(hip_deg) > 0.01f) {
        _servo->setJointAngleDirect(IDX_R_HIP_PITCH, hip_deg);
        _servo->setJointAngleDirect(IDX_L_HIP_PITCH, hip_deg);
    }
    // Negated for same reason as MotionManager path — see comment above.
    if (fabsf(torso_deg) > 0.01f) {
        _servo->setJointAngleDirect(IDX_TORSO_PITCH, -torso_deg);
    }
}

// ---------------------------------------------------------------------------
void BalanceController::_applyRollCorrection(float ankle_deg, float hip_deg, float torso_deg) {
    // Mirrors _applyPitchCorrection() exactly — same two-path structure.
    //
    // JointModel handles direction inversion per channel, so submitting the
    // same signed angleDeg to both left and right roll joints produces
    // anatomically symmetric corrections automatically:
    //   IDX_R_ANKLE_ROLL: direction +1 → positive cmd = rightward tilt correction
    //   IDX_L_ANKLE_ROLL: direction -1 → same signed cmd mirrors correctly to left

    // ── Path A: MotionManager wired (normal runtime path) ────────────────────
    if (_motionManager) {
        _motionManager->submit(SOURCE_BALANCE, IDX_R_ANKLE_ROLL, ankle_deg);
        _motionManager->submit(SOURCE_BALANCE, IDX_L_ANKLE_ROLL, ankle_deg);
        if (fabsf(hip_deg) > 0.01f) {
            _motionManager->submit(SOURCE_BALANCE, IDX_R_HIP_ROLL, hip_deg);
            _motionManager->submit(SOURCE_BALANCE, IDX_L_HIP_ROLL, hip_deg);
        }
        // Torso roll: single joint. Guard matches pitch convention.
        // Direction is currently assumed +1 (not yet verified on hardware).
        // If correction worsens the lean, set roll_correction_sign = -1 from UI.
        if (fabsf(torso_deg) > 0.01f) {
            _motionManager->submit(SOURCE_BALANCE, IDX_TORSO_ROLL, torso_deg);
        }
        return;
    }

    // ── Path B: MotionManager absent — direct write fallback ─────────────────
    if (!_servo) return;
    Serial.println("[BalanceController] WARNING: MotionManager not wired — "
                   "using direct servo write for roll. Wire setMotionManager() in setup().");
    _servo->setJointAngleDirect(IDX_R_ANKLE_ROLL, ankle_deg);
    _servo->setJointAngleDirect(IDX_L_ANKLE_ROLL, ankle_deg);
    if (fabsf(hip_deg) > 0.01f) {
        _servo->setJointAngleDirect(IDX_R_HIP_ROLL, hip_deg);
        _servo->setJointAngleDirect(IDX_L_HIP_ROLL, hip_deg);
    }
    if (fabsf(torso_deg) > 0.01f) {
        _servo->setJointAngleDirect(IDX_TORSO_ROLL, torso_deg);
    }
}