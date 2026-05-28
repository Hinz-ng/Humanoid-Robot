// =============================================================================
// FILE:    torso_stabilizer.cpp
// MODULE:  control
// LAYER:   3 — Torso Stabilizer (joint-space)
//
// Standalone torso pitch/roll PD. See torso_stabilizer.h for the control-law
// rationale (PD, no integral).
// =============================================================================

#include "torso_stabilizer.h"
#include "motion_manager.h"
#include "joint_config.h"   // IDX_TORSO_PITCH / IDX_TORSO_ROLL
#include "oe_control.h"     // oe_is_estopped()
#include <math.h>

// ---------------------------------------------------------------------------
void TorsoStabilizer::resetOutputState() {
    _pitchShape        = {};
    _rollShape         = {};
    _pitchRateFiltered = 0.0f;
    _rollRateFiltered  = 0.0f;
    _state             = {};
}

// ---------------------------------------------------------------------------
//  _computePD — single-axis PD with derivative LPF, deadband, and clamp.
//  No gain-boost, no servo-lag predictor (lean PD). Returns clamped cmd (deg).
// ---------------------------------------------------------------------------
float TorsoStabilizer::_computePD(float angle, float setpoint, float angleRate,
                                   float& rateFilt, float lpfAlpha, float deadband,
                                   float kp, float kd, float sign, float maxCorr,
                                   float& errorOut) const {
    rateFilt = lpfAlpha * rateFilt + (1.0f - lpfAlpha) * angleRate;
    const float error = angle - setpoint;
    errorOut = error;
    if (fabsf(error) < deadband) return 0.0f;
    const float u_raw = (kp * error + kd * rateFilt) * sign;
    return constrain(u_raw, -maxCorr, maxCorr);
}

// ---------------------------------------------------------------------------
//  _shape — output shaping pipeline:
//    rawCmd → [zero bypass | delta clamp] → IIR → velocity damp → final clamp
// ---------------------------------------------------------------------------
float TorsoStabilizer::_shape(TorsoShapeState& s, float rawCmd,
                               float angularRate_rs) {
    float clamped;
    if (fabsf(rawCmd) < 1e-4f) {
        clamped = 0.0f;  // zero input → free IIR decay
    } else {
        clamped = constrain(rawCmd,
                            s.filtered_cmd_deg - _cfg.max_output_rate_deg_per_tick,
                            s.filtered_cmd_deg + _cfg.max_output_rate_deg_per_tick);
    }
    float smoothed = _cfg.output_iir_alpha * s.filtered_cmd_deg
                   + (1.0f - _cfg.output_iir_alpha) * clamped;
    s.prev_cmd_deg     = clamped;
    s.filtered_cmd_deg = smoothed;
    float damped = smoothed - _cfg.output_damping_kv * angularRate_rs;
    return constrain(damped, -_cfg.max_output_deg, _cfg.max_output_deg);
}

// ---------------------------------------------------------------------------
void TorsoStabilizer::update(const IMUState& state) {
    _state = {};
    if (_mm == nullptr || !state.valid || oe_is_estopped()) return;

    // Torso pitch
    if (_cfg.pitch_enabled) {
        float err;
        float u = _computePD(state.pitch, _cfg.pitch_setpoint_rad, state.pitchRate,
                             _pitchRateFiltered, _cfg.derivative_lpf_alpha,
                             _cfg.pitch_deadband_rad, _cfg.Kp, _cfg.Kd,
                             _cfg.correction_sign, _cfg.max_correction_deg, err);
        float shaped = _shape(_pitchShape, u, _pitchRateFiltered);
        _mm->submit(SOURCE_BALANCE, IDX_TORSO_PITCH, shaped);
        _state.pitch_error    = err;
        _state.pitch_u_clamped = u;
        _state.pitch_cmd_deg  = shaped;
        _state.pitch_active   = true;
    }

    // Torso roll
    if (_cfg.roll_enabled) {
        float err;
        float u = _computePD(state.roll, _cfg.roll_setpoint_rad, state.rollRate,
                             _rollRateFiltered, _cfg.roll_derivative_lpf_alpha,
                             _cfg.roll_deadband_rad, _cfg.Kp_roll, _cfg.Kd_roll,
                             _cfg.roll_correction_sign, _cfg.max_roll_correction_deg, err);
        float shaped = _shape(_rollShape, u, _rollRateFiltered);
        _mm->submit(SOURCE_BALANCE, IDX_TORSO_ROLL, shaped);
        _state.roll_error    = err;
        _state.roll_u_clamped = u;
        _state.roll_cmd_deg  = shaped;
        _state.roll_active   = true;
    }
}
