// =============================================================================
// FILE:    torso_stabilizer.cpp
// MODULE:  control
// LAYER:   3 — Torso Stabilizer (joint-space)
// =============================================================================

#include "torso_stabilizer.h"
#include "motion_manager.h"
#include "joint_config.h"   // IDX_TORSO_PITCH / IDX_TORSO_ROLL
#include <math.h>

// ---------------------------------------------------------------------------
void TorsoStabilizer::resetOutputState() {
    _pitchShape = {};
    _rollShape  = {};
    _state      = {};
}

// ---------------------------------------------------------------------------
//  _shape() — output shaping pipeline.
//
//  Replicated from BalanceController::_shapeOutput() so the torso joint
//  response is identical to the pre-refactor joint-space torso path:
//    rawCmd → [zero bypass | delta clamp] → IIR → velocity damp → final clamp
// ---------------------------------------------------------------------------
float TorsoStabilizer::_shape(JointOutputState& s, float rawCmd,
                               float angularRate_rs) {
    float clamped;
    if (fabsf(rawCmd) < 1e-4f) {
        // Zero input → free IIR decay (BUG-01 zero-bypass parity).
        clamped = 0.0f;
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
void TorsoStabilizer::update(const BalancePDOutput& pd) {
    _state = {};
    if (_mm == nullptr || !pd.valid) return;

    // Torso pitch — negated so it pushes the CoM opposite to the leg
    // correction (parity with the pre-refactor BalanceController torso path:
    // negation applied before shaping so the IIR tracks mechanical direction).
    if (_cfg.enabled && pd.pitch_active && _cfg.pitch_ratio > 0.001f) {
        float cmd    = pd.pitch_u_clamped * _cfg.pitch_ratio;
        float shaped = _shape(_pitchShape, -cmd, pd.pitch_rate_filt);
        _mm->submit(SOURCE_BALANCE, IDX_TORSO_PITCH, shaped);
        _state.pitch_cmd_deg = shaped;
        _state.pitch_active  = true;
    }
    // Torso roll — negated so the torso leans opposite to the leg correction
    // (mechanical direction inversion vs. the pre-refactor path).
    if (_cfg.enabled && pd.roll_active && _cfg.roll_ratio > 0.001f) {
        float cmd    = pd.roll_u_clamped * _cfg.roll_ratio;
        float shaped = _shape(_rollShape, -cmd, pd.roll_rate_filt);
        _mm->submit(SOURCE_BALANCE, IDX_TORSO_ROLL, shaped);
        _state.roll_cmd_deg = shaped;
        _state.roll_active  = true;
    }
}
