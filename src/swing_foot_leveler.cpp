// =============================================================================
// FILE:    swing_foot_leveler.cpp
// MODULE:  swing_foot_leveler
// LAYER:   3.5 — peer of WeightShift, FootContact
// LAST CHANGED: 2026-05-16 | Hinz | Phase L0 introduction
// =============================================================================

#include "swing_foot_leveler.h"
#include <math.h>

// ---------------------------------------------------------------------------
void SwingFootLeveler::init(const StateEstimator* leftFootEst,
                             const StateEstimator* rightFootEst) {
    _footEst[sideIdx(Side::LEFT)]  = leftFootEst;
    _footEst[sideIdx(Side::RIGHT)] = rightFootEst;
    reset();
}

// ---------------------------------------------------------------------------
void SwingFootLeveler::reset() {
    _pitch_lpf_rad      = 0.0f;
    _roll_lpf_rad       = 0.0f;
    _pitch_bias_iir_deg = 0.0f;
    _roll_bias_iir_deg  = 0.0f;
    _seeded             = false;

    _state.armed                  = false;
    _state.foot_pitch_rad         = 0.0f;
    _state.foot_roll_rad          = 0.0f;
    _state.ankle_pitch_bias_deg   = 0.0f;
    _state.ankle_roll_bias_deg    = 0.0f;
    _state.envelope_scale         = 0.0f;
    _state.applied_pitch_bias_deg = 0.0f;
    _state.applied_roll_bias_deg  = 0.0f;
    _state.foot_imu_valid         = false;
}

// ---------------------------------------------------------------------------
//  _envelope — smoothstep-shaped gain from liftFrac.
//
//  Returns [0..1] based on where the swing arc is:
//    before engage_lift_frac  → 0
//    engage → peak            → smoothstep ramp up to 1
//    peak   → disengage       → smoothstep ramp down to 0
//    after disengage_lift_frac → 0
//
//  Smoothstep (3t² − 2t³) gives zero first-derivative at both endpoints,
//  so the bias ramps in and out without an abrupt step.
// ---------------------------------------------------------------------------
float SwingFootLeveler::_envelope(float lf, const SwingFootLevelerConfig& cfg) {
    if (lf <= cfg.engage_lift_frac)      return 0.0f;
    if (lf >= cfg.disengage_lift_frac)   return 0.0f;
    if (lf <= cfg.peak_lift_frac) {
        const float t = (lf - cfg.engage_lift_frac)
                      / (cfg.peak_lift_frac - cfg.engage_lift_frac);
        return 3.0f*t*t - 2.0f*t*t*t;   // smoothstep
    }
    // Ramp down from peak to disengage.
    const float t = (lf - cfg.peak_lift_frac)
                  / (cfg.disengage_lift_frac - cfg.peak_lift_frac);
    return 1.0f - (3.0f*t*t - 2.0f*t*t*t);
}

// ---------------------------------------------------------------------------
//  update — main per-tick computation.
//
//  Always runs the full pipeline (LPF → P → envelope → clamp → IIR)
//  regardless of the enabled flag. The enabled flag only gates the IIR
//  accumulation: when disabled, the IIR is fed 0 so it decays smoothly to
//  zero (and anklePitchBiasFor() returns 0). This means:
//    - _state.ankle_pitch/roll_bias_deg (would-be) is non-zero in L0 —
//      the operator can verify sign and magnitude without actuating.
//    - _state.applied_pitch/roll_bias_deg is always 0 in L0.
// ---------------------------------------------------------------------------
void SwingFootLeveler::update(Side swingSide, float liftFrac) {
    // 1. Reset IIR/LPF state on side change.
    if (_seeded && swingSide != _lastSide) {
        reset();
    }
    _lastSide          = swingSide;
    _state.activeSide  = swingSide;

    // 2. Read swing-foot IMU.
    const size_t idx = sideIdx(swingSide);
    if (_footEst[idx] == nullptr) {
        _state.foot_imu_valid         = false;
        _state.ankle_pitch_bias_deg   = 0.0f;
        _state.ankle_roll_bias_deg    = 0.0f;
        _state.applied_pitch_bias_deg = 0.0f;
        _state.applied_roll_bias_deg  = 0.0f;
        _state.armed                  = false;
        return;
    }
    const IMUState& footState = _footEst[idx]->getState();
    _state.foot_imu_valid = footState.valid;

    // 3. Compute envelope (always, even if IMU is invalid — needed for decay).
    _state.envelope_scale = _envelope(liftFrac, _cfg);

    // 4. Apply input LPF. Seed on first valid sample to avoid an initial spike.
    if (footState.valid) {
        if (!_seeded) {
            _pitch_lpf_rad = footState.pitch;
            _roll_lpf_rad  = footState.roll;
            _seeded        = true;
        } else {
            const float a  = _cfg.input_lpf_alpha;
            _pitch_lpf_rad = a * _pitch_lpf_rad + (1.0f - a) * footState.pitch;
            _roll_lpf_rad  = a * _roll_lpf_rad  + (1.0f - a) * footState.roll;
        }
    }
    _state.foot_pitch_rad = _pitch_lpf_rad;
    _state.foot_roll_rad  = _roll_lpf_rad;

    // 5. Apply deadband then compute raw P output.
    //    NOTE on semantics (changed from initial L0 draft):
    //    - "would-be" telemetry below is PRE-envelope, PRE-IIR, PRE-enable.
    //      It is non-zero whenever the foot has tilt beyond the deadband,
    //      regardless of swing phase or enabled state. This enables bench-test
    //      sign/magnitude verification by tilting the foot in-hand without
    //      having to run a gait cycle.
    //    - "applied" path further down is POST-envelope, POST-IIR, gated by
    //      enabled. Zero outside SWINGING or when disabled.
    float p_err = _pitch_lpf_rad;
    float r_err = _roll_lpf_rad;
    if (fabsf(p_err) < _cfg.deadband_rad) p_err = 0.0f;
    if (fabsf(r_err) < _cfg.deadband_rad) r_err = 0.0f;

    // Negative sign: foot tilts +pitch (forward) → ankle dorsiflexes (-bias)
    // to push sole flat. Verify on hardware in L0 telemetry by tilting foot.
    const float raw_p = -_cfg.kp_pitch_deg_per_rad * p_err;
    const float raw_r = -_cfg.kp_roll_deg_per_rad  * r_err;

    // Would-be: clamped, pre-envelope, pre-IIR. Stored for telemetry.
    const float clamped_p = constrain(raw_p, -_cfg.max_pitch_bias_deg, +_cfg.max_pitch_bias_deg);
    const float clamped_r = constrain(raw_r, -_cfg.max_roll_bias_deg,  +_cfg.max_roll_bias_deg);
    _state.ankle_pitch_bias_deg = clamped_p;
    _state.ankle_roll_bias_deg  = clamped_r;

    // 6. Apply envelope and enable gate to produce IIR input.
    //    env_scale=0 outside SWINGING -> IIR fed 0 -> applied decays to 0.
    //    enabled=false -> IIR fed 0 -> applied decays to 0 -> getter returns 0.
    const float env_scale   = _state.envelope_scale;
    const float gated_p     = clamped_p * env_scale;
    const float gated_r     = clamped_r * env_scale;
    const float iir_input_p = _cfg.enabled ? gated_p : 0.0f;
    const float iir_input_r = _cfg.enabled ? gated_r : 0.0f;
    const float oAlpha      = _cfg.output_iir_alpha;
    _pitch_bias_iir_deg = oAlpha * _pitch_bias_iir_deg + (1.0f - oAlpha) * iir_input_p;
    _roll_bias_iir_deg  = oAlpha * _roll_bias_iir_deg  + (1.0f - oAlpha) * iir_input_r;

    // 7. Publish applied values + armed flag.
    //    armed = enabled + IMU valid. Decoupled from envelope so the UI badge
    //    distinguishes "controller is on" from "currently actuating".
    //    env_scale (published separately as lvl_env) provides actuating-now
    //    visual feedback. This pairing solves the L0 verifiability problem.
    _state.applied_pitch_bias_deg = _pitch_bias_iir_deg;
    _state.applied_roll_bias_deg  = _roll_bias_iir_deg;
    _state.armed = _cfg.enabled && footState.valid;
}

// ---------------------------------------------------------------------------
float SwingFootLeveler::anklePitchBiasFor(Side side) const {
    if (!_cfg.enabled)             return 0.0f;
    if (!_state.foot_imu_valid)    return 0.0f;
    if (_state.envelope_scale <= 0.0f) return 0.0f;
    if (side != _state.activeSide) return 0.0f;
    return _state.applied_pitch_bias_deg;
}

// ---------------------------------------------------------------------------
float SwingFootLeveler::ankleRollBiasFor(Side side) const {
    if (!_cfg.enabled)             return 0.0f;
    if (!_state.foot_imu_valid)    return 0.0f;
    if (_state.envelope_scale <= 0.0f) return 0.0f;
    if (side != _state.activeSide) return 0.0f;
    return _state.applied_roll_bias_deg;
}
