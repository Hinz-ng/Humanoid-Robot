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

    // 5. Apply deadband then compute P output × envelope.
    //    This is the "would-be" bias — always published for L0 telemetry
    //    so the operator can verify sign and magnitude before L1.
    float p_err = _pitch_lpf_rad;
    float r_err = _roll_lpf_rad;
    if (fabsf(p_err) < _cfg.deadband_rad) p_err = 0.0f;
    if (fabsf(r_err) < _cfg.deadband_rad) r_err = 0.0f;

    // Negative sign: foot tilts +pitch (forward) → ankle pitches backward
    // (dorsiflexion) to push sole flat. Verify on hardware in L0 telemetry.
    const float raw_p = -_cfg.kp_pitch_deg_per_rad * p_err;
    const float raw_r = -_cfg.kp_roll_deg_per_rad  * r_err;

    // Would-be value = P × envelope, clamped. Stored for telemetry.
    const float env_scale = _state.envelope_scale;
    float would_be_p = raw_p * env_scale;
    float would_be_r = raw_r * env_scale;
    would_be_p = constrain(would_be_p, -_cfg.max_pitch_bias_deg, +_cfg.max_pitch_bias_deg);
    would_be_r = constrain(would_be_r, -_cfg.max_roll_bias_deg,  +_cfg.max_roll_bias_deg);
    _state.ankle_pitch_bias_deg = would_be_p;
    _state.ankle_roll_bias_deg  = would_be_r;

    // 6. IIR: feed the would-be value when enabled; feed 0 when disabled so
    //    the output decays smoothly to zero. Decay also happens naturally when
    //    envelope_scale = 0 (would_be = 0 regardless of enabled).
    const float iir_input_p = _cfg.enabled ? would_be_p : 0.0f;
    const float iir_input_r = _cfg.enabled ? would_be_r : 0.0f;
    const float oAlpha      = _cfg.output_iir_alpha;
    _pitch_bias_iir_deg = oAlpha * _pitch_bias_iir_deg + (1.0f - oAlpha) * iir_input_p;
    _roll_bias_iir_deg  = oAlpha * _roll_bias_iir_deg  + (1.0f - oAlpha) * iir_input_r;

    // 7. Publish applied values and armed flag.
    _state.applied_pitch_bias_deg = _pitch_bias_iir_deg;
    _state.applied_roll_bias_deg  = _roll_bias_iir_deg;
    _state.armed = _cfg.enabled && footState.valid && env_scale > 0.0f;
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
