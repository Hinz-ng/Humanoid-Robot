// =============================================================================
// FILE:    foot_impact.cpp
// MODULE:  Foot Impact Detection (per-foot)
// LAYER:   3.5
// SEE:     foot_impact.h for purpose, signal definition, and architectural
//          notes. Implementation only here.
//
// LAST CHANGED: 2026-05-18 | Hinz | Initial L0 implementation.
// =============================================================================
#include "foot_impact.h"
#include "project_wide_defs.h"
#include <math.h>

// -----------------------------------------------------------------------------
// clampConfig
//
// Defensive clamping for runtime-tunable fields. Operators tune live via
// WebSocket; an out-of-range value should not be allowed to silently
// disable detection or destabilise the filter. Public + static so the
// WebComm parser can pre-clamp a staged config at parse time (B1).
// -----------------------------------------------------------------------------
FootImpactConfig FootImpactDetector::clampConfig(FootImpactConfig c) {
    if (c.threshold_g < 0.05f) c.threshold_g = 0.05f;   // floor: noise-level
    if (c.threshold_g > 20.0f) c.threshold_g = 20.0f;   // ceiling: well above any plausible hit
    if (c.lpf_alpha   < 0.05f) c.lpf_alpha   = 0.05f;   // floor: don't fully freeze the LPF
    if (c.lpf_alpha   > 1.0f)  c.lpf_alpha   = 1.0f;    // ceiling: pass-through
    if (c.refractory_ms > 2000) c.refractory_ms = 2000; // sanity: max 2 s lockout
    return c;
}

// -----------------------------------------------------------------------------
// init / reset / setConfig
// -----------------------------------------------------------------------------
void FootImpactDetector::init() {
    // Load defaults from project_wide_defs.h.
    FootImpactConfig c;
    c.enabled       = FOOT_IMPACT_ENABLED_DEFAULT;
    c.threshold_g   = FOOT_IMPACT_THRESHOLD_G_DEFAULT;
    c.lpf_alpha     = FOOT_IMPACT_LPF_ALPHA_DEFAULT;
    c.refractory_ms = FOOT_IMPACT_REFRACTORY_MS_DEFAULT;
    setConfig(c);
    reset();
}

void FootImpactDetector::reset() {
    for (int i = 0; i < SIDE_COUNT; ++i) {
        _state[i] = FootImpactSideState{};
        _priv[i]  = PrivSide{};
    }
}

void FootImpactDetector::setConfig(const FootImpactConfig& c) {
    _cfg = clampConfig(c);
}

// -----------------------------------------------------------------------------
// update — two-side dispatch.
//
// Called once per 400 Hz tick from main.cpp, AFTER foot IMU axis remap and
// either before or in parallel with the foot StateEstimator updates. Order
// versus the estimator does not matter — they consume the same RawIMUData
// but produce independent outputs.
// -----------------------------------------------------------------------------
void FootImpactDetector::update(const RawIMUData& lf, const RawIMUData& rf) {
    const uint32_t now_ms = millis();
    _updateSide(Side::LEFT,  lf, now_ms);
    _updateSide(Side::RIGHT, rf, now_ms);
}

// -----------------------------------------------------------------------------
// _updateSide — core per-side signal pipeline.
//
// Steps:
//   1. Carry forward the previous detected=false (sticky lasts one tick only).
//   2. If raw is invalid: hold magnitudes, armed=false, return.
//   3. Compute current accel magnitude in g.
//   4. Seed LPF on first valid sample (no delta available yet).
//   5. LPF the magnitude.
//   6. Compute delta as first difference of LPF output.
//   7. Compute armed: enabled && seeded && refractory expired.
//   8. If armed && |delta| > threshold: raise detected, record timestamp,
//      latch last_magnitude/last_delta, advance refractory.
// -----------------------------------------------------------------------------
void FootImpactDetector::_updateSide(Side s, const RawIMUData& raw, uint32_t now_ms) {
    const int idx = sideIdx(s);
    FootImpactSideState& st = _state[idx];
    PrivSide&            pv = _priv[idx];

    // Step 1: clear sticky detected.
    st.detected = false;

    // Step 2: mirror raw validity. Hold filter state untouched if invalid
    // so we don't propagate garbage into the LPF when readings resume.
    st.valid = raw.valid;
    if (!raw.valid) {
        st.armed = false;
        // Note: ms_since_last continues to advance below.
        if (pv.last_detect_ms > 0) st.ms_since_last = now_ms - pv.last_detect_ms;
        return;
    }

    // Step 3: |a| in g.
    //   ax/ay/az are int16 raw counts in robot frame (post-remap by main.cpp).
    //   FOOT_IMU_ACCEL_SCALE_LSB_PER_G is defined in project_wide_defs.h
    //   and depends on the LSM6DSV full-scale setting (currently +/-2 g).
    const float ax_g = static_cast<float>(raw.accel_x) / FOOT_IMU_ACCEL_SCALE_LSB_PER_G;
    const float ay_g = static_cast<float>(raw.accel_y) / FOOT_IMU_ACCEL_SCALE_LSB_PER_G;
    const float az_g = static_cast<float>(raw.accel_z) / FOOT_IMU_ACCEL_SCALE_LSB_PER_G;
    const float a_mag_g = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);

    // Step 4: seed on first valid sample.
    if (!pv.seeded) {
        pv.a_mag_filt_g      = a_mag_g;
        pv.prev_a_mag_filt_g = a_mag_g;
        pv.seeded            = true;
        st.magnitude_g       = a_mag_g;
        st.delta_g           = 0.0f;
        st.armed             = false; // not armed on the seed tick
        return;
    }

    // Step 5: LPF.
    pv.prev_a_mag_filt_g = pv.a_mag_filt_g;
    pv.a_mag_filt_g      = _cfg.lpf_alpha * a_mag_g
                         + (1.0f - _cfg.lpf_alpha) * pv.a_mag_filt_g;

    // Step 6: delta.
    const float delta_g = pv.a_mag_filt_g - pv.prev_a_mag_filt_g;

    // Step 7: armed test.
    const bool refractory_active = (now_ms < pv.refractory_until_ms);
    st.armed = _cfg.enabled && pv.seeded && !refractory_active;

    // Publish continuous fields.
    st.magnitude_g    = pv.a_mag_filt_g;
    st.delta_g        = delta_g;
    st.ms_since_last  = (pv.last_detect_ms == 0) ? 0 : (now_ms - pv.last_detect_ms);

    // Step 8: threshold check.
    if (st.armed && fabsf(delta_g) > _cfg.threshold_g) {
        st.detected           = true;
        st.detect_count++;
        st.last_magnitude_g   = pv.a_mag_filt_g;
        st.last_delta_g       = delta_g;
        pv.last_detect_ms     = now_ms;
        pv.refractory_until_ms = now_ms + _cfg.refractory_ms;
        st.ms_since_last      = 0;
    }
}
