// =============================================================================
// FILE:    foot_contact.cpp
// LAYER:   3.5 — predicates
// LAST CHANGED: 2026-05-13 | Hinz | F2 introduction
// =============================================================================

#include "foot_contact.h"
#include <math.h>

bool isFootFlat(const IMUState& state,
                float neutralPitchRad,
                float neutralRollRad,
                const FootContactConfig& cfg) {
    if (cfg.require_valid && !state.valid) return false;
    return (fabsf(state.pitch - neutralPitchRad) < cfg.flat_pitch_threshold_rad)
        && (fabsf(state.roll  - neutralRollRad)  < cfg.flat_roll_threshold_rad);
}

bool isFootFlat(const IMUState& state, const FootContactConfig& cfg) {
    // Zero-neutral form preserved for existing (control-path) callers — bit
    // identical to the pre-tare behaviour. The neutral-aware overload above is
    // used by telemetry so the gate snapshot and on-screen booleans stay equal.
    return isFootFlat(state, 0.0f, 0.0f, cfg);
}

bool isFootLoaded(const IMUState& state,
                  float neutralPitchRad,
                  const FootContactConfig& cfg) {
    if (cfg.require_valid && !state.valid) return false;
    const float delta_rad = (state.pitch - neutralPitchRad) * cfg.loaded_pitch_sign;
    return delta_rad >= cfg.loaded_pitch_deviation_rad;
}