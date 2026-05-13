// =============================================================================
// FILE:    foot_contact.cpp
// LAYER:   3.5 — predicates
// LAST CHANGED: 2026-05-13 | Hinz | F2 introduction
// =============================================================================

#include "foot_contact.h"
#include <math.h>

bool isFootFlat(const IMUState& state, const FootContactConfig& cfg) {
    if (cfg.require_valid && !state.valid) return false;
    return (fabsf(state.pitch) < cfg.flat_pitch_threshold_rad)
        && (fabsf(state.roll)  < cfg.flat_roll_threshold_rad);
}

bool isFootLoaded(const IMUState& state,
                  float neutralPitchRad,
                  const FootContactConfig& cfg) {
    if (cfg.require_valid && !state.valid) return false;
    const float delta_rad = (state.pitch - neutralPitchRad) * cfg.loaded_pitch_sign;
    return delta_rad >= cfg.loaded_pitch_deviation_rad;
}