// =============================================================================
// FILE:    foot_contact.h
// MODULE:  foot_contact
// LAYER:   3.5 — predicates (no state, no control)
//
// PURPOSE:
//   Pure functions that classify a foot's IMUState as flat/tilted and
//   loaded/unloaded. Designed so any future consumer (lift gate, voltage-
//   drift tool, telemetry display) reads from one source of truth.
//
// CONTRACT:
//   - Pure: same input -> same output, no internal state.
//   - Returns false on invalid IMUState. Callers do NOT re-check valid.
//   - Thresholds live in FootContactConfig. One instance per foot is
//     supported but in practice both feet share the same config.
//
// LAST CHANGED: 2026-05-13 | Hinz | F2 introduction
// =============================================================================

#ifndef FOOT_CONTACT_H
#define FOOT_CONTACT_H

#include <Arduino.h>
#include "state_estimator.h"  // IMUState
#include "side.h"             // Side / sideIdx — Phase 1 refactor

// ---------------------------------------------------------------------------
// FootContactConfig — all tuneables in one place. WS-tunable in F2.
// ---------------------------------------------------------------------------
struct FootContactConfig {

    // --- isFootFlat thresholds ---
    // Sole is "flat" when both |pitch| < this AND |roll| < this.
    // 0.035 rad ~= 2.0 deg. Loosen toward 0.05 rad if foot LPF is noisy
    // during in-place stepping; tighten toward 0.025 rad after F1 telemetry
    // confirms the noise floor at rest is < 0.5 deg.
    float flat_pitch_threshold_rad = 0.035f;
    float flat_roll_threshold_rad  = 0.05f;

    // --- isFootLoaded asymmetric threshold ---
    // foot pitch under load deviates from neutral by ~this magnitude due to
    // servo droop in the ankle pitch joint. Sign is determined empirically
    // in F1+F2 telemetry — see FootContactConfig::loaded_pitch_sign.
    //
    // Starting magnitude is conservative; tune from telemetry by recording
    // each foot's pitch at rest unloaded and pitch when a stance-leg pose
    // is commanded (single-leg stance from the UI). The delta is the
    // expected load signature.
    float loaded_pitch_deviation_rad = 0.025f;   // ~1.4 deg

    // --- Sign of expected deviation under load ---
    // +1.0: loaded foot pitches forward of unloaded neutral.
    // -1.0: loaded foot pitches backward of unloaded neutral.
    // Set empirically in F2 by observing pitch sign during single-leg stance
    // with the unloaded leg suspended. Do NOT derive mathematically; servo
    // droop direction is a property of the gearbox + neutral calibration.
    float loaded_pitch_sign = +1.0f;

    // --- Validity gate (shared by both predicates) ---
    // When IMUState.valid is false, both predicates return false regardless
    // of pitch / roll content. Kept here as a config field (rather than a
    // hardcoded check) so a future "trust stale state for N ticks" policy
    // can be added without touching the call sites.
    bool require_valid = true;
};

// ---------------------------------------------------------------------------
// isFootFlat
//   true iff |pitch| < cfg.flat_pitch_threshold_rad AND
//             |roll|  < cfg.flat_roll_threshold_rad AND
//             state.valid (when cfg.require_valid).
// ---------------------------------------------------------------------------
bool isFootFlat(const IMUState& state, const FootContactConfig& cfg);

// ---------------------------------------------------------------------------
// isFootLoaded
//   true iff (state.pitch - neutralPitchRad) * loaded_pitch_sign
//                                >= loaded_pitch_deviation_rad
//        AND state.valid (when cfg.require_valid).
//
//   neutralPitchRad is the operator-captured "unloaded neutral pitch" for
//   this foot. F4 (voltage-drift tool) will capture this at session start;
//   for F2 it is supplied by the caller and may be zero on first use.
// ---------------------------------------------------------------------------
bool isFootLoaded(const IMUState& state,
                  float neutralPitchRad,
                  const FootContactConfig& cfg);

// ---------------------------------------------------------------------------
// FootContactSnapshot - precomputed per-foot predicate output.
//
// Populated once per tick in main.cpp from the latest IMU states + neutral
// pitch values + FootContactConfig, then passed by const-pointer to consumers
// (GaitController for the lift gate; WebComm for telemetry). This guarantees:
//   1. UI display and gate evaluation see IDENTICAL booleans - what you see
//      on screen is the actual gate input.
//   2. GaitController has no direct dependency on FootContactConfig, IMUState
//      sign conventions, or per-foot neutral storage.
//   3. Unit testing the gate logic is trivial - construct a snapshot literal.
//
// `valid` distinguishes "we have meaningful data for both feet" from "one or
// both estimators are still calibrating / NACKing". A snapshot with valid=false
// is allowed downstream, but predicate consumers must treat the per-foot bools
// as untrusted.
// ---------------------------------------------------------------------------
struct FootContactSnapshot {
    // Phase 1 refactor: canonical representation is now arrays indexed by Side.
    // The reference members below alias the array entries so existing callers
    // that read `leftFlat` / `rightFlat` / etc. continue to compile unchanged.
    // Reference members make this struct non-aggregate and non-copy-assignable;
    // it is only constructed on the stack each tick (main.cpp) and passed by
    // const-pointer to consumers, so this is acceptable.
    bool flat[SIDE_COUNT]   = { false, false };
    bool loaded[SIDE_COUNT] = { false, false };
    bool valid[SIDE_COUNT]  = { false, false };

    // Aggregate validity — was `valid` (scalar) before Phase 1. Renamed because
    // `valid` is now the per-side array. Only writer (main.cpp) updated; no
    // external reader of the old aggregate scalar exists.
    bool allValid = false;

    // ── Backward-compat references — DO NOT add new uses ─────────────────────
    // Keep main.cpp / webcomm.cpp / gait_controller.cpp compiling unchanged
    // through Phase 1. Removed in Phase 7 alongside their callers.
    bool& leftFlat    = flat[sideIdx(Side::LEFT)];
    bool& leftLoaded  = loaded[sideIdx(Side::LEFT)];
    bool& leftValid   = valid[sideIdx(Side::LEFT)];
    bool& rightFlat   = flat[sideIdx(Side::RIGHT)];
    bool& rightLoaded = loaded[sideIdx(Side::RIGHT)];
    bool& rightValid  = valid[sideIdx(Side::RIGHT)];
};

#endif // FOOT_CONTACT_H
