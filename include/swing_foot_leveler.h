// =============================================================================
// FILE:    swing_foot_leveler.h
// MODULE:  swing_foot_leveler
// LAYER:   3.5 — peer of WeightShift, FootContact
//
// PURPOSE:
//   Closed-loop P controller that drives the airborne foot's sole toward
//   horizontal during swing phase using the per-foot LSM6DSV IMU.
//   Emits an additive ankle pitch/roll bias on top of the IK-computed
//   flat-foot output — never replaces the IK.
//
// ROLLOUT PHASES:
//   L0 (current) — enabled=false; controller computes but does not actuate.
//                  Telemetry shows would-be corrections for sign/magnitude
//                  verification before any servo movement.
//   L1           — enabled via CMD:SWING_LEVELER_TUNE:enabled=1.
//   L2           — reduce stepHeightMm toward 12 mm after L1 validation.
//
// LAST CHANGED: 2026-05-16 | Hinz | Phase L0 introduction
// =============================================================================

#ifndef SWING_FOOT_LEVELER_H
#define SWING_FOOT_LEVELER_H

#include <Arduino.h>
#include "state_estimator.h"   // IMUState
#include "side.h"              // Side, sideIdx, SIDE_COUNT
#include "project_wide_defs.h" // SWING_LEVELER_* constants

// ---------------------------------------------------------------------------
// SwingFootLevelerConfig — all runtime-tunable parameters.
// Defaults mirror the Section 14 constants in project_wide_defs.h.
// Override at runtime via CMD:SWING_LEVELER_TUNE over WebSocket.
// ---------------------------------------------------------------------------
struct SwingFootLevelerConfig {
    // Master enable. Default OFF — L0 is passive (telemetry only).
    bool  enabled = false;

    // Proportional gain: ankle bias = -kp * foot_angle_rad.
    // Foot tilts forward (+pitch rad) → ankle dorsiflexes (−bias deg).
    // Verify sign on hardware in L0 telemetry before enabling in L1.
    float kp_pitch_deg_per_rad = SWING_LEVELER_KP_PITCH_DEG_PER_RAD;
    float kp_roll_deg_per_rad  = SWING_LEVELER_KP_ROLL_DEG_PER_RAD;

    // Hard clamp on bias magnitude. Leaves headroom for IK flat-foot output.
    float max_pitch_bias_deg = SWING_LEVELER_MAX_PITCH_BIAS_DEG;
    float max_roll_bias_deg  = SWING_LEVELER_MAX_ROLL_BIAS_DEG;

    // Input-side deadband (rad). Suppresses IMU noise.
    float deadband_rad = SWING_LEVELER_DEADBAND_RAD;

    // IIR LPF on foot angle input (pre-controller).
    // 0 = no smoothing, 0.9 = heavy.
    float input_lpf_alpha = SWING_LEVELER_INPUT_LPF_ALPHA;

    // IIR on output bias (post-controller).
    float output_iir_alpha = SWING_LEVELER_OUTPUT_IIR_ALPHA;

    // Lift-fraction envelope corners. Smoothstep ramp up then ramp down.
    //   liftFrac <  engage_lift_frac      → zero bias (near ground)
    //   engage   <= liftFrac <= peak      → full bias
    //   peak     <  liftFrac < disengage  → decays to zero
    //   liftFrac >= disengage_lift_frac   → zero bias (pre-touchdown)
    float engage_lift_frac    = SWING_LEVELER_ENGAGE_LIFT_FRAC;
    float peak_lift_frac      = SWING_LEVELER_PEAK_LIFT_FRAC;
    float disengage_lift_frac = SWING_LEVELER_DISENGAGE_LIFT_FRAC;
};

// ---------------------------------------------------------------------------
// SwingFootLevelerState — read-only diagnostic snapshot.
// Updated every tick regardless of enabled flag so L0 telemetry shows
// would-be corrections before actuation is turned on.
// ---------------------------------------------------------------------------
struct SwingFootLevelerState {
    bool    armed                 = false;  // enabled && in envelope && IMU valid
    Side    activeSide            = Side::LEFT;
    float   foot_pitch_rad        = 0.0f;  // input-LPF filtered foot pitch
    float   foot_roll_rad         = 0.0f;  // input-LPF filtered foot roll
    // "Would-be" bias: P output × envelope, clamped, BEFORE the IIR and
    // BEFORE the enabled gate. Non-zero in L0 so sign/magnitude can be
    // verified without actuating.
    float   ankle_pitch_bias_deg  = 0.0f;
    float   ankle_roll_bias_deg   = 0.0f;
    float   envelope_scale        = 0.0f;  // lift-frac shaping factor [0..1]
    // Applied bias: post-IIR output. Zero when disabled (enabled=false).
    // This is what _submitFootTargetIK adds to the ankle angle.
    float   applied_pitch_bias_deg = 0.0f;
    float   applied_roll_bias_deg  = 0.0f;
    bool    foot_imu_valid         = false; // mirrors swing foot estimator valid
};

// ---------------------------------------------------------------------------
// SwingFootLeveler
// ---------------------------------------------------------------------------
class SwingFootLeveler {
public:
    SwingFootLeveler() = default;

    // Wire foot estimator pointers. main.cpp owns the estimators; leveler
    // holds non-owning pointers and reads on each update(). Calls reset().
    void init(const StateEstimator* leftFootEst, const StateEstimator* rightFootEst);

    // Hard-reset all transient IIR/LPF state. Does NOT touch config.
    // Call on every swing-leg switch and on entry to RECOVERY_STEP / oe_clear().
    void reset();

    // Per-tick update. Called from GaitController::update() after swing side
    // and lift fraction are known, before _submitFootTargetIK.
    //   swingSide: which leg is airborne this tick.
    //   liftFrac:  [0,1]; 0 = grounded, 1 = sin peak.
    // Pass liftFrac=0 in non-SWINGING sub-states — envelope gates output to
    // zero and the IIR decays smoothly toward zero.
    void update(Side swingSide, float liftFrac);

    // Bias accessors for _submitFootTargetIK.
    // Returns 0.0f when: disabled, foot IMU invalid, outside envelope,
    // OR requested side != activeSide (stance leg always gets zero bias).
    float anklePitchBiasFor(Side side) const;
    float ankleRollBiasFor (Side side) const;

    void                           setConfig(const SwingFootLevelerConfig& c) { _cfg = c; }
    const SwingFootLevelerConfig&  getConfig() const { return _cfg; }
    const SwingFootLevelerState&   getState()  const { return _state; }

private:
    const StateEstimator* _footEst[SIDE_COUNT] = { nullptr, nullptr };
    SwingFootLevelerConfig _cfg;
    SwingFootLevelerState  _state;

    // IIR/LPF filter state — persists across ticks within one swing.
    float _pitch_lpf_rad      = 0.0f;
    float _roll_lpf_rad       = 0.0f;
    float _pitch_bias_iir_deg = 0.0f;
    float _roll_bias_iir_deg  = 0.0f;
    bool  _seeded             = false;  // true after first valid foot sample
    Side  _lastSide           = Side::LEFT;

    // Smoothstep-shaped envelope from liftFrac.
    // Zero velocity at engage and disengage edges; full value at peak.
    static float _envelope(float liftFrac, const SwingFootLevelerConfig& cfg);
};

#endif // SWING_FOOT_LEVELER_H
