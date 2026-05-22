// =============================================================================
// FILE:    torso_stabilizer.h
// MODULE:  control
// LAYER:   3 — Torso Stabilizer (joint-space)
//
// Joint-space torso pitch/roll stabiliser, extracted from BalanceController in
// the balance/gait consolidation refactor. Leg balance is task-space
// (BalanceController emits dx_mm/dy_mm); the torso joints are NOT in the IK
// chain, so they remain a separate joint-space controller.
//
// TorsoStabilizer does NOT run its own PD. It consumes BalanceController's
// already-clamped pitch/roll PD output (BalancePDOutput) and scales it by the
// torso ratios. This guarantees parity with the pre-refactor torso behaviour —
// one PD loop, no duplicated Smith predictor / deadband / gain boost.
// =============================================================================

#ifndef TORSO_STABILIZER_H
#define TORSO_STABILIZER_H

#include <Arduino.h>
#include "balance_controller.h"   // BalancePDOutput, JointOutputState

class MotionManager;

struct TorsoConfig {
    // Master enable for the torso pitch + roll submissions. WS: "torso_bal"
    bool  enabled = true;

    // Raw gains: torso joint cmd = PD-clamped u (deg) × ratio. Default 0 — the
    // torso contributes nothing until explicitly tuned. WS: "torso", "t_roll"
    float pitch_ratio = 0.0f;
    float roll_ratio  = 0.0f;

    // Output shaping — seeded identical to the BalanceConfig values the torso
    // submission used before the refactor, so torso behaviour is unchanged.
    float output_iir_alpha             = 0.60f;  // WS: "torso_iir"
    float max_output_rate_deg_per_tick = 0.30f;
    float output_damping_kv            = 0.0f;
    float max_output_deg               = 15.0f;  // final clamp
};

struct TorsoState {
    float pitch_cmd_deg = 0.0f;  // last shaped torso pitch cmd submitted
    float roll_cmd_deg  = 0.0f;  // last shaped torso roll cmd submitted
    bool  pitch_active  = false;
    bool  roll_active   = false;
};

class TorsoStabilizer {
public:
    void setMotionManager(MotionManager* mm) { _mm = mm; }
    void setConfig(const TorsoConfig& cfg)   { _cfg = cfg; }
    const TorsoConfig& getConfig() const     { return _cfg; }
    TorsoState         getState()  const     { return _state; }

    // Zero the IIR shaping state. Call from oe_clear() after ESTOP.
    void resetOutputState();

    // Submits torso pitch/roll joint commands at SOURCE_BALANCE, derived from
    // BalanceController's PD output. Call once per tick, AFTER
    // BalanceController::update() and BEFORE GaitController::update().
    void update(const BalancePDOutput& pd);

private:
    MotionManager*   _mm = nullptr;
    TorsoConfig      _cfg;
    TorsoState       _state;
    JointOutputState _pitchShape;
    JointOutputState _rollShape;

    float _shape(JointOutputState& s, float rawCmd, float angularRate_rs);
};

#endif // TORSO_STABILIZER_H
