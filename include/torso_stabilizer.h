// =============================================================================
// FILE:    torso_stabilizer.h
// MODULE:  control
// LAYER:   3 — Torso Stabilizer (joint-space)
//
// Standalone joint-space torso pitch/roll stabiliser. Reads the torso IMUState
// directly, runs its own pitch + roll PD, shapes/filters its own output, and
// submits the torso joints to MotionManager at SOURCE_BALANCE. It shares NO
// state with BalanceController — separate enables, gains, filters, clamps,
// shaping, telemetry, and runtime state.
//
// CONTROL LAW — PD (no integral). Rationale:
//   - ~1–2° servo backlash makes an integrator wind up inside the mechanical
//     deadband and limit-cycle once backlash is taken up;
//   - the upright setpoint carries no persistent external torque, so there is
//     no genuine steady-state error for an I term to remove — the only residual
//     is backlash/servo positioning error, which integral cannot fix;
//   - low rigidity + servo lag already cost phase margin; an I term adds lag and
//     can resonate with step cadence.
//   PD gives a deterministic, inspectable response suited to a secondary
//   stabiliser. The torso joints are NOT in the IK chain (leg balance is
//   task-space dx/dy), so this controller is fully independent of leg balance.
// =============================================================================

#ifndef TORSO_STABILIZER_H
#define TORSO_STABILIZER_H

#include <Arduino.h>
#include "state_estimator.h"   // IMUState

class MotionManager;

// Per-joint output-shaping state (IIR + delta clamp). Owned by TorsoStabilizer;
// independent of BalanceController's joint-state array.
struct TorsoShapeState {
    float prev_cmd_deg     = 0.0f;  // pre-IIR command (diagnostic)
    float filtered_cmd_deg = 0.0f;  // IIR state — tracks smoothed trajectory
};

struct TorsoConfig {
    // --- Per-axis enables (independent of BalanceController) ---
    bool pitch_enabled = false;   // WS: TORSO_PITCH_ON / TORSO_PITCH_OFF
    bool roll_enabled  = false;   // WS: TORSO_ROLL_ON  / TORSO_ROLL_OFF

    // --- Pitch PD ---
    float pitch_setpoint_rad  = 0.0f;   // rad.          WS: "setpoint"
    float Kp                  = 0.0f;   // deg/rad.      WS: "Kp"
    float Kd                  = 0.0f;   // deg/(rad/s).  WS: "Kd"
    float pitch_deadband_rad  = 0.035f; // rad ≈ 2°.     WS: "deadband"
    float derivative_lpf_alpha = 0.85f; // [0,1).        WS: "d_lpf"
    float correction_sign     = -1.0f;  // ±1. Torso leans opposite body tilt.
                                        //               WS: "sign"  (verify on HW)
    float max_correction_deg  = 30.0f;  // deg.          WS: "max"

    // --- Roll PD ---
    float roll_setpoint_rad        = 0.0f;   // rad.          WS: "sp_r"
    float Kp_roll                  = 0.0f;   // deg/rad.      WS: "Kp_r"
    float Kd_roll                  = 0.0f;   // deg/(rad/s).  WS: "Kd_r"
    float roll_deadband_rad        = 0.035f; // rad.          WS: "db_r"
    float roll_derivative_lpf_alpha = 0.85f; // [0,1).        WS: "dlpf_r"
    // Inverted from -1.0f → +1.0f: the previous default drove the torso roll
    // correction in the destabilising direction on hardware. +1.0f moves the
    // correction the physically stabilising way (torso rolls to oppose the
    // measured tilt). Isolated to torso roll only — pitch uses correction_sign;
    // ankle-roll / weight-shift / gait roll bias / foot lateral offsets are in
    // other modules and are unaffected. WS: "sign_r".
    float roll_correction_sign     = +1.0f;  // ±1.           WS: "sign_r" (verify HW)
    float max_roll_correction_deg  = 15.0f;  // deg.          WS: "max_r"

    // --- Output shaping (shared pipeline, both axes) ---
    float output_iir_alpha             = 0.60f; // [0,0.99). WS: "iir"
    float max_output_rate_deg_per_tick = 2.0f; // deg/tick. WS: "rate"
    float output_damping_kv            = 0.0f;  // deg/(rad/s). WS: "damping"
    float max_output_deg               = 30.0f; // final clamp. WS: "out_max"
};

struct TorsoState {
    float pitch_error    = 0.0f;  // rad
    float pitch_u_clamped = 0.0f; // deg, post-PD pre-shaping
    float pitch_cmd_deg  = 0.0f;  // deg, last shaped cmd submitted
    bool  pitch_active   = false;
    float roll_error     = 0.0f;  // rad
    float roll_u_clamped = 0.0f;  // deg
    float roll_cmd_deg   = 0.0f;  // deg
    bool  roll_active    = false;
};

class TorsoStabilizer {
public:
    void setMotionManager(MotionManager* mm) { _mm = mm; }
    void setConfig(const TorsoConfig& cfg)   { _cfg = cfg; }
    const TorsoConfig& getConfig() const     { return _cfg; }
    TorsoState         getState()  const     { return _state; }

    // Zero the IIR shaping + telemetry state. Call from oe_estop()/oe_clear().
    void resetOutputState();

    // Runs the torso pitch/roll PD on the torso IMUState and submits the
    // shaped joint commands at SOURCE_BALANCE. Call once per tick, after
    // StateEstimator::update(). No dependency on BalanceController.
    void update(const IMUState& state);

private:
    MotionManager*  _mm = nullptr;
    TorsoConfig     _cfg;
    TorsoState      _state;
    TorsoShapeState _pitchShape;
    TorsoShapeState _rollShape;
    float           _pitchRateFiltered = 0.0f;
    float           _rollRateFiltered  = 0.0f;

    // PD on one axis → clamped command (deg). Updates rateFilt in place.
    float _computePD(float angle, float setpoint, float angleRate,
                     float& rateFilt, float lpfAlpha, float deadband,
                     float kp, float kd, float sign, float maxCorr,
                     float& errorOut) const;

    // Output shaping: zero-bypass → delta clamp → IIR → velocity damp → clamp.
    float _shape(TorsoShapeState& s, float rawCmd, float angularRate_rs);
};

#endif // TORSO_STABILIZER_H
