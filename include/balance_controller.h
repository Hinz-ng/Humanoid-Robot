// =============================================================================
// FILE:    balance_controller.h
// MODULE:  control
// LAYER:   3 — Balance Controller
//
// LAST CHANGED: 2026-04-08 | Hinz | Task-1 gain boost, Task-2 ankle bias,
//                                    C-01 API compat, C-02 bias-when-disabled
// =============================================================================

#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#include <Arduino.h>
#include "state_estimator.h"
#include "servo_control.h"
#include "joint_config.h"

class MotionManager;

struct BalanceConfig {

    bool pitch_enabled = false;
    bool roll_enabled  = false;

    // --- Pitch ---
    // DEFAULT CHANGE: Kp reduced from 20.0 → 10.0.
    // This is INTENTIONAL for the nonlinear gain boost (TASK-1):
    //   At Kp=10 + boost_factor=2.0: small errors use Kp=10 (no oscillation),
    //   large errors use Kp=20 (same recovery as before). Net effect: stability
    //   near equilibrium is improved while large-disturbance response is unchanged.
    // After flashing: re-tune from WebSocket using CMD:BALANCE_TUNE:Kp=X
    float pitch_setpoint_rad = 0.0f;   // rad. WS: "setpoint"
    float Kp = 10.0f;                  // deg/rad. WS: "Kp"
    float Kd = 0.3f;                   // deg/(rad/s). WS: "Kd"

    // --- TASK-1: Nonlinear gain boost (pitch) ---
    // |error| < threshold              → effective_kp = Kp
    // threshold <= |error| < 2×thresh  → linear ramp: Kp → Kp × factor
    // |error| >= 2×threshold           → effective_kp = Kp × factor
    //
    // Design intent: oscillations occur at |error| < ~0.05 rad near equilibrium.
    // threshold=0.10 rad keeps those in the flat-Kp stable zone. Large disturbances
    // (|error| > 0.15 rad) hit the boost zone, matching old Kp=20 recovery.
    // Set factor=1.0 to disable boost and use flat Kp (equivalent to old behaviour).
    // WS keys: "boost_thresh" (rad), "boost_factor"
    float gain_boost_threshold_rad = 0.10f;  // rad (≈5.7°)
    float gain_boost_factor        = 2.0f;   // dimensionless; 1.0 = disabled

    float pitch_deadband_rad       = 0.035f;  // rad ≈ 2°. WS: "deadband"
    float derivative_lpf_alpha     = 0.85f;   // [0,1). WS: "d_lpf"
    float servo_lag_compensation_s = 0.030f;  // s. WS: "tau"

    float ankle_ratio = 1.0f;  // WS: "ankle"
    float hip_ratio   = 0.0f;  // WS: "hip"
    float torso_ratio = 0.0f;  // WS: "torso"

    float correction_sign    = 1.0f;   // ±1. WS: "sign"
    float max_correction_deg = 15.0f;  // deg. WS: "max"

     // --- Fall detection ---
    // Disabled by default. Enable only after gains are tuned and the robot
    // can stand stably — accidental triggers during early testing cause
    // violent servo snaps. Re-enable via CMD:BALANCE_TUNE:fall_det=1.
    bool    fall_detection_enabled = false;
    float   fall_threshold_rad     = 0.70f;  // rad
    uint8_t fall_confirm_ticks     = 5;      // at 400 Hz ≈ 12.5 ms

    // --- Output shaping ---
    float output_iir_alpha             = 0.60f;  // [0,0.99). WS: "iir"
    float max_output_rate_deg_per_tick = 0.30f;  // deg/tick. WS: "rate"
    float output_damping_kv            = 0.0f;   // deg/(rad/s). WS: "damping"

    // --- Roll ---
    // DEFAULT CHANGE: Kp_roll 20.0 → 15.0, max_roll_correction_deg 10.0 → 8.0.
    // Kp_roll=15 with boost_factor=2.0 restores the effective Kp=20 for large roll
    // errors while providing a stable operating point at equilibrium.
    // max_roll_correction_deg: 8° is conservative; raise to 10 if roll correction
    // feels insufficient after confirming roll_correction_sign on hardware.
    float Kp_roll = 15.0f;  // deg/rad. Previous: 20.0. WS: "Kp_r"
    float Kd_roll = 0.2f;   // deg/(rad/s). WS: "Kd_r"

    // TASK-1 parity for roll.
    // WS keys: "boost_thresh_r", "boost_factor_r"
    float gain_boost_threshold_roll_rad = 0.10f;
    float gain_boost_factor_roll        = 2.0f;

    float roll_setpoint_rad    = 0.0f;  // rad. Injected by WeightShift. WS: "sp_r"
    float roll_correction_sign = 1.0f;  // ±1. WS: "sign_r"

    float ankle_roll_ratio = 1.0f;  // WS: "a_roll"
    float hip_roll_ratio   = 0.0f;  // WS: "h_roll"
    float torso_roll_ratio = 0.0f;  // WS: "t_roll"

    // DEFAULT CHANGE: max_roll_correction_deg 10.0 → 8.0. WS: "max_r"
    float max_roll_correction_deg   = 8.0f;   // deg. Previous: 10.0
    float roll_deadband_rad         = 0.035f; // rad. WS: "roll_db"
    float roll_derivative_lpf_alpha = 0.85f;  // [0,1). WS: "roll_dlpf"

    // --- TASK-2: Ankle roll bias from WeightShift ---
    // Set each tick by WeightShift::update() via _bal->getConfig()/setConfig().
    // _applyRollCorrection() adds these AFTER the IIR smoother — additive to
    // the shaped balance correction, not replacing it.
    // Applied even when roll_enabled=false (C-02 fix — see update()).
    // Sign: LEFT body tilt → bias_l > 0, bias_r < 0 (matches balance convention).
    // MUST NOT be set by WebSocket or any code other than WeightShift::update().
    float ankle_roll_bias_l_deg = 0.0f;
    float ankle_roll_bias_r_deg = 0.0f;
};

enum BalanceJointIdx : uint8_t {
    BJI_R_ANKLE_PITCH = 0,
    BJI_L_ANKLE_PITCH = 1,
    BJI_R_HIP_PITCH   = 2,
    BJI_L_HIP_PITCH   = 3,
    BJI_TORSO_PITCH   = 4,
    BJI_R_ANKLE_ROLL  = 5,
    BJI_L_ANKLE_ROLL  = 6,
    BJI_R_HIP_ROLL    = 7,
    BJI_L_HIP_ROLL    = 8,
    BJI_TORSO_ROLL    = 9,
    BJI_COUNT         = 10
};

struct JointOutputState {
    float prev_cmd_deg     = 0.0f;
    float filtered_cmd_deg = 0.0f;
};

struct BalanceState {
    bool  active         = false;
    float pitch_error    = 0.0f;
    float u_raw          = 0.0f;
    float u_clamped      = 0.0f;
    float effective_kp   = 0.0f;  // actual Kp this tick (with boost applied)
    float ankle_cmd_deg  = 0.0f;
    float hip_cmd_deg    = 0.0f;
    float torso_cmd_deg  = 0.0f;
    bool  pitch_active   = false;
    bool  roll_active    = false;
    float roll_error          = 0.0f;
    float u_roll_raw          = 0.0f;
    float u_roll_clamped      = 0.0f;
    float effective_kp_roll   = 0.0f;
    float ankle_roll_cmd_deg  = 0.0f;
    float hip_roll_cmd_deg    = 0.0f;
    float torso_roll_cmd_deg  = 0.0f;
    bool  fell = false;
};

class BalanceController {
public:
    explicit BalanceController(ServoControl* servo = nullptr);

    void         init();
    BalanceState update(const IMUState& state);

    // Zero all IIR, derivative filters, fall counter. Does NOT touch _cfg.
    // Call from oe_clear() in main.cpp after ESTOP to prevent command spikes.
    void resetOutputState();

    // C-01 FIX: backward-compatible alias. main.cpp calls resetState().
    // Both names call the same implementation. Do not remove either.
    inline void resetState() { resetOutputState(); }

    void                 setConfig(const BalanceConfig& cfg) { _cfg = cfg; }
    const BalanceConfig& getConfig() const                   { return _cfg; }
    BalanceState         getLastState() const                { return _lastState; }
    void                 setMotionManager(MotionManager* mm) { _motionManager = mm; }

private:
    ServoControl*    _servo;
    MotionManager*   _motionManager = nullptr;
    BalanceConfig    _cfg;
    BalanceState     _lastState;
    uint8_t          _fallTickCount     = 0;
    float            _pitchRateFiltered = 0.0f;
    float            _rollRateFiltered  = 0.0f;
    JointOutputState _jointState[BJI_COUNT];
    bool             _prevPitchEnabled  = false;
    bool             _prevRollEnabled   = false;

    float _shapeOutput(uint8_t jIdx, float rawCmd, float angularRate_rs);

    static float _computeEffectiveKp(float absError, float baseKp,
                                     float threshold, float boostFactor);

    void _applyPitchCorrection(float ankle_deg, float hip_deg,
                               float torso_deg, float pitchRate_rs);
    void _applyRollCorrection(float ankle_deg, float hip_deg,
                              float torso_deg, float rollRate_rs);
};

#endif // BALANCE_CONTROLLER_H