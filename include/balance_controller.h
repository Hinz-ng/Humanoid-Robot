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
#include "gait_types.h"
#include "side.h"   // Side — Phase 3 setAnkleRollBias overload

class MotionManager;

struct BalanceConfig {

    bool pitch_enabled = false;
    bool roll_enabled  = false;

    // --- Pitch ---
    // Kp TUNING HISTORY: 20.0 → 10.0 (gain-boost introduction) → shipped 8.0.
    // The nonlinear gain boost below (shipped boost_factor 1.3) raises the
    // effective Kp for large errors, so the flat Kp can sit lower without
    // giving up large-disturbance response.
    // After flashing: re-tune from WebSocket using CMD:BALANCE_TUNE:Kp=X
    //
    // *** AUDIT A-05 (2026-07): pitch_setpoint_rad = 10.0 RADIANS (~573°) is
    // almost certainly a units entry error (degrees typed into a rad field —
    // torso's analogous default is 0.0). It is dormant only because
    // pitch_enabled ships false. DO NOT arm pitch balance (PITCH_ON /
    // BALANCE_ON) without first sending BALANCE_TUNE:setpoint=0 (or a sane
    // small value): with this default the error is ≈ -10 rad, the controller
    // saturates, and dx rails to its clamp — a stability event. The
    // BALANCE_TUNE "setpoint" key is also unclamped at parse time. Default
    // retained pending human decision (default changes are out of audit
    // scope); change to 0.0f + add a ±0.5 rad parse clamp once confirmed. ***
    float pitch_setpoint_rad = 10.0f;  // rad. WS: "setpoint"
    float Kp = 8.0f;                  // deg/rad. WS: "Kp"
    float Kd = 0.1f;                   // deg/(rad/s). WS: "Kd"

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
    float gain_boost_factor        = 1.3f;   // dimensionless; 1.0 = disabled

    float pitch_deadband_rad       = 0.035f;  // rad ≈ 2°. WS: "deadband"
    float derivative_lpf_alpha     = 0.85f;   // [0,1). WS: "d_lpf"
    float servo_lag_compensation_s = 0.030f;  // s. WS: "tau"

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
    float roll_correction_sign = -1.0f;  // ±1. WS: "sign_r"

    // DEFAULT CHANGE: max_roll_correction_deg 10.0 → 8.0. WS: "max_r"
    float max_roll_correction_deg   = 8.0f;   // deg. Previous: 10.0
    float roll_deadband_rad         = 0.035f; // rad. WS: "roll_db"
    float roll_derivative_lpf_alpha = 0.85f;  // [0,1). WS: "roll_dlpf"

    // --- TASK-2: Ankle roll bias from WeightShift ---
    // Set each tick by WeightShift::update() through the dedicated setters below.
    // _submitAnkleRollBias() adds these AFTER the IIR smoother — additive to
    // the shaped balance correction, not replacing it.
    // Applied even when roll_enabled=false (C-02 fix — see update()).
    // Sign: LEFT body tilt → bias_l > 0, bias_r < 0 (matches balance convention).
    // MUST NOT be set by WebSocket or any code other than WeightShift::update().
    float ankle_roll_bias_l_deg = 0.0f;
    float ankle_roll_bias_r_deg = 0.0f;

    // --- Swing-foot leveler ankle ROLL bias ---
    // Pushed each tick from GaitController via setSwingLevelerRollBias()
    // after SwingFootLeveler::update() runs. ADDED to ankle roll at submission
    // time in _submitAnkleRollBias() — composes additively with the WeightShift
    // ankle_roll_bias_l/r_deg field (different concerns, different sources).
    //
    // Sign convention matches the leveler's internal convention: a single signed
    // value per side, written directly to the swing-side joint command. The
    // non-swing side's field is always 0 (the leveler returns 0 there).
    //
    // Routed through SOURCE_BALANCE so it survives priority arbitration when
    // WeightShift is also injecting a roll bias on the same tick (SOURCE_BALANCE
    // > SOURCE_GAIT).
    //
    // MUST NOT be set from WebSocket. Only GaitController writes via setter.
    float swing_leveler_roll_bias_l_deg = 0.0f;
    float swing_leveler_roll_bias_r_deg = 0.0f;

    // Conversion gain: PD output (deg) → task-space body shift (mm).
    // At h_sagittal ≈ 185 mm: 1° ≈ 3.2 mm. Default 3.0 is conservative.
    // WS: "dx_per_deg", "dy_per_deg"
    float pitch_to_dx_mm_per_deg = 3.0f;
    float roll_to_dy_mm_per_deg  = 3.0f;

    // --- Catch-step recovery — Stage 0 detection ---
    // Triggers recovery_flag when primary controller cannot stabilise.
    // Stage 0: flag-only, no actuation. Default disabled — tune thresholds
    // so flag never fires during M2 walking before enabling.
    //
    // Detection: (tilt OR saturation) persistent for recovery_confirm_ticks.
    // Saturation counted on u_clamped / u_roll_clamped ≥ 95% of clamp ceiling.
    // WS keys: "rcv_pitch", "rcv_roll", "rcv_sat", "rcv_conf"
    bool     recovery_enabled              = false;
    bool     recovery_suppress_during_gait = true;   // F6 mitigation
    float    recovery_tilt_pitch_rad       = 0.15f;  
    float    recovery_tilt_roll_rad        = 0.13f;  
    uint16_t recovery_saturation_ticks     = 20;     // ~50 ms at 400 Hz
    uint16_t recovery_confirm_ticks        = 10;     // ~25 ms debounce
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
    bool  pitch_active   = false;
    bool  roll_active    = false;
    float roll_error          = 0.0f;
    float u_roll_raw          = 0.0f;
    float u_roll_clamped      = 0.0f;
    float effective_kp_roll   = 0.0f;
    float ankle_roll_cmd_deg  = 0.0f;
    float hip_roll_cmd_deg    = 0.0f;
    bool  fell = false;

    // --- Catch-step recovery telemetry (Stage 0) ---
    bool     recovery_flag     = false;  // true when debounced conditions met
    uint16_t saturation_count  = 0;      // current saturation streak (ticks)
    uint16_t recovery_debounce = 0;      // current debounce streak (ticks)
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
    // Phase 3: dual-parameter form preserved for callers that set both biases
    // atomically (oe_clear in main.cpp). Prefer the Side-keyed overload below
    // for new code — positional parameter order is the original silent-swap
    // hazard this refactor closes.
    void                 setAnkleRollBias(float leftDeg, float rightDeg) {
        _cfg.ankle_roll_bias_l_deg = leftDeg;
        _cfg.ankle_roll_bias_r_deg = rightDeg;
    }
    // Phase 3: Side-keyed single-side setter. Calling twice per tick (once for
    // each Side) is equivalent to the dual-parameter form above.
    void                 setAnkleRollBias(Side side, float deg) {
        if (side == Side::LEFT)  _cfg.ankle_roll_bias_l_deg = deg;
        else                      _cfg.ankle_roll_bias_r_deg = deg;
    }
    // Side-keyed setter for swing-foot leveler ankle roll bias.
    // Called once per side per tick from GaitController::update() after the
    // leveler's own update(). Uses targeted-field write (not setConfig) to
    // avoid the BalanceConfig WiFi-task RMW race documented in CODING_STANDARD.
    void                 setSwingLevelerRollBias(Side side, float deg) {
        if (side == Side::LEFT)  _cfg.swing_leveler_roll_bias_l_deg = deg;
        else                      _cfg.swing_leveler_roll_bias_r_deg = deg;
    }
    void                 setRollSetpointRad(float rad) { _cfg.roll_setpoint_rad = rad; }

    // Task-space output API.
    // computeCorrection() runs the pitch + roll PD in mm space. Pure computation —
    // does NOT submit to any joints. Called every tick from update().
    // Intermediate clamped-degree values are cached in _ts_pitch_u_clamped /
    // _ts_roll_u_clamped for BalanceState telemetry (p_err / p_u broadcast).
    BalanceCorrection computeCorrection(const IMUState& state);

    // Returns the most recent correction. Valid=false until update() has run once.
    BalanceCorrection getLastCorrection() const { return _lastCorrection; }

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

    // Ankle-roll ownership moved to GaitController (consolidation refactor):
    // it composes flat-foot + CoM-shift bias + swing-prep + leveler in one
    // SOURCE_GAIT submission. BalanceController no longer submits ankle roll;
    // its leg authority is task-space only (computeCorrection → dx_mm/dy_mm).

    // Task-space filter state.
    BalanceCorrection _lastCorrection;
    float _dx_filtered_mm    = 0.0f;
    float _dy_filtered_mm    = 0.0f;
    // Intermediate PD values cached by computeCorrection for BalanceState
    // telemetry (surfaced as pitch_error / u_clamped, broadcast p_err / p_u).
    float _ts_pitch_u_clamped = 0.0f;
    float _ts_roll_u_clamped  = 0.0f;
    float _ts_pitch_error     = 0.0f;
    float _ts_roll_error      = 0.0f;

    // Stage 0: recovery detection counters.
    // Incremented each tick saturated / debounced; clamped at the respective
    // threshold (uint16_t max > 65 k ticks, but we clamp before that).
    uint16_t _saturationCount  = 0;
    uint16_t _recoveryDebounce = 0;

    // Populates out.recovery_flag / saturation_count / recovery_debounce.
    // Called from both the task-space and legacy branches of update(), just
    // before _lastState = out, so that out.u_clamped is already populated.
    void _updateRecoveryDetection(BalanceState& out, const IMUState& state);
};

#endif // BALANCE_CONTROLLER_H
