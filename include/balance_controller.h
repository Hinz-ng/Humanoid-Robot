// =============================================================================
// FILE:    balance_controller.h
// MODULE:  control
// LAYER:   3 — Balance Controller
//
// PURPOSE:
//   PD pitch + roll balance controller with a full output-shaping pipeline.
//   All joint commands pass through: delta clamp → IIR smooth → velocity damp
//   before submission to MotionManager. This limits effective command bandwidth
//   to ~20–50 Hz (from 400 Hz control loop), eliminates slew-rate spikes, and
//   applies angular-rate damping at the output stage — independent of the PD
//   error path. Derived from techniques in the UVC biped codebase (movSv,
//   footCont delta clamp, fbAV output damping).
//
// INPUTS:  IMUState (from state_estimator.h)
// OUTPUTS: SOURCE_BALANCE joint commands via MotionManager
//
// DEPENDENCIES:
//   state_estimator.h — IMUState
//   servo_control.h   — ServoControl, IDX_* constants
//   joint_config.h    — IDX_* channel constants
//   motion_manager.h  — MotionManager (forward-declared)
//
// LAST CHANGED: 2026-04-05 | Hinz | Add output shaping pipeline + roll parity
// =============================================================================

#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#include <Arduino.h>
#include "state_estimator.h"
#include "servo_control.h"
#include "joint_config.h"

class MotionManager;

// ---------------------------------------------------------------------------
// BalanceConfig — all tuning parameters in one struct.
//
// TUNING ORDER (never tune two things simultaneously):
//   1. Verify correction_sign on hardware (lean fwd → ankles push back).
//   2. Kd=0, increase Kp from 5 until oscillation just starts, back off 20%.
//   3. Add Kd until oscillations damp (0.1–0.5 range).
//   4. Tune output_iir_alpha (0.60 default). Higher = more smoothing / slower.
//   5. Tune max_output_rate_deg_per_tick (0.30 default). Lower = slower ramp.
//   6. Enable output_damping_kv (0.0 default) only if limit cycles persist.
//   7. Repeat for roll after pitch is confirmed stable.
// ---------------------------------------------------------------------------
struct BalanceConfig {

    // --- Master switches ---
    bool pitch_enabled = false;
    bool roll_enabled  = false;

    // --- Pitch upright target ---
    // Positive = leaning forward. Adjust if robot consistently leans at rest.
    float pitch_setpoint_rad   = 0.0f;  // rad

    // --- Pitch PD gains ---
    // Kp units: deg/rad.  Kd units: deg/(rad/s).
    // SAFE RANGES for this robot (35–40 cm, ~50 ms servo lag):
    //   Kp: 5–20.  Above 20 eliminates phase margin — do NOT exceed.
    //   Kd: 0.1–0.8.  Above 1.0, gyro shot noise dominates.
    // Ceiling enforced by BALANCE_KP_MAX / BALANCE_KD_MAX in project_wide_defs.h.
    float Kp = 10.0f;   // deg/rad
    float Kd = 0.3f;    // deg/(rad/s)

    // --- Pitch error deadband ---
    // Corrections below this pitch error are suppressed.
    // Sized to match servo backlash (1–3°). UVC uses 0.033 rad; matched here.
    // Wider → less jitter at rest. Narrower → more responsive to small lean.
    float pitch_deadband_rad = 0.035f;  // rad (≈ 2°)

    // --- Pitch derivative LPF ---
    // IIR on pitchRate before Kd multiplication. Attenuates 400 Hz gyro noise.
    // Recalculate if loop rate changes: alpha ≈ 1 - 1/(tau_s * loop_hz)
    //   alpha=0.85 at 400 Hz → tau ≈ 14 ms.
    float derivative_lpf_alpha = 0.85f;  // dimensionless [0, 1)

    // --- Smith predictor (servo lag compensation) ---
    // Advances the pitch estimate by one servo lag period to partially cancel
    // mechanical delay in the phase response.
    // tau=0.030s is the calibrated default. Set to 0 to disable.
    float servo_lag_compensation_s = 0.030f;  // seconds

    // --- Pitch joint distribution ---
    // ankle + hip + torso must sum to 1.0. Firmware normalises on receipt.
    float ankle_ratio = 1.0f;  // dimensionless
    float hip_ratio   = 0.0f;  // dimensionless
    float torso_ratio = 0.0f;  // dimensionless

    // --- Pitch correction direction ---
    // +1: positive pitch (lean fwd) → positive joint correction.
    // Verify empirically: lean fwd → ankles should dorsi-flex, not plantar-flex.
    float correction_sign = 1.0f;

    // --- Pitch output clamp ---
    // Hard magnitude limit on PD output before joint split. Start at 15°.
    float max_correction_deg = 15.0f;  // deg

    // --- Fall detection ---
    // If |pitch| OR |roll| exceeds threshold for fall_confirm_ticks consecutive
    // ticks, ESTOP fires. Do not raise above 0.70 rad (~40°).
    float  fall_threshold_rad  = 0.70f;  // rad
    uint8_t fall_confirm_ticks = 3;      // ticks at 400 Hz (~7.5 ms)

    // =========================================================================
    //  OUTPUT SHAPING PIPELINE
    //  All balance commands pass through this after ratio split:
    //    raw → delta_clamp → IIR_smooth → velocity_damp → submit
    //
    //  This limits effective command bandwidth independently of PD gains,
    //  preventing the 400 Hz control loop from overdriving servos.
    //  Derived from UVC biped: movSv (IIR), footCont delta clamp, fbAV damping.
    // =========================================================================

    // IIR smoother on joint commands (applied post-delta-clamp).
    // Limits effective command bandwidth: f_c ≈ (1-alpha) * loop_hz / (2π)
    //   alpha=0.60 at 400 Hz → f_c ≈ 38 Hz (target: 20–50 Hz).
    //   alpha=0.80 at 400 Hz → f_c ≈ 13 Hz (more smoothing, slower response).
    // Recalculate if loop rate changes.
    // 0 = no smoothing (passthrough).  1 = command frozen (never set this).
    float output_iir_alpha = 0.60f;  // dimensionless [0, 1)

    // Per-tick delta clamp — limits how fast each joint command can change.
    // Units: deg/tick.  At 400 Hz: 0.30 deg/tick = 120 deg/s slew rate.
    // Prevents step-command spikes from exciting leg structural resonance.
    // Replaces the old ankle-only rate limiter (_prevAnkleCmd path).
    // WebSocket key "rate" maps to this field (backward compatible).
    // 120 deg/s is well below ankle no-load speed (706 deg/s) but prevents
    // impulsive inputs that mechanical backlash would amplify into chatter.
    float max_output_rate_deg_per_tick = 0.30f;  // deg/tick

    // Velocity damping applied AFTER IIR smoothing at the output stage.
    // Counteracts angular velocity directly — independent of the PD error term.
    // Mirrors UVC's `A0W[s] = k0 - 0.003 * fbAV` pattern.
    // Units: deg / (rad/s).  Kv=0 → disabled (safe default for first tuning).
    // After delta+IIR are stable, try 0.5–2.0 if near-upright oscillation persists.
    float output_damping_kv = 0.0f;  // deg/(rad/s)

    // =========================================================================
    //  ROLL CONTROLLER PARAMETERS
    //  Feature parity with pitch: same PD structure + same output shaping path.
    //  ankle_roll + hip_roll + torso_roll ratios must sum to 1.0.
    // =========================================================================

    float Kp_roll                  = 5.0f;   // deg/rad  — lower than Kp_pitch; smaller stability margin
    float Kd_roll                  = 0.2f;   // deg/(rad/s)
    float roll_setpoint_rad        = 0.0f;   // rad — target roll (0 = upright)
    float roll_correction_sign     = 1.0f;   // ±1 — verify empirically before enabling
    float ankle_roll_ratio         = 1.0f;   // dimensionless
    float hip_roll_ratio           = 0.0f;   // dimensionless
    float torso_roll_ratio         = 0.0f;   // dimensionless
    float max_roll_correction_deg  = 8.0f;   // deg — tighter than pitch (smaller stability margin)

    // Roll parity fields — brought to feature parity with pitch controller.
    // Without these, roll had no derivative filtering or deadband, causing
    // asymmetric dynamics between sagittal and frontal loops.
    float roll_deadband_rad         = 0.035f;  // rad — matches pitch_deadband_rad
    float roll_derivative_lpf_alpha = 0.85f;   // matches derivative_lpf_alpha
};

// ---------------------------------------------------------------------------
// BalanceJointIdx — index into the per-joint output shaping state array.
// Pitch joints: 0–4.  Roll joints: 5–9.
// Order must match _jointState array indexing in BalanceController.
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// JointOutputState — persistent per-joint state for the output shaping pipeline.
// One instance per entry in BalanceJointIdx. Zeroed on init() and on ESTOP clear.
// ---------------------------------------------------------------------------
struct JointOutputState {
    // Post-delta-clamp value from last tick. The IIR uses filtered_cmd_deg
    // as its reference — prev_cmd_deg is kept separately for diagnostics.
    float prev_cmd_deg     = 0.0f;  // deg — delta-clamped output, last tick

    // IIR smoother state. This is the actual smoothed trajectory the servo tracks.
    // Updated every tick regardless of deadband (so IIR decays toward zero when
    // inside deadband and there is no active command).
    float filtered_cmd_deg = 0.0f;  // deg — IIR output, persists across ticks
};

// ---------------------------------------------------------------------------
// BalanceState — telemetry snapshot from one update() call.
// Values reported are PRE-shaping (control law output before IIR/delta/damp)
// so UI gain tuning reflects the raw PD math, not the shaped trajectory.
// ---------------------------------------------------------------------------
struct BalanceState {
    bool  active        = false;
    float pitch_error   = 0.0f;   // rad — measured_pitch − setpoint
    float u_raw         = 0.0f;   // deg — PD output before clamp
    float u_clamped     = 0.0f;   // deg — PD output after magnitude clamp
    float ankle_cmd_deg = 0.0f;   // deg — pre-shaping ankle pitch command
    float hip_cmd_deg   = 0.0f;   // deg — pre-shaping hip pitch command
    float torso_cmd_deg = 0.0f;   // deg — pre-shaping torso pitch command
    bool  pitch_active  = false;
    bool  roll_active   = false;
    float roll_error         = 0.0f;   // rad
    float u_roll_raw         = 0.0f;   // deg
    float u_roll_clamped     = 0.0f;   // deg
    float ankle_roll_cmd_deg = 0.0f;   // deg
    float hip_roll_cmd_deg   = 0.0f;   // deg
    float torso_roll_cmd_deg = 0.0f;   // deg
    bool  fell          = false;
};

// ---------------------------------------------------------------------------
// BalanceController
// ---------------------------------------------------------------------------
class BalanceController {
public:
    explicit BalanceController(ServoControl* servo = nullptr);

    // Call once in setup() after servoController.init().
    void init();

    // Main entry point — call every tick inside the 400 Hz gate in main.cpp,
    // AFTER stateEstimator.update() has run.
    BalanceState update(const IMUState& state);

    void                 setConfig(const BalanceConfig& cfg) { _cfg = cfg; }
    const BalanceConfig& getConfig() const                   { return _cfg; }
    BalanceState         getLastState() const                { return _lastState; }

    void setMotionManager(MotionManager* mm) { _motionManager = mm; }

private:
    ServoControl*  _servo;
    MotionManager* _motionManager = nullptr;
    BalanceConfig  _cfg;
    BalanceState   _lastState;

    // --- Fall detection state ---
    uint8_t _fallTickCount = 0;

    // --- Derivative filter state (pitch and roll — kept at controller level) ---
    // These feed the PD Kd term and are also passed to _applyXCorrection() for
    // use as the velocity_damping angular rate source.
    float _pitchRateFiltered = 0.0f;  // rad/s — IIR-filtered pitchRate
    float _rollRateFiltered  = 0.0f;  // rad/s — IIR-filtered rollRate (parity)

    // --- Per-joint output shaping state ---
    // Indexed by BalanceJointIdx. All entries zeroed in init() and on ESTOP.
    JointOutputState _jointState[BJI_COUNT];

    // -------------------------------------------------------------------------
    //  _shapeOutput() — single output shaping pipeline for every balance joint.
    //
    //  Pipeline: raw → delta_clamp → IIR_smooth → velocity_damp → return
    //
    //  jIdx:          which joint (BalanceJointIdx enum value)
    //  rawCmd:        pre-shaped command (deg from neutral), after ratio split
    //  angularRate_rs: filtered angular rate for velocity damping (rad/s)
    //                  pass _pitchRateFiltered for pitch joints,
    //                  pass _rollRateFiltered  for roll  joints.
    //
    //  Returns the final shaped command in deg, ready to submit to MotionManager.
    //  Updates _jointState[jIdx] as a side effect — must be called every tick,
    //  even when rawCmd=0 (inside deadband), so IIR decays toward zero smoothly.
    // -------------------------------------------------------------------------
    float _shapeOutput(uint8_t jIdx, float rawCmd, float angularRate_rs);

    // -------------------------------------------------------------------------
    //  _applyPitchCorrection() — shapes and submits pitch corrections.
    //  pitchRate_rs: _pitchRateFiltered (passed in so it's not re-read from state).
    // -------------------------------------------------------------------------
    void _applyPitchCorrection(float ankle_deg, float hip_deg, float torso_deg,
                               float pitchRate_rs);

    // -------------------------------------------------------------------------
    //  _applyRollCorrection() — shapes and submits roll corrections.
    //  rollRate_rs: _rollRateFiltered.
    // -------------------------------------------------------------------------
    void _applyRollCorrection(float ankle_deg, float hip_deg, float torso_deg,
                              float rollRate_rs);
};

#endif // BALANCE_CONTROLLER_H