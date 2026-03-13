#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

// =============================================================================
// balance_controller.h
// Purpose : PD pitch balance controller — all tuning knobs in one place.
//
// TUNING GUIDE (do in order, don't skip steps):
//   1. Flash with enabled=false. Confirm robot stands neutral. Verify IMU sign:
//      lean forward → pitch goes positive. If not, flip pitch_gyro_sign in FilterConfig.
//   2. Set Kd=0, Kp=5, correction_sign=1. Enable. Push robot gently forward.
//      If ankles correct in the right direction, sign is correct.
//      If corrections make the lean worse, set correction_sign=-1.
//   3. Increase Kp slowly until the robot just starts to oscillate (~10-15).
//   4. Increase Kd until oscillations stop (~0.3-0.8). Expect some chatter if too high.
//   5. Increase Kp again. Alternate Kp/Kd until corrections feel crisp and damped.
//   6. If robot leans at rest after tuning, adjust pitch_setpoint_rad.
//      Example: consistently leans 3° forward → set setpoint to +0.052 rad.
//   7. Only after ankle is stable: enable hip_ratio=0.2 and tune from there.
//
// RUNTIME TUNING (no reflash needed):
//   Send via WebSocket: CMD:BAL_CONFIG:Kp=12.0,Kd=0.6,setpoint=0.0,max_corr=15.0
//   Toggle:             CMD:BAL_ENABLE  /  CMD:BAL_DISABLE
//
// JOINT CHANNELS USED:
//   IDX_R_ANKLE_PITCH (ch 1,  dir +1)
//   IDX_L_ANKLE_PITCH (ch 14, dir -1)
//   IDX_R_HIP_PITCH   (ch 5,  dir +1)   — only if hip_ratio > 0
//   IDX_L_HIP_PITCH   (ch 10, dir -1)   — only if hip_ratio > 0
//
//   Direction is handled by JointModel — passing the same jointDeg value to
//   both left and right joints moves them anatomically symmetrically.
// =============================================================================

#include <Arduino.h>
#include "state_estimator.h"   // IMUState
#include "servo_control.h"     // ServoControl, IDX_* constants
#include "joint_config.h"      // IDX_R_ANKLE_PITCH, etc.

// ---------------------------------------------------------------------------
// BalanceConfig — all tuning parameters in one struct.
// Modify these values directly, or update at runtime via WebSocket.
// Every field is a named constant with a comment — no magic numbers elsewhere.
// ---------------------------------------------------------------------------
struct BalanceConfig {

    // --- Master switch ---
    // Must be explicitly set to true. Safe default is off.
    bool  enabled              = false;

    // --- Upright target ---
    // The pitch (rad) the robot should hold. Start at 0.0 (perfectly vertical).
    // If the robot consistently leans at rest, trim this: e.g. 0.052 = +3 deg forward.
    // Range: roughly ±0.17 rad (±10°). Beyond that, the robot is visibly not upright.
    float pitch_setpoint_rad   = 0.0f;

    // --- PD gains ---
    // Kp: how strongly to react to position error. Too high → oscillates.
    // Kd: how strongly to react to rate (velocity). Too high → high-freq chatter.
    // Start with Kd=0 to find Kp first, then add Kd to damp.
    float Kp                   = 10.0f;
    float Kd                   = 0.5f;

    // --- Joint distribution ---
    // ankle_ratio: fraction of the control output applied to ankle pitch joints.
    //              Start at 1.0 (full authority to ankles). Tune this first.
    // hip_ratio:   fraction applied to hip pitch joints.
    //              Start at 0.0. Enable only after ankles are stable.
    //              Range 0.1–0.4. Hip assists with larger disturbances.
    float ankle_ratio          = 1.0f;
    float hip_ratio            = 0.0f;

    // --- Correction direction ---
    // +1.0: positive pitch (leaning forward) → positive joint correction (corrects lean).
    // -1.0: flip this if corrections make the lean worse instead of better.
    // Verify empirically after first enable. Do not guess.
    float correction_sign      = 1.0f;

    // --- Output clamp ---
    // Maximum joint angle correction in degrees. Hard limit applied after PD.
    // Prevents a bad Kp from commanding destructive 90° corrections.
    // Start at 15°. Tighten to 10° once tuning is stable.
    float max_correction_deg   = 15.0f;

    // --- Fall threshold ---
    // If |pitch| exceeds this, the robot has fallen past recoverable range.
    // Triggers ESTOP to protect servos. ~30° = 0.524 rad.
    // Do not raise above 0.70 rad (~40°) — the physics don't support recovery there.
    float fall_threshold_rad   = 0.524f;
};

// ---------------------------------------------------------------------------
// BalanceState — output snapshot from one update() call.
// Broadcast to UI at 20 Hz so you can watch the controller in real time.
// ---------------------------------------------------------------------------
struct BalanceState {
    bool  active        = false;   // true when enabled AND estimator valid AND no estop
    float pitch_error   = 0.0f;   // measured_pitch - setpoint (rad). 0 = on target.
    float u_raw         = 0.0f;   // PD output before clamping (deg). Shows saturation.
    float u_clamped     = 0.0f;   // PD output after clamping (deg). What joints receive.
    float ankle_cmd_deg = 0.0f;   // final command to each ankle pitch joint (deg from neutral)
    float hip_cmd_deg   = 0.0f;   // final command to each hip pitch joint (deg from neutral)
    bool  fell          = false;   // true if fall_threshold was exceeded this tick → ESTOP fired
};

// ---------------------------------------------------------------------------
// BalanceController
// ---------------------------------------------------------------------------
class BalanceController {
public:
    // Constructor. Pass a ServoControl pointer (same pattern as WebComm).
    explicit BalanceController(ServoControl* servo = nullptr);

    // Call once in setup() after servoController.init().
    void init();

    // Main entry point — call every tick inside the 400 Hz gate in main.cpp,
    // AFTER stateEstimator.update() has run.
    // Returns a BalanceState for telemetry. Writes joint corrections to hardware.
    BalanceState update(const IMUState& state);

    // Runtime config — WebComm calls these when CMD:BAL_CONFIG / BAL_ENABLE arrives.
    void                 setConfig(const BalanceConfig& cfg) { _cfg = cfg; }
    const BalanceConfig& getConfig() const                   { return _cfg; }
    BalanceState         getLastState() const                { return _lastState; }

private:
    ServoControl* _servo;
    BalanceConfig _cfg;
    BalanceState  _lastState;

    // Applies computed ankle/hip corrections to hardware immediately.
    // Uses applyBalanceOffset() which bypasses smooth stepping (immediate=true).
    //
    // FUTURE: when the MotionArbiter pattern is implemented (Phase 3, walking),
    // this call will be replaced with arbiter->setBalanceDelta(ch, deg) so the
    // gait generator and balance controller can coexist on the same joints.
    void _applyPitchCorrection(float ankle_deg, float hip_deg);
};

#endif // BALANCE_CONTROLLER_H