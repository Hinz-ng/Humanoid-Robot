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
//   Send via WebSocket: CMD:BALANCE_TUNE:Kp=12.0,Kd=0.6,setpoint=0.0,ankle=0.8,hip=0.1,torso=0.1,max=15.0
//   NOTE: ankle + hip + torso must sum to 1.0. Firmware normalises automatically.
//   Toggle:             CMD:BALANCE_ON  /  CMD:BALANCE_OFF
//

// JOINT CHANNELS USED:
//   IDX_R_ANKLE_PITCH  (ch 1,  dir +1)
//   IDX_L_ANKLE_PITCH  (ch 14, dir -1)
//   IDX_R_HIP_PITCH    (ch 5,  dir +1)  — only if hip_ratio > 0
//   IDX_L_HIP_PITCH    (ch 6,  dir -1)  — only if hip_ratio > 0
//   IDX_TORSO_PITCH    (ch 9,  dir +1)  — only if torso_ratio > 0 (direction UNVERIFIED)
//
//   Direction is handled by JointModel — passing the same jointDeg value to
//   both left and right joints moves them anatomically symmetrically.
// =============================================================================

#include <Arduino.h>
#include "state_estimator.h"   // IMUState
#include "servo_control.h"     // ServoControl, IDX_* constants
#include "joint_config.h"      // IDX_R_ANKLE_PITCH, etc.

// Forward declaration — we only need a pointer; the full type is in motion_manager.h.
class MotionManager;

// ---------------------------------------------------------------------------
// BalanceConfig — all tuning parameters in one struct.
// Modify these values directly, or update at runtime via WebSocket.
// Every field is a named constant with a comment — no magic numbers elsewhere.
// ---------------------------------------------------------------------------
struct BalanceConfig {

    // --- Master switches (independently toggleable for testing) ---
    // pitch_enabled: enable the sagittal (forward/back) PD controller.
    // roll_enabled:  enable the frontal (left/right) PD controller.
    // Both default off — enable via WebSocket CMD:PITCH_ON / CMD:ROLL_ON.
    bool  pitch_enabled        = false;
    bool  roll_enabled         = false;

    // --- Upright target ---
    // The pitch (rad) the robot should hold. Start at 0.0 (perfectly vertical).
    // If the robot consistently leans at rest, trim this: e.g. 0.052 = +3 deg forward.
    // Range: roughly ±0.17 rad (±10°). Beyond that, the robot is visibly not upright.
    float pitch_setpoint_rad   = 0.0f;

    // --- PD gains ---
    // Units: Kp is deg/rad, Kd is deg/(rad/s).
    // Because error is in radians (small numbers), Kp must be large to produce
    // visible corrections. At 10° lean (0.175 rad), Kp=10 gives only 1.75° output.
    // Useful range for this robot: Kp=50–120, Kd=0.5–3.0.
    // Tuning order: set Kd=0, increase Kp to edge of oscillation, then add Kd to damp.
    float Kp                   = 10.0f;   // deg/rad
    float Kd                   = 0.5f;    // deg/(rad/s)
   
    // --- Joint distribution ---
    // Three ratios must sum to 1.0. Firmware normalises after every BALANCE_TUNE.
    //
    // ankle_ratio: fraction sent to ankle pitch joints. Start at 1.0. Tune first.
    // hip_ratio:   fraction sent to hip pitch joints. Start at 0.0. Enable after
    //              ankles are stable. Typical range: 0.1–0.4.
    // torso_ratio: fraction sent to torso pitch joint. Start at 0.0. Enable last,
    //              only after ankle + hip strategy is confirmed stable.
    float ankle_ratio          = 1.0f;
    float hip_ratio            = 0.0f;
    float torso_ratio          = 0.0f;

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

// =========================================================================
    //  ROLL CONTROLLER PARAMETERS
    //  Mirror of the pitch fields above. Tune after pitch is stable.
    //  ankle_roll_ratio + hip_roll_ratio + torso_roll_ratio must sum to 1.0.
    //  Firmware normalises after every BALANCE_TUNE roll field update.
    //
    //  CAUTION: verify roll_correction_sign empirically before enabling.
    //  Tilt right → observe roll sign in telemetry → set sign so correction
    //  pushes back toward upright (not further into the lean).
    // =========================================================================

    float Kp_roll                 = 5.0f;    // deg/rad  — start lower than Kp_pitch;
                                              //            5.6° stability margin is tight
    float Kd_roll                 = 0.2f;    // deg/(rad/s)
    float roll_setpoint_rad       = 0.0f;    // rad — target roll (0 = upright)
    float roll_correction_sign    = 1.0f;    // ±1 — verify empirically on first enable
    float ankle_roll_ratio        = 1.0f;    // fraction of u_roll sent to ankle roll joints
    float hip_roll_ratio          = 0.0f;    // fraction of u_roll sent to hip roll joints
    float torso_roll_ratio        = 0.0f;    // fraction of u_roll sent to torso roll joint
    float max_roll_correction_deg = 8.0f;    // hard clamp — tighter than pitch (5.6° margin)

    
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
    float torso_cmd_deg = 0.0f;   // final command to torso pitch joint (deg from neutral)
    bool  pitch_active       = false;   // true when pitch controller produced output
    bool  roll_active        = false;   // true when roll controller produced output
    float roll_error         = 0.0f;   // measured_roll - roll_setpoint (rad)
    float u_roll_raw         = 0.0f;   // roll PD output before clamping (deg)
    float u_roll_clamped     = 0.0f;   // roll PD output after clamping (deg)
    float ankle_roll_cmd_deg = 0.0f;   // command to each ankle roll joint (deg from neutral)
    float hip_roll_cmd_deg   = 0.0f;   // command to each hip roll joint (deg from neutral)
    float torso_roll_cmd_deg = 0.0f;   // command to torso roll joint (deg from neutral)
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

    // Wire the joint authority layer. Call once in setup() after motionManager.init().
    // Optional: if not called, _applyPitchCorrection() falls back to direct writes.
    void setMotionManager(MotionManager* mm)                { _motionManager = mm; }

private:
    ServoControl*  _servo;
    MotionManager* _motionManager = nullptr;  // joint authority layer — wired in setup()
    BalanceConfig  _cfg;
    BalanceState   _lastState;

    // Applies computed ankle/hip corrections.
    // Routes through MotionManager when wired (priority arbitration).
    // Falls back to direct ServoControl writes when MotionManager is absent
    // (preserves the pre-MotionManager behaviour for compatibility/testing).
    void _applyPitchCorrection(float ankle_deg, float hip_deg, float torso_deg);

    // Applies computed ankle/hip/torso roll corrections.
    // Identical routing logic to _applyPitchCorrection — uses roll IDX constants.
    void _applyRollCorrection(float ankle_deg, float hip_deg, float torso_deg);
};

#endif // BALANCE_CONTROLLER_H
