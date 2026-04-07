// ============================================================
// FILE:    weight_shift.h
// MODULE:  gait
// LAYER:   3.5 — Gait / Motion (between balance and servo layers)
//
// PURPOSE:
//   Shifts the robot's centre of mass laterally in preparation
//   for single-leg support and walking. Two mechanisms:
//     1. Roll setpoint injection into BalanceConfig — the balance
//        controller then drives ankle roll joints to the new lean target.
//     2. Stance-leg hip roll command via SOURCE_GAIT — additive CoM shift.
//        Only enabled when hip_shift_deg > 0; balance hip ratio defaults to 0.
//
// INPUTS:  trigger(ShiftDirection) — LEFT / RIGHT / NONE (return to center)
// OUTPUTS: BalanceConfig::roll_setpoint_rad (modified each tick)
//          SOURCE_GAIT submissions to MotionManager for stance hip + torso
//
// DEPENDENCIES:
//   balance_controller.h — BalanceConfig, BalanceController
//   motion_manager.h     — MotionManager, SOURCE_GAIT
//   joint_config.h       — IDX_* channel constants
//
// LAST CHANGED: 2026-03-26  |  Hinz  |  Initial implementation.
// ============================================================

#ifndef WEIGHT_SHIFT_H
#define WEIGHT_SHIFT_H

#include <Arduino.h>
#include "joint_config.h"
#include "motion_manager.h"

// Forward declaration — avoids circular include with balance_controller.h.
// WeightShift needs to call _bal->setConfig() / _bal->getConfig().
class BalanceController;
struct BalanceConfig;

// ---------------------------------------------------------------------------
// ShiftDirection — which way to move the CoM.
// Numeric values are chosen so that (int)dir is a natural sign multiplier:
//   LEFT  = +1 → progress ramps to +1.0
//   RIGHT = -1 → progress ramps to -1.0
//   NONE  =  0 → progress ramps to  0.0 (return to center)
// ---------------------------------------------------------------------------
enum class ShiftDirection : int8_t {
    NONE  =  0,   // "center" in WebSocket protocol — returns to zero lean
    LEFT  =  1,
    RIGHT = -1,
};

// ---------------------------------------------------------------------------
// WeightShiftConfig — all tuning parameters in one struct.
// All defaults are conservative (setpoint only, no hip/torso, slow ramp).
// Enable hip_shift_deg and torso_shift_deg only after setpoint-only shift is stable.
// ---------------------------------------------------------------------------
struct WeightShiftConfig {

    // Roll setpoint injected into the balance controller (radians).
    // Positive = lean right, negative = lean left (matches state_estimator sign convention).
    // WeightShift negates the sign correctly for each direction — set this as a magnitude.
    //   0.05 rad ≈ 2.9°. Start here. Do not exceed 0.10 rad until stability is confirmed.
    float setpoint_shift_rad = 0.050f;  // rad — magnitude; sign applied per direction

    // Ankle roll feedforward (degrees from neutral), applied to BOTH ankles.
    // This is the PRIMARY CoM shift driver. Set ankle_roll_ratio=0 in
    // BalanceConfig when using this — SOURCE_BALANCE (priority 2) overrides
    // SOURCE_GAIT (priority 1) on the same joint if that ratio is non-zero.
    //
    // Sign convention mirrors _applyRollCorrection() in balance_controller.cpp:
    //   progress > 0 (LEFT shift): IDX_L_ANKLE_ROLL = +cmd, IDX_R_ANKLE_ROLL = -cmd
    // Flip sign via the UI if shift direction is wrong on hardware.
    //
    // Tuning path:
    //   1. Confirm setpoint-only shift works (leave this at 0).
    //   2. Set ankle_roll_ratio=0 in BalanceConfig (UI slider).
    //   3. Start at 5°, verify direction, increase toward ~13°.
    float ankle_shift_deg    = 0.0f;   // deg — 0 = disabled; ~13° for single-leg stance

    // Forward ankle pitch tilt applied during weight shift (degrees, joint-relative).
    // Scaled by |progress| so it ramps in smoothly with the weight shift.
    // Applied equally to both ankles regardless of shift direction (left or right).
    // JointModel direction fields (+1 right, -1 left) produce symmetric motion
    // automatically — do NOT negate for one side manually.
    //
    // To adjust: change ANKLE_PITCH_FORWARD_TILT_DEG in project_wide_defs.h.
    // To reverse direction: set a negative value here or negate the constant.
    // To disable: set to 0.0f.
    float ankle_pitch_tilt_deg = ANKLE_PITCH_FORWARD_TILT_DEG;  // deg — default 3.0°

    // Hip roll command sent to STANCE-SIDE hip only (degrees from neutral).
    // Enable only after ankle feedforward shift is stable.
    // If correction goes the wrong way, negate this value.
    float hip_shift_deg      = 0.0f;   // deg — 0 = disabled

    // Torso roll command sent to IDX_TORSO_ROLL (degrees from neutral).
    // Single joint — direction unverified on hardware; verify before enabling.
    // Start at 0 (disabled).
    float torso_shift_deg    = 0.0f;   // deg — 0 = disabled

    // Time to ramp from no-shift to full-shift (and back), in milliseconds.
    // Short ramp → large impulse → fall risk.
    // Start at 500 ms. Reduce only after the full shift is stable.
    float ramp_ms            = 500.0f;  // ms

    // ── Swing leg lift ────────────────────────────────────────────────────
    // Feedforward lift applied to the swing leg (opposite to shift direction)
    // via SOURCE_GAIT. No UI sliders — tune here before reflashing.
    //
    // Clearance model (from robot_geometry.h):
    //   L_THIGH=96mm, L_SHANK=95.4mm, L_ANKLE_TO_SOLE=15.4mm → neutral=206.8mm
    //   formula: clearance = 206.8 − (L_THIGH×cos(θh) + L_SHANK×cos(θh−θk) + 15.4)
    //   θh=15°, θk=40° → clearance = 12.2mm (flat floor adequate).
    //   If leg geometry changes, recalculate before changing these values.
    //
    // PRIORITY CONFLICT GUARD:
    //   SOURCE_GAIT (1) < SOURCE_BALANCE (2). If BalanceConfig::hip_ratio > 0,
    //   the balance controller overrides the swing hip pitch command.
    //   Keep hip_ratio = 0 in BalanceConfig while using swing leg lift.
    //   Knee joints are never touched by the balance controller — no conflict there.
    float swing_hip_extension_deg = 15.0f;  // deg — magnitude; applied as negative joint angle
    float swing_knee_flexion_deg  = 60.0f;  // deg — magnitude; applied as positive joint angle

    // Progress fraction at which the lift begins, in [0, 1).
    // lift_scale ramps linearly from 0 at threshold to 1 at |progress|=1.0.
    // Ensures sufficient weight transfer before the foot leaves the ground.
    // Start at 0.5 — raise toward 0.7 if the initial lift destabilises the shift.
    float swing_lift_threshold    = 0.50f;  // dimensionless
};

// ---------------------------------------------------------------------------
// WeightShiftState — observable state for telemetry and UI feedback.
// ---------------------------------------------------------------------------
struct WeightShiftState {
    ShiftDirection direction    = ShiftDirection::NONE;
    float          progress     = 0.0f;   // −1.0 = full right, 0.0 = center, +1.0 = full left
    bool           ramping      = false;  // true while ramp is in progress
};

// ---------------------------------------------------------------------------
// WeightShift
// ---------------------------------------------------------------------------
class WeightShift {
public:
    WeightShift() = default;

    // -------------------------------------------------------------------------
    //  init()  —  wire up dependencies. Call once in setup() after both objects
    //  are constructed. Both pointers are required; null either → no-op on update().
    // -------------------------------------------------------------------------
    void init(BalanceController* bal, MotionManager* mm);

    // -------------------------------------------------------------------------
    //  trigger()  —  start a ramp toward the given direction.
    //  Safe to call while already ramping: resumes from current progress.
    //  Calling trigger(NONE) ramps back to center (roll_setpoint_rad → 0).
    // -------------------------------------------------------------------------
    void trigger(ShiftDirection dir);

    // -------------------------------------------------------------------------
    //  update()  —  advance ramp, inject setpoint, submit SOURCE_GAIT commands.
    //  MUST be called before balanceController.update() in the 400 Hz gate so
    //  the injected setpoint is used by the balance controller on the same tick.
    //  dt_s: seconds since last call — pass the same dt used by the balance loop.
    // -------------------------------------------------------------------------
    void update(float dt_s);

    WeightShiftConfig       getConfig() const { return _cfg; }
    void                    setConfig(const WeightShiftConfig& cfg) { _cfg = cfg; }
    const WeightShiftState& getState()  const { return _state; }

private:
    BalanceController* _bal            = nullptr;
    MotionManager*     _mm             = nullptr;
    WeightShiftConfig  _cfg;
    WeightShiftState   _state;
    float              _targetProgress        = 0.0f;   // ramp target: −1, 0, or +1
    float              _lastInjectedSetpointRad = 0.0f;  // prevents redundant setConfig() calls
};

#endif // WEIGHT_SHIFT_H
