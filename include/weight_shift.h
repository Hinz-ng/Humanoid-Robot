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
    NONE  =  0,
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

    // Hip roll command sent to STANCE-SIDE hip only (degrees from neutral).
    // The swing leg's hip roll is deliberately NOT commanded by this module.
    // Start at 0 (disabled). Enable only after ankle-only shift is stable.
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
    float              _targetProgress = 0.0f;  // ramp target: −1, 0, or +1
};

#endif // WEIGHT_SHIFT_H