// =============================================================================
// FILE:    weight_shift.h
// MODULE:  gait
// LAYER:   3.5 — Gait / Motion
//
// TASK-2 CHANGES:
//   1. Phased ankle shifting: swing ankle ramps first, stance follows after delay.
//   2. Ankle roll conflict fix: SOURCE_GAIT ankle roll removed. Bias injected
//      into BalanceConfig so balance controller layers additively on top.
//   3. Same-direction shift: both ankles tilt in same physical direction.
//      Sign convention matches balance controller's _applyRollCorrection.
//   4. Per-ankle progress: _rightAnkleProgress / _leftAnkleProgress track
//      each ankle independently with separate delay timers.
//
// LAST CHANGED: 2026-04-08 | Hinz | Task-2 phased shift + ankle bias
// =============================================================================

#ifndef WEIGHT_SHIFT_H
#define WEIGHT_SHIFT_H

#include <Arduino.h>
#include "joint_config.h"
#include "motion_manager.h"

class BalanceController;
struct BalanceConfig;

enum class ShiftDirection : int8_t {
    NONE  =  0,
    LEFT  =  1,
    RIGHT = -1,
};

// ---------------------------------------------------------------------------
struct WeightShiftConfig {
    // Roll setpoint injected into BalanceConfig (rad, magnitude).
    float setpoint_shift_rad = 0.050f;

    // Ankle roll amplitude (deg). Injected as bias into BalanceConfig, not via
    // SOURCE_GAIT. Set ankle_roll_ratio=0 in BalanceConfig to prevent the
    // balance loop from fighting its own baseline.
    // DEFAULT CHANGE: was 0.0f in previous codebase. Changed to 5.0f to enable
    // ankle roll feedforward by default. This is injected as a baseline offset
    // into BalanceConfig each tick — NOT submitted via SOURCE_GAIT.
    //
    // IMPORTANT: At boot, if roll controller is also enabled, the 5° bias will
    // produce a visible 5° lean in the direction of any active weight shift.
    // This is correct behaviour. To disable the default bias, set to 0.0f here
    // or send: CMD:WEIGHT_SHIFT_TUNE:ankle=0 before triggering weight shifts.
    float ankle_shift_deg = 15.0f;
    
    // TASK-2: Phase delay between swing and stance ankle ramps (ms).
    // Swing ankle starts immediately; stance starts after this delay.
    // Set to 0 for simultaneous ramp (disables phasing).
    float shift_phase_delay_ms = 500.0f;

    // Ankle pitch tilt during shift. Scaled by |progress|, both ankles.
    float ankle_pitch_tilt_deg = ANKLE_PITCH_FORWARD_TILT_DEG;

    // Hip roll on stance leg. 0 = disabled.
    float hip_shift_deg = 0.0f;

    // Torso roll (direction unverified). 0 = disabled.
    float torso_shift_deg = 0.0f;

    // Time for each ankle to ramp to full shift position (ms).
    float ramp_ms = 500.0f;

    // Swing leg lift parameters.
    float swing_hip_extension_deg = 0.0f;
    float swing_knee_flexion_deg  = 0.0f;
    float swing_lift_threshold    = 0.50f;  // [0, 1)
};

// ---------------------------------------------------------------------------
struct WeightShiftState {
    ShiftDirection direction      = ShiftDirection::NONE;
    float          progress       = 0.0f;  // average progress [-1, +1]
    float          right_progress = 0.0f;  // right ankle progress
    float          left_progress  = 0.0f;  // left ankle progress
    bool           ramping        = false;
};

// ---------------------------------------------------------------------------
class WeightShift {
public:
    WeightShift() = default;

    void init(BalanceController* bal, MotionManager* mm);

    // Trigger a shift. Swing starts immediately, stance waits phase_delay_ms.
    // trigger(NONE) returns both to center with no delay.
    void trigger(ShiftDirection dir);

    // Call every tick BEFORE balanceController.update().
    void update(float dt_s);

    WeightShiftConfig       getConfig() const { return _cfg; }
    void                    setConfig(const WeightShiftConfig& cfg) { _cfg = cfg; }
    const WeightShiftState& getState()  const { return _state; }

private:
    BalanceController* _bal = nullptr;
    MotionManager*     _mm  = nullptr;
    WeightShiftConfig  _cfg;
    WeightShiftState   _state;

    // Per-ankle independent ramp state.
    float _rightAnkleProgress = 0.0f;
    float _leftAnkleProgress  = 0.0f;
    float _targetRight        = 0.0f;
    float _targetLeft         = 0.0f;
    float _rightDelayRemaining_ms = 0.0f;
    float _leftDelayRemaining_ms  = 0.0f;

    float _lastInjectedSetpointRad = 0.0f;
};

#endif // WEIGHT_SHIFT_H