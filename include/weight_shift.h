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
    // Default 0 keeps manual shift buttons motionless until the operator
    // explicitly dials in a lean target from the UI.
    float setpoint_shift_rad = 0.050f;

    // Ankle roll amplitude (deg). Injected as bias into BalanceConfig, not via
    // SOURCE_GAIT. Set ankle_roll_ratio=0 in BalanceConfig to prevent the
    // balance loop from fighting its own baseline. Default 0 keeps the
    // panel's first left/right trigger as a no-op until tuning is intentional.
    float ankle_shift_deg = 5.0f;
    
    // TASK-2: Phase delay between swing and stance ankle ramps (ms).
    // Swing ankle starts immediately; stance starts after this delay.
    // Set to 0 for simultaneous ramp (disables phasing).
    float shift_phase_delay_ms = 500.0f;

    // Stage 2: Body lateral shift (mm) at progress = ±1. Replaces hip_shift_deg.
    // Translates to a y_mm offset on both feet (each in its own frontal frame).
    // Sign convention: progress = +1 (LEFT shift) → body moves left.
    //   Right leg y_mm += lateral_shift_mm  (right foot more outward of right hip)
    //   Left  leg y_mm -= lateral_shift_mm  (left foot more inward  of left  hip)
    // 0 = disabled. Negative values intentionally invert the mapping so hip
    // adduction / inward-foot experiments can be tested from the UI.
    // Tune magnitude to ~10–25 mm during Stage 2 hardware testing.
    float lateral_shift_mm = 0.0f;

    // Stage 2: Body forward shift (mm) at |progress| = 1. Replaces ankle_pitch_tilt_deg.
    // Both feet receive an x_mm offset of -|progress| × forward_lean_mm so the foot
    // sits behind the hip pitch axis when the body leans forward. 0 = disabled.
    float forward_lean_mm = 0.0f;

    // IIR smoothing factor for the forward-lean ramp (was ankle_tilt_smooth_alpha).
    // Applied as: smoothed = alpha*smoothed + (1-alpha)*raw. The lateral path uses
    // progress directly (already smoothed by the ramp); only the |progress|-driven
    // forward lean has a derivative discontinuity at zero crossing that needs IIR.
    // 0.90 mirrors UVC DYI_DAMPING_FACTOR. τ ≈ 25 ms at 400 Hz.
    float shift_smooth_alpha = 0.90f;

    // Torso roll (direction unverified). 0 = disabled.
    float torso_shift_deg = 0.0f;

    // Time for each ankle to ramp to full shift position (ms).
    float ramp_ms = 500.0f;
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

    // Hard-reset all internal shift state immediately.
    // Used by oe_clear() so no stale lean or ankle bias survives re-enable.
    void forceCenterImmediate();

    WeightShiftConfig       getConfig() const { return _cfg; }
    void                    setConfig(const WeightShiftConfig& cfg) { _cfg = cfg; }
    const WeightShiftState& getState()  const { return _state; }

    // Stage 2: Returns this tick's task-space contribution to FootTarget for one
    // leg. Caller adds the offsets to FootTarget.x_mm / FootTarget.y_mm before
    // LegIK::solve. Returns zero offsets if estopped. See WeightShiftConfig:
    // forward_lean_mm (signed by |progress|, smoothed) and lateral_shift_mm
    // (signed by progress and per-leg side).
    void getStanceShift(bool isRightLeg,
                        float& x_offset_mm,
                        float& y_offset_mm) const;

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

    // Stage 2: IIR-smoothed body forward offset in mm. Was _ankleTiltCmdSmoothed
    // in deg-space; the smoothing now lives at the FootTarget.x_mm boundary.
    float _smoothedForwardLean_mm = 0.0f;
};

#endif // WEIGHT_SHIFT_H
