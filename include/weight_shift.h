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

// Direction names refer to the WIRE PROTOCOL value, NOT the physical CoM
// direction. With the adduction-led strategy and lateral_shift_mm < 0:
//   ShiftDirection::LEFT  (progress = +1) → CoM moves to the RIGHT foot
//                                            (right leg becomes stance, adducts;
//                                             left leg becomes swing,    abducts)
//   ShiftDirection::RIGHT (progress = -1) → CoM moves to the LEFT  foot
//
// The UI button "Shift Left ←" sends WEIGHT_SHIFT:right so that the
// user-facing label matches the physical effect. Do not "fix" the enum
// values without also reversing every dependent formula and comment block.
enum class ShiftDirection : int8_t {
    NONE  =  0,
    LEFT  =  1,   // wire-protocol name; physically shifts CoM RIGHT
    RIGHT = -1,   // wire-protocol name; physically shifts CoM LEFT
};

// Which leg is currently the swing leg, if any. Reported by GaitController via
// setSwingLegMask(); WeightShift uses it to scale the swing-side ankle_roll
// bias by (1 - liftFrac) so the airborne foot returns to the IK-computed flat
// ankle. NONE = both legs grounded -> full bias on both sides.
enum class SwingLeg : int8_t {
    NONE  = 0,
    LEFT  = 1,
    RIGHT = 2,
};

// ---------------------------------------------------------------------------
struct WeightShiftConfig {
    // Roll setpoint injected into BalanceConfig (rad, magnitude).
    // 0.050 rad ~= 2.9 deg. Active immediately on first left/right trigger;
    // suppressed when balance is in task_space_output mode.
    float setpoint_shift_rad = 0.050f;

    // Ankle roll amplitude (deg). Injected as bias into BalanceConfig, not via
    // SOURCE_GAIT. Set ankle_roll_ratio=0 in BalanceConfig to prevent the
    // balance loop from fighting its own baseline. 15.5 deg is the post-tuning
    // working default; override via WEIGHT_SHIFT_TUNE:ankle=...
    float ankle_shift_deg = 15.0f;
    
    // TASK-2: Phase delay between swing and stance ankle ramps (ms).
    // Swing ankle starts immediately; stance starts after this delay.
    // Set to 0 for simultaneous ramp (disables phasing).
    float shift_phase_delay_ms = 500.0f;

    // Stage 2: Body lateral shift (mm) at progress = ±1.
    // SIGN CONVENTION (current default is negative — adduction strategy):
    //   Negative: swing-side foot adducts inward toward the body midline while
    //             stance-side foot abducts. Net effect at the hip: foot tracks
    //             under CoM (adduction-led shift). This is the working default
    //             for first-step development.
    //   Positive: body translates laterally over a stationary stance foot
    //             (translation-led shift). Use only with the GaitController
    //             expecting the legacy convention.
    // Magnitude: tune in ~5 mm increments. Working range -20 to -5 mm; floor
    // at -50 / ceiling at +50 enforced by WebComm parser.
    // TUNING NOTE: Increased magnitude from -10 → -15 mm. More CoM transfer
    // means more weight on the stance leg at gate-open time, reducing drag risk.
    // If the robot leans excessively to one side, reduce toward -12 mm.
    float lateral_shift_mm = -20.0f;

    // Stage 2: Body forward shift (mm) at |progress| = 1. Replaces ankle_pitch_tilt_deg.
    // Both feet receive an x_mm offset of -|progress| × forward_lean_mm so the foot
    // sits behind the hip pitch axis when the body leans forward. 0 = disabled.
    float forward_lean_mm = 5.0f;

    // IIR smoothing factor for the forward-lean ramp (was ankle_tilt_smooth_alpha).
    // Applied as: smoothed = alpha*smoothed + (1-alpha)*raw. The lateral path uses
    // progress directly (already smoothed by the ramp); only the |progress|-driven
    // forward lean has a derivative discontinuity at zero crossing that needs IIR.
    // 0.90 mirrors UVC DYI_DAMPING_FACTOR. τ ≈ 25 ms at 400 Hz.
    float shift_smooth_alpha = 0.90f;

    // Torso roll (direction unverified). 0 = disabled.
    float torso_shift_deg = 0.0f;

    // Time for each ankle to ramp to full shift position (ms).
    // TUNING NOTE: Increased from 500 → 700 ms. Slower ramp gives ankle servos
    // more time to execute the commanded bias under load. The IMU gate blocks
    // foot lift regardless of ramp completion, so a longer ramp here only costs
    // per-step time, not stability. Reduce back toward 500 after first stable steps.
    float ramp_ms = 500.0f;

    // Minimum dwell at progress = ±1 before an opposite-direction trigger is
    // honoured. 0 disables the gate (legacy behaviour).
    float min_dwell_ms = 250.0f;
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

    // Publish swing-leg state for next tick's bias masking. Sticky: last value
    // persists if not called. Caller (GaitController) MUST republish each tick
    // - call setSwingLegMask(SwingLeg::NONE, 0.0f) when grounded.
    //
    //   liftFrac: [0, 1]; 0 = grounded, 1 = peak swing. Clamped internally.
    //
    // The swing leg's bias is scaled by (1 - liftFrac); stance leg unchanged.
    // Internal coupling only - does NOT change update()'s signature so existing
    // call sites in main.cpp remain valid.
    void setSwingLegMask(SwingLeg swing, float liftFrac);

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
    uint32_t _settledAtMs = 0;

    // Stage 2: IIR-smoothed body forward offset in mm. Was _ankleTiltCmdSmoothed
    // in deg-space; the smoothing now lives at the FootTarget.x_mm boundary.
    float _smoothedForwardLean_mm = 0.0f;

    // Swing-leg bias mask published by GaitController each tick. Defaults to
    // NONE so a forgotten publish leaves bias unscaled (safe legacy behavior).
    SwingLeg _swingMask     = SwingLeg::NONE;
    float    _swingLiftFrac = 0.0f;
};

#endif // WEIGHT_SHIFT_H
