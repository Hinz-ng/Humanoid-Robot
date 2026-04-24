// =============================================================================
// FILE:    gait_controller.h
// MODULE:  gait
// LAYER:   3.5 — Gait / Motion
//
// PURPOSE:
//   Open-loop walking gait generator. Produces continuous alternating-leg
//   foot trajectories in task space (x_mm, h_mm, y_mm per leg), passes them
//   through LegIK::solve(), and submits results via MotionManager at
//   SOURCE_GAIT priority.
//
//   ARCHITECTURE:
//     GaitController submits SOURCE_GAIT (priority 1).
//     BalanceController submits SOURCE_BALANCE (priority 2, wins on shared joints).
//     For open-loop walking: disable balance (CMD:PITCH_OFF, CMD:ROLL_OFF).
//     For closed-loop walking later: enable balance; it corrects on top of gait.
//
//   LATERAL SHIFT:
//     Handled internally via sinusoidal y-modulation of both foot targets.
//     WeightShift is reset to center on start() — no coupling during walking.
//     This keeps the first open-loop attempt self-contained.
//
//   STEP CYCLE:
//     One _phase variable [0,1) advances over one half-cycle (one leg's motion).
//     When it wraps, legs swap roles. Both legs always use identical trajectory
//     shapes — no special-case code for left vs. right.
//
// INPUTS:  GaitConfig (tunable at runtime via CMD:GAIT_TUNE), float dt_s
// OUTPUTS: IK-solved joint angles via MotionManager::submit(SOURCE_GAIT, ...)
//
// DEPENDENCIES:
//   leg_ik.h         — LegIK::solve(), FootTarget, LegIKResult, IKStatus
//   motion_manager.h — MotionManager::submit(), SOURCE_GAIT
//   weight_shift.h   — WeightShift::trigger() (reset only, on start/stop)
//   oe_control.h     — oe_is_estopped() gate
//   joint_config.h   — IDX_* channel constants
//
// LAST CHANGED: 2026-04-22 | Hinz | Initial implementation
// =============================================================================

#ifndef GAIT_CONTROLLER_H
#define GAIT_CONTROLLER_H

#include <Arduino.h>
#include "leg_ik.h"
#include "motion_manager.h"
#include "weight_shift.h"

// =============================================================================
//  GaitConfig — all tunable parameters. Adjust at runtime via CMD:GAIT_TUNE.
//
//  DESIGN NOTE:
//   These are task-space parameters. They describe the foot's motion in the
//   body frame, NOT servo angles. LegIK converts them. This separation is
//   intentional: you can tune step shape without understanding the joint chain.
// =============================================================================
struct GaitConfig {

    // ── Step geometry ─────────────────────────────────────────────────────────

    // Total foot travel back-to-front per step (mm).
    // Foot moves from -step_length/2 to +step_length/2 during swing.
    // At 20mm: foot moves ±10mm around center. Conservative first value.
    // Do not exceed 40mm before verifying IK clearance at the new value.
    float step_length_mm   = 20.0f;  // mm. Total forward foot travel per step.

    // Peak foot clearance above ground at mid-swing (mm).
    // Minimum for flat ground: ~8mm. Increase if robot scuffs.
    // CONSTRAINT: stance_height_mm - step_height_mm must be >= 130mm.
    // At defaults: 160 - 15 = 145mm. ✓
    float step_height_mm   = 15.0f;  // mm. Foot lift at mid-swing.

    // Nominal sagittal chain drop hip-pitch-axis → ankle-pitch-axis (mm).
    // Higher = more upright stance (less knee bend, closer to singularity).
    // At 160mm: knee ≈ 67° — confirmed by IK validation in setup().
    // Safe range: 130mm (squatted) to 185mm (singularity — avoid).
    // Run LegIK::validateRoundtrip() if you change this.
    float stance_height_mm = 160.0f; // mm. Nominal sagittal chain height.

    // Each foot's lateral offset outward from centerline at rest (mm).
    // CONSTRAINT: stance_width_mm + lateral_shift_mm drives the worst-case
    //   hip_roll angle. At defaults (25+12=37mm), worst case at mid-swing
    //   is hip_roll = atan2(37, 119.6) = 17.2° < IK_HIP_ROLL_MAX (20°). ✓
    float stance_width_mm  = 25.0f;  // mm. Foot-to-centerline lateral offset.

    // Lateral CoM shift amplitude (mm). Body shifts over stance foot by this
    // amount at mid-swing via sinusoidal y-modulation of foot targets.
    // Physics: the stance foot appears to move inward relative to the hip
    //   (body leans over it), while the swing foot moves outward.
    // CONSTRAINT: keep stance_width + lateral_shift < ~37mm to stay within
    //   IK hip roll limit at mid-swing (h_frontal ≈ 120mm at full lift).
    //   Default 12mm is safe. Do not raise past 15mm without re-checking IK.
    float lateral_shift_mm = 12.0f;  // mm. Peak CoM lateral shift from center.

    // ── Timing ───────────────────────────────────────────────────────────────

    // Duration of one complete gait cycle (both legs each take one step) (s).
    // Half-cycle = cycle_period_s / 2 = time for one leg to complete its motion.
    // Start slow (1.6s). Decrease only after first walking attempt is stable.
    // TRADEOFF: shorter → faster but less time for servos to reach targets.
    float cycle_period_s   = 1.6f;   // s. Full gait cycle duration.

    // Fraction of each half-cycle [0, 0.40) spent in double-support before
    // the swing foot lifts. Swing foot is clamped to its start position during
    // this window, giving the lateral shift time to begin.
    // At 0.20 + cycle 1.6s: double-support = 0.20 × 0.8s = 160ms per step.
    float double_support_frac = 0.20f;  // [0, 0.40). Pre-liftoff double support.

    bool enabled = false;  // Controlled via start()/stop() or CMD:GAIT_START/STOP.
};

// =============================================================================
//  GaitState — read-only telemetry. Not for external modification.
// =============================================================================
struct GaitState {
    bool  active         = false;   // true while gait is running
    float phase          = 0.0f;   // [0,1) within current half-cycle
    bool  right_is_swing = true;   // which leg is currently airborne

    // Task-space foot targets in body frame (mm) — for telemetry display.
    float x_right_mm     = 0.0f;   // right foot x target (forward positive)
    float h_right_mm     = 0.0f;   // right foot sagittal chain height
    float x_left_mm      = 0.0f;
    float h_left_mm      = 0.0f;

    // IK solution quality. DOMAIN_ERROR triggers stop(). LIMIT_CLAMPED continues.
    IKStatus right_ik_status = IKStatus::OK;
    IKStatus left_ik_status  = IKStatus::OK;
};

// =============================================================================
//  GaitController
// =============================================================================
class GaitController {
public:
    GaitController() = default;

    // Call once in setup(), AFTER motionManager.init() and weightShift.init().
    void init(MotionManager* mm, WeightShift* ws);

    // Start walking. Resets phase to 0, right leg swings first.
    // Calls weightShift.trigger(NONE) so WeightShift doesn't fight the gait's
    // IK-computed ankle/hip roll commands.
    void start();

    // Stop walking gracefully. Resets WeightShift to center.
    // Does not call oe_estop() — servos stay powered at last commanded position.
    void stop();

    // Call every tick in the 400Hz gate in this order:
    //   weightShift.update(dt)       <- runs but is neutral after start()
    //   gaitController.update(dt)    <- THIS
    //   balanceController.update()
    //   motionManager.flush()
    void update(float dt_s);

    void              setConfig(const GaitConfig& cfg) { _cfg = cfg; }
    const GaitConfig& getConfig() const                { return _cfg; }
    const GaitState&  getState()  const                { return _state; }

private:
    MotionManager* _mm = nullptr;
    WeightShift*   _ws = nullptr;
    GaitConfig     _cfg;
    GaitState      _state;

    // Submit SOURCE_GAIT commands for all 5 joints of one leg.
    // Identical anatomical angles for both legs — JointModel direction fields
    // in joint_config.cpp handle physical mirroring for left-side joints.
    void _submitLeg(bool isRight, const LegIKResult& ik);
};

#endif // GAIT_CONTROLLER_H