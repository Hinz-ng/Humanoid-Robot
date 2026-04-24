// =============================================================================
// FILE:    gait_controller.cpp
// MODULE:  gait
// LAYER:   3.5 — Gait / Motion
//
// TRAJECTORY MATH:
//
//   SWING FOOT (cosine interpolation):
//     x_swing = -(L/2) * cos(π * swing_phase)
//     h_swing = h_stance - h_lift * sin(π * swing_phase)
//     y_swing = stance_width + lateral_shift * sin(π * phase)
//
//     Derivation of x formula:
//       At swing_phase=0: x = -L/2 * cos(0) = -L/2  (foot at back)
//       At swing_phase=1: x = -L/2 * cos(π) = +L/2  (foot at front)
//       Velocity ∝ sin(π*t): zero at endpoints → smooth liftoff and landing. ✓
//
//     Derivation of h formula:
//       sin(0)=0 and sin(π)=0 → foot at ground level at start and end.
//       sin(π/2)=1 → max clearance at mid-swing. ✓
//       h DECREASES during swing because h is chain drop (downward positive).
//       Decreasing h = foot rising. ✓
//
//   STANCE FOOT (linear):
//     x_stance = (L/2) * (1 - 2 * phase)
//     h_stance = stance_height_mm   (constant — body at fixed height)
//     y_stance = stance_width - lateral_shift * sin(π * phase)
//
//     Physical meaning: body moves forward at constant speed; in body frame
//     the stance foot moves backward linearly. Linear is correct for constant-
//     speed motion. Cosine (smooth) would imply sinusoidal body acceleration.
//
//   POSITION CONTINUITY (algebraic proof):
//     At phase=1 (old swing becomes new stance):
//       x_swing_end   = -(L/2)*cos(π*1) = +L/2
//       x_stance_start (new phase=0) = (L/2)*(1-0) = +L/2 ✓
//     At phase=1 (old stance becomes new swing):
//       x_stance_end  = (L/2)*(1-2*1) = -L/2
//       x_swing_start (new swing_phase=0) = -(L/2)*cos(0) = -L/2 ✓
//     No position jump at any transition. ✓
//
//   LATERAL SHIFT:
//     Both foot y-targets are modulated by sin(π * phase), which is 0 at
//     transitions and 1 at mid-cycle. Direction is opposite for each foot:
//       stance foot y DECREASES (foot appears inward → body leans over it)
//       swing foot  y INCREASES (foot appears outward)
//     This is equivalent to the UVC "dy" CoM correction.
//     Implemented here rather than via WeightShift to avoid ramp_ms timing
//     coupling for the open-loop first attempt.
//
//   DOUBLE-SUPPORT WINDOW:
//     For phase in [0, ds_frac), swing_phase is clamped to 0.0 so the swing
//     foot stays at x=-L/2, h=stance_height (on the ground). This prevents
//     the foot from lifting before the lateral shift has started.
//     Note: lateral shift DOES begin at phase=0 (sin(0)=0, but increases
//     immediately), so the body starts leaning as soon as the step begins.
//
// LAST CHANGED: 2026-04-22 | Hinz | Initial implementation
// =============================================================================

#include "gait_controller.h"
#include "joint_config.h"
#include "oe_control.h"
#include <math.h>

// ---------------------------------------------------------------------------
void GaitController::init(MotionManager* mm, WeightShift* ws) {
    if (mm == nullptr) {
        Serial.println("[GaitController] ERROR: null MotionManager — update() will no-op.");
    }
    _mm    = mm;
    _ws    = ws;  // ws may be null (lateral shift still works via y-modulation)
    _state = {};
    Serial.println("[GaitController] Initialized.");
}

// ---------------------------------------------------------------------------
void GaitController::start() {
    if (_mm == nullptr) {
        Serial.println("[GaitController] start() failed: MotionManager not wired.");
        return;
    }

    // Reset phase and choose the starting swing leg.
    // Right leg swings first so the robot's first motion is predictable.
    _state              = {};
    _state.active       = true;
    _state.right_is_swing = true;
    _state.phase        = 0.0f;
    _cfg.enabled        = true;

    // Take WeightShift to neutral. While it ramps to zero over its configured
    // ramp_ms, GaitController's SOURCE_GAIT submissions will overwrite any
    // WeightShift SOURCE_GAIT commands on shared joints. The ankle roll BIAS
    // from WeightShift (injected into BalanceConfig) will also fade to zero.
    // This ensures clean joint ownership once the gait starts.
    if (_ws) _ws->trigger(ShiftDirection::NONE);

    Serial.printf("[GaitController] START: step_len=%.1fmm  lift=%.1fmm  "
                  "h=%.1fmm  width=%.1fmm  lat=%.1fmm  period=%.2fs  ds=%.2f\n",
                  _cfg.step_length_mm, _cfg.step_height_mm,
                  _cfg.stance_height_mm, _cfg.stance_width_mm,
                  _cfg.lateral_shift_mm, _cfg.cycle_period_s,
                  _cfg.double_support_frac);
}

// ---------------------------------------------------------------------------
void GaitController::stop() {
    _cfg.enabled  = false;
    _state.active = false;

    // Return body to centered lateral stance. The balance controller (if enabled)
    // will hold the upright pose. WeightShift ramps back to center over ramp_ms.
    if (_ws) _ws->trigger(ShiftDirection::NONE);

    Serial.println("[GaitController] STOP. WeightShift centering.");
}

// ---------------------------------------------------------------------------
void GaitController::update(float dt_s) {
    if (!_cfg.enabled || _mm == nullptr) return;

    // Always stop cleanly on e-stop rather than submitting during recovery.
    if (oe_is_estopped()) {
        _cfg.enabled  = false;
        _state.active = false;
        // Do not call stop() — oe_estop() has already disabled servo output.
        // start() is required before the next walk attempt.
        return;
    }

    // ── Advance phase ─────────────────────────────────────────────────────────
    // Each half-cycle spans cycle_period_s / 2 seconds.
    // Phase 0→1 covers one full leg motion (stance start → stance end / swing).
    const float half_period_s = _cfg.cycle_period_s * 0.5f;
    if (half_period_s < 0.05f) {
        // Guard: cycle_period_s < 0.1s is pathological — ignore to prevent
        // the phase from advancing multiple steps per tick and skipping joints.
        return;
    }

    _state.phase += dt_s / half_period_s;

    if (_state.phase >= 1.0f) {
        _state.phase -= 1.0f;
        _state.right_is_swing = !_state.right_is_swing;
        // Log transition for debugging. Remove after first stable walk.
        Serial.printf("[GaitController] Step. Now swinging: %s\n",
                      _state.right_is_swing ? "RIGHT" : "LEFT");
    }

    const float phase = _state.phase;  // [0, 1), local alias for readability

    // ── Compute swing_phase (double-support-adjusted) ─────────────────────────
    // swing_phase 0→1 drives the actual airborne trajectory.
    // During the double-support windows (phase < ds, or phase > 1-ds),
    // swing_phase is clamped so the foot stays on the ground.
    //
    // WHY clamp to 0.0 / 1.0 rather than linear interpolation:
    //   At swing_phase=0: x=-L/2, h=stance_height (foot at back, on ground).
    //   At swing_phase=1: x=+L/2, h=stance_height (foot at front, on ground).
    //   Both are valid "foot on ground" positions, so the clamp is physically safe.
    //   The stance foot is still moving (linear x), so the transition is smooth.
    const float ds = _cfg.double_support_frac;
    float swing_phase;

    if (phase < ds) {
        // Pre-liftoff: foot stationary at back position.
        swing_phase = 0.0f;
    } else if (phase > (1.0f - ds)) {
        // Post-landing: foot stationary at front position.
        swing_phase = 1.0f;
    } else {
        // Active airborne phase: normalize [ds, 1-ds] to [0, 1].
        const float airborne_frac = 1.0f - 2.0f * ds;
        // Guard: ds >= 0.5 makes airborne_frac <= 0.
        if (airborne_frac < 0.05f) {
            swing_phase = 0.0f;
        } else {
            swing_phase = (phase - ds) / airborne_frac;
        }
    }

    // ── Lateral shift (both feet, sinusoidal) ─────────────────────────────────
    // lateral_frac goes 0→1→0 over the half-cycle, peaking at phase=0.5.
    // At phase=0 and 1 (transitions): lateral_frac=0 → no shift (smooth handoff).
    //
    // WHY sin(π * phase) and not something else:
    //   Zero at transitions, maximum at mid-stance. This matches when the body
    //   is farthest from the touchdown event and most over the stance foot.
    const float lateral_frac = sinf(M_PI * phase);  // [0, 1], symmetric
    const float lateral_mm   = _cfg.lateral_shift_mm * lateral_frac;

    // ── Swing foot targets ────────────────────────────────────────────────────
    const float L      = _cfg.step_length_mm;
    const float h_nom  = _cfg.stance_height_mm;
    const float h_lift = _cfg.step_height_mm;
    const float width  = _cfg.stance_width_mm;

    // x: cosine interpolation — smooth zero-velocity liftoff and landing.
    // Derivation: x = -L/2 * cos(π * t) gives -L/2 at t=0, +L/2 at t=1.
    const float swing_x_mm = -0.5f * L * cosf(M_PI * swing_phase);

    // h: decreases from h_nom by h_lift at peak (sin peak = 1 at swing_phase=0.5).
    // Smaller h = foot closer to hip = foot is higher off the ground. ✓
    const float swing_h_mm = h_nom - h_lift * sinf(M_PI * swing_phase);

    // y: swing foot moves OUTWARD (body appears to lean away from swing side).
    // y is in each leg's own outward frame (+outward). Adding lateral_mm moves
    // the foot further from the body centerline, which shifts the body inward.
    const float swing_y_mm  = width + lateral_mm;

    // ── Stance foot targets ───────────────────────────────────────────────────
    // x: linear recession — foot moves from +L/2 to -L/2 as phase goes 0→1.
    // Linear is physically correct for constant body velocity.
    const float stance_x_mm = 0.5f * L * (1.0f - 2.0f * phase);

    // h: constant — body maintains the same height throughout stance.
    const float stance_h_mm = h_nom;

    // y: stance foot moves INWARD (foot appears under body, body leans over it).
    // Subtracting lateral_mm moves the foot closer to centerline in hip frame,
    // which in the world frame means the body has shifted outward over the foot.
    const float stance_y_mm = width - lateral_mm;

    // ── Build FootTargets ─────────────────────────────────────────────────────
    // h_frontal_mm = 0 tells LegIK::solve() to auto-derive from h_sagittal
    // using CHAIN_HEIGHT_DELTA_MM = 25.4mm (from robot_geometry.h).
    FootTarget swingTarget, stanceTarget;

    swingTarget.x_mm          = swing_x_mm;
    swingTarget.h_sagittal_mm = swing_h_mm;
    swingTarget.y_mm          = swing_y_mm;
    swingTarget.h_frontal_mm  = 0.0f;     // auto-derive
    swingTarget.isRightLeg    = _state.right_is_swing;

    stanceTarget.x_mm          = stance_x_mm;
    stanceTarget.h_sagittal_mm = stance_h_mm;
    stanceTarget.y_mm          = stance_y_mm;
    stanceTarget.h_frontal_mm  = 0.0f;
    stanceTarget.isRightLeg    = !_state.right_is_swing;

    // ── Solve IK ──────────────────────────────────────────────────────────────
    const LegIKResult swingIK  = LegIK::solve(swingTarget);
    const LegIKResult stanceIK = LegIK::solve(stanceTarget);

    // Update telemetry before the IK guard so the UI shows the last attempted target.
    const bool rightSwings = _state.right_is_swing;
    _state.x_right_mm = rightSwings ? swing_x_mm  : stance_x_mm;
    _state.h_right_mm = rightSwings ? swing_h_mm  : stance_h_mm;
    _state.x_left_mm  = rightSwings ? stance_x_mm : swing_x_mm;
    _state.h_left_mm  = rightSwings ? stance_h_mm : swing_h_mm;
    _state.right_ik_status = rightSwings ? swingIK.status  : stanceIK.status;
    _state.left_ik_status  = rightSwings ? stanceIK.status : swingIK.status;

    // DOMAIN_ERROR means the target is geometrically impossible (h too small,
    // or NaN in solver). Stop immediately — do not submit garbage angles.
    // UNREACHABLE (input clamped to max reach) and LIMIT_CLAMPED (soft limit
    // applied) are still submittable — the solver returns valid, safe angles.
    if (swingIK.status == IKStatus::DOMAIN_ERROR ||
        stanceIK.status == IKStatus::DOMAIN_ERROR) {
        Serial.printf("[GaitController] IK DOMAIN_ERROR — stopping. "
                      "swing=%d stance=%d\n",
                      static_cast<int>(swingIK.status),
                      static_cast<int>(stanceIK.status));
        stop();
        return;
    }

    // ── Submit joint commands ─────────────────────────────────────────────────
    if (rightSwings) {
        _submitLeg(true,  swingIK);   // right = swing
        _submitLeg(false, stanceIK);  // left  = stance
    } else {
        _submitLeg(false, swingIK);   // left = swing
        _submitLeg(true,  stanceIK);  // right = stance
    }
}

// ---------------------------------------------------------------------------
//  _submitLeg — map IK result to the correct IDX_* channels and submit.
//
//  IMPORTANT: Submit IDENTICAL anatomical angles for both legs. Do NOT negate
//  for the left leg here. JointModel reads direction=-1 from joint_config.cpp
//  for mirrored joints and applies the negation internally in setJointAngle().
//  If you negate here AND joint_config.cpp has direction=-1, the motion
//  doubles-negates and produces the wrong direction.
// ---------------------------------------------------------------------------
void GaitController::_submitLeg(bool isRight, const LegIKResult& ik) {
    // Safety: never submit on DOMAIN_ERROR (angles may be NaN or garbage).
    if (ik.status == IKStatus::DOMAIN_ERROR) return;

    const uint8_t hip_pitch   = isRight ? IDX_R_HIP_PITCH   : IDX_L_HIP_PITCH;
    const uint8_t knee_pitch  = isRight ? IDX_R_KNEE_PITCH  : IDX_L_KNEE_PITCH;
    const uint8_t ankle_pitch = isRight ? IDX_R_ANKLE_PITCH : IDX_L_ANKLE_PITCH;
    const uint8_t hip_roll    = isRight ? IDX_R_HIP_ROLL    : IDX_L_HIP_ROLL;
    const uint8_t ankle_roll  = isRight ? IDX_R_ANKLE_ROLL  : IDX_L_ANKLE_ROLL;

    _mm->submit(SOURCE_GAIT, hip_pitch,   ik.hip_pitch_deg);
    _mm->submit(SOURCE_GAIT, knee_pitch,  ik.knee_pitch_deg);
    _mm->submit(SOURCE_GAIT, ankle_pitch, ik.ankle_pitch_deg);
    _mm->submit(SOURCE_GAIT, hip_roll,    ik.hip_roll_deg);
    _mm->submit(SOURCE_GAIT, ankle_roll,  ik.ankle_roll_deg);

    // Hip yaw is intentionally NOT submitted — zero (straight ahead) is correct
    // for forward walking. The UI or neutral config holds hip yaw at zero.
}