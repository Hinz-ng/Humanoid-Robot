// =============================================================================
// FILE:    leg_ik.cpp
// MODULE:  kinematics
// LAYER:   3.5 — Gait / Motion
//
// SAGITTAL MATH:
//   r       = sqrt(x² + h²)                          reach magnitude
//   cos(ψ)  = (r² − L1² − L2²) / (2·L1·L2)          knee angle from straight
//   ψ       = acos(cos_ψ)                             [0°, 180°]; 0 = straight
//   φ_reach = atan2(x, h)                             reach direction from −Z
//   cos(γ)  = (L1² + r² − L2²) / (2·L1·r)           hip's triangle interior angle
//   hip     = φ_reach + γ                             posterior-knee config
//   ankle   = ψ − hip                                 flat-foot: sole horizontal
//
//   Posterior-knee means the knee protrudes FORWARD of the hip-ankle line.
//   This matches human anatomy and is confirmed correct for this robot:
//   the UVC reference uses K0W = acos(k/LEG) + asin(x/k), which is identical
//   for the symmetric-limb (L1=L2) approximation.
//
//   Flat-foot derivation:
//   Shank angle from vertical = hip − knee (negative = shank tilts backward).
//   Ankle must cancel this: ankle = −(hip − knee) = knee − hip.
//   Example: hip=33°, knee=67° → shank at −34° → ankle = +34° (dorsiflexion). ✓
//
// FRONTAL MATH:
//   hip_roll   = atan2(y, h_frontal)
//   ankle_roll = −hip_roll                            flat-foot lateral constraint
//
//   Matches UVC core.cpp: A1W[s] = −k1, where k1 = CHG_SVA·atan(y/h). ✓
//
// CHAIN HEIGHT RELATIONSHIP:
//   h_frontal = h_sagittal − CHAIN_HEIGHT_DELTA_MM (= 25.4mm)
//   Derived from confirmed chain lengths: (L_THIGH + L_SHANK) − L_FRONTAL_MM.
//   The per-joint axis offsets (29.8mm, 5.9mm) are NOT used here — they disagree
//   with the direct chain measurement by 1.5mm. Direct measurement wins.
//
// LAST CHANGED: 2026-04-21 | Corrected frontal height derivation.
// =============================================================================

#include "leg_ik.h"
#include <math.h>

// ── clampJoint ────────────────────────────────────────────────────────────────

float LegIK::clampJoint(float deg, float lo, float hi, bool& clamped) {
    if (deg < lo) { clamped = true; return lo; }
    if (deg > hi) { clamped = true; return hi; }
    return deg;
}

// ── forwardSagittal (FK) ──────────────────────────────────────────────────────
//
// Ankle pitch axis position relative to hip pitch axis.
// Thigh vector: (L1·sin(hip), L1·cos(hip))
// Shank vector: (L2·sin(hip−knee), L2·cos(hip−knee))
// This is signed — hip−knee is negative when shank leans back relative to thigh.

void LegIK::forwardSagittal(float hip_deg, float knee_deg,
                              float& x_out, float& h_out) {
    const float hip_rad   = hip_deg  * DEG_TO_RAD;
    const float shank_rad = (hip_deg - knee_deg) * DEG_TO_RAD;
    x_out = L_THIGH * sinf(hip_rad)   + L_SHANK * sinf(shank_rad);
    h_out = L_THIGH * cosf(hip_rad)   + L_SHANK * cosf(shank_rad);
}

// ── solveSagittal ─────────────────────────────────────────────────────────────

LegIK::SagittalSol LegIK::solveSagittal(float x_mm, float h_mm) {
    SagittalSol sol;
    const float L1 = L_THIGH;   // 96.0mm
    const float L2 = L_SHANK;   // 95.4mm
    const float maxReach = (L1 + L2) * IK_MAX_REACH_FRACTION;  // ~189.5mm

    // ── Reach and singularity guard ───────────────────────────────────────────
    const float r = sqrtf(x_mm * x_mm + h_mm * h_mm);
    sol.reach_mm  = r;

    // Scale along the reach direction rather than clamping x and h independently.
    // Independent clamping would silently alter the intended foot direction.
    float ex = x_mm, eh = h_mm, er = r;
    if (r > maxReach) {
        const float scale = maxReach / r;
        ex = x_mm * scale;
        eh = h_mm * scale;
        er = maxReach;
        sol.status = IKStatus::UNREACHABLE;
    }

    // Guard: h must be meaningfully positive. Near zero means the foot is
    // almost at the same height as the hip — mechanically impossible.
    if (eh < 10.0f) {
        sol.status = IKStatus::DOMAIN_ERROR;
        return sol;
    }

    const float er_sq = er * er;

    // ── Step 1: Knee flexion (law of cosines) ─────────────────────────────────
    // cos(ψ) = (r² − L1² − L2²) / (2·L1·L2)
    // Float arithmetic can push cos marginally outside [−1, 1] — guard explicitly.
    float cos_psi = (er_sq - L1*L1 - L2*L2) / (2.0f * L1 * L2);
    cos_psi       = constrain(cos_psi, -1.0f, 1.0f);
    sol.knee_pitch_deg = acosf(cos_psi) * RAD_TO_DEG;  // [0°, 180°]; 0 = straight

    // ── Step 2: Hip pitch ─────────────────────────────────────────────────────
    // φ_reach: angle of reach vector from downward vertical.
    //   atan2(x, h): x=0 → 0°, x>0 → positive (foot ahead = hip tilts forward). ✓
    const float phi_reach = atan2f(ex, eh);

    // γ: hip's interior angle in the thigh-shank-reach triangle.
    // law of cosines: L2² = L1² + r² − 2·L1·r·cos(γ)
    float cos_gamma = (L1*L1 + er_sq - L2*L2) / (2.0f * L1 * er);
    cos_gamma       = constrain(cos_gamma, -1.0f, 1.0f);
    const float gamma = acosf(cos_gamma);  // always ≥ 0

    // Posterior-knee: hip = reach_direction + triangle_angle.
    // Verification: x=0, some knee bend → hip = 0 + γ > 0 (thigh forward). ✓
    sol.hip_pitch_deg = (phi_reach + gamma) * RAD_TO_DEG;

    // ── Step 3: Ankle pitch (flat-foot constraint) ──────────────────────────────
    // Shank angle from vertical = hip − knee (negative = shank tilts backward).
    // Geometrically, ankle = knee − hip keeps the foot flat.
    // Hardware sign correction: the physical ankle pitch servo produces
    // plantar flexion for positive joint angles (confirmed on hardware — the
    // direction=+1 in joint_config maps increasing pulse to plantar flexion,
    // opposite to the assumed convention). Negating the IK output aligns
    // the geometric constraint with the hardware sign:
    //   ankleGeometric = +(knee − hip) → requires plantar flexion
    //   ankleHardware  = -(knee − hip) → produces dorsiflexion ✓
    // If the robot is re-calibrated and direction is flipped in joint_config,
    // remove this negation and restore the positive formula.
    sol.ankle_pitch_deg = -(sol.knee_pitch_deg - sol.hip_pitch_deg);

    // ── Step 4: Soft limits ───────────────────────────────────────────────────
    bool clamped = false;
    sol.hip_pitch_deg   = clampJoint(sol.hip_pitch_deg,
                                     IK_HIP_PITCH_MIN_DEG, IK_HIP_PITCH_MAX_DEG, clamped);
    sol.knee_pitch_deg  = clampJoint(sol.knee_pitch_deg,
                                     IK_KNEE_MIN_DEG, IK_KNEE_MAX_DEG, clamped);
    // Do NOT re-derive ankle from clamped hip/knee. Record the deviation as
    // flat_foot_error_deg in LegIKResult so the caller can decide.
    sol.ankle_pitch_deg = clampJoint(sol.ankle_pitch_deg,
                                     IK_ANKLE_PITCH_MIN_DEG, IK_ANKLE_PITCH_MAX_DEG, clamped);

    if (clamped && sol.status == IKStatus::OK) sol.status = IKStatus::LIMIT_CLAMPED;
    return sol;
}

// ── solveFrontal ──────────────────────────────────────────────────────────────
//
// The frontal chain is modelled as a single rigid link (hip roll → ankle roll).
// Knee flexion does not appear in the frontal plane for small-to-moderate roll
// angles — valid for the lateral ranges this robot will use.
//
// DIRECTION VERIFICATION (first hardware run):
//   Command y=20mm (outward) on the right leg only with balance disabled.
//   Right ankle sole should tilt OUTWARD (inversion).
//   If it tilts inward: add a sign flip on ankle_roll_deg here.
//   If left and right behave asymmetrically: check direction=−1 for left ankle
//   roll in joint_config.cpp.

LegIK::FrontalSol LegIK::solveFrontal(float y_mm, float h_frontal_mm) {
    FrontalSol sol;

    // Guard: frontal height below ~20mm implies near-collapse — atan2 would
    // return extreme angles outside any joint range.
    if (h_frontal_mm < 20.0f) {
        sol.status = IKStatus::DOMAIN_ERROR;
        return sol;
    }

    // Clamp lateral reach to maxReach = h_frontal × IK_MAX_REACH_FRACTION.
    // This is consistent with the sagittal singularity guard.
    const float maxY = h_frontal_mm * IK_MAX_REACH_FRACTION;
    float ey = y_mm;
    if (fabsf(y_mm) > maxY) {
        ey         = copysignf(maxY, y_mm);
        sol.status = IKStatus::UNREACHABLE;
    }

    // hip_roll = atan2(y, h_frontal).
    // y=0 → hip_roll=0. y>0 (outward) → positive abduction. ✓
    sol.hip_roll_deg   = atan2f(ey, h_frontal_mm) * RAD_TO_DEG;

    // Flat-foot: ankle inverts to cancel hip abduction. Sole stays horizontal.
    // Matches UVC: A1W[s] = −k1 where k1 = CHG_SVA·atan(y/h). ✓
    sol.ankle_roll_deg = -sol.hip_roll_deg;

    bool clamped = false;
    sol.hip_roll_deg   = clampJoint(sol.hip_roll_deg,
                                    IK_HIP_ROLL_MIN_DEG, IK_HIP_ROLL_MAX_DEG, clamped);
    sol.ankle_roll_deg = clampJoint(sol.ankle_roll_deg,
                                    IK_ANKLE_ROLL_MIN_DEG, IK_ANKLE_ROLL_MAX_DEG, clamped);
    if (clamped && sol.status == IKStatus::OK) sol.status = IKStatus::LIMIT_CLAMPED;
    return sol;
}

// ── solve ─────────────────────────────────────────────────────────────────────

LegIKResult LegIK::solve(const FootTarget& t) {
    LegIKResult result;

    // ── Derive frontal height from sagittal ───────────────────────────────────
    // Use the confirmed chain-length delta, not per-axis offsets.
    // CHAIN_HEIGHT_DELTA_MM = (L_THIGH + L_SHANK) − L_FRONTAL_MM = 25.4mm.
    // h_frontal is always CHAIN_HEIGHT_DELTA_MM shorter than h_sagittal.
    const float h_frontal = (t.h_frontal_mm > 0.0f)
                            ? t.h_frontal_mm
                            : t.h_sagittal_mm - CHAIN_HEIGHT_DELTA_MM;

    // ── Sagittal ──────────────────────────────────────────────────────────────
    const SagittalSol sag = solveSagittal(t.x_mm, t.h_sagittal_mm);
    result.hip_pitch_deg   = sag.hip_pitch_deg;
    result.knee_pitch_deg  = sag.knee_pitch_deg;
    result.ankle_pitch_deg = sag.ankle_pitch_deg;
    result.sagittal_reach_mm  = sag.reach_mm;
    result.sagittal_reach_pct = sag.reach_mm / (L_THIGH + L_SHANK) * 100.0f;

    // ── Frontal ───────────────────────────────────────────────────────────────
    const FrontalSol frt = solveFrontal(t.y_mm, h_frontal);
    result.hip_roll_deg   = frt.hip_roll_deg;
    result.ankle_roll_deg = frt.ankle_roll_deg;

    // ── Compose status (worst of the two planes) ──────────────────────────────
    result.status = (static_cast<uint8_t>(sag.status) >
                     static_cast<uint8_t>(frt.status))
                    ? sag.status : frt.status;

    // ── Flat-foot diagnostic ──────────────────────────────────────────────────
    // Ideal ankle = knee − hip (before any limit clamping).
    // After clamping, what was actually output may differ.
    const float ideal_ankle = result.knee_pitch_deg - result.hip_pitch_deg;
    result.flat_foot_error_deg = result.ankle_pitch_deg - ideal_ankle;
    if (fabsf(result.flat_foot_error_deg) > 0.01f) result.any_joint_at_limit = true;

    // ── NaN guard (final safety net) ─────────────────────────────────────────
    if (isnan(result.hip_pitch_deg)  || isnan(result.knee_pitch_deg)  ||
        isnan(result.ankle_pitch_deg) || isnan(result.hip_roll_deg)   ||
        isnan(result.ankle_roll_deg)) {
        result        = LegIKResult{};
        result.status = IKStatus::DOMAIN_ERROR;
    }

    return result;
}

// ── validateRoundtrip ─────────────────────────────────────────────────────────
//
// Solves IK, applies FK to the result, checks the FK position closes back to
// the original target. A pass confirms the sagittal math is self-consistent.
// Run this before using the IK in any motion loop.

bool LegIK::validateRoundtrip(const FootTarget& t, float toleranceMm) {
    const char* leg = t.isRightLeg ? "RIGHT" : "LEFT";
    Serial.printf("\n[LegIK] Roundtrip (%s)  x=%.1f  h_sag=%.1f  y=%.1f\n",
                  leg, t.x_mm, t.h_sagittal_mm, t.y_mm);

    const LegIKResult ik = solve(t);
    if (ik.status == IKStatus::DOMAIN_ERROR) {
        Serial.println("  FAIL — DOMAIN_ERROR, cannot validate.");
        return false;
    }

    Serial.printf("  IK output: hip=%.4f°  knee=%.4f°  ankle=%.4f°  "
                  "h_roll=%.4f°  a_roll=%.4f°\n",
                  ik.hip_pitch_deg, ik.knee_pitch_deg, ik.ankle_pitch_deg,
                  ik.hip_roll_deg, ik.ankle_roll_deg);
    Serial.printf("  Reach: %.2fmm = %.1f%%  status=%d\n",
                  ik.sagittal_reach_mm, ik.sagittal_reach_pct,
                  static_cast<int>(ik.status));

    // FK: recover sagittal foot position from computed angles.
    float fk_x, fk_h;
    forwardSagittal(ik.hip_pitch_deg, ik.knee_pitch_deg, fk_x, fk_h);
    const float err_x  = fk_x - t.x_mm;
    const float err_h  = fk_h - t.h_sagittal_mm;
    const float pos_err = sqrtf(err_x * err_x + err_h * err_h);

    Serial.printf("  FK:  x=%.4f  h=%.4f  |pos_err|=%.5fmm"
                  "  (Δx=%.5f  Δh=%.5f)\n", fk_x, fk_h, pos_err, err_x, err_h);
    Serial.printf("  Flat-foot error: %.5f° (0 = ideal; nonzero if ankle limit hit)\n",
                  ik.flat_foot_error_deg);

    // Frontal check: recover y from hip_roll and h_frontal.
    const float h_frontal = (t.h_frontal_mm > 0.0f)
                            ? t.h_frontal_mm
                            : t.h_sagittal_mm - CHAIN_HEIGHT_DELTA_MM;
    const float y_check = h_frontal * tanf(ik.hip_roll_deg * DEG_TO_RAD);
    Serial.printf("  Frontal: h_frontal=%.2f  y_recovered=%.4f  Δy=%.5f\n",
                  h_frontal, y_check, y_check - t.y_mm);

    const bool passed = (pos_err < toleranceMm)
                      && (fabsf(ik.flat_foot_error_deg) < 0.1f);
    Serial.printf("  %s  pos_err=%.5fmm  tol=%.2fmm  ff_err=%.5f°\n\n",
                  passed ? "PASS" : "FAIL", pos_err, toleranceMm,
                  ik.flat_foot_error_deg);
    return passed;
}

// ── validateNeutral ───────────────────────────────────────────────────────────
//
// Given MEASURED joint angles at the robot's neutral standing pose, computes
// the foot position those angles imply via FK, then runs IK and verifies the
// angles are recovered within tolerance.
// This detects inconsistencies between geometry and physical calibration.
// The expected angles MUST come from physical measurement — do not guess.

bool LegIK::validateNeutral(float expHip, float expKnee, float expAnkle,
                              float toleranceDeg) {
    Serial.printf("\n[LegIK] Neutral compare  expected: hip=%.3f°  knee=%.3f°  "
                  "ankle=%.3f°  tol=%.1f°\n",
                  expHip, expKnee, expAnkle, toleranceDeg);

    // What foot position do these angles imply?
    float x_impl, h_impl;
    forwardSagittal(expHip, expKnee, x_impl, h_impl);
    Serial.printf("  Implied foot: x=%.3f mm  h_sag=%.3f mm\n", x_impl, h_impl);

    FootTarget t;
    t.x_mm          = x_impl;
    t.h_sagittal_mm = h_impl;

    const LegIKResult ik = solve(t);
    if (!ik.ok()) {
        Serial.printf("  FAIL — solver returned status=%d\n",
                      static_cast<int>(ik.status));
        return false;
    }

    const float err_hip   = ik.hip_pitch_deg   - expHip;
    const float err_knee  = ik.knee_pitch_deg  - expKnee;
    const float err_ankle = ik.ankle_pitch_deg - expAnkle;
    const float max_err   = fmaxf(fabsf(err_hip),
                             fmaxf(fabsf(err_knee), fabsf(err_ankle)));

    Serial.printf("  IK: hip=%.4f°  knee=%.4f°  ankle=%.4f°\n",
                  ik.hip_pitch_deg, ik.knee_pitch_deg, ik.ankle_pitch_deg);
    Serial.printf("  Δhip=%.5f°  Δknee=%.5f°  Δankle=%.5f°  max=%.5f°\n",
                  err_hip, err_knee, err_ankle, max_err);

    const bool passed = (max_err < toleranceDeg);
    Serial.printf("  %s  max_err=%.5f°  tol=%.1f°\n\n",
                  passed ? "PASS" : "FAIL", max_err, toleranceDeg);
    return passed;
}