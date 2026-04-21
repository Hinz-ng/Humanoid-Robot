// =============================================================================
// FILE:    leg_ik_test.cpp
// PURPOSE: Unit tests for LegIK. Not compiled in production builds.
//          Activate: add -DUNIT_TEST to platformio.ini build_flags.
//          Call runLegIKTests() once from setup() for a diagnostic run.
// =============================================================================

#ifdef UNIT_TEST

#include "leg_ik.h"
#include <Arduino.h>

// Helper: compare two floats within an epsilon.
static bool near(float a, float b, float eps) { return fabsf(a - b) < eps; }

// ---------------------------------------------------------------------------
//  T1 — x=0, nominal walking height (160mm sagittal)
//  Verified by hand: at h=160mm, r=160mm.
//    cos(knee) = (160²−96²−95.4²)/(2·96·95.4) = 7282.84/18316.8 = 0.3976 → 66.6°
//    phi_reach = 0, cos(gamma) = (96²+160²−95.4²)/(2·96·160) = 25714.84/30720 = 0.8373 → 32.9°
//    hip = 32.9°, ankle = 66.6−32.9 = 33.7°
// ---------------------------------------------------------------------------
static bool test_nominalHeight() {
    FootTarget t;
    t.x_mm = 0.0f; t.h_sagittal_mm = 160.0f;
    const LegIKResult r = LegIK::solve(t);
    if (!r.ok()) { Serial.println("[T1] FAIL — solver error"); return false; }
    const bool pass = near(r.hip_pitch_deg, 32.9f, 0.5f)
                   && near(r.knee_pitch_deg, 66.6f, 0.5f)
                   && near(r.ankle_pitch_deg, 33.7f, 0.5f)
                   && near(r.flat_foot_error_deg, 0.0f, 0.05f);
    Serial.printf("[T1] nominal_height: %s  hip=%.3f knee=%.3f ankle=%.3f\n",
                  pass ? "PASS" : "FAIL",
                  r.hip_pitch_deg, r.knee_pitch_deg, r.ankle_pitch_deg);
    return pass;
}

// ---------------------------------------------------------------------------
//  T2 — FK roundtrip at nominal height, foot centered and offset forward
// ---------------------------------------------------------------------------
static bool test_roundtrip_centered() {
    return LegIK::validateRoundtrip({0.0f, 160.0f, 0.0f, 0.0f, true}, 0.5f);
}
static bool test_roundtrip_forward() {
    return LegIK::validateRoundtrip({20.0f, 155.0f, 0.0f, 0.0f, true}, 0.5f);
}

// ---------------------------------------------------------------------------
//  T3 — Frontal: y=20mm outward, h_sag=160mm
//  h_frontal = 160 − 25.4 = 134.6mm
//  Expected hip_roll = atan2(20, 134.6) = 8.45°
//  Expected ankle_roll = −8.45°
// ---------------------------------------------------------------------------
static bool test_frontal_20mm() {
    FootTarget t;
    t.x_mm = 0.0f; t.h_sagittal_mm = 160.0f; t.y_mm = 20.0f;
    const float h_frontal      = 160.0f - CHAIN_HEIGHT_DELTA_MM;
    const float expected_roll  = atan2f(20.0f, h_frontal) * RAD_TO_DEG;
    const LegIKResult r        = LegIK::solve(t);
    const bool pass = r.ok()
                   && near(r.hip_roll_deg,    expected_roll,  0.05f)
                   && near(r.ankle_roll_deg, -expected_roll,  0.05f);
    Serial.printf("[T3] frontal_20mm: %s  h_frontal=%.2f  expected_roll=%.4f°  "
                  "got h_roll=%.4f°  a_roll=%.4f°\n",
                  pass ? "PASS" : "FAIL",
                  h_frontal, expected_roll, r.hip_roll_deg, r.ankle_roll_deg);
    return pass;
}

// ---------------------------------------------------------------------------
//  T4 — Left/right symmetry: IK returns identical anatomical angles for both
//       legs given identical targets. JointModel direction fields handle mirroring.
// ---------------------------------------------------------------------------
static bool test_LR_symmetry() {
    FootTarget tR; tR.x_mm = 15.0f; tR.h_sagittal_mm = 155.0f;
                   tR.y_mm = 10.0f; tR.isRightLeg = true;
    FootTarget tL = tR;             tL.isRightLeg = false;
    const LegIKResult rR = LegIK::solve(tR);
    const LegIKResult rL = LegIK::solve(tL);
    const bool pass = near(rR.hip_pitch_deg,   rL.hip_pitch_deg,   0.001f)
                   && near(rR.knee_pitch_deg,  rL.knee_pitch_deg,  0.001f)
                   && near(rR.ankle_pitch_deg, rL.ankle_pitch_deg, 0.001f)
                   && near(rR.hip_roll_deg,    rL.hip_roll_deg,    0.001f)
                   && near(rR.ankle_roll_deg,  rL.ankle_roll_deg,  0.001f);
    Serial.printf("[T4] LR_symmetry: %s\n", pass ? "PASS" : "FAIL");
    if (!pass)
        Serial.printf("     R: hip=%.4f knee=%.4f roll=%.4f\n"
                      "     L: hip=%.4f knee=%.4f roll=%.4f\n",
                      rR.hip_pitch_deg, rR.knee_pitch_deg, rR.hip_roll_deg,
                      rL.hip_pitch_deg, rL.knee_pitch_deg, rL.hip_roll_deg);
    return pass;
}

// ---------------------------------------------------------------------------
//  T5 — Unreachable target (exceeds max reach): status must be UNREACHABLE
//       and angles must still be numerically valid (not NaN).
// ---------------------------------------------------------------------------
static bool test_unreachable() {
    FootTarget t;
    t.x_mm = 0.0f; t.h_sagittal_mm = L_THIGH + L_SHANK + 20.0f;  // 211.4mm — over max
    const LegIKResult r = LegIK::solve(t);
    const bool pass = (r.status == IKStatus::UNREACHABLE)
                   && !isnan(r.hip_pitch_deg)
                   && !isnan(r.knee_pitch_deg);
    Serial.printf("[T5] unreachable: %s  status=%d  hip=%.3f  knee=%.3f\n",
                  pass ? "PASS" : "FAIL",
                  static_cast<int>(r.status), r.hip_pitch_deg, r.knee_pitch_deg);
    return pass;
}

// ---------------------------------------------------------------------------
//  T6 — DOMAIN_ERROR: h near zero must not produce NaN
// ---------------------------------------------------------------------------
static bool test_domain_error() {
    FootTarget t;
    t.x_mm = 0.0f; t.h_sagittal_mm = 5.0f;  // below 10mm guard
    const LegIKResult r = LegIK::solve(t);
    const bool pass = (r.status == IKStatus::DOMAIN_ERROR)
                   && !isnan(r.hip_pitch_deg);
    Serial.printf("[T6] domain_error: %s  status=%d\n",
                  pass ? "PASS" : "FAIL", static_cast<int>(r.status));
    return pass;
}

// ---------------------------------------------------------------------------
//  T7 — Neutral validation: uses measured neutral angles from joint_config.cpp.
//  INSTRUCTION: replace 0,0,0 with the joint-relative angles that describe
//  the robot's physical neutral standing pose once measured.
//  For now, tests geometric self-consistency at x=0, h=160mm (not the physical
//  neutral — this is a geometric exercise until hardware angles are measured).
// ---------------------------------------------------------------------------
static bool test_neutral_placeholder() {
    // At x=0, h=160mm the IK produces: hip≈32.9, knee≈66.6, ankle≈33.7.
    // Feed those back as expected to confirm the path is consistent.
    FootTarget t; t.x_mm = 0.0f; t.h_sagittal_mm = 160.0f;
    const LegIKResult ik = LegIK::solve(t);
    return LegIK::validateNeutral(ik.hip_pitch_deg,
                                   ik.knee_pitch_deg,
                                   ik.ankle_pitch_deg, 0.1f);
}

// ---------------------------------------------------------------------------
void runLegIKTests() {
    Serial.println("\n[LegIK] ══ Unit Tests ════════════════════════════════════");
    int pass = 0, total = 0;
    auto run = [&](bool r) { total++; if (r) pass++; };
    run(test_nominalHeight());
    run(test_roundtrip_centered());
    run(test_roundtrip_forward());
    run(test_frontal_20mm());
    run(test_LR_symmetry());
    run(test_unreachable());
    run(test_domain_error());
    run(test_neutral_placeholder());
    Serial.printf("[LegIK] ══ %d / %d PASSED ══════════════════════════════════\n\n",
                  pass, total);
}

#endif // UNIT_TEST