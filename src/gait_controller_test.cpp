// =============================================================================
// FILE:    gait_controller_test.cpp
// PURPOSE: Unit tests for GaitController trajectory math (Stage 1+2 invariants).
//          Activate: add -DUNIT_TEST to platformio.ini build_flags.
//          Call runGaitControllerTests() once from setup() for a diagnostic run.
//          No hardware or MotionManager required for any test below.
// =============================================================================

#ifdef UNIT_TEST

#include "gait_controller.h"
#include "leg_ik.h"
#include "project_wide_defs.h"
#include <Arduino.h>
#include <math.h>

static bool near(float a, float b, float eps = 0.001f) { return fabsf(a - b) < eps; }

// ---------------------------------------------------------------------------
// T1 — Default config defaults are sane and stay inside IK reach budget.
//   stanceHeightMm must be <= 99% of (L_THIGH+L_SHANK); h at peak swing
//   (stanceHeightMm − GAIT_STEP_HEIGHT_MM) must be >= 130 mm.
static bool test_configDefaults() {
    GaitController g;
    const GaitConfig& c = g.getConfig();
    const float h_min = c.stanceHeightMm - GAIT_STEP_HEIGHT_MM;
    const bool pass = (h_min >= 130.0f) && (c.stanceHeightMm <= 189.0f);
    Serial.printf("[GaitT1] config_defaults: %s  h_min=%.1f stance=%.1f\n",
                  pass ? "PASS" : "FAIL", h_min, c.stanceHeightMm);
    return pass;
}

// ---------------------------------------------------------------------------
// T2 — Swing fraction endpoints: sin(0)=0, sin(π)=0, sin(π/2)=1.
//   Mirrors GaitController::_swingFrac(localPhase).
static bool test_swingFracEndpoints() {
    const float s0  = sinf(0.0f * static_cast<float>(M_PI));
    const float s1  = sinf(1.0f * static_cast<float>(M_PI));
    const float s05 = sinf(0.5f * static_cast<float>(M_PI));
    const bool pass = near(s0, 0.0f) && near(s1, 0.0f) && near(s05, 1.0f);
    Serial.printf("[GaitT2] swing_frac_endpoints: %s  s0=%.3f s1=%.3f s05=%.3f\n",
                  pass ? "PASS" : "FAIL", s0, s1, s05);
    return pass;
}

// ---------------------------------------------------------------------------
// T3 — Default neutral stance and peak-swing FootTargets resolve through IK.
//   Catches "did I make stance height unreachable" before hardware time.
static bool test_neutralAndSwingReachable() {
    GaitController g;
    const GaitConfig& c = g.getConfig();

    FootTarget tNeutral;
    tNeutral.isRightLeg    = true;
    tNeutral.x_mm          = GAIT_STANCE_X_OFFSET_MM;
    tNeutral.y_mm          = 0.0f;
    tNeutral.h_sagittal_mm = c.stanceHeightMm;
    tNeutral.h_frontal_mm  = 0.0f;
    tNeutral.valid         = true;

    FootTarget tSwingPeak  = tNeutral;
    tSwingPeak.h_sagittal_mm = c.stanceHeightMm - GAIT_STEP_HEIGHT_MM;

    const LegIKResult rN = LegIK::solve(tNeutral);
    const LegIKResult rS = LegIK::solve(tSwingPeak);

    const bool pass = (rN.status != IKStatus::DOMAIN_ERROR)
                   && (rN.status != IKStatus::UNREACHABLE)
                   && (rS.status != IKStatus::DOMAIN_ERROR)
                   && (rS.status != IKStatus::UNREACHABLE);
    Serial.printf("[GaitT3] neutral_swing_reachable: %s  "
                  "rN.status=%d reach=%.1f%%  rS.status=%d reach=%.1f%%\n",
                  pass ? "PASS" : "FAIL",
                  static_cast<int>(rN.status), rN.sagittal_reach_pct,
                  static_cast<int>(rS.status), rS.sagittal_reach_pct);
    return pass;
}

// ---------------------------------------------------------------------------
// T4 — Worst-case Stage 2 lateral shift stays inside IK reach.
//   At lateral_shift_mm = 50 (parser cap) and stanceHeightMm = 185:
//   FootTarget.y_mm = 50 → IK must not return DOMAIN_ERROR / UNREACHABLE.
static bool test_lateralShiftReachable() {
    FootTarget t;
    t.isRightLeg    = true;
    t.x_mm          = GAIT_STANCE_X_OFFSET_MM;
    t.y_mm          = 50.0f;
    t.h_sagittal_mm = GAIT_STANCE_HEIGHT_MM;
    t.h_frontal_mm  = 0.0f;
    t.valid         = true;

    const LegIKResult r = LegIK::solve(t);
    const bool pass = (r.status != IKStatus::DOMAIN_ERROR)
                   && (r.status != IKStatus::UNREACHABLE);
    Serial.printf("[GaitT4] lateral_shift_reachable: %s  status=%d reach=%.1f%%\n",
                  pass ? "PASS" : "FAIL",
                  static_cast<int>(r.status), r.sagittal_reach_pct);
    return pass;
}

// ---------------------------------------------------------------------------
void runGaitControllerTests() {
    Serial.println("\n[GaitCtrl] ══ Unit Tests ══════════════════════════════════");
    int pass = 0, total = 0;
    auto run = [&](bool r) { total++; if (r) pass++; };
    run(test_configDefaults());
    run(test_swingFracEndpoints());
    run(test_neutralAndSwingReachable());
    run(test_lateralShiftReachable());
    Serial.printf("[GaitCtrl] ══ %d / %d PASSED ═══════════════════════════════\n\n",
                  pass, total);
}

#endif // UNIT_TEST
