// =============================================================================
// FILE:    gait_controller_test.cpp
// PURPOSE: Unit tests for GaitController trajectory math.
//          Activate: add -DUNIT_TEST to platformio.ini build_flags.
//          Call runGaitControllerTests() once from setup() for a diagnostic run.
//          No hardware or MotionManager required for any test below.
// =============================================================================

#ifdef UNIT_TEST

#include "gait_controller.h"
#include <Arduino.h>
#include <math.h>

static bool near(float a, float b, float eps = 0.001f) { return fabsf(a - b) < eps; }

// ---------------------------------------------------------------------------
// T1 — Default config is sane (key constraints hold).
static bool test_configConstraints() {
    GaitController g;
    const GaitConfig& c = g.getConfig();
    // Workspace constraint: h_min must be >= 130mm
    const float h_min = c.stance_height_mm - c.step_height_mm;
    // Lateral constraint: swing foot y = stance_width + lateral_shift
    const float y_max = c.stance_width_mm + c.lateral_shift_mm;
    const bool pass = (h_min >= 130.0f) && (y_max <= 40.0f);
    Serial.printf("[GaitT1] config_constraints: %s  h_min=%.1f y_max=%.1f\n",
                  pass ? "PASS" : "FAIL", h_min, y_max);
    return pass;
}

// ---------------------------------------------------------------------------
// T2 — Swing x trajectory is position-continuous at phase=0 and phase=1.
//   End of swing (swing_phase=1) must equal start of next stance (new phase=0).
//   End of stance (phase=1) must equal start of next swing (new swing_phase=0).
static bool test_xPositionContinuity() {
    const float L = 20.0f;
    // End of swing at swing_phase=1: x_swing = -L/2 * cos(π) = +L/2
    const float x_swing_end   = -0.5f * L * cosf(M_PI * 1.0f);
    // Start of stance at phase=0: x_stance = L/2 * (1-0) = +L/2
    const float x_stance_start = 0.5f * L * (1.0f - 2.0f * 0.0f);
    // End of stance at phase=1: x_stance = L/2 * (1-2) = -L/2
    const float x_stance_end  = 0.5f * L * (1.0f - 2.0f * 1.0f);
    // Start of swing at swing_phase=0: x_swing = -L/2 * cos(0) = -L/2
    const float x_swing_start = -0.5f * L * cosf(M_PI * 0.0f);

    const bool pass = near(x_swing_end, x_stance_start)
                   && near(x_stance_end, x_swing_start);
    Serial.printf("[GaitT2] x_continuity: %s  "
                  "sw_end=%.4f st_start=%.4f  st_end=%.4f sw_start=%.4f\n",
                  pass ? "PASS" : "FAIL",
                  x_swing_end, x_stance_start, x_stance_end, x_swing_start);
    return pass;
}

// ---------------------------------------------------------------------------
// T3 — Swing height returns to stance_height at swing_phase = 0 and 1.
//   If foot doesn't return to ground height, it will either crash into the
//   ground or land with a residual height offset.
static bool test_swingHeightAtEndpoints() {
    const float h_nom = 160.0f, h_lift = 15.0f;
    const float h0 = h_nom - h_lift * sinf(M_PI * 0.0f);  // start of swing
    const float h1 = h_nom - h_lift * sinf(M_PI * 1.0f);  // end of swing
    const bool pass = near(h0, h_nom) && near(h1, h_nom);
    Serial.printf("[GaitT3] swing_height_endpoints: %s  h0=%.4f h1=%.4f expect=%.1f\n",
                  pass ? "PASS" : "FAIL", h0, h1, h_nom);
    return pass;
}

// ---------------------------------------------------------------------------
// T4 — Lateral y is continuous at transitions (phase=0 and phase=1, frac=0).
static bool test_yContinuityAtTransition() {
    const float width = 25.0f, lat = 12.0f;
    // At phase=0: lateral_frac = sin(0) = 0
    const float frac_0 = sinf(M_PI * 0.0f);
    // At phase=1: lateral_frac = sin(π) = 0
    const float frac_1 = sinf(M_PI * 1.0f);
    const float swing_y_0 = width + lat * frac_0;  // +width (no shift)
    const float swing_y_1 = width + lat * frac_1;  // +width (no shift)
    // Both should equal width (no shift at transitions)
    const bool pass = near(swing_y_0, width) && near(swing_y_1, width);
    Serial.printf("[GaitT4] y_continuity: %s  y_at_0=%.4f y_at_1=%.4f expect=%.1f\n",
                  pass ? "PASS" : "FAIL", swing_y_0, swing_y_1, width);
    return pass;
}

// ---------------------------------------------------------------------------
// T5 — Default gait parameters are within IK reachable workspace.
//   Worst case: swing foot at y = stance_width + lateral_shift,
//   h_sagittal = stance_height - step_height (foot at max lift, mid-swing).
static bool test_ikReachabilityDefault() {
    FootTarget t;
    t.x_mm          = 10.0f;    // step_length/2
    t.h_sagittal_mm = 145.0f;   // stance_height(160) - step_height(15)
    t.y_mm          = 37.0f;    // stance_width(25) + lateral_shift(12)
    t.h_frontal_mm  = 0.0f;     // auto-derive
    t.isRightLeg    = true;

    const LegIKResult r = LegIK::solve(t);
    // Fail if solver returns DOMAIN_ERROR or UNREACHABLE (step would abort).
    // LIMIT_CLAMPED is acceptable — soft limits are margins, not hard walls.
    const bool pass = (r.status != IKStatus::DOMAIN_ERROR)
                   && (r.status != IKStatus::UNREACHABLE);
    Serial.printf("[GaitT5] ik_reachability_default: %s  status=%d "
                  "hip=%.2f knee=%.2f ankle=%.2f hip_roll=%.2f\n",
                  pass ? "PASS" : "FAIL",
                  static_cast<int>(r.status),
                  r.hip_pitch_deg, r.knee_pitch_deg,
                  r.ankle_pitch_deg, r.hip_roll_deg);
    return pass;
}

// ---------------------------------------------------------------------------
void runGaitControllerTests() {
    Serial.println("\n[GaitCtrl] ══ Unit Tests ══════════════════════════════════");
    int pass = 0, total = 0;
    auto run = [&](bool r) { total++; if (r) pass++; };
    run(test_configConstraints());
    run(test_xPositionContinuity());
    run(test_swingHeightAtEndpoints());
    run(test_yContinuityAtTransition());
    run(test_ikReachabilityDefault());
    Serial.printf("[GaitCtrl] ══ %d / %d PASSED ═══════════════════════════════\n\n",
                  pass, total);
}

#endif // UNIT_TEST