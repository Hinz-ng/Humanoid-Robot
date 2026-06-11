// =============================================================================
// FILE:    steering_controller_test.cpp
// PURPOSE: Unit tests for the Stage-1 hip-yaw steering law (STEERING_PLAN §9.1).
//          Activate: add -DUNIT_TEST to platformio.ini build_flags.
//          Call runSteeringControllerTests() once from setup() for a diagnostic
//          run. No hardware or MotionManager wiring required.
//
// WHY THE STATIC HELPERS:
//   The full update() path is gated by oe_is_estopped(), which returns TRUE
//   during the boot-time test run (tests execute before oe_begin()/oe_loop()
//   release OE). So update() parks the yaw output and never submits off-hardware.
//   The law and the slew are therefore verified through the pure static helpers
//   SteeringController::yawLawDeg() / slewToward(); the disabled-no-op is the one
//   integration check that is estop-independent (the enabled gate returns first).
//   The live submit + direction is verified on hardware (STEERING_PLAN gates 2-3).
// =============================================================================

#ifdef UNIT_TEST

#include "steering_controller.h"
#include "motion_manager.h"
#include "side.h"
#include <Arduino.h>
#include <math.h>

static bool s_near(float a, float b, float eps = 0.001f) { return fabsf(a - b) < eps; }

// (a) enabled=false → true no-op: zero submissions and zeroed telemetry, even
//     with an active RUNNING input and a full turnCmd.
static bool test_disabledNoSubmit() {
    MotionManager      mm;          // slots zero-initialised by ctor
    SteeringController sc;
    sc.init(&mm);

    SteeringConfig cfg;             // enabled defaults false
    cfg.enabled = false;
    cfg.turnCmd = 1.0f;
    sc.setConfig(cfg);
    sc.setGaitInput({ /*active=*/true, Side::LEFT, /*halfPhase=*/0.5f });
    sc.update(0.0025f);

    const SteeringState& st = sc.getState();
    const bool pass = (mm.pendingCount() == 0)
                   && s_near(st.yawCmdDeg, 0.0f)
                   && s_near(st.turnEff, 0.0f)
                   && (st.active == false);
    Serial.printf("[SteerT1] disabled_no_submit: %s  pending=%d yaw=%.3f active=%d\n",
                  pass ? "PASS" : "FAIL", mm.pendingCount(), st.yawCmdDeg, (int)st.active);
    return pass;
}

// (b) turnEff=0 (turnCmd=0) → yaw=0 at every phase, both sides.
static bool test_turnZeroYawZero() {
    bool ok = true;
    const float phases[] = { 0.0f, 0.25f, 0.5f, 0.75f, 1.0f };
    for (float h : phases) {
        ok &= s_near(SteeringController::yawLawDeg(0.0f, 2.5f, +1.0f, 15.0f, Side::LEFT,  h), 0.0f);
        ok &= s_near(SteeringController::yawLawDeg(0.0f, 2.5f, +1.0f, 15.0f, Side::RIGHT, h), 0.0f);
    }
    Serial.printf("[SteerT2] turn_zero_yaw_zero: %s\n", ok ? "PASS" : "FAIL");
    return ok;
}

// (c) Accumulation + halfSign: over a swing (h:0→1) the anatomical Δq is
//     +yawStep on a LEFT swing and −yawStep on a RIGHT swing. With the stance
//     relation (right-stance: dψ=+dq; left-stance: dψ=−dq) BOTH footfalls
//     rotate the body the SAME way by +yawStep.
static bool test_accumulationHalfSign() {
    const float yawStep = 2.5f;
    // Half A: swing LEFT, stance RIGHT.
    const float qA0 = SteeringController::yawLawDeg(1.0f, yawStep, +1.0f, 15.0f, Side::LEFT, 0.0f);
    const float qA1 = SteeringController::yawLawDeg(1.0f, yawStep, +1.0f, 15.0f, Side::LEFT, 1.0f);
    const float dqA = qA1 - qA0;                 // expect +yawStep
    // Half B: swing RIGHT, stance LEFT.
    const float qB0 = SteeringController::yawLawDeg(1.0f, yawStep, +1.0f, 15.0f, Side::RIGHT, 0.0f);
    const float qB1 = SteeringController::yawLawDeg(1.0f, yawStep, +1.0f, 15.0f, Side::RIGHT, 1.0f);
    const float dqB = qB1 - qB0;                 // expect −yawStep

    const float bodyA = +dqA;                    // right-stance: dψ = +dq
    const float bodyB = -dqB;                    // left-stance:  dψ = −dq

    const bool pass = s_near(dqA, +yawStep) && s_near(dqB, -yawStep)
                   && s_near(bodyA, +yawStep) && s_near(bodyB, +yawStep)
                   && (bodyA > 0.0f) && (bodyB > 0.0f);   // same direction
    Serial.printf("[SteerT3] accumulation_halfsign: %s  dqA=%.2f dqB=%.2f bodyA=%.2f bodyB=%.2f\n",
                  pass ? "PASS" : "FAIL", dqA, dqB, bodyA, bodyB);
    return pass;
}

// (d) Boundary continuity: halfSign·(s−0.5)=+0.5 at A-end (LEFT,h=1) and
//     B-start (RIGHT,h=0) → the yaw command is identical across the role swap.
static bool test_boundaryContinuity() {
    const float aEnd   = SteeringController::yawLawDeg(1.0f, 2.5f, +1.0f, 15.0f, Side::LEFT,  1.0f);
    const float bStart = SteeringController::yawLawDeg(1.0f, 2.5f, +1.0f, 15.0f, Side::RIGHT, 0.0f);
    // Shared value should be turnEff*yawStep*turnSign*(+0.5) = 1.25.
    const bool pass = s_near(aEnd, bStart) && s_near(aEnd, 1.25f);
    Serial.printf("[SteerT4] boundary_continuity: %s  aEnd=%.3f bStart=%.3f\n",
                  pass ? "PASS" : "FAIL", aEnd, bStart);
    return pass;
}

// (e) Slew: a mid-step command jump produces a bounded per-tick change; the
//     output never jumps instantaneously and converges to the target.
static bool test_slewBounded() {
    const float maxStep = 2.0f * 0.0025f;        // turnSlewPerSec * dt = 0.005
    bool ok = true;
    // Single steps are capped at ±maxStep.
    ok &= s_near(SteeringController::slewToward(0.0f, 1.0f, maxStep),  +maxStep);
    ok &= s_near(SteeringController::slewToward(0.0f, -1.0f, maxStep), -maxStep);
    // Within one step of the target → snaps exactly (no overshoot).
    ok &= s_near(SteeringController::slewToward(0.998f, 1.0f, maxStep), 1.0f);
    // A run of ticks from 0 toward +1: every step bounded, monotone, converges.
    float e = 0.0f; float prev = 0.0f; bool bounded = true;
    for (int i = 0; i < 1000 && e < 0.999f; ++i) {
        e = SteeringController::slewToward(e, 1.0f, maxStep);
        if (fabsf(e - prev) > maxStep + 1e-6f) bounded = false;
        prev = e;
    }
    ok &= bounded && s_near(e, 1.0f, 0.01f);
    Serial.printf("[SteerT5] slew_bounded: %s  converged=%.3f\n", ok ? "PASS" : "FAIL", e);
    return ok;
}

// (f) Double-support hold: at halfPhase=0, s=0 → yaw = -0.5·turnEff·yawStep·
//     turnSign·halfSign. For LEFT swing (halfSign=+1): -1.25.
static bool test_doubleSupportHold() {
    const float yaw = SteeringController::yawLawDeg(1.0f, 2.5f, +1.0f, 15.0f, Side::LEFT, 0.0f);
    const float expected = -0.5f * 1.0f * 2.5f * 1.0f * (+1.0f);   // -1.25
    const bool pass = s_near(yaw, expected);
    Serial.printf("[SteerT6] double_support_hold: %s  yaw=%.3f expected=%.3f\n",
                  pass ? "PASS" : "FAIL", yaw, expected);
    return pass;
}

// (g) Clamp: a raw command beyond ±maxYawDeg is clamped both directions.
static bool test_clamp() {
    // raw = 1 * 100 * 1 * (+1) * (1-0.5) = 50  → clamp +15.
    const float hi = SteeringController::yawLawDeg(1.0f, 100.0f, +1.0f, 15.0f, Side::LEFT, 1.0f);
    // raw = -50 → clamp -15.
    const float lo = SteeringController::yawLawDeg(-1.0f, 100.0f, +1.0f, 15.0f, Side::LEFT, 1.0f);
    const bool pass = s_near(hi, 15.0f) && s_near(lo, -15.0f);
    Serial.printf("[SteerT7] clamp: %s  hi=%.2f lo=%.2f\n", pass ? "PASS" : "FAIL", hi, lo);
    return pass;
}

// ===========================================================================
//  Stage 1.1 — per-step yaw BUDGET (SteeringYawBudget). Driven directly (the
//  struct is pure; the full update() path is estop-gated off-hardware).
// ===========================================================================

// Bb1 — cmd consistency: over a synthetic half at constant turnEff, the summed
//       commanded deltas telescope to yawStep*turnEff*turnSign regardless of N.
static bool test_budgetCmdConsistency() {
    const float turnEff = 1.0f, yawStep = 2.5f, sign = STEERING_TURN_SIGN, maxYaw = 15.0f;
    const float halfSign = +1.0f;   // swing LEFT
    SteeringYawBudget b;
    auto q = [&](float h){ return SteeringController::yawLawDeg(turnEff, yawStep, sign, maxYaw, Side::LEFT, h); };
    b.openWindow(q(0.0f), /*stanceIsLeft=*/false);
    const int N = 40;
    for (int i = 0; i <= N; ++i) { float h = (float)i / N; b.tick(q(h), halfSign, 0,0,0, false, 0.0025f); }
    b.latch(STEERING_RATIO_MIN_CMD_DEG);
    const float expected = turnEff * yawStep * sign;
    const bool pass = s_near(b.cmdStep, expected, 0.01f);
    Serial.printf("[SteerB1] budget_cmd_consistency: %s  cmd=%.3f exp=%.3f\n",
                  pass ? "PASS" : "FAIL", b.cmdStep, expected);
    return pass;
}

// Bb2 — mid-half slew: with turnEff ramping during the window, the commanded
//       accumulation is the exact telescoped sum halfSign*(qLast - qFirst).
static bool test_budgetMidHalfSlew() {
    const float yawStep = 2.5f, sign = STEERING_TURN_SIGN, maxYaw = 15.0f, halfSign = +1.0f;
    SteeringYawBudget b;
    auto q = [&](float teff, float h){ return SteeringController::yawLawDeg(teff, yawStep, sign, maxYaw, Side::LEFT, h); };
    const int N = 50;
    const float q0 = q(0.0f, 0.0f);
    b.openWindow(q0, false);
    float qLast = q0;
    for (int i = 0; i <= N; ++i) {
        float h = (float)i / N, teff = (float)i / N;   // slew 0→1 across the half
        float qq = q(teff, h);
        b.tick(qq, halfSign, 0,0,0, false, 0.0025f);
        qLast = qq;
    }
    b.latch(STEERING_RATIO_MIN_CMD_DEG);
    const float expected = halfSign * (qLast - q0);
    const bool pass = s_near(b.cmdStep, expected, 0.001f);
    Serial.printf("[SteerB2] budget_midhalf_slew: %s  cmd=%.3f exp=%.3f\n",
                  pass ? "PASS" : "FAIL", b.cmdStep, expected);
    return pass;
}

// Bb3 — role swap: opening the next window re-primes qAnatPrev (no spurious cmd
//       delta at the boundary) and the stance-foot capture flips correctly.
static bool test_budgetRoleSwap() {
    const float yawStep = 2.5f, sign = STEERING_TURN_SIGN, maxYaw = 15.0f;
    SteeringYawBudget b;
    auto qA = [&](float h){ return SteeringController::yawLawDeg(1.0f, yawStep, sign, maxYaw, Side::LEFT,  h); };
    auto qB = [&](float h){ return SteeringController::yawLawDeg(1.0f, yawStep, sign, maxYaw, Side::RIGHT, h); };
    // Window A: swing LEFT → stance RIGHT → stanceIsLeft=false.
    b.openWindow(qA(0.0f), /*stanceIsLeft=*/false);
    const bool aStanceOk = (b.stanceIsLeft == false);
    for (int i = 0; i <= 20; ++i) { float h = (float)i / 20; b.tick(qA(h), +1.0f, 0,0,0, false, 0.0025f); }
    b.latch(STEERING_RATIO_MIN_CMD_DEG);
    // Boundary → Window B: swing RIGHT → stance LEFT → stanceIsLeft=true.
    const float qB0 = qB(0.0f);
    b.openWindow(qB0, /*stanceIsLeft=*/true);
    b.tick(qB0, -1.0f, 0,0,0, false, 0.0025f);          // first B tick at the primed command
    const bool noSpurious = s_near(b.cmdAccum, 0.0f, 0.0001f);
    const bool bStanceOk  = (b.stanceIsLeft == true);
    const bool pass = aStanceOk && bStanceOk && noSpurious;
    Serial.printf("[SteerB3] budget_role_swap: %s  aStanceL0=%d bStanceL1=%d cmdAccum=%.4f\n",
                  pass ? "PASS" : "FAIL", (int)aStanceOk, (int)bStanceOk, b.cmdAccum);
    return pass;
}

// Bb4 — post-touchdown partition: the locked buckets equal the deltas summed
//       only while touchdownLocked, exactly (cmd via telescoping, body via dt).
static bool test_budgetPostTdPartition() {
    const float yawStep = 2.5f, sign = STEERING_TURN_SIGN, maxYaw = 15.0f, halfSign = +1.0f;
    const float torsoDps = 8.0f, dt = 0.0025f;
    SteeringYawBudget b;
    auto q = [&](float h){ return SteeringController::yawLawDeg(1.0f, yawStep, sign, maxYaw, Side::LEFT, h); };
    const int N = 40, split = N / 2;
    b.openWindow(q(0.0f), false);
    float qSplit = 0, qLast = 0; int lockedTicks = 0;
    for (int i = 0; i <= N; ++i) {
        float h = (float)i / N; float qq = q(h);
        bool locked = (i > split);
        b.tick(qq, halfSign, torsoDps, 0, 0, locked, dt);
        if (i == split) qSplit = qq;
        if (i == N)     qLast  = qq;
        if (locked)     lockedTicks++;
    }
    b.latch(STEERING_RATIO_MIN_CMD_DEG);
    const float expCmdPost  = halfSign * (qLast - qSplit);
    const float expBodyPost = torsoDps * dt * (float)lockedTicks;
    const bool cmdOk  = s_near(b.cmdPostTdStep,  expCmdPost,  0.01f);
    const bool bodyOk = s_near(b.bodyPostTdStep, expBodyPost, 0.001f);
    const bool pass = cmdOk && bodyOk;
    Serial.printf("[SteerB5] budget_posttd_partition: %s  cmdPost=%.3f(exp %.3f) bodyPost=%.3f(exp %.3f)\n",
                  pass ? "PASS" : "FAIL", b.cmdPostTdStep, expCmdPost, b.bodyPostTdStep, expBodyPost);
    return pass;
}

// Bb6 — disabled: with enabled=false, update() leaves every budget field at 0
//       (estop-independent: the enabled gate returns first).
static bool test_budgetDisabledZero() {
    MotionManager mm; SteeringController sc; sc.init(&mm);
    SteeringConfig cfg; cfg.enabled = false; cfg.turnCmd = 1.0f; sc.setConfig(cfg);
    sc.setYawRates(0.5f, 0.5f, 0.5f);
    sc.setGaitInput({ true, Side::LEFT, 0.5f, true });
    sc.update(0.0025f);
    const SteeringState& s = sc.getState();
    const bool pass = s_near(s.cmdYawStepDeg, 0) && s_near(s.bodyYawStepDeg, 0)
                   && s_near(s.stanceFootStepDeg, 0) && s_near(s.swingFootStepDeg, 0)
                   && s_near(s.realizationRatio, 0) && s_near(s.meanRealizationRatio, 0)
                   && s_near(s.yawRateDps, 0);
    Serial.printf("[SteerB6] budget_disabled_zero: %s\n", pass ? "PASS" : "FAIL");
    return pass;
}

// ---------------------------------------------------------------------------
void runSteeringControllerTests() {
    Serial.println("\n[Steering] ══ Unit Tests ══════════════════════════════════");
    int pass = 0, total = 0;
    auto run = [&](bool r) { total++; if (r) pass++; };
    run(test_disabledNoSubmit());
    run(test_turnZeroYawZero());
    run(test_accumulationHalfSign());
    run(test_boundaryContinuity());
    run(test_slewBounded());
    run(test_doubleSupportHold());
    run(test_clamp());
    // Stage 1.1 per-step yaw budget.
    run(test_budgetCmdConsistency());
    run(test_budgetMidHalfSlew());
    run(test_budgetRoleSwap());
    run(test_budgetPostTdPartition());
    run(test_budgetDisabledZero());
    Serial.printf("[Steering] ══ %d / %d PASSED ═══════════════════════════════\n\n",
                  pass, total);
}

#endif // UNIT_TEST
