// =============================================================================
// FILE:    steering_controller.h
// MODULE:  steering_controller
// LAYER:   3.5 — peer of WeightShift / SwingFootLeveler (gait / motion authoring)
//
// PURPOSE:
//   Stage-1 open-loop hip-yaw turning. Owns the two otherwise-undriven hip-yaw
//   joints (ch4 right dir+1, ch7 left dir-1, both ±50°) and submits an
//   anatomical yaw command to BOTH legs at SOURCE_GAIT so the body turns while
//   GaitMode::RUNNING. The yaw law is direct kinematics — no IK, no FootTarget,
//   no balance coupling. This is the firmware analogue of WeightShift: a small
//   author that GaitController feeds and main.cpp ticks before flush().
//
// THE LAW (per tick, identical anatomical command to both legs):
//   s        = smoothstep(halfPhase) = h*h*(3 - 2h)
//   halfSign = (swingSide == LEFT) ? +1 : -1
//   yaw      = clamp(turnEff * yawStepDeg * turnSign * halfSign * (s - 0.5),
//                    ±maxYawDeg)
//   submit SOURCE_GAIT yaw to LEFT and RIGHT hip-yaw (direction field mirrors).
//
//   halfSign flips because +q rotates the body CCW on right-stance (ψ=φ_R+q_R)
//   and CW on left-stance (ψ=φ_L−q_L); dropping it yields pure heading
//   oscillation with zero net turn. (s−0.5) gives zero yaw-rate at touchdown so
//   the planted foot does not scrub. turnEff (a slewed image of turnCmd) makes
//   the output C¹ regardless of when the operator moves the command.
//
// CONCURRENCY:
//   update() runs on the 400 Hz control task; WebComm writes config/turnCmd from
//   the AsyncTCP task. External writes therefore use the targeted single-field
//   setters below (each an atomic 32-bit store), NOT a getConfig→modify→setConfig
//   read-modify-write, which the control task could read torn. (Same hazard
//   CLAUDE.md documents for BalanceConfig/TorsoConfig.)
//
// SHIPS INERT: enabled defaults false → update() submits nothing → bit-identical
//   firmware until an operator enables it at runtime. See STEERING_PLAN.md.
//
// LAST CHANGED: 2026-06-11 | Hinz | Stage 1.1 — sign baked, per-step yaw budget
//               characterization telemetry (SteeringYawBudget). Law unchanged.
// =============================================================================

#ifndef STEERING_CONTROLLER_H
#define STEERING_CONTROLLER_H

#include <Arduino.h>
#include <math.h>               // fabsf (SteeringYawBudget::latch)
#include "motion_manager.h"     // MotionManager, SOURCE_GAIT
#include "side.h"               // Side, LegJoint, channelFor
#include "project_wide_defs.h"  // GAIT_STEER_* / STEERING_* defaults

// ---------------------------------------------------------------------------
// SteeringConfig — runtime-tunable parameters.
//   turnCmd is RUNTIME control (streamed by the operator joystick on CMD:STEER);
//   the rest are CONFIG (set rarely on CMD:STEER_TUNE). Both land in this one
//   struct — there is no second owner of steering state.
// ---------------------------------------------------------------------------
struct SteeringConfig {
    bool  enabled        = GAIT_STEER_ENABLED_DEFAULT;       // false → submits nothing
    float turnCmd        = 0.0f;                             // [-1,+1] setpoint; 0 = straight
    float yawStepDeg     = GAIT_STEER_YAW_STEP_DEG_DEFAULT;  // body-yaw per footfall (deg)
    // Hardware-verified direction (baked, NOT operator-tunable). See
    // STEERING_TURN_SIGN in project_wide_defs.h. The field + the yawLawDeg()
    // parameter are retained, but no runtime control writes it any more.
    float turnSign       = STEERING_TURN_SIGN;
    float maxYawDeg      = GAIT_STEER_MAX_YAW_DEG;           // |output| clamp ≪ ±50° limit
    float turnSlewPerSec = GAIT_STEER_TURN_SLEW_PER_SEC;     // |turnEff| change cap (units/s)
};

// Gait-derived inputs pushed once per tick by GaitController.
// Explicit constructors (rather than relying on aggregate init) so the
// positional brace form `{active, swingSide, halfPhase}` at the GaitController
// call site compiles under gnu++11, where default member initializers make a
// struct a non-aggregate.
struct SteeringInput {
    bool  active         = false;     // mode == RUNNING (yaw allowed)
    Side  swingSide      = Side::LEFT;// current swing leg → halfSign
    float halfPhase      = 0.0f;      // [0,1) within the half-cycle
    bool  touchdownLocked = false;    // gait touchdown latch (Stage 1.1 budget bucket)
    SteeringInput() = default;
    // touchdownLocked defaulted so the Stage-1 3-arg form still compiles.
    SteeringInput(bool a, Side s, float h, bool td = false)
        : active(a), swingSide(s), halfPhase(h), touchdownLocked(td) {}
};

// ---------------------------------------------------------------------------
// SteeringYawBudget — per-half-step yaw accounting (Stage 1.1 characterization).
//
// Pure, self-contained accumulator: no estop, no I/O, no member of the control
// path. Integrates, over one half-step window, the COMMANDED body yaw (summed
// from the submitted joint deltas — exact, model-free) against the MEASURED body
// yaw and the two foot yaws (gyro-Z integrals). A pre/post-touchdown split shows
// where in the step the rotation happens. SteeringController owns one instance
// and feeds it after its gates; the unit tests drive it directly (the full
// update() path is estop-gated and not exercisable off-hardware).
//
// SIGN: dCmd = halfSign·Δqanat. qanat already contains the law's halfSign, and
// the stance relation contributes another (dψ=+dq on right-stance, −dq on left),
// so halfSign·Δqanat is the sign-consistent body-yaw increment. Over a full half
// this telescopes to yawStepDeg·turnEff·turnSign (see unit tests).
// ---------------------------------------------------------------------------
struct SteeringYawBudget {
    // Live window accumulators (degrees).
    float cmdAccum    = 0.0f;
    float bodyAccum   = 0.0f;
    float stanceAccum = 0.0f;
    float swingAccum  = 0.0f;
    float cmdPostTd   = 0.0f;
    float bodyPostTd  = 0.0f;
    float qAnatPrevDeg = 0.0f;
    bool  primed       = false;   // qAnatPrev valid for this window
    bool  stanceIsLeft = false;   // physical stance foot, captured at window open

    // Latched results of the LAST completed window (degrees / ratio).
    float cmdStep=0, bodyStep=0, stanceStep=0, swingStep=0;
    float cmdPostTdStep=0, bodyPostTdStep=0, ratio=0;

    void reset() { *this = SteeringYawBudget{}; }

    // Start a new window: clear accumulators, prime qAnatPrev to the current
    // command (so the first in-window delta is 0), capture the stance foot.
    void openWindow(float qAnatDeg, bool stanceFootIsLeft) {
        cmdAccum = bodyAccum = stanceAccum = swingAccum = 0.0f;
        cmdPostTd = bodyPostTd = 0.0f;
        qAnatPrevDeg = qAnatDeg;
        primed       = true;
        stanceIsLeft = stanceFootIsLeft;
    }

    // One control tick inside the open window. halfSign = stance sign
    // (+1 right-stance / −1 left-stance). Foot rates are already resolved to
    // stance/swing by the caller using this window's stanceIsLeft.
    void tick(float qAnatDeg, float halfSign,
              float torsoDps, float stanceDps, float swingDps,
              bool tdLocked, float dt) {
        if (!primed) { qAnatPrevDeg = qAnatDeg; primed = true; }
        const float dCmd = halfSign * (qAnatDeg - qAnatPrevDeg);
        qAnatPrevDeg = qAnatDeg;
        cmdAccum    += dCmd;
        bodyAccum   += torsoDps  * dt;
        stanceAccum += stanceDps * dt;
        swingAccum  += swingDps  * dt;
        if (tdLocked) { cmdPostTd += dCmd; bodyPostTd += torsoDps * dt; }
    }

    // Close the window: publish results + the realization ratio (guarded so a
    // near-zero command does not divide noise into a huge number).
    void latch(float ratioMinCmdDeg) {
        cmdStep = cmdAccum; bodyStep = bodyAccum;
        stanceStep = stanceAccum; swingStep = swingAccum;
        cmdPostTdStep = cmdPostTd; bodyPostTdStep = bodyPostTd;
        ratio = (fabsf(cmdStep) >= ratioMinCmdDeg) ? (bodyStep / cmdStep) : 0.0f;
    }
};

// Read-only telemetry snapshot (STEER: packet).
struct SteeringState {
    float turnCmd   = 0.0f;   // last commanded setpoint
    float turnEff   = 0.0f;   // slewed effective command actually used by the law
    float yawCmdDeg = 0.0f;   // anatomical yaw submitted to both legs this tick
    bool  active    = false;  // input active AND not estopped this tick

    // ---- Stage 1.1 characterization (telemetry only) ----------------------
    // Live torso yaw rate (deg/s), lightly IIR-smoothed for display.
    float yawRateDps = 0.0f;
    // Live per-foot yaw rates (deg/s, sign-aligned, raw). Published so the
    // gyro-sign gate (rotate the robot CCW → all three must read positive) can
    // be checked from the steering panel; feet are a different IMU model than the
    // torso, so their sign is not guaranteed to match under one sign constant.
    float footLeftYawDps  = 0.0f;
    float footRightYawDps = 0.0f;
    // Per-footfall yaw budget, latched at each swing-side flip (degrees):
    //   cmd*   = DERIVED  (summed submitted joint deltas — the commanded body yaw)
    //   body*  = MEASURED (torso gyro-Z integral)
    //   stance/swing* = MEASURED (per-foot gyro-Z integral over the window)
    //   *PostTd = the portion accumulated while the gait touchdown latch was set
    //   realizationRatio = bodyYawStepDeg / cmdYawStepDeg (guarded; relative A/B)
    float cmdYawStepDeg     = 0.0f;
    float bodyYawStepDeg    = 0.0f;
    float stanceFootStepDeg = 0.0f;
    float swingFootStepDeg  = 0.0f;
    float cmdYawPostTdDeg   = 0.0f;
    float bodyYawPostTdDeg  = 0.0f;
    float realizationRatio  = 0.0f;
    // 6-step rolling means (same idea as the GaitState step-timing ring).
    float meanCmdYawStepDeg     = 0.0f;
    float meanBodyYawStepDeg    = 0.0f;
    float meanStanceFootStepDeg = 0.0f;
    float meanSwingFootStepDeg  = 0.0f;
    float meanRealizationRatio  = 0.0f;
};

// ---------------------------------------------------------------------------
class SteeringController {
public:
    SteeringController() = default;

    // Link the joint authority layer. Call once in setup().
    void init(MotionManager* mm) { _mm = mm; }

    // ---- Config read (single reader; telemetry / broadcast echo) -----------
    SteeringConfig       getConfig() const { return _cfg; }
    const SteeringState& getState()  const { return _state; }

    // ---- Targeted ATOMIC setters (cross-task safe — see header concurrency) ----
    // setTurnCmd is the HOT path (joystick rate). Each writes a single field.
    void setTurnCmd(float v)        { _cfg.turnCmd        = constrain(v, -1.0f, 1.0f); }
    void setEnabled(bool e)         { _cfg.enabled        = e; }
    void setYawStepDeg(float d)     { _cfg.yawStepDeg     = d; }
    void setTurnSlewPerSec(float s) { _cfg.turnSlewPerSec = s; }
    // NOTE: setTurnSign() removed — the sign is hardware-verified and baked into
    // STEERING_TURN_SIGN (project_wide_defs.h); no operator control writes it.

    // setConfig is retained ONLY for unit-test construction. Production writes go
    // through the field setters above to avoid the cross-task torn-read race.
    void setConfig(const SteeringConfig& cfg) { _cfg = cfg; }

    // Push this tick's gait state (mode/swingSide/halfPhase/touchdownLocked).
    // Called from GaitController::update() right after the swing-leg mask publish.
    // Detects the footfall boundary here (swing-side flip while active) so the
    // budget window can be latched on the next update().
    void setGaitInput(const SteeringInput& in) {
        if (in.active && _in.active && in.swingSide != _in.swingSide) {
            _footfallPending = true;
        }
        _in = in;
    }

    // Inject the three measured yaw rates (rad/s, robot/world z-up CCW+ via
    // IMUState::yawRate). Called from main.cpp each control tick BEFORE update(),
    // after the estimators run. Converted to deg/s and sign-aligned here. Pass 0
    // for a foot whose estimator is invalid (footstrike flicker) so its integral
    // does not accumulate garbage.
    void setYawRates(float torsoRadS, float leftFootRadS, float rightFootRadS) {
        _torsoYawDps     = torsoRadS     * RAD_TO_DEG * STEERING_YAW_RATE_SIGN;
        _leftFootYawDps  = leftFootRadS  * RAD_TO_DEG * STEERING_YAW_RATE_SIGN;
        _rightFootYawDps = rightFootRadS * RAD_TO_DEG * STEERING_YAW_RATE_SIGN;
    }

    // Run the yaw law and submit to MotionManager. Call once per control tick,
    // AFTER gaitController.update() and BEFORE motionManager.flush().
    void update(float dt_s);

    // ---- Pure helpers (no estop, no submit, no member state) ---------------
    // Exposed static so the unit tests can verify the law and the slew directly:
    // the full update() path is gated by oe_is_estopped(), which is true during
    // the boot-time test run, so the submit path is not exercisable off-hardware.

    // The yaw law: anatomical hip-yaw command (deg) for a given effective
    // command. This is the value update() submits to BOTH legs (the JointConfig
    // `direction` field mirrors left/right). Clamped to ±maxYawDeg.
    static float yawLawDeg(float turnEff, float yawStepDeg, float turnSign,
                           float maxYawDeg, Side swingSide, float halfPhase);

    // One slew step of `current` toward `target`, change capped at ±maxStep.
    static float slewToward(float current, float target, float maxStep);

private:
    MotionManager* _mm = nullptr;
    SteeringConfig _cfg;
    SteeringInput  _in;
    SteeringState  _state;

    float _turnEff = 0.0f;   // slewed image of turnCmd; the law uses THIS, not turnCmd

    // ---- Stage 1.1 characterization state (telemetry only) ----------------
    float _torsoYawDps     = 0.0f;   // measured yaw rates (deg/s, sign-aligned)
    float _leftFootYawDps  = 0.0f;
    float _rightFootYawDps = 0.0f;
    float _measYawRateFilt = 0.0f;   // IIR-smoothed torso rate for the live display

    SteeringYawBudget _budget;       // per-half-step yaw accounting
    bool  _wasActive       = false;  // edge-detect active→inactive to reset
    bool  _footfallPending = false;  // swing-side flip seen; latch on next update()

    // 6-step rolling ring for the headline budget metrics.
    static constexpr uint8_t RING = STEERING_STEP_RING_SIZE;
    float   _ringCmd[RING]    = {0};
    float   _ringBody[RING]   = {0};
    float   _ringStance[RING] = {0};
    float   _ringSwing[RING]  = {0};
    float   _ringRatio[RING]  = {0};
    uint8_t _ringIdx = 0;
    uint8_t _ringCount = 0;

    // Copy the just-latched budget into _state, push the ring, recompute means.
    void _publishBudgetStep();
    // Zero the published step fields + ring (active→inactive / disable).
    void _clearBudgetTelemetry();
};

#endif // STEERING_CONTROLLER_H
