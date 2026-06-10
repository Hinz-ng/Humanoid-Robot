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
// LAST CHANGED: 2026-06-09 | Hinz | Stage 1 open-loop hip-yaw steering
// =============================================================================

#ifndef STEERING_CONTROLLER_H
#define STEERING_CONTROLLER_H

#include <Arduino.h>
#include "motion_manager.h"     // MotionManager, SOURCE_GAIT
#include "side.h"               // Side, LegJoint, channelFor
#include "project_wide_defs.h"  // GAIT_STEER_* defaults

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
    float turnSign       = +1.0f;                            // ±1 hardware direction flip
    float maxYawDeg      = GAIT_STEER_MAX_YAW_DEG;           // |output| clamp ≪ ±50° limit
    float turnSlewPerSec = GAIT_STEER_TURN_SLEW_PER_SEC;     // |turnEff| change cap (units/s)
};

// Gait-derived inputs pushed once per tick by GaitController.
// Explicit constructors (rather than relying on aggregate init) so the
// positional brace form `{active, swingSide, halfPhase}` at the GaitController
// call site compiles under gnu++11, where default member initializers make a
// struct a non-aggregate.
struct SteeringInput {
    bool  active    = false;          // mode == RUNNING (yaw allowed)
    Side  swingSide = Side::LEFT;     // current swing leg → halfSign
    float halfPhase = 0.0f;           // [0,1) within the half-cycle
    SteeringInput() = default;
    SteeringInput(bool a, Side s, float h) : active(a), swingSide(s), halfPhase(h) {}
};

// Read-only telemetry snapshot (STEER: packet).
struct SteeringState {
    float turnCmd   = 0.0f;   // last commanded setpoint
    float turnEff   = 0.0f;   // slewed effective command actually used by the law
    float yawCmdDeg = 0.0f;   // anatomical yaw submitted to both legs this tick
    bool  active    = false;  // input active AND not estopped this tick
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
    void setTurnSign(float s)       { _cfg.turnSign       = (s >= 0.0f) ? +1.0f : -1.0f; }
    void setTurnSlewPerSec(float s) { _cfg.turnSlewPerSec = s; }

    // setConfig is retained ONLY for unit-test construction. Production writes go
    // through the field setters above to avoid the cross-task torn-read race.
    void setConfig(const SteeringConfig& cfg) { _cfg = cfg; }

    // Push this tick's gait state (mode/swingSide/halfPhase). Called from
    // GaitController::update() right after the swing-leg mask publish.
    void setGaitInput(const SteeringInput& in) { _in = in; }

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
};

#endif // STEERING_CONTROLLER_H
