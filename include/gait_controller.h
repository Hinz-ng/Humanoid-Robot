// =============================================================================
// FILE:    gait_controller.h
// MODULE:  gait
// LAYER:   3.5 — Gait / Motion
// PURPOSE: Phase-based stepping-in-place controller (Phase 1).
//          Drives lateral CoM shift via WeightShift and swing foot lift via
//          MotionManager SOURCE_GAIT. Phase only advances within each half-cycle
//          once the weight shift gate is satisfied (CoM stable over stance foot
//          before swing foot lifts), preventing premature lifts that cause falls.
// INPUTS:  IMUState (roll — used for Phase 2 gate logging), dt_s, WeightShift*
// OUTPUTS: SOURCE_GAIT hip/knee commands via MotionManager;
//          trigger() calls to WeightShift for lateral CoM shift
// DEPENDENCIES: motion_manager.h, state_estimator.h, project_wide_defs.h
// LAST CHANGED: 2026-04-25 | Hinz | Phase 1 stepping in place
// =============================================================================

#ifndef GAIT_CONTROLLER_H
#define GAIT_CONTROLLER_H

#include <Arduino.h>
#include "joint_config.h"
#include "motion_manager.h"
#include "state_estimator.h"
#include "project_wide_defs.h"
#include "gait_types.h"
#include "leg_ik.h"

class WeightShift;  // forward declaration — full include in .cpp only

// ---------------------------------------------------------------------------
//  Step execution sub-state (within each half-cycle).
//
//  WAIT_SHIFT: CoM ramp in progress; half-phase holds at 0; no foot lift.
//              Exits to SWINGING when |wsProgress| >= liftGateThreshold.
//  SWINGING:   Gate open; half-phase advances; foot follows a sin(x) arc
//              from 0 (grounded) to 1 (peak) back to 0 (grounded).
// ---------------------------------------------------------------------------
enum class StepSubState : uint8_t {
    WAIT_SHIFT  = 0,
    SWINGING    = 1,
    // Inserted between SWINGING (previous half done) and WAIT_SHIFT (next half).
    // Weight shift is centered (trigger NONE) and the FSM holds for
    // GAIT_STABILIZE_MS so structural and IIR transients damp before the next
    // direction is commanded. Prevents back-to-back shifts from destabilising.
    STABILIZING = 2,
    STOPPING    = 3,
};

// ---------------------------------------------------------------------------
//  GaitMode — top-level controller mode.
// ---------------------------------------------------------------------------
enum class GaitMode : uint8_t {
    IDLE        = 0,  // no motion; WeightShift returns to center on stop
    RUNNING     = 1,  // continuous stepping in place
    SINGLE_STEP = 2,  // complete exactly one full cycle (both legs), then IDLE
    WSHIFT_ONLY = 3,  // CoM shift only — no foot lift (initial CoM verification)
    POSE_STEP   = 4,  // pose-by-pose: pauses at gate-open, peak, and landing
                      // call nextPose() each time to advance to the next pose
};

// ---------------------------------------------------------------------------
//  GaitConfig — all runtime-tunable parameters.
//  Defaults sourced from project_wide_defs.h; override via setConfig() or
//  CMD:GAIT_TUNE over WebSocket.
// ---------------------------------------------------------------------------
struct GaitConfig {
    // Stepping rate (cycles/sec). One cycle = both legs take one step.
    // Half-cycle duration = 1 / (2 × phaseRateHz).
    // Start at 0.3–0.4 Hz; raise only after single-leg stance is confirmed.
    float phaseRateHz       = GAIT_PHASE_RATE_HZ;   // WS key: "phaseRate"

    // Fraction of |weight shift progress| [0, 1) required before the swing
    // foot is allowed to lift. Gate opens when |wsProgress| >= this value.
    // 0.70 = 70% shift complete. Lower to 0.50 if gate never opens.
    // Raise toward 0.90 if robot falls when foot leaves ground.
    float liftGateThreshold = 0.70f;  // [0, 1). WS: "liftGate"

    // Phase 2 IMU roll threshold (rad) — logged for diagnostics but not yet
    // used as a gate. Will gate foot lift in Phase 2 after single-leg stance
    // validation confirms the threshold value is reliable.
    float wsThresholdRad    = GAIT_WEIGHT_SHIFT_THRESHOLD_RAD;  // WS: "wsThresh"

    // Stance hip→ankle drop in mm passed to LegIK as h_sagittal_mm.
    // Sourced from GAIT_STANCE_HEIGHT_MM. Override via WS:"stanceH_mm".
    // Range guard: LegIK rejects < 10mm and clamps reach to 99% of
    // (L_THIGH + L_SHANK).
    float stanceHeightMm    = GAIT_STANCE_HEIGHT_MM;

    // Swing foot peak vertical clearance (mm). Bounded by the caller so
    // stanceHeightMm - stepHeightMm stays inside IK's safe range.
    float stepHeightMm      = GAIT_STEP_HEIGHT_MM;
};

// ---------------------------------------------------------------------------
//  GaitState — read-only snapshot broadcast to WebSocket each tick.
// ---------------------------------------------------------------------------
struct GaitState {
    GaitMode     mode       = GaitMode::IDLE;
    StepSubState subState   = StepSubState::WAIT_SHIFT;
    // Display phase [0, 1): currentHalf × 0.5 + halfPhase × 0.5.
    float        phase      = 0.0f;
    float        liftL_deg  = 0.0f;   // left hip extension sent to MotionManager (deg)
    float        liftR_deg  = 0.0f;   // right hip extension sent to MotionManager (deg)
    float        wsProgress = 0.0f;   // WeightShift progress at last tick (signed, [-1,+1])
    bool         liftGateOK = false;  // |wsProgress| in correct direction >= liftGateThreshold
    bool         running    = false;  // true when mode != IDLE
    // POSE_STEP hold point: 0=running, 1=gate-open, 2=peak swing, 3=foot landed.
    uint8_t      poseHold   = 0;

    // Stage 1 IK telemetry. Updated each tick by _submitFootTargetIK().
    uint8_t      lastIKStatusL = 0;   // IKStatus enum value
    uint8_t      lastIKStatusR = 0;
    float        lastReachPctL = 0.0f;
    float        lastReachPctR = 0.0f;
};

// ---------------------------------------------------------------------------
//  GaitController
// ---------------------------------------------------------------------------
class GaitController {
public:
    GaitController() = default;

    // Wire dependencies. Call in setup() AFTER weightShift.init().
    // ws may be nullptr — CoM shift disabled; foot lift still works.
    void init(MotionManager* mm, WeightShift* ws = nullptr);

    void start(GaitMode mode);  // start a mode; start(IDLE) is equivalent to stop()
    void stop();                 // return to IDLE; centers WeightShift
    void singleStep();           // start SINGLE_STEP mode (one full cycle, then IDLE)
    // Advance to the next pause point in POSE_STEP mode.
    // Ignored if mode != POSE_STEP or not currently paused.
    void nextPose();

    // Call every tick from main.cpp, after weightShift.update() and before
    // balanceController.update(). imuState is used for Phase 2 roll logging.
    void update(float dt_s, const IMUState& imuState);

    GaitConfig       getConfig() const          { return _cfg; }
    void             setConfig(const GaitConfig& cfg) { _cfg = cfg; }
    const GaitState& getState()  const          { return _state; }

private:
    MotionManager* _mm = nullptr;
    WeightShift*   _ws = nullptr;

    GaitConfig   _cfg;
    GaitState    _state;

    // Sub-cycle tracking.
    // _currentHalf: 0 = right foot swings (LEFT shift), 1 = left foot swings (RIGHT shift).
    // _halfPhase:   [0, 1) within current half-cycle; advances only in SWINGING sub-state.
    // _subState:    mirrors _state.subState; kept local for clarity in update().
    uint8_t      _currentHalf = 0;
    float        _halfPhase   = 0.0f;
    StepSubState _subState    = StepSubState::WAIT_SHIFT;

    // Last direction sent to WeightShift (avoids redundant trigger calls).
    // +1 = LEFT shift, -1 = RIGHT shift, 0 = none/center.
    int8_t _lastShiftDir = 0;

    // POSE_STEP pause state.
    // 0 = not paused. 1 = held at gate-open. 2 = held at peak. 3 = held at landing.
    uint8_t _poseHoldPt = 0;

    // millis() timestamp at which the current STABILIZING substate began.
    // Compared against GAIT_STABILIZE_MS to decide when to release into WAIT_SHIFT.
    uint32_t _stabilizeStartMs = 0;
    uint32_t _stopStartMs      = 0;
    float    _stopSwingFrac    = 0.0f;
    uint8_t  _stopHalf         = 0;

    // Trigger WeightShift direction matching the given half-cycle.
    void  _triggerShiftForHalf(uint8_t half);

    // Complete current half-cycle: advance to next half, re-gate, or stop.
    void  _advanceHalf();

    // sin(localPhase × π) → swing fraction [0, 1] from grounded to peak and back.
    float _swingFrac(float localPhase) const;

    // Build the FootTarget for one leg this tick.
    //   Stance leg: y_mm=0, x_mm=0, h_sagittal_mm = stanceHeightMm.
    //   Swing  leg: x_mm=0, y_mm=0, h_sagittal_mm = stanceHeightMm − liftMm.
    //                where liftMm = swingFrac × GAIT_STEP_HEIGHT_MM (Stage 1: vertical only).
    // Returns target with valid=true on success. valid=false means the
    // caller should not submit (used for IDLE / paused states).
    FootTarget _buildFootTarget(bool isRight, float swingFrac, bool isSwing) const;

    // Resolve a FootTarget through LegIK and submit the joint angles via
    // SOURCE_GAIT. Logs (rate-limited) and skips submission on
    // IKStatus::DOMAIN_ERROR. Returns the LegIKResult so callers can update
    // telemetry without re-solving.
    LegIKResult _submitFootTargetIK(bool isRight, const FootTarget& t);

    // Re-assert both legs' stance baseline for this tick.
    // Used whenever gait or WeightShift owns the legs, even if no swing is active.
    void _submitStanceBothLegs();
};

#endif // GAIT_CONTROLLER_H
