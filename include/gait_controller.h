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
#include "foot_contact.h"
#include "side.h"            // Side / LegJoint / channelFor — Phase 4 / Phase 5
#include "weight_shift.h"    // ShiftDirection enum — Phase 5 _lastShiftDir type

// WeightShift class still forward-declared below for the pointer member.
struct BalanceState; // forward declaration — full include in gait_controller.cpp
class BalanceController; // forward declaration
class SwingFootLeveler;  // forward declaration — full include in gait_controller.cpp
class FootImpactDetector; // forward declaration — full include in gait_controller.cpp
                          // Touchdown-awareness Stage 1: consumer of the
                          // per-side impact detector. Detector remains gait-
                          // agnostic; the policy (when/how to latch) lives here.

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
//  RecoverySubState — sub-states inside RECOVERY_STEP mode (Stage 2).
//
//  TRIGGERED → PRE_LIFT → LIFTING → LANDING → DAMP_HOLD → SOFT_RETURN → COMPLETE → (back to IDLE)
//  Open-loop timing: RECOVERY_PRE_LIFT_MS / RECOVERY_LIFT_MS / RECOVERY_LAND_MS /
//  RECOVERY_DAMP_HOLD_MS / RECOVERY_SOFT_RETURN_MS.
// ---------------------------------------------------------------------------
enum class RecoverySubState : uint8_t {
    TRIGGERED   = 0,  // direction resolved; PRE_LIFT starts next tick
    PRE_LIFT    = 1,  // hold stance counter-shift while swing foot remains planted
    LIFTING     = 2,  // swing foot rising on sin arc to peak height + lateral offset
    LANDING     = 3,  // swing foot descending onto target lateral position
    DAMPING     = 4,  // Stage 1 alias — kept for telemetry compat (= DAMP_HOLD)
    DAMP_HOLD   = 4,  // explicit name — same numeric value
    SOFT_RETURN = 5,  // Stage 2: foot offset decays smoothly back to centered stance
    COMPLETE    = 6,  // return to IDLE on this tick; clear all recovery state
};

// ---------------------------------------------------------------------------
//  GaitMode — top-level controller mode.
// ---------------------------------------------------------------------------
enum class GaitMode : uint8_t {
    IDLE          = 0,  // no motion; WeightShift returns to center on stop
    RUNNING       = 1,  // continuous stepping in place
    SINGLE_STEP   = 2,  // complete exactly one full cycle (both legs), then IDLE
    WSHIFT_ONLY   = 3,  // CoM shift only — no foot lift (initial CoM verification)
    POSE_STEP     = 4,  // pose-by-pose: pauses at gate-open, peak, and landing
                        // call nextPose() each time to advance to the next pose
    RECOVERY_STEP = 5,  // Stage 1: protective lateral catch step on disturbance
};

// ---------------------------------------------------------------------------
//  LiftGateMode - independent gate options on top of the always-required
//  kinematic check (progressOK). These are NOT monotonic refinements; each
//  mode picks ONE extra predicate the swing foot must satisfy.
//
//  progressOK is always required (except in OFF). The extra predicate is
//  selected per mode:
//
//    OFF        - kinematic only. Diagnostic. Foot lifts as soon as
//                 |wsProgress| >= liftGateThreshold.
//    IMU_ROLL   - torso roll past wsThresholdRad in correct direction.
//                 Only reliable in joint-space balance mode with WeightShift
//                 injecting roll setpoint. Useless in task-space mode where
//                 lateral CoM transfer is by foot placement, not torso tilt.
//    STANCE_FLAT - stance foot's pitch and roll within flat threshold.
//                  F3 headline. Works in both joint-space and task-space modes.
//                  Currently the most reliable gate (per hardware tests).
//    STANCE_LOADED - stance foot's pitch shows the loaded-deviation signature.
//                    Requires isFootLoaded() to be tuned via F4 voltage-drift
//                    calibration. Opt-in for tuning sessions only.
//    COM_LATERAL   - neutral-corrected lateral CoM observation in mm past threshold.
//
//  Default at boot is STANCE_FLAT - empirically the only sub-gate that
//  reliably passes on this platform. Promote / demote at runtime via UI.
//  IMU_ROLL is kept as a selectable option for joint-space sessions and as a
//  diagnostic measurement (always evaluated, always broadcast for telemetry).
// ---------------------------------------------------------------------------
enum class LiftGateMode : uint8_t {
    OFF           = 0,  // kinematic only
    IMU_ROLL      = 1,  // torso roll past threshold (legacy joint-space)
    STANCE_FLAT   = 2,  // stance foot's pitch + roll inside flat threshold
    STANCE_LOADED = 3,  // stance foot pitch shows load signature (needs F4)
    // Observed lateral CoM displacement from torso IMU + inverted-pendulum
    // model. Works in both joint-space and task-space balance modes.
    // Neutral-corrected via GaitConfig::torsoRollNeutralRad -- robust to
    // small chassis/mounting asymmetries that bias the raw roll reference.
    // Recommended over IMU_ROLL once neutrals are tared.
    COM_LATERAL   = 4,
    STANCE_FLAT_COM_LATERAL = 5,
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
    float liftGateThreshold = 0.85f;  // [0, 1). WS: "liftGate"

    // Phase 2 IMU roll threshold (rad) — logged for diagnostics but not yet
    // used as a gate. Will gate foot lift in Phase 2 after single-leg stance
    // validation confirms the threshold value is reliable.
    float wsThresholdRad    = GAIT_WEIGHT_SHIFT_THRESHOLD_RAD;  // WS: "wsThresh"

    // Phase F3: which single extra predicate the lift gate requires beyond
    // progressOK. See LiftGateMode block above.
    // Default STANCE_FLAT — verified working on hardware; IMU_ROLL and LOADED
    // each have known prerequisites and are not safe boot defaults.
    LiftGateMode liftGateMode = LiftGateMode::STANCE_FLAT;

    // Stance hip→ankle drop in mm passed to LegIK as h_sagittal_mm.
    // Sourced from GAIT_STANCE_HEIGHT_MM. Override via WS:"stanceH_mm".
    // Range guard: LegIK rejects < 10mm and clamps reach to 99% of
    // (L_THIGH + L_SHANK).
    float stanceHeightMm    = GAIT_STANCE_HEIGHT_MM;

    // Swing foot peak vertical clearance (mm). Bounded by the caller so
    // stanceHeightMm - stepHeightMm stays inside IK's safe range.
    float stepHeightMm      = GAIT_STEP_HEIGHT_MM;

    // Forward stride per cycle (mm). Swing leg's foot moves linearly from
    // -stepLengthMm/2 (behind hip) to +stepLengthMm/2 (ahead of hip) over
    // its swing phase. Stance leg is unaffected (Stage 2 — push-back is Stage 3).
    //
    // Hard-capped at stanceHeightMm × 0.20 by the WebComm parser to protect
    // hip pitch IK soft limits. Default 0.0 keeps this opt-in: existing
    // GAIT_START with no GAIT_TUNE will produce identical Phase 1 behavior.
    //
    // Tuning order: confirm Phase 1 unchanged at stepLengthMm=0, then
    // ramp 0 → 10 → 15 → 20 mm in 5 mm increments. Watch lastReachPctL/R
    // and IK status — stop on first LIMIT_CLAMPED. Do not raise phaseRateHz
    // and stepLengthMm in the same session.
    float stepLengthMm = GAIT_STEP_LENGTH_MM;

    // W1 — swing-envelope apex fraction. _swingFrac() warps localPhase so the
    // vertical lift apex lands here instead of at 0.5. Below 0.5 = faster
    // lift, slower descent (lower vertical speed approaching touchdown).
    // 0.5 reproduces the legacy symmetric raised cosine exactly. Clamped to
    // [0.05, 0.95] in _swingFrac(). WS key: "swingApex".
    float swingApexFrac = GAIT_SWING_APEX_FRAC_DEFAULT;

    // W2 — continuous lateral CoM trim. When enabled, observed lateral CoM
    // displacement is added to both feet's y_mm every tick (see
    // _comLateralTrimMm()). Independent of LiftGateMode::COM_LATERAL, which
    // is a one-shot gate; this trim is a continuous foot-placement correction.
    //   comLatTrimEnabled — master enable. Default ON: the trim is the W2
    //                       headline. Toggle OFF for A/B baseline runs.
    //   comLatTrimGain    — mm of foot trim per mm of observed CoM offset.
    //   comLatTrimMaxMm   — per-tick cap on the applied trim magnitude.
    bool  comLatTrimEnabled = true;                            // WS: "comLatTrim"
    float comLatTrimGain    = GAIT_COM_LAT_TRIM_GAIN_DEFAULT;   // WS: "comLatGain"
    float comLatTrimMaxMm   = GAIT_COM_LAT_TRIM_MAX_MM_DEFAULT; // WS: "comLatMax"

    // W5 — velocity-damping extension of the lateral CoM trim.
    //   comLatTrimKd       — Kd: mm of foot trim per (mm/s) of lateral CoM
    //                        velocity (units = seconds). Default 0 → W5 inert,
    //                        trim stays bit-identical to displacement-only W2.
    //   comLatTrimRateLpfAlpha — IIR retention factor for the roll-rate LPF.
    //                        0 = raw rate (noisy); →1 = heavy smoothing (laggy).
    //                        The LPF is mandatory — raw gyro rate would feed
    //                        high-frequency hiss straight into the Kd term.
    float comLatTrimKd            = GAIT_COM_LAT_TRIM_KD_DEFAULT;     // WS: "comLatKd"
    float comLatTrimRateLpfAlpha  = GAIT_COM_LAT_TRIM_RATE_LPF_ALPHA; // WS: "comLatRateLpf"

    // Stage 2.2 — sagittal capture-point step placement.
    // When enabled, swing leg's stride x_mm is augmented by:
    //   capture_offset = clamp(vx_CoM × inv_omega0 × captureGain,
    //                          -captureOffsetMaxMm, +captureOffsetMaxMm)
    // Composed onto stepLengthMm BEFORE per-tick stride formula evaluates,
    // so the swing foot lands at +stepLengthMm/2 + capture_offset and the
    // stance leg retreats with the unmodified stepLengthMm (capture is a
    // foot-placement correction, not a body-translation correction).
    // Default disabled — opt-in via UI toggle. Enables side-by-side comparison
    // of pre-Stage-2.2 vs post-Stage-2.2 walking on identical settings.
    bool  captureEnabled    = false;       // WS: "captureEnabled"
    float captureGain       = GAIT_CAPTURE_GAIN_DEFAULT;  // WS: "captureGain"
    float captureOffsetMaxMm = GAIT_CAPTURE_OFFSET_MAX_MM; // WS: "captureMax"

    // Stage 2 -- support-transfer dwell + timeout. See project_wide_defs.h
    // for rationale. 0 on either field reverts to pre-Stage-2 behavior
    // on that axis (no dwell / no timeout).
    uint32_t liftDwellMs    = GAIT_LIFT_DWELL_MS_DEFAULT;     // WS: "dwellMs"
    uint32_t waitShiftMaxMs = GAIT_WAIT_SHIFT_MAX_MS_DEFAULT; // WS: "waitMaxMs"

    // Stage 3 -- Lateral CoM observation gate. See project_wide_defs.h
    // for the model and tuning guidance. Threshold compared in mm.
    float comLateralGateMm = GAIT_COM_LATERAL_GATE_MM_DEFAULT;  // WS: "comLatGateMm"

    // Torso IMU neutral biases (rad). Subtracted from imuRoll / imuPitch
    // wherever "tilt relative to standing rest" is the meaningful signal.
    // Set via CMD:TORSO_TARE (captures current imuState; requires IDLE
    // and valid IMU) or directly via CMD:GAIT_TUNE:torsoNeutralRoll=...,
    // torsoNeutralPitch=.... Not persisted across reboots -- re-tare on
    // each power cycle or after any chassis change.
    //
    // CONSUMER MAP: torsoRollNeutralRad -> COM_LATERAL gate only (current).
    //               torsoPitchNeutralRad -> reserved for future sagittal use.
    // If a second consumer appears, promote these to a shared trim store.
    float torsoRollNeutralRad  = 0.0f;
    float torsoPitchNeutralRad = 0.0f;

    // -----------------------------------------------------------------------
    // Touchdown-awareness (Stage 0 + Stage 1).
    //
    // Consumer-side policy for the gait-agnostic FootImpactDetector signal.
    // When enabled, the swing leg's gait-generated stride progression term
    // and lift trajectory are frozen at touchdown so the foot stops scrubbing
    // forward through the body frame. The stance leg's retreat formula keeps
    // running unchanged, so the body advances naturally over the planted foot.
    //
    // touchdownEnabled    — master enable. Default ON: the latch is the
    //                       primary fix for the post-touchdown scrub failure
    //                       (see touchdown-awareness_integration_plan.md §2).
    //                       Toggle OFF for A/B baseline measurements.
    //
    // touchdownGatePhase  — minimum _halfPhase at which an impact detection
    //                       is allowed to fire the latch. Filters out
    //                       false-positive impacts during early/mid swing.
    //                       Default 0.7: late enough to be safely past peak,
    //                       generous enough to catch touchdowns ~5 mm
    //                       early due to floor variation.
    // -----------------------------------------------------------------------
    bool  touchdownEnabled   = true;
    float touchdownGatePhase = 0.7f;  // [0..1)

    // -----------------------------------------------------------------------
    // W4 — Touchdown compliance.
    //
    // Event-triggered geometric yield. When the early-contact / swing-
    // truncation latch fires (see touchdown-awareness block in update()),
    // the effective stance height is briefly shortened by a small offset so
    // both legs flex slightly and absorb the touchdown impulse, then the
    // offset decays smoothly back to zero. Transient and self-clearing — no
    // persistent posture bias, no force-control loop, no new sensing.
    //
    // This reuses the existing touchdown event verbatim: compliance fires
    // from the same `if (imp.detected)` branch that sets the latch. It does
    // NOT add a detection path of its own.
    //
    //   complianceEnabled — master enable. Default ON. Toggle OFF for A/B
    //                       baseline runs.
    //   complianceDepthMm — magnitude of the (negative) stance-height offset
    //                       applied at the event, in mm. Applied as -depth.
    //   complianceDecayMs — time over which the offset smoothsteps back to
    //                       zero after the event.
    bool     complianceEnabled = GAIT_TD_COMPLIANCE_ENABLED_DEFAULT;  // WS: "enabled"
    float    complianceDepthMm = GAIT_TD_COMPLIANCE_DEPTH_MM_DEFAULT; // WS: "depthMm"
    uint32_t complianceDecayMs = GAIT_TD_COMPLIANCE_DECAY_MS_DEFAULT; // WS: "decayMs"

    // -----------------------------------------------------------------------
    // Touchdown trim suppression — phase-gate of the W2/W5 lateral CoM trim.
    //
    // The foot-strike at touchdown injects a short torso-roll transient into
    // the IMU that is not a real CoM excursion. Without gating, it leaks into
    // _comLateralTrimMm() and the support polygon chases a phantom. When this
    // suppression is active the trim is held (or zeroed — see freeze flag)
    // while the touchdown latch is set and for touchdownTrimHoldMs afterward.
    //
    //   touchdownTrimSuppressEnabled — master enable. Default ON. Toggle OFF
    //                       to restore the pre-suppression trim behaviour
    //                       exactly (A/B baseline).
    //   touchdownTrimHoldMs — ms the suppression extends past the moment the
    //                       touchdown latch clears, so the post-strike roll
    //                       ring-down is rejected too. 0 = suppress only while
    //                       the latch itself is held.
    //   touchdownTrimFreezeHold — suppression mode. true = freeze the trim at
    //                       its last clean (pre-touchdown) value; false =
    //                       force the trim output to 0 during the window.
    //                       UI exposes a button to flip this live.
    // -----------------------------------------------------------------------
    bool     touchdownTrimSuppressEnabled = GAIT_TD_TRIM_SUPPRESS_ENABLED_DEFAULT;
    uint32_t touchdownTrimHoldMs          = GAIT_TD_TRIM_HOLD_MS_DEFAULT;
    bool     touchdownTrimFreezeHold      = GAIT_TD_TRIM_FREEZE_DEFAULT;
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

    // Phase F3: per-sub-gate booleans, evaluated every tick. Surface in the
    // GAIT: WS packet so the UI can show why a higher-mode gate is closed
    // even when the operator believes it should be open.
    //   gateImuRollOK     - imuRoll past threshold in correct signed direction
    //   gateStanceFlat    - isFootFlat() for the stance foot
    //   gateStanceLoaded  - isFootLoaded() for the stance foot
    // gateStanceIsLeft mirrors (_swingSide == Side::RIGHT) so the UI can label
    // the sub-gates with the foot they refer to without re-deriving the mapping.
    // Default at boot is OFF (kinematic only) — matches pre-F3 effective behaviour
    // where useImuGate was false in task-space mode. IMU_ROLL is opt-in and is only
    // reliable when joint-space balance is armed; do not promote in task-space mode.
    bool gateImuRollOK    = false;
    bool gateStanceFlat   = false;
    bool gateStanceLoaded = false;
    bool gateStanceIsLeft = false;

    // Stage 2 -- support-transfer telemetry. All values reset on entry to
    // WAIT_SHIFT and on transitions out of it.
    //   waitShiftElapsedMs: ms since current WAIT_SHIFT entry. Counts in
    //                       all WAIT_SHIFT ticks; useful for tuning.
    //   gateDwellMs:        ms since gate first went continuously OK in
    //                       this WAIT_SHIFT episode. 0 if gate currently
    //                       not OK (i.e. dwell counter has been reset).
    //   waitShiftTimedOut:  set true for one tick on the transition that
    //                       fired the timeout abort. UI uses this as an
    //                       edge to log/highlight the event.
    uint32_t waitShiftElapsedMs = 0;
    uint32_t gateDwellMs        = 0;
    bool     waitShiftTimedOut  = false;

    // Stage 3 -- Lateral CoM observation telemetry.
    //   comLateralMm:     signed neutral-corrected displacement.
    //                     Positive = body shifted to robot's right.
    //                     Computed every tick regardless of gate mode so
    //                     it's available for tuning the threshold against
    //                     measured behavior.
    //   gateComLateralOK: |comLateralMm * expectedSign| >= comLateralGateMm.
    float comLateralMm     = 0.0f;
    bool  gateComLateralOK = false;

    // W2 — lateral CoM trim actually applied to each foot's y_mm this tick
    // (signed magnitude; the per-leg sign flip happens in _buildFootTarget).
    // Zero when comLatTrimEnabled is false, IMU invalid, or estopped.
    // W5: comLatTrimMm now includes BOTH the displacement (Kp) and velocity
    // (Kd) contributions; the two fields below break it apart for tuning.
    float comLatTrimMm     = 0.0f;

    // W5 — velocity-damping telemetry. Both are diagnostic only; nothing in
    // the control stack reads them.
    //   comLatRollRateFilt  filtered torso roll rate (rad/s) feeding the Kd
    //                       term. Mirrors the live LPF state every tick, even
    //                       in IDLE, so the UI can show how quiet the signal is
    //                       before the operator raises Kd.
    //   comLatTrimVelMm     the velocity contribution alone (mm, pre-clamp):
    //                       H_COM_MM * rollRate_filt * comLatTrimKd. Zero when
    //                       the trim is inactive or comLatTrimKd is 0.
    float comLatRollRateFilt = 0.0f;
    float comLatTrimVelMm    = 0.0f;

    // Stage 1 IK telemetry. Updated each tick by _submitFootTargetIK().
    uint8_t      lastIKStatusL = 0;   // IKStatus enum value
    uint8_t      lastIKStatusR = 0;
    float        lastReachPctL = 0.0f;
    float        lastReachPctR = 0.0f;
    // Stage 1.1 — flat-foot diagnostic. Nonzero means the ankle pitch was
    // clamped to its soft limit; the sole is no longer flat against the ground.
    // Treat |value| > 2° as a tuning red flag — gait should pause/abort.
    float        lastFlatFootErrL_deg = 0.0f;
    float        lastFlatFootErrR_deg = 0.0f;
    // Stage 3 — Cumulative body-frame x displacement this cycle, in mm.
    // Computed as _halfPhase × stepLengthMm. Reset to 0 on _advanceHalf().
    // Telemetry only — does NOT feed back into FootTarget.
    float        cycleXDispl_mm = 0.0f;

    // Stage 2.2 — last computed sagittal capture offset (mm). Telemetry only.
    // Sign matches stride x: positive = foot placed further forward than
    // the open-loop nominal. At zero CoM velocity or with capture disabled,
    // this stays at 0. Updated whenever _buildFootTarget() runs on the swing
    // leg — value reflects the offset applied to that tick's swing foot.
    float captureOffsetMm = 0.0f;

    // Recovery telemetry (Stage 1+). Valid only when mode == RECOVERY_STEP.
    RecoverySubState recoverySub       = RecoverySubState::TRIGGERED;
    int8_t           recoveryDirX      = 0;   // sagittal fall direction: -1/0/+1
    int8_t           recoveryDirY      = 0;   // lateral  fall direction: -1/0/+1
    uint32_t         recoveryElapsedMs = 0;   // ms since current sub-state began

    // Stage 2 recovery telemetry. Valid only when mode == RECOVERY_STEP.
    float    rcvStepX_mm   = 0.0f;   // sagittal step magnitude at trigger (signed)
    float    rcvStepY_mm   = 0.0f;   // lateral  step magnitude at trigger (signed)
    float    rcvMagnitude  = 0.0f;   // sqrt(stepX² + stepY²) at trigger
    // Stage 2 stance-leg counter-shift telemetry (Issue 2 visibility).
    // Set per-tick by _runRecoveryStep() so the UI can confirm stance abduction
    // is happening even before it becomes visually obvious.
    float    rcvStanceY_mm = 0.0f;

    // ------------------------------------------------------------------
    // Touchdown-awareness telemetry (Stage 0).
    //
    // All fields refer to the CURRENT swing half-cycle. They reset on
    // _advanceHalf(), start(), and the IDLE branch of start(IDLE) so the
    // UI never sees stale values across role swaps. Telemetry-only —
    // nothing in the control stack reads these fields.
    //
    //   touchdownSeen      sticky: true from first gated detection until
    //                      the half-cycle ends.
    //   touchdownPhase     _halfPhase value at first gated detection.
    //   touchdownSwingX_mm swing leg's commanded stride-x contribution
    //                      (mm) at first gated detection.
    //   touchdownSide      0 = LEFT swing, 1 = RIGHT swing (matches Side enum).
    //   touchdownLocked    Stage 1 latch state. Mirrors the private
    //                      _touchdownLocked member so the UI can show
    //                      whether the foot is being held in place this tick.
    // ------------------------------------------------------------------
    bool    touchdownSeen      = false;
    float   touchdownPhase     = 0.0f;
    float   touchdownSwingX_mm = 0.0f;
    uint8_t touchdownSide      = 0;
    bool    touchdownLocked    = false;

    // ------------------------------------------------------------------
    // W4 — Touchdown-compliance telemetry. Telemetry-only — the control
    // stack reads the private _complianceOffsetMm member, not these.
    //
    //   complianceActive    true while a compliance event is decaying
    //                        (offset non-zero this tick).
    //   complianceOffsetMm  signed stance-height offset applied this tick
    //                        (<= 0). Added to BOTH legs' h_sagittal_mm in
    //                        _buildFootTarget. 0 when no event is active.
    // ------------------------------------------------------------------
    bool    complianceActive   = false;
    float   complianceOffsetMm = 0.0f;

    // ------------------------------------------------------------------
    // Touchdown trim-suppression telemetry. Telemetry-only — the control
    // stack reads the private _trimSuppressActive member, not this.
    //
    //   trimSuppressActive   true on any tick the W2/W5 lateral trim is
    //                        being suppressed (latch held or inside the
    //                        post-latch hold window). Recomputed every
    //                        tick by _updateTrimSuppression().
    // ------------------------------------------------------------------
    bool    trimSuppressActive = false;

};

// ---------------------------------------------------------------------------
//  GaitController
// ---------------------------------------------------------------------------
class GaitController {
public:
    GaitController() = default;

    // Wire dependencies. Call in setup() AFTER weightShift.init().
    // ws may be nullptr — CoM shift disabled; foot lift still works.
    void init(MotionManager* mm, WeightShift* ws = nullptr, BalanceController* bal = nullptr);

    void start(GaitMode mode);  // start a mode; start(IDLE) is equivalent to stop()
    void stop();                 // return to IDLE; centers WeightShift
    void singleStep();           // start SINGLE_STEP mode (one full cycle, then IDLE)
    // Advance to the next pause point in POSE_STEP mode.
    // Ignored if mode != POSE_STEP or not currently paused.
    void nextPose();
    // Capture the current IMU pitch/roll as the torso neutrals.
    // Rejected unless mode == IDLE and imuState.valid -- taring while
    // moving or before IMU calibration would store transient garbage.
    // Returns true on success.
    bool tareToTorso(const IMUState& imuState);

    // Call every tick from main.cpp, after weightShift.update().
    // bc:       task-space balance correction (Stage 3). Pass nullptr for legacy mode.
    // balState: current or previous tick's BalanceState. Pass nullptr to disable
    //           catch-step detection. The recovery_flag field drives triggerRecovery().
    void update(float dt_s, const IMUState& imuState,
                const BalanceCorrection*   bc       = nullptr,
                const BalanceState*        balState = nullptr,
                const FootContactSnapshot* footSnap = nullptr);

    // Trigger a single catch-step recovery in the direction implied by the tilt.
    // No-op if mode == RECOVERY_STEP (no re-trigger mid-step) or estopped.
    // Called automatically from update() when balState->recovery_flag fires, or
    // manually from WebComm test commands.
    void triggerRecovery(float tilt_pitch_rad, float tilt_roll_rad);

    GaitConfig       getConfig() const          { return _cfg; }
    void             setConfig(const GaitConfig& cfg) { _cfg = cfg; }
    const GaitState& getState()  const          { return _state; }

    // NOTE: servo-neutral drift compensation is NOT a gait concern. It lives
    // in JointModel as a runtime session trim on the joint↔servo conversion,
    // so every command source (UI / gait / balance / recovery) inherits it
    // through one path. See JointModel::captureSessionNeutral().

    // Wire swing-foot leveler. Call once in setup() after leveler.init().
    // When wired, _submitFootTargetIK applies leveler bias on the swing leg.
    void setSwingFootLeveler(SwingFootLeveler* l) { _swingLeveler = l; }

    // Wire foot-impact detector. Optional — nullptr disables touchdown
    // latching entirely (Stage 0 telemetry will also stay quiet). The
    // detector remains owned by main.cpp; GaitController is a read-only
    // consumer.
    void setFootImpactDetector(FootImpactDetector* d) { _impact = d; }

private:
    MotionManager*     _mm           = nullptr;
    WeightShift*       _ws           = nullptr;
    BalanceController* _bal          = nullptr;
    SwingFootLeveler*  _swingLeveler = nullptr;
    FootImpactDetector* _impact      = nullptr;   // Stage 0/1 — observational

    GaitConfig   _cfg;
    GaitState    _state;

    // Sub-cycle tracking.
    // _swingSide:   which leg is currently the swing leg. Phase 5 refactor —
    //               replaces uint8_t _currentHalf where 0 meant LEFT-swings,
    //               1 meant RIGHT-swings (hardware-verified mapping that now
    //               lives in the type system, not in a comment).
    // _halfPhase:   [0, 1) within current half-cycle; advances only in SWINGING sub-state.
    // _subState:    mirrors _state.subState; kept local for clarity in update().
    Side         _swingSide   = Side::LEFT;
    float        _halfPhase   = 0.0f;
    StepSubState _subState    = StepSubState::WAIT_SHIFT;

    // Last direction sent to WeightShift (avoids redundant trigger calls).
    // ShiftDirection::NONE means no shift commanded. Phase 5 refactor —
    // replaces int8_t with the canonical wire-protocol enum.
    ShiftDirection _lastShiftDir = ShiftDirection::NONE;

    IMUState _lastImuState{};   // cached at top of update() for trajectory helpers

    // W5 — low-pass-filtered torso roll rate (rad/s). Single IIR state, advanced
    // once per tick by _updateRollRateFilter(); read (never written) by the
    // const trim helpers. Kept as a member because _comLateralTrimMm() is const
    // and runs several times per tick — the filter must step exactly once.
    float _rollRateFilt = 0.0f;

    // POSE_STEP pause state.
    // 0 = not paused. 1 = held at gate-open. 2 = held at peak. 3 = held at landing.
    uint8_t _poseHoldPt = 0;

    // millis() timestamp at which the current STABILIZING substate began.
    // Compared against GAIT_STABILIZE_MS to decide when to release into WAIT_SHIFT.
    uint32_t _stabilizeStartMs = 0;
    // Stage 2 -- dwell + timeout bookkeeping. Both are millis() timestamps.
    // _waitShiftStartMs: set on every entry to WAIT_SHIFT. Drives the
    //                    timeout comparison.
    // _gateOkSinceMs:    set when liftGateOK first goes true within a
    //                    WAIT_SHIFT episode. Reset to 0 if the gate
    //                    goes false again. Drives the dwell comparison.
    uint32_t _waitShiftStartMs = 0;
    uint32_t _gateOkSinceMs    = 0;
    uint32_t _stopStartMs      = 0;
    float    _stopSwingFrac    = 0.0f;
    Side     _stopSwingSide    = Side::LEFT;   // Phase 5 — was uint8_t _stopHalf

    // Stage 1: catch-step recovery private state.
    RecoverySubState _rcvSub          = RecoverySubState::TRIGGERED;
    int8_t           _rcvDirX         = 0;     // sagittal direction at trigger
    int8_t           _rcvDirY         = 0;     // lateral direction at trigger
    uint32_t         _rcvSubStartMs   = 0;     // millis() when _rcvSub last changed
    Side             _rcvSwingSide    = Side::LEFT; // which leg is the recovery swing leg
                                                     // Phase 5 refactor — was bool _rcvSwingIsRight

    // Stage 2: per-trigger step magnitudes and refractory lockout.
    float    _rcvStepX_mm       = 0.0f;  // sagittal step magnitude this recovery (signed)
    float    _rcvStepY_mm       = 0.0f;  // lateral step magnitude this recovery (signed)
    uint32_t _rcvLockoutUntilMs = 0;     // millis() until which re-trigger is blocked

    // -----------------------------------------------------------------------
    // Touchdown-awareness latch (Stage 1).
    //
    // The latch is per-half-cycle local state. When set, the swing leg's
    // stride x contribution and h_sagittal are frozen at the values
    // snapshotted at the moment of detection. Everything else (WeightShift
    // contribution, BalanceCorrection on the stance leg, swing-foot leveler,
    // capture offset on subsequent half-cycles) keeps composing normally.
    //
    // Freezing h prevents the gait from commanding foot penetration after
    // ground contact: continuing to drive h toward stanceHeightMm would push
    // the foot into the floor, creating a reaction force that shoves the body
    // backward — the exact failure mode this subsystem was built to prevent.
    //
    // The latch is reset in _advanceHalf() (next half), start() (any mode
    // entry), stop() implicitly (STOPPING ramps from current state, then
    // start(IDLE) clears), and on triggerRecovery() entry.
    //
    // Detector and gate-phase live in _cfg (touchdownEnabled, touchdownGatePhase).
    // Detector pointer lives in _impact (set via setFootImpactDetector()).
    // -----------------------------------------------------------------------
    bool     _touchdownLocked      = false;
    float    _touchdownLockedX_mm  = 0.0f;   // snapshot of stride x at lock
    float    _touchdownLockedH_mm  = 0.0f;   // snapshot of h_sagittal_mm at lock
    float    _touchdownLockedPhase = 0.0f;   // _halfPhase at lock

    // Clears the latch and the public telemetry mirror. Call at any half-
    // cycle boundary, mode change, or recovery entry to keep the per-swing
    // semantics tight.
    void _clearTouchdownLatch();

    // -----------------------------------------------------------------------
    // W4 — Touchdown-compliance event state.
    //
    // The applied offset is recomputed every tick from the time elapsed
    // since the triggering event, so only the start timestamp + active flag
    // are stored. Deliberately INDEPENDENT of the touchdown latch (separate
    // timer, not cleared by _clearTouchdownLatch()) so the offset keeps
    // decaying smoothly across the half-cycle boundary instead of snapping
    // to zero when the next swing begins.
    //
    //   _complianceStartMs   millis() at the triggering event.
    //   _complianceOffsetMm  cached offset for this tick (<= 0). Read by the
    //                        const _buildFootTarget(); recomputed by
    //                        _updateCompliance().
    //   _complianceActive    true while the decay is in progress.
    // -----------------------------------------------------------------------
    uint32_t _complianceStartMs  = 0;
    float    _complianceOffsetMm = 0.0f;
    bool     _complianceActive   = false;

    // Recompute the touchdown-compliance stance-height offset for this tick.
    // Smoothstep decay from -complianceDepthMm to 0 over complianceDecayMs;
    // self-clears once the decay window elapses. Updates _complianceOffsetMm
    // and the GaitState telemetry mirror. Call once per tick in update().
    void _updateCompliance();

    // Zero the compliance event state + telemetry mirror. Call on mode entry
    // (start), init, and recovery entry — NOT at half-cycle boundaries, where
    // the offset must be allowed to keep decaying.
    void _resetCompliance();

    // -----------------------------------------------------------------------
    // Touchdown trim suppression — phase-gate state for the W2/W5 trim.
    //
    // _comLateralTrimMm() is const and runs several times per tick, so the
    // suppression decision is made ONCE per tick by _updateTrimSuppression()
    // and cached here; the const trim helper only reads these members. This
    // mirrors the _rollRateFilt / _updateRollRateFilter() pattern.
    //
    //   _trimSuppressUntilMs  millis() deadline through which suppression
    //                         stays active after the touchdown latch clears.
    //                         Refreshed every tick the latch is held.
    //   _trimSuppressActive   true on any tick the trim is suppressed (latch
    //                         held OR now < _trimSuppressUntilMs). Read by
    //                         _comLateralTrimMm().
    //   _trimLastValidMm      last un-suppressed raw trim (mm). Held and
    //                         returned by _comLateralTrimMm() in freeze mode.
    // -----------------------------------------------------------------------
    uint32_t _trimSuppressUntilMs = 0;
    bool     _trimSuppressActive  = false;
    float    _trimLastValidMm     = 0.0f;

    // Recompute the touchdown trim-suppression decision for this tick and keep
    // the freeze snapshot fresh. Must run once per update(), before any trim
    // helper or _buildFootTarget() reads the result.
    void _updateTrimSuppression();

    // Zero the trim-suppression state + telemetry mirror. Call on mode entry
    // (start), init — NOT at half-cycle boundaries, where the post-latch hold
    // window must be allowed to keep running.
    void _resetTrimSuppression();

    void _enterRcvSub(RecoverySubState s);
    // imuState required by COMPLETE case to decide whether to chain a second step.
    void _runRecoveryStep(float dt_s, const IMUState& imuState);

    // Publish current swing-leg state to WeightShift each tick so WS can
    // scale the swing side's ankle_roll bias internally. The call site lives
    // inside update() so the publication is always paired with state
    // advancement.
    void _publishSwingLegMaskToWS() const;

    // Trigger WeightShift direction matching the given swing side.
    // Phase 5 refactor: was _triggerShiftForHalf(uint8_t half). The body of
    // this function is the single place in the codebase where the wire-protocol
    // swing-side → ShiftDirection mapping is performed.
    void  _triggerShiftForSwing(Side swing);

    // Complete current half-cycle: advance to next half, re-gate, or stop.
    void  _advanceHalf();

    // sin(localPhase × π) → swing fraction [0, 1] from grounded to peak and back.
    float _swingFrac(float localPhase) const;

    // -----------------------------------------------------------------------
    //  _captureOffsetMm — sagittal capture-point step-placement correction.
    //
    //  Returns the mm offset to add to the swing leg's stride. Zero when
    //  capture is disabled, IMU invalid, or estopped. Sign convention:
    //    vx_CoM > 0 (body advancing forward) → positive offset
    //                                          (foot lands further forward)
    //
    //  Capping logic (composite, conservative):
    //    raw      = vx_CoM × INV_OMEGA0 × captureGain
    //    by_field = clamp(raw, ±captureOffsetMaxMm)
    //    by_step  = clamp(by_field, ±stepLengthMm)
    //
    //  by_step is the second cap: the capture offset alone never exceeds the
    //  open-loop nominal stride. Without this, a noisy IMU spike could double
    //  the stride in one tick.
    //
    //  IK reach is checked separately at submit time via the existing
    //  IK status path — this function is intentionally pure-math.
    // -----------------------------------------------------------------------
    float _captureOffsetMm() const;

    // -----------------------------------------------------------------------
    //  _comLateralTrimMm — W2+W5 continuous lateral CoM foot-placement trim.
    //
    //  Returns the signed mm trim to add to each foot's y_mm. Sign convention:
    //    CoM displaced to robot-right (comLateral > 0) → positive return →
    //    right foot abducts (+y), left foot adducts (-y) in _buildFootTarget
    //    → body translates right over the feet (feet track under the CoM).
    //
    //  W5: the return value is now displacement (Kp) PLUS velocity (Kd):
    //    trim = comLatDisp * comLatTrimGain + comLatVel * comLatTrimKd
    //  with comLatVel derived from the filtered roll rate (_rollRateFilt).
    //  When comLatTrimKd == 0 the velocity term vanishes and the result is
    //  bit-identical to the displacement-only W2 trim.
    //
    //  Returns 0 when comLatTrimEnabled is false, the controller is IDLE, the
    //  IMU is invalid, or estopped. The IDLE gate keeps the trim from layering
    //  a static lateral offset over the idle pose; the STOPPING ramp is faded
    //  separately by strideScale in _buildFootTarget(). Computed from
    //  _lastImuState / _rollRateFilt (not _state.*), so it is consistent on
    //  every _buildFootTarget path. Intentionally pure-math.
    //
    //  Touchdown trim suppression: when _trimSuppressActive is set this tick,
    //  this returns either the frozen last-clean trim or 0 (per the freeze
    //  config flag) instead of the live computation — see _comLateralTrimRawMm
    //  and _updateTrimSuppression().
    // -----------------------------------------------------------------------
    float _comLateralTrimMm() const;

    // W2/W5 — the raw lateral trim computation (displacement + velocity, then
    // clamped), with NO touchdown suppression applied. _comLateralTrimMm()
    // wraps this with the suppression gate; _updateTrimSuppression() reads it
    // to keep the freeze snapshot fresh. Intentionally pure-math.
    float _comLateralTrimRawMm() const;

    // W5 — true when the lateral CoM trim is live this tick (enabled, not IDLE,
    // IMU valid, not estopped). Single predicate shared by _comLateralTrimMm()
    // and _comLatVelTrimMm() so their gating can never drift apart.
    bool  _comLatTrimActive() const;

    // W5 — the velocity (Kd) contribution to the trim, in mm, BEFORE the shared
    // ±comLatTrimMaxMm clamp. Telemetry only: lets the UI show how much of the
    // applied trim is damping vs. displacement. Returns 0 when the trim is
    // inactive or comLatTrimKd is 0.
    float _comLatVelTrimMm() const;

    // W5 — advance the roll-rate low-pass filter by one tick. Must be called
    // exactly once per update(), before any trim helper runs. Zeroes the
    // filter state when the IMU is invalid so a stale rate cannot survive an
    // IMU dropout and re-enter the Kd term on the next valid tick.
    void  _updateRollRateFilter(const IMUState& imuState);

    // Build the FootTarget for one leg this tick.
    //   Stance leg: y_mm=0, x_mm=0, h_sagittal_mm = stanceHeightMm.
    //   Swing  leg: x_mm=0, y_mm=0, h_sagittal_mm = stanceHeightMm − liftMm.
    //                where liftMm = swingFrac × GAIT_STEP_HEIGHT_MM (Stage 1: vertical only).
    // If bc != nullptr and isSwing=false, mergeBalanceCorrection is applied to the
    // stance target before returning. Swing leg never receives the correction.
    // Returns target with valid=true on success. valid=false means the
    // caller should not submit (used for IDLE / paused states).
    // localPhase: position within the current half-cycle [0, 1).
    //   Stance leg ignores it. Swing leg uses it for linear stride.
    //   Pass 0.0f from contexts that don't have a meaningful phase
    //   (stop ramp, idle stance refresh).
    FootTarget _buildFootTarget(bool isRight, float swingFrac, bool isSwing,
                                float localPhase = 0.0f,
                                const BalanceCorrection* bc = nullptr,
                                float strideScale = 1.0f) const;

    // Resolve a FootTarget through LegIK and submit the joint angles via
    // SOURCE_GAIT. Logs (rate-limited) and skips submission on
    // IKStatus::DOMAIN_ERROR. Returns the LegIKResult so callers can update
    // telemetry without re-solving.
    // Phase 4: Side-typed canonical form. Channel selection routes through
    // channelFor(side, LegJoint::*) instead of the legacy isRight ternaries.
    LegIKResult _submitFootTargetIK(Side side, const FootTarget& t);
    // Phase 4 backward-compat shim — keep bool-typed call sites compiling
    // until they migrate. Removed in Phase 7.
    LegIKResult _submitFootTargetIK(bool isRight, const FootTarget& t) {
        return _submitFootTargetIK(sideFromIsRight(isRight), t);
    }

    // Re-assert both legs' stance baseline for this tick.
    // Used whenever gait or WeightShift owns the legs, even if no swing is active.
    // bc: merged into each stance leg's FootTarget when not nullptr.
    void _submitStanceBothLegs(const BalanceCorrection* bc = nullptr);

    // Stage 3: Submit both legs using their gait-cycle role (swing / stance) at
    // the given localPhase. Applies stride formula to BOTH legs even when neither
    // is actively lifting, so legs hold accumulated body displacement through
    // WAIT_SHIFT and STABILIZING. Used in active gait modes only — IDLE and
    // STOPPING continue to use _submitStanceBothLegs (no stride).
    void _submitCycleStanceBoth(float localPhase, const BalanceCorrection* bc = nullptr);
};

#endif // GAIT_CONTROLLER_H
