#ifndef JOINT_MODEL_H
#define JOINT_MODEL_H

#include <Arduino.h>
#include "joint_config.h"
#include "servo_driver.h"

// =============================================================================
//  JOINT MODEL  —  Layer 2: Joint Abstraction
// =============================================================================
//
//  This class represents the robot's joints as mechanical entities.
//  It is the bridge between motion code (which thinks in body angles)
//  and hardware (which only understands servo pulse widths).
//
//  WHAT THIS LAYER KNOWS ABOUT:
//    ✓ Each joint's neutral position
//    ✓ Each joint's direction (which way positive angles move the servo)
//    ✓ Each joint's safe range limits
//    ✓ How to convert between joint angles and servo pulses
//    ✓ Smooth stepping toward target positions
//
//  WHAT THIS LAYER DOES NOT KNOW ABOUT:
//    ✗ Which movement sequences to run (that's the Motion layer above)
//    ✗ I2C, PCA9685, or any hardware (that's ServoDriver below)
//    ✗ WebSocket, UI, or connectivity
//
//  TWO ANGLE COORDINATE SYSTEMS
//  ──────────────────────────────
//  Joint angle (motion-frame):
//    - Degrees FROM neutral. 0 = neutral standing pose.
//    - Positive = anatomical flexion direction.
//    - Clamped to [minAngle, maxAngle] from JOINT_CONFIG.
//    - This is what motion code should use.
//
//  Absolute servo angle (hardware-frame):
//    - Physical servo position, 0°–270°.
//    - This is what the UI sliders and pose files use.
//    - setAbsoluteAngle() / getAbsoluteAngle() are the bridge.
//
//  Conversion (from joint_config.h):
//    servoAngle = effectiveNeutral + direction × jointAngle
//
//  FACTORY NEUTRAL vs. RUNTIME SESSION TRIM
//  ─────────────────────────────────────────
//    factory neutral  : JOINT_CONFIG[ch].neutralDeg — the immutable
//                       compile-time calibration constant. Never written
//                       at runtime.
//    session trim     : _sessionTrimDeg[ch] — a per-joint runtime offset
//                       (servo degrees), zero at boot, that compensates
//                       servo-neutral drift between power cycles. Set by
//                       captureSessionNeutral(); not persisted.
//    effective neutral: factory neutral + session trim. This is what the
//                       conversion above actually uses.
//
//  The trim lives here because JointModel is the single joint↔servo
//  conversion boundary — applying it in jointAngleToPulse() and its inverse
//  means every command source (UI, gait, balance, recovery, IK) inherits
//  the correction through one code path, with no controller-local offsets.
//
//  SMOOTH STEPPING
//  ───────────────
//  All set*() calls update a _targetPulse. The update() method advances
//  _currentPulse toward _targetPulse by stepSize µs per cycle, writing to
//  hardware only when the difference exceeds the deadband. This prevents
//  abrupt servo jumps.
//
//  Tune stepSize and deadband via public fields.
// =============================================================================

class JointModel {
public:
    JointModel();

    // Link this model to a hardware driver. Must be called before any set* calls.
    void init(ServoDriver* driver);

    // =========================================================================
    //  PRIMARY API  —  use this in all new motion code
    // =========================================================================

    // Set a joint by body-relative angle (degrees FROM neutral).
    //
    //   jointId   : channel index (0–15). Use IDX_* constants from joint_map.h.
    //   angleDeg  : degrees from neutral. Positive = anatomical flexion.
    //               Automatically clamped to [minAngle, maxAngle].
    //   immediate : true  → write to hardware right now (use during init/reset)
    //               false → smooth stepping loop applies it gradually
    //
    // Example — flex right knee 30°:
    //   model.setJointAngle(IDX_R_KNEE_PITCH, 30.0f);
    //
    // Example — stand neutral:
    //   model.setJointAngle(IDX_R_KNEE_PITCH, 0.0f);
    //
    void setJointAngle(uint8_t jointId, float angleDeg, bool immediate = false);

    // Set all joints to their neutral pose (jointAngle = 0 for every joint).
    // immediate = true for startup/reset; false for smooth transition.
    void moveToNeutral(bool immediate = true);

    // =========================================================================
    //  COMPATIBILITY API  —  for code that uses absolute servo angles
    // =========================================================================
    //  The web UI sliders and saved poses use absolute servo angles (0°–270°).
    //  These methods let that code work unchanged while limits are still applied.

    // Set a joint by absolute servo angle (0°–270°).
    // Converts to joint-relative internally, so limits are still enforced.
    void setAbsoluteAngle(uint8_t jointId, float absoluteDeg, bool immediate = false);

    // =========================================================================
    //  GETTERS  —  for state broadcasts and pose save/load
    // =========================================================================

    // Current target servo angle in degrees (absolute, 0°–270°).
    // Returns the target (what we're aiming for), not the smoothed current.
    float getAbsoluteAngle(uint8_t jointId) const;

    // Current target joint-relative angle (degrees from neutral).
    float getJointAngle(uint8_t jointId) const;

    // Current target pulse width in µs.
    int   getServoPulse(uint8_t jointId) const;

    // =========================================================================
    //  RUNTIME SESSION NEUTRAL TRIM  —  servo-drift compensation
    // =========================================================================
    //  A per-joint runtime offset (servo degrees) added to the factory neutral
    //  at the joint↔servo conversion. Compensates servo-neutral drift between
    //  power cycles WITHOUT mutating the JOINT_CONFIG calibration constants.
    //  Every command source inherits it automatically. Session-scoped — zero
    //  at boot, not persisted across reboots.

    // Snapshot the current commanded pose as the new neutral: for each joint,
    // sets trim = (current commanded servo angle − factory neutral), so the
    // present physical pose becomes jointAngle 0. No servo motion — it is a
    // pure reference-frame redefinition. Rejected (returns false, trim
    // untouched) if any joint is further than NEUTRAL_TRIM_MAX_DEG from its
    // factory neutral — a guard against capturing a non-neutral pose.
    // Caller must ensure no controller is actively driving the joints.
    bool  captureSessionNeutral();

    // Zero every joint's session trim — revert to pure factory neutral.
    void  clearSessionNeutral();

    // Programmatic per-joint session trim setter (servo-degree offset from
    // factory neutral). Unlike captureSessionNeutral(), this writes one joint
    // at a time and does NOT snapshot the current pose. Used to encode a
    // posture parameter (e.g. stance-width inset) as the session neutral so
    // all command sources inherit the offset automatically.
    // No servo motion — only the joint-coordinate origin shifts.
    void  setSessionTrim(uint8_t jointId, float trimDeg);

    // Session-trim queries (telemetry / UI).
    float getSessionTrim(uint8_t jointId)     const;  // signed servo-deg, one joint
    float getEffectiveNeutralDeg(uint8_t jointId) const;  // factory neutral + trim
    bool  hasSessionTrim()    const;  // true if any joint's trim is non-zero
    float maxSessionTrimDeg() const;  // largest |trim| across all joints

    // =========================================================================
    //  TRANSMISSION GAIN  —  per-joint mechanical ratio
    // =========================================================================
    //  Ratio: (anatomical deviation) / (commanded servo deviation). For a 4-bar
    //  linkage, the bone rotates less than the servo by this factor — see the
    //  measurement log in joint_config.cpp.
    //
    //  The seam: jointAngleToPulse() divides the (already-clamped, anatomical)
    //  joint deviation by gain before computing the servo angle, so the bone
    //  ends up rotating by the requested anatomical amount.
    //    gain == 1.0       → bit-identical to pre-correction conversion
    //    deviation == 0    → no change (neutral pose unaffected)
    //    gain ≤ 0          → treated as 1.0 (defensive against bad config)
    //
    //  Runtime override: setTransmissionGain() lets the WebComm tune sweep the
    //  gain without reflashing. Defaults are sourced from JOINT_CONFIG at
    //  construction time. Not persisted — reboot restores the compile-time
    //  defaults (joint_config.cpp is the source of truth).
    void  setTransmissionGain(uint8_t jointId, float gain);
    float getTransmissionGain(uint8_t jointId) const;

    // The "amplified" (post-gain) anatomical-to-servo deviation for one joint,
    // in servo-side degrees from neutral. Used by telemetry to show what the
    // servo is actually being commanded vs what the bone should reach.
    //   correctedServoDeviationDeg(ch, anatomicalDeg) = anatomicalDeg / gain
    // (sign already encoded; multiply by JointConfig::direction to get the
    // signed offset that lands at the servo register.)
    float correctedServoDeviationDeg(uint8_t jointId, float anatomicalDeg) const;

    // Saturation flag: set to true when the corrected servo angle clipped the
    // 0–270° hardware envelope on the last conversion. Sticky until cleared.
    // Mirrors how the rest of the code surfaces silent saturations: visible,
    // broadcast, never silent. Public for read-only access by telemetry.
    bool  transmissionClampHit[NUM_JOINTS];

    // Clear all transmission-clamp flags (call after a tune knob change so
    // the operator sees fresh saturations, not stale ones from prior gains).
    void  clearTransmissionClamp();

    // =========================================================================
    //  SMOOTH STEPPING LOOP  —  call once per loop()
    // =========================================================================

    // Advances all servos one step toward their targets.
    // Self-limits to 50 Hz internally.
    void update();

    // =========================================================================
    //  PER-JOINT SPEED CONTROL
    // =========================================================================
    // Speed is in degrees per second and applies only to smooth-stepped moves.
    // immediate=true writes (balance controller) are never rate-limited here.
    //
    // Internals: deg/s is converted to µs/tick inside update() using actual
    // elapsed dt so speed tracking is accurate even if the update rate drifts.

    // Set movement speed for one joint (deg/s).
     // Floor: JOINT_SPEED_MIN_DEG_S (project_wide_defs.h).
    // Ceiling: JointConfig::noLoadSpeedDegS for this channel (joint_config.h).
    void  setJointSpeed(uint8_t jointId, float speedDegPerSec);

    // Set the same speed for every joint at once. Useful for UI "apply all".
    void  setAllJointsSpeed(float speedDegPerSec);

    // Query current configured speed for one joint.
    float getJointSpeed(uint8_t jointId) const;

    // =========================================================================
    //  POSE-TRANSITION SPEED  —  dedicated cap for Neutral/Stance moves
    // =========================================================================
    // A second speed used only for deliberate pose transitions, kept separate
    // from the per-joint gait/IK cap above. armTransition() latches a one-shot:
    // the NEXT smooth-stepped write per joint (any source) is flagged to slew at
    // the transition speed until it reaches target, then the flag self-clears.
    // disarmTransition() drops the latch (the per-joint flags persist to arrival).
    // Routed through ServoControl; MotionManager::flush() disarms after applying
    // each batch so the latch scopes to exactly one pose submission.
    void  armTransition();
    void  disarmTransition();
    void  setTransitionSpeed(float speedDegPerSec);  // clamped [MIN, DEFAULT]
    float getTransitionSpeed() const;

    // Per-channel deadband (µs). Write only occurs when |target - current| > deadband[ch].
    // Default 4 µs for all channels. Increase for channels with known oscillation
    // (e.g., torso pitch when running under balance control).
    // Tune: 4 µs ≈ 0.54° for 270° servo. 8 µs ≈ 1.08°.
    float deadband[NUM_JOINTS];

    // Set deadband for one channel (µs). Minimum 1 µs to prevent zero-deadband jitter.
    void setDeadband(uint8_t jointId, float deadbandUs);

private:
    ServoDriver* _driver;  // Pointer to Layer 1 — set by init(), never owned

   // Per-channel state — both in µs for consistent math throughout.
    float    _currentPulse[NUM_JOINTS];     // Actual hardware position (smoothed)
    uint16_t _targetPulse[NUM_JOINTS];      // Desired position (set by set*() calls)

    // Per-joint movement speed for the smooth-stepping loop (deg/s).
    // Converted to µs/tick inside update() using actual dt.
    // Defaults to JOINT_SPEED_DEFAULT_DEG_S (project_wide_defs.h) at construction.
    float _speedDegPerSec[NUM_JOINTS];

    // Pose-transition slew (deg/s) and per-joint engagement. _transitionArmed is
    // a one-shot latch consumed by setJointAngle() on smooth writes; a flagged
    // joint slews at _transitionSpeedDegPerSec in update() until it reaches target,
    // then its flag self-clears. See armTransition() above.
    float _transitionSpeedDegPerSec;
    bool  _useTransitionSpeed[NUM_JOINTS];
    bool  _transitionArmed;

    unsigned long _lastUpdateMs;  // For rate limiting in update()

    // Per-joint runtime session trim (servo degrees). effectiveNeutral =
    // JOINT_CONFIG[ch].neutralDeg + _sessionTrimDeg[ch]. Zero at boot; set
    // only by captureSessionNeutral() / cleared by clearSessionNeutral().
    // See the factory-vs-runtime block at the top of this file.
    float _sessionTrimDeg[NUM_JOINTS];

    // Per-joint runtime transmission gain. Initialised from JOINT_CONFIG at
    // construction; modifiable at runtime via setTransmissionGain() so the
    // WebComm tune knob can sweep without reflashing. Reboot restores the
    // compile-time defaults — joint_config.cpp is the persisted source.
    float _transmissionGain[NUM_JOINTS];

    // -------------------------------------------------------------------------
    //  Private helpers
    //
    //  clampToLimits is static — it depends only on JOINT_CONFIG limit fields.
    //  jointAngleToPulse / absoluteDegToJointAngle are instance methods: they
    //  read _sessionTrimDeg so the conversion uses the effective neutral.
    // -------------------------------------------------------------------------

    // Clamp angleDeg to the joint's configured [minAngle, maxAngle].
    static float clampToLimits(uint8_t jointId, float angleDeg);

    // Full pipeline: joint angle → safe servo angle → pulse.
    // Uses effectiveNeutral (factory neutral + session trim).
    int   jointAngleToPulse(uint8_t jointId, float angleDeg) const;

    // Inverse: absolute servo degrees → joint-relative degrees.
    // Uses effectiveNeutral — the exact inverse of jointAngleToPulse's step 1.
    float absoluteDegToJointAngle(uint8_t jointId, float absoluteDeg) const;
};

#endif // JOINT_MODEL_H