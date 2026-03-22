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
//    servoAngle = neutralDeg + direction × jointAngle
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
    // Clamped to [JOINT_SPEED_MIN_DEG_S, JOINT_SPEED_MAX_DEG_S] from project_wide_defs.h.
    void  setJointSpeed(uint8_t jointId, float speedDegPerSec);

    // Set the same speed for every joint at once. Useful for UI "apply all".
    void  setAllJointsSpeed(float speedDegPerSec);

    // Query current configured speed for one joint.
    float getJointSpeed(uint8_t jointId) const;

    // deadband: µs gap below which no hardware write occurs (default 4).
    // Prevents constant tiny PWM jitter at rest.
    // stepSize is DEPRECATED — use setJointSpeed() instead. Left in place so
    // any legacy call sites still compile; it is no longer read by update().
    float deadband;

private:
    ServoDriver* _driver;  // Pointer to Layer 1 — set by init(), never owned

   // Per-channel state — both in µs for consistent math throughout.
    float    _currentPulse[NUM_JOINTS];     // Actual hardware position (smoothed)
    uint16_t _targetPulse[NUM_JOINTS];      // Desired position (set by set*() calls)

    // Per-joint movement speed for the smooth-stepping loop (deg/s).
    // Converted to µs/tick inside update() using actual dt.
    // Defaults to JOINT_SPEED_DEFAULT_DEG_S (project_wide_defs.h) at construction.
    float _speedDegPerSec[NUM_JOINTS];

    unsigned long _lastUpdateMs;  // For rate limiting in update()

    // -------------------------------------------------------------------------
    //  Private helpers  (all static — they depend only on JOINT_CONFIG data)
    // -------------------------------------------------------------------------

    // Clamp angleDeg to the joint's configured [minAngle, maxAngle].
    static float clampToLimits(uint8_t jointId, float angleDeg);

    // Full pipeline: joint angle → safe servo angle → pulse.
    static int   jointAngleToPulse(uint8_t jointId, float angleDeg);

    // Inverse: absolute servo degrees → joint-relative degrees.
    static float absoluteDegToJointAngle(uint8_t jointId, float absoluteDeg);
};

#endif // JOINT_MODEL_H