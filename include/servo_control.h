#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Arduino.h>
#include "project_wide_defs.h"   // I2C pins, joint_map.h IDX_* defines
#include "joint_config.h"        // JOINT_CONFIG[], NUM_JOINTS
#include "servo_driver.h"        // Layer 1: hardware
#include "joint_model.h"         // Layer 2: joint abstraction
#include <LittleFS.h>

// =============================================================================
//  SERVO CONTROL  —  Layer 3: Motion Coordinator
// =============================================================================
//
//  This class is the top of the control stack. It:
//    • Wires the three layers together at startup (owns ServoDriver + JointModel)
//    • Exposes the motion API used by WebComm and gait modules
//    • Handles pose save/load from LittleFS
//    • Preserves the exact public API that WebComm and SquatGait depend on
//
//  LAYER OWNERSHIP
//  ───────────────
//  ServoControl  →  owns JointModel  →  owns (via pointer) ServoDriver
//                                                  ↓
//                                            PCA9685 hardware
//
//  ServoControl composes both lower layers internally.
//  Callers (WebComm, SquatGait) hold a ServoControl* — nothing changes for them.
//
//  WHAT CHANGED INTERNALLY (invisible to callers)
//  ───────────────────────────────────────────────
//  OLD: ServoControl owned Adafruit_PWMServoDriver directly, did all pulse math
//       inline, mixed joint mapping with hardware writes.
//
//  NEW: ServoControl delegates all hardware writes to ServoDriver (Layer 1)
//       and all joint math to JointModel (Layer 2). It only coordinates.
//
//  PUBLIC API IS IDENTICAL — existing callers compile without changes.
// =============================================================================

class ServoControl {
public:
    ServoControl();

    // -------------------------------------------------------------------------
    //  Startup — call once in setup()
    //  Initialises I2C, PCA9685, all joints to neutral.
    // -------------------------------------------------------------------------
    void init();

    // -------------------------------------------------------------------------
    //  Smooth stepping loop — call every loop()
    // -------------------------------------------------------------------------
    void update();

    // =========================================================================
    //  MOTION API — used by WebComm, SquatGait, and future motion modules
    // =========================================================================

    // Set a joint by absolute servo angle (0°–270°).
    // Used by the web UI sliders. Internally converts to joint-relative and
    // enforces limits before writing.
    void  setTargetAngle(uint8_t channel, float absoluteDeg, bool immediate = false);
    float getTargetAngle(uint8_t channel);

    // Immediate joint-relative write — bypasses smooth stepping.
    // Use ONLY from real-time control loops (balance controller).
    // angleDeg: degrees FROM neutral, same frame as setJointAngle().
    void setJointAngleDirect(uint8_t channel, float angleDeg);
    
    // Set a joint by raw pulse width (µs). Limits are still enforced.
    void setTargetPulse(uint8_t channel, int pulse_us, bool immediate = false);
    int  getTargetPulse(uint8_t channel);

    // Move a joint by a signed offset from neutral (absolute servo-frame degrees).
    // Used by gait modules: setGaitOffset(ch, 30) moves servo 30° above neutral.
    void setGaitOffset(uint8_t channel, float absoluteOffsetDeg);

    // =========================================================================
    //  SPEED CONTROL
    // =========================================================================
    // Sets movement speed in deg/s for smooth-stepped writes (immediate=false).
    // The balance controller uses immediate=true — speed settings have no effect
    // on real-time balance corrections.
    void  setJointSpeed(uint8_t channel, float speedDegPerSec);
    void  setAllJointsSpeed(float speedDegPerSec);
    float getJointSpeed(uint8_t channel) const;
    
    // Named semantic movement: "flex" or "extend" by magnitude degrees.
    // Internally calls JointModel::setJointAngle — magnitude maps 1:1 to degrees.
    void applyNamedMovement(uint8_t channel, const char* movementName,
                            float magnitude, bool immediate = false);

    // =========================================================================
    //  POSE MANAGEMENT
    // =========================================================================
    void   saveCurrentPose(String name);
    void   loadPose(String name);
    String listPoses();

    // Reset all joints to neutral and prime PCA9685 registers with safe values.
// Must be called before re-enabling OE after any e-stop or fall event.
// Uses immediate=true so _currentPulse, _targetPulse, and hardware registers
// are all synchronised before the caller sets OE LOW.
void resetToNeutral();

private:
    // The two owned layers — constructed in-place (not heap allocated).
    ServoDriver _driver;  // Layer 1: PCA9685 communication
    JointModel  _model;   // Layer 2: joint abstraction + smooth stepping
};

#endif // SERVO_CONTROL_H