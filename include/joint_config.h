#ifndef JOINT_CONFIG_H
#define JOINT_CONFIG_H

#include <Arduino.h>

// =============================================================================
//  JOINT CONFIGURATION  —  THE SINGLE FILE YOU EDIT FOR CALIBRATION
// =============================================================================
//
//  This file is the only place in the codebase that contains per-joint
//  hardware values. Edit here to change neutral poses, inversion, or limits.
//  No other file needs to change for a calibration update.
//
//  HOW THE ANGLE SYSTEM WORKS
//  ───────────────────────────
//  Every joint has two angle frames:
//
//    "Joint angle"  — degrees FROM neutral, positive = anatomical flexion.
//                     This is what motion code uses. Range: [minAngle, maxAngle].
//
//    "Servo angle"  — absolute physical servo position, 0°–270°.
//                     This is what the hardware needs.
//
//  Conversion formula (applied automatically by JointModel):
//
//    servoAngle = neutralDeg + direction * jointAngle
//
//  Example: neutralDeg=130, direction=+1, jointAngle=+30
//    → servoAngle = 130 + (1 × 30) = 160°   ← servo moves up
//
//  Example: neutralDeg=130, direction=−1, jointAngle=+30
//    → servoAngle = 130 + (−1 × 30) = 100°  ← servo moves down (mirrored)
//
//  DIRECTION FIELD
//  ───────────────
//  direction = +1  → increasing servo pulse = anatomical flexion
//  direction = −1  → decreasing servo pulse = anatomical flexion
//                    (use for mirrored joints on the left side, or joints
//                     where the servo is mounted facing the opposite way)
//
//  LIMIT FIELDS
//  ────────────
//  minAngle : most negative joint angle allowed. Must be ≤ 0.
//             Usually 0 for joints that only flex one way (e.g. knees).
//  maxAngle : most positive joint angle allowed. Must be ≥ 0.
//
//  Start with conservative values (±20–30°) and widen after hardware testing.
//  Hard-stop contact damages servos — be careful with knees especially.
//
//  TUNING WORKFLOW
//  ───────────────
//  1. Power on. Robot stands at neutralDeg values.
//  2. Verify standing pose looks correct. Adjust neutralDeg if any joint is off.
//  3. Manually move each joint to its physical end stops. Measure degrees.
//  4. Set limits to ~5° inside those physical stops.
//  5. Mark calibrated joints with "// OK" instead of "// TUNE".
// =============================================================================

// Total number of PCA9685 channels managed (including unused stubs).
// Must match the length of JOINT_CONFIG[] below.
#define NUM_JOINTS  16

struct JointConfig {
    uint8_t     channel;     // PCA9685 channel 0–15
    const char* name;        // Snake_case label — used in debug output and UI
    float       neutralDeg;  // Absolute servo angle (°) at neutral standing pose. TUNE.
    int         direction;   // +1 or −1. See direction convention above.
    float       minAngle;    // Min joint angle (° from neutral, ≤ 0). TUNE.
    float       maxAngle;    // Max joint angle (° from neutral, ≥ 0). TUNE.
};

// =============================================================================
//  CONFIGURATION TABLE  —  one row per channel
//
//  Column order:  channel | name | neutralDeg | direction | minAngle | maxAngle
// =============================================================================

// Declared as extern so it lives in joint_config.cpp — avoids multiple-definition
// errors when this header is included by multiple translation units.
extern const JointConfig JOINT_CONFIG[NUM_JOINTS];

// Channel index aliases — use these instead of raw channel numbers in motion code.
// Example: setJointAngle(IDX_R_KNEE_PITCH, 30.0f)
#define IDX_R_ANKLE_ROLL   0
#define IDX_R_ANKLE_PITCH  1
#define IDX_R_KNEE_PITCH   2
#define IDX_R_HIP_ROLL     3
#define IDX_R_HIP_YAW      4
#define IDX_R_HIP_PITCH    5
// channels 8,9,10,11: unused stubs
#define IDX_L_HIP_PITCH   6
#define IDX_L_HIP_YAW     7
#define IDX_L_HIP_ROLL    12
#define IDX_L_KNEE_PITCH  13
#define IDX_L_ANKLE_PITCH 14
#define IDX_L_ANKLE_ROLL  15

#endif // JOINT_CONFIG_H