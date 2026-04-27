#ifndef PROJECT_WIDE_DEFS_H
#define PROJECT_WIDE_DEFS_H

#include <Arduino.h>

// =============================================================================
//  PROJECT-WIDE DEFINITIONS
// =============================================================================
//
//  Joint metadata and limits live in joint_config.h.
//  Servo hardware limits live in servo_driver.h.
// =============================================================================

// --- Hardware pins ---
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9

// --- WiFi ---
#define AP_SSID "HumanoidRobot"
#define AP_PASS "robot1234"

// --- Joint smooth-stepping speed limits (deg/s) ---
// Applied to all smooth-stepped (immediate=false) writes: UI sliders, pose loads, gait.
// Immediate writes (balance controller) bypass the smooth-stepper entirely.
//
// DEFAULT_DEG_S: comfortable default for UI interaction — not too sluggish, not snappy.
// Recalculate MAX_DEG_S if SD_ANGLE_RANGE changes (servo_driver.h):
//   max_us_per_tick = (MAX_DEG_S / SD_ANGLE_RANGE) * (SD_USMAX - SD_USMIN) * UPDATE_INTERVAL_S
#define JOINT_SPEED_DEFAULT_DEG_S   60.0f   // All joints at boot (~11–18% of no-load speed)
#define JOINT_SPEED_MIN_DEG_S        1.0f   // Floor — prevents silent frozen-joint bugs
// No global MAX: upper clamp is now per-joint (JointConfig::noLoadSpeedDegS).

// --- Single source of truth for joint metadata is joint_config.h ---

// No global MAX: upper clamp is now per-joint (JointConfig::noLoadSpeedDegS).

// --- Balance controller safe gain limits ---
// Hard ceiling enforced by WebComm when parsing BALANCE_TUNE commands.
// Derived from phase-margin analysis: at ~50ms servo lag and ω_n ≈ 6.3 rad/s,
// Kp above 20 deg/rad eliminates phase margin entirely. Kd above 1.0 amplifies
// gyro shot noise faster than it damps the pendulum.
// Recalculate if CoM height or servo model changes.
#define BALANCE_KP_MAX   30.0f   // deg/rad — do not raise without re-measuring servo lag
#define BALANCE_KD_MAX    1.0f   // deg/(rad/s)

// --- Weight shift ankle pitch forward tilt ---
// Joint-relative ankle pitch offset applied during weight shift (degrees).
// Positive = forward lean direction. Negate to reverse. Set to 0 to disable.
// Both ankles receive this value; JointModel direction fields produce symmetry.
#define ANKLE_PITCH_FORWARD_TILT_DEG  -4.0f
 
// ============================================================
// GAIT PARAMETERS — GaitController (weight_shift evolution)
// ============================================================
constexpr float GAIT_PHASE_RATE_HZ          = 0.4f;   // start slow; cycles/sec
constexpr float GAIT_STEP_HEIGHT_MM         = 20.0f;  // swing foot clearance
constexpr float GAIT_STANCE_WIDTH_MM        = 40.0f;  // lateral CoM amplitude
constexpr float GAIT_STEP_LENGTH_MM         = 30.0f;  // Phase 3+; forward stride
constexpr float GAIT_WEIGHT_SHIFT_THRESHOLD_RAD = 0.10f; // ~5.7° — min roll before lift

// Stance leg's nominal hip→ankle vertical drop in mm. Used by GaitController
// to construct FootTargets. Must stay inside LegIK's safe range (130–189 mm).
// Lower = deeper crouch = more knee flexion at neutral.
constexpr float GAIT_STANCE_HEIGHT_MM           = 185.0f;

#endif // PROJECT_WIDE_DEFS_H