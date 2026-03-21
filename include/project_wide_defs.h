#ifndef PROJECT_WIDE_DEFS_H
#define PROJECT_WIDE_DEFS_H

#include <Arduino.h>
#include "state_estimator.h"

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
#define JOINT_SPEED_DEFAULT_DEG_S   60.0f   // All joints at boot
#define JOINT_SPEED_MIN_DEG_S        1.0f   // Prevents zero-speed (frozen joint) bugs
#define JOINT_SPEED_MAX_DEG_S      360.0f   // One full servo sweep per second (physical ceiling)

// --- Single source of truth for joint metadata is joint_config.h ---

#endif // PROJECT_WIDE_DEFS_H
