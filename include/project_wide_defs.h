#ifndef PROJECT_WIDE_DEFS_H
#define PROJECT_WIDE_DEFS_H

#include <Arduino.h>
#include "state_estimator.h"

// =============================================================================
//  PROJECT-WIDE DEFINITIONS
// =============================================================================
//
//  NUM_SERVOS, USMIN, USMAX, ANGLE_RANGE, SERVO_FREQ are defined in joint_map.h.
//  This file re-exports them via the include below for backward compatibility.
// =============================================================================

// --- Hardware pins ---
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9

// --- WiFi ---
#define AP_SSID "HumanoidRobot"
#define AP_PASS "robot1234"

// --- Single source of truth for joint metadata, pulse math, and movement API ---
// --- All existing references to JOINT_MAP[i].neutralAngle continue to work  ---

#endif // PROJECT_WIDE_DEFS_H