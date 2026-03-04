#ifndef PROJECT_WIDE_DEFS_H
#define PROJECT_WIDE_DEFS_H

#include <Arduino.h>

// =============================================================================
//  PROJECT-WIDE DEFINITIONS
// =============================================================================
//  NOTE: JointConfig and JOINT_MAP have been migrated to joint_map.h/cpp.
//  All servo metadata (neutral pulses, movement deltas, channel names) lives
//  exclusively in joint_map.h. Include that file for joint data.
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
#include "joint_map.h"

#endif // PROJECT_WIDE_DEFS_H