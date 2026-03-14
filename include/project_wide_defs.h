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

// --- Single source of truth for joint metadata is joint_config.h ---

#endif // PROJECT_WIDE_DEFS_H
