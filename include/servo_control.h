#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "project_wide_defs.h"
#include <LittleFS.h>

class ServoControl {
public:
    ServoControl();
    void init();
    void update();
    void setTargetAngle(uint8_t channel, float angle, bool immediate = false);
    void setGaitOffset(uint8_t channel, float offsetDegrees);
    
    // SSOT State Access
    float getTargetAngle(uint8_t channel);

    // Pose Management
    void saveCurrentPose(String name);
    void loadPose(String name);
    String listPoses();

private:
    Adafruit_PWMServoDriver pwm;
    
    // Authoritative State Arrays
    float currentPulse[NUM_SERVOS];
    uint16_t targetPulse[NUM_SERVOS];
    float targetAngles[NUM_SERVOS]; // <-- NEW: SSOT Array
    float activeOffsets[NUM_SERVOS];
    
    float stepSize;
    float deadband;
    unsigned long lastUpdate;

    uint16_t angleToPulse(uint8_t channel, float angle);
};

#endif