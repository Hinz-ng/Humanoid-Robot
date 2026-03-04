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

    // Angle-based API (existing, unchanged)
    void  setTargetAngle(uint8_t channel, float angle, bool immediate = false);
    void  setGaitOffset(uint8_t channel, float offsetDegrees);
    float getTargetAngle(uint8_t channel);

    // Pulse-based API
    void setTargetPulse(uint8_t channel, int pulse_us, bool immediate = false);

    // --- FIX: magnitude changed from int to float so fractional degrees work ---
    // Semantic named movement: "flex" or "extend" scaled by float magnitude.
    // Internally calls pulse_for_named_movement() from joint_map.h.
    void applyNamedMovement(uint8_t channel, const char* movement_name,
                            float magnitude, bool immediate = false);  // CHANGED: int → float

    int getTargetPulse(uint8_t channel);

    // Pose management
    void   saveCurrentPose(String name);
    void   loadPose(String name);
    String listPoses();

private:
    Adafruit_PWMServoDriver pwm;

    float    currentPulse[NUM_SERVOS];
    uint16_t targetPulse[NUM_SERVOS];
    float    targetAngles[NUM_SERVOS];
    float    activeOffsets[NUM_SERVOS];

    float         stepSize;
    float         deadband;
    unsigned long lastUpdate;
};

#endif // SERVO_CONTROL_H