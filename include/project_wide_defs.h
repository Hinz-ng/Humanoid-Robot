#ifndef PROJECT_WIDE_DEFS_H
#define PROJECT_WIDE_DEFS_H

#include <Arduino.h>

#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
#define NUM_SERVOS 16
#define SERVO_FREQ 50
#define USMIN 500  
#define USMAX 2500 
#define ANGLE_RANGE 270.0f

#define AP_SSID "HumanoidRobot"
#define AP_PASS "robot1234"

// Centralized metadata structure. 
// 'direction' is retained strictly for documentation/reference. 
// It is NO LONGER used in backend math.
struct JointConfig {
    const char* name;
    int direction;      
    float neutralAngle; 
};

// --- AUTHORITATIVE NEUTRAL POSE & METADATA ---
static const JointConfig JOINT_MAP[NUM_SERVOS] = {
    {"Right Ankle Roll",   1, 131.0f}, // CH 0
    {"Right Ankle Pitch",  1, 125.0f}, // CH 1
    {"Right Knee Pitch",   1, 133.0f}, // CH 2
    {"Right Hip Roll",     1, 130.0f}, // CH 3
    {"Right Hip Yaw",      1, 130.0f}, // CH 4
    {"Right Hip Pitch",    1, 145.0f}, // CH 5
    {"Unused 6",           1, 135.0f}, {"Unused 7", 1, 135.0f},
    {"Unused 8",           1, 135.0f}, {"Unused 9", 1, 135.0f},
    {"Left Hip Pitch",     1, 145.0f}, // CH 10
    {"Left Hip Yaw",       1, 130.0f}, // CH 11
    {"Left Hip Roll",      1, 134.0f}, // CH 12
    {"Left Knee Pitch",    1, 128.0f}, // CH 13
    {"Left Ankle Pitch",   1, 125.0f}, // CH 14
    {"Left Ankle Roll",    1, 127.0f}  // CH 15
};
#endif