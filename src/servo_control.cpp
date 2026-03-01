#include "servo_control.h"
#include <Wire.h>

ServoControl::ServoControl() : pwm(Adafruit_PWMServoDriver()) {
    stepSize = 10.0f; 
    deadband = 4.0f;  
    lastUpdate = 0;
}

void ServoControl::init() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    pwm.begin();
    pwm.setOscillatorFrequency(27000000); 
    pwm.setPWMFreq(SERVO_FREQ);

    for(int i=0; i<NUM_SERVOS; i++) {
        activeOffsets[i] = 0.0f;
        setTargetAngle(i, JOINT_MAP[i].neutralAngle, true); 
    }
}

float ServoControl::getTargetAngle(uint8_t channel) {
    if (channel >= NUM_SERVOS) return 0.0f;
    return targetAngles[channel];
}

uint16_t ServoControl::angleToPulse(uint8_t channel, float angle) {
    float a = constrain(angle, 0.0f, ANGLE_RANGE);
    return (uint16_t)((a / ANGLE_RANGE) * (USMAX - USMIN) + USMIN);
}

void ServoControl::setTargetAngle(uint8_t channel, float angle, bool immediate) {
    if (channel >= NUM_SERVOS) return;
    
    // UPDATE SSOT
    targetAngles[channel] = constrain(angle, 0.0f, ANGLE_RANGE);
    
    uint16_t p = angleToPulse(channel, targetAngles[channel]);
    targetPulse[channel] = p;
    
    if (immediate) {
        currentPulse[channel] = (float)p;
        pwm.writeMicroseconds(channel, p);
    }
}

void ServoControl::setGaitOffset(uint8_t channel, float offsetDegrees) {
    if (channel >= NUM_SERVOS) return;
    activeOffsets[channel] = offsetDegrees;
    float finalAngle = JOINT_MAP[channel].neutralAngle + offsetDegrees;
    setTargetAngle(channel, finalAngle);
}

void ServoControl::update() {
    if (millis() - lastUpdate < 20) return;
    lastUpdate = millis();

    for(int i=0; i<NUM_SERVOS; i++) {
        float diff = (float)targetPulse[i] - currentPulse[i];
        if (abs(diff) > deadband) {
            if (currentPulse[i] < targetPulse[i]) {
                currentPulse[i] += stepSize;
            } else {
                currentPulse[i] -= stepSize;
            }
            pwm.writeMicroseconds(i, (uint16_t)currentPulse[i]);
        }
    }
}

void ServoControl::saveCurrentPose(String name) {
    String filename = "/pose_" + name + ".txt";
    File file = LittleFS.open(filename, FILE_WRITE);
    if (!file) { Serial.println("Failed to open file for saving"); return; }
    for (int i = 0; i < NUM_SERVOS; i++) {
        file.println(targetAngles[i]);
    }
    file.close();
}

void ServoControl::loadPose(String name) {
    String filename = "/pose_" + name + ".txt";
    File file = LittleFS.open(filename, FILE_READ);
    if (!file) { Serial.println("Pose not found!"); return; }
    for (int i=0; i<NUM_SERVOS; i++) {
        if (!file.available()) break;
        String line = file.readStringUntil('\n');
        float angle = line.toFloat();
        setTargetAngle(i, angle, false); 
    }
    file.close();
}

String ServoControl::listPoses() {
    String list = "POSES:";
    File root = LittleFS.open("/");
    File file = root.openNextFile();
    while(file){
        String fname = file.name();
        if (fname.startsWith("pose_")) {
            list += fname.substring(5, fname.length() - 4) + ","; 
        }
        file = root.openNextFile();
    }
    return list;
}