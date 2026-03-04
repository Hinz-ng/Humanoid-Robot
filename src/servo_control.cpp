#include "servo_control.h"
#include <Wire.h>

ServoControl::ServoControl() : pwm(Adafruit_PWMServoDriver()) {
    stepSize   = 10.0f;
    deadband   =  4.0f;
    lastUpdate = 0;
}

void ServoControl::init() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);

    for (int i = 0; i < NUM_SERVOS; i++) {
        activeOffsets[i] = 0.0f;
        setTargetAngle(i, JOINT_MAP[i].neutralAngle, true);
    }
}

// =============================================================================
//  ANGLE-BASED API
// =============================================================================

float ServoControl::getTargetAngle(uint8_t channel) {
    if (channel >= NUM_SERVOS) return 0.0f;
    return targetAngles[channel];
}

void ServoControl::setTargetAngle(uint8_t channel, float angle, bool immediate) {
    if (channel >= NUM_SERVOS) return;

    float clamped = angle;
    if (clamped < 0.0f)        clamped = 0.0f;
    if (clamped > ANGLE_RANGE) clamped = ANGLE_RANGE;
    targetAngles[channel] = clamped;

    uint16_t p = (uint16_t)deg_to_pulse(clamped);
    targetPulse[channel]  = p;

    if (immediate) {
        currentPulse[channel] = (float)p;
        pwm.writeMicroseconds(channel, p);
    }
}

void ServoControl::setGaitOffset(uint8_t channel, float offsetDegrees) {
    if (channel >= NUM_SERVOS) return;
    activeOffsets[channel] = offsetDegrees;
    setTargetAngle(channel, JOINT_MAP[channel].neutralAngle + offsetDegrees);
}

// =============================================================================
//  PULSE-BASED API
// =============================================================================

void ServoControl::setTargetPulse(uint8_t channel, int pulse_us, bool immediate) {
    if (channel >= NUM_SERVOS) return;
    int p = clamp_pulse(pulse_us);
    targetPulse[channel]  = (uint16_t)p;
    targetAngles[channel] = pulse_to_deg(p);  // keep degree SSOT in sync

    // --- ADD START: debug log ---
    Serial.printf("[ServoCtrl] setTargetPulse ch=%d  pulse=%d  (%.1f°)  immediate=%d\n",
                  channel, p, targetAngles[channel], immediate);
    // --- ADD END ---

    if (immediate) {
        currentPulse[channel] = (float)p;
        pwm.writeMicroseconds(channel, (uint16_t)p);
    }
}

// --- FIX: magnitude is now float (was int) ---
void ServoControl::applyNamedMovement(uint8_t channel, const char* movement_name,
                                      float magnitude, bool immediate) {
    if (channel >= NUM_SERVOS) return;
    // --- ADD START: debug log before computing pulse ---
    Serial.printf("[ServoCtrl] applyNamedMovement ch=%d  move=%s  magnitude=%.2f\n",
                  channel, movement_name, magnitude);
    // --- ADD END ---
    int p = pulse_for_named_movement(channel, movement_name, magnitude);
    setTargetPulse(channel, p, immediate);
}

int ServoControl::getTargetPulse(uint8_t channel) {
    if (channel >= NUM_SERVOS) return USMIN;
    return (int)targetPulse[channel];
}

// =============================================================================
//  HARDWARE UPDATE LOOP
// =============================================================================

void ServoControl::update() {
    if (millis() - lastUpdate < 20) return;
    lastUpdate = millis();

    for (int i = 0; i < NUM_SERVOS; i++) {
        float diff = (float)targetPulse[i] - currentPulse[i];
        if (fabsf(diff) > deadband) {
            currentPulse[i] += (diff > 0) ? stepSize : -stepSize;
            pwm.writeMicroseconds(i, (uint16_t)currentPulse[i]);
        }
    }
}

// =============================================================================
//  POSE MANAGEMENT
// =============================================================================

void ServoControl::saveCurrentPose(String name) {
    String filename = "/pose_" + name + ".txt";
    File file = LittleFS.open(filename, FILE_WRITE);
    if (!file) { Serial.println("Failed to open file for saving"); return; }
    for (int i = 0; i < NUM_SERVOS; i++) file.println(targetAngles[i]);
    file.close();
}

void ServoControl::loadPose(String name) {
    String filename = "/pose_" + name + ".txt";
    File file = LittleFS.open(filename, FILE_READ);
    if (!file) { Serial.println("Pose not found!"); return; }
    for (int i = 0; i < NUM_SERVOS; i++) {
        if (!file.available()) break;
        setTargetAngle(i, file.readStringUntil('\n').toFloat(), false);
    }
    file.close();
}

String ServoControl::listPoses() {
    String list = "POSES:";
    File root = LittleFS.open("/");
    File file = root.openNextFile();
    while (file) {
        String fname = file.name();
        if (fname.startsWith("pose_"))
            list += fname.substring(5, fname.length() - 4) + ",";
        file = root.openNextFile();
    }
    return list;
}