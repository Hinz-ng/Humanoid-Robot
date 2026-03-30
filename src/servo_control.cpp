#include "servo_control.h"
#include <Wire.h>

// =============================================================================
//  SERVO CONTROL — Implementation
// =============================================================================
//
//  Every method here is a thin coordinator. The actual work happens in:
//    _driver  → servo_driver.cpp  (Layer 1: PCA9685 hardware)
//    _model   → joint_model.cpp   (Layer 2: joint math + smooth stepping)
//
//  If you find yourself adding joint math or hardware calls directly here,
//  that logic belongs in one of the layers below instead.
// =============================================================================

ServoControl::ServoControl() {
    // Layer objects are constructed in-place with their default constructors.
    // No explicit initialisation needed here.
}

// -----------------------------------------------------------------------------
//  init()  —  Wire up all layers and move robot to neutral standing pose.
// -----------------------------------------------------------------------------
void ServoControl::init() {
    // Step 1: Start I2C bus.
    //         Both PCA9685 boards and the BMI160 IMU share this bus.
    //         Wire.begin() must be called exactly once before any I2C device.
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Serial.printf("[ServoControl] I2C started on SDA=%d SCL=%d\n",
                  I2C_SDA_PIN, I2C_SCL_PIN);

    // Step 2: Initialise the hardware driver (PCA9685).
    _driver.init();

    // Step 3: Link the joint model to the hardware driver.
    _model.init(&_driver);
    
    // Torso pitch has balance controller writing at 400 Hz.
// Wider deadband suppresses oscillatory writes that produce visible jitter.
// 10 µs ≈ 1.35° for 270° servo — wide enough to damp, tight enough to track setpoint.
_model.setDeadband(IDX_TORSO_PITCH, 10.0f);
    // Step 4: Drive all joints to neutral immediately (no smooth stepping).
    //         The OE pin is still held high at this point, so servos receive
    //         the pulse but won't move until oe_loop() releases the OE hold.
    _model.moveToNeutral(/*immediate=*/true);

    Serial.println("[ServoControl] All joints at neutral. Awaiting OE release.");
}

// -----------------------------------------------------------------------------
//  update()  —  Advance smooth stepping. Call every loop().
// -----------------------------------------------------------------------------
void ServoControl::update() {
    _model.update();
}

// =============================================================================
//  MOTION API
// =============================================================================

void ServoControl::setTargetAngle(uint8_t channel, float absoluteDeg, bool immediate) {
    // Absolute angle path: used by the web UI sliders and pose loading.
    // JointModel converts to joint-relative and enforces limits internally.
    _model.setAbsoluteAngle(channel, absoluteDeg, immediate);
}

float ServoControl::getTargetAngle(uint8_t channel) {
    // Returns the target absolute servo angle (°) for state broadcast to UI.
    return _model.getAbsoluteAngle(channel);
}

void ServoControl::setJointAngleDirect(uint8_t channel, float angleDeg) {
    // immediate=true skips the smooth-stepping queue.
    // Joint limits are still enforced inside JointModel::setJointAngle().
    _model.setJointAngle(channel, angleDeg, /*immediate=*/true);
}

void ServoControl::setTargetPulse(uint8_t channel, int pulse_us, bool immediate) {
    // Convert pulse → absolute angle → joint model (which enforces limits).
    float absoluteDeg = ServoDriver::pulseToDeg(pulse_us);
    _model.setAbsoluteAngle(channel, absoluteDeg, immediate);
}

int ServoControl::getTargetPulse(uint8_t channel) {
    return _model.getServoPulse(channel);
}

void ServoControl::setGaitOffset(uint8_t channel, float absoluteOffsetDeg) {
    // absoluteOffsetDeg is in servo-frame degrees relative to neutral.
    // Example: setGaitOffset(ch, 30.0f) → servo moves to neutral+30° regardless
    // of joint direction. Used by gait code that computes absolute offsets.
    if (channel >= NUM_JOINTS) return;
    float targetAbsDeg = JOINT_CONFIG[channel].neutralDeg + absoluteOffsetDeg;
    _model.setAbsoluteAngle(channel, targetAbsDeg, /*immediate=*/false);
}

void ServoControl::applyNamedMovement(uint8_t channel, const char* movementName,
                                      float magnitude, bool immediate) {
    // In the new joint model, positive joint angle = anatomical flexion by definition.
    // "flex"   → setJointAngle(+magnitude) — moves in flexion direction
    // "extend" → setJointAngle(-magnitude) — moves in extension direction
    //
    // magnitude is in degrees (1:1 mapping — no US_PER_DEG scaling needed
    // because joint model works natively in degrees, not µs).
    if (channel >= NUM_JOINTS) return;

    float jointAngle = 0.0f;

    if (strcmp(movementName, "flex") == 0) {
        jointAngle = +magnitude;
    } else if (strcmp(movementName, "extend") == 0) {
        jointAngle = -magnitude;
    } else {
        Serial.printf("[ServoControl] WARNING: unknown movement '%s' on ch=%d\n",
                      movementName, channel);
        return;
    }

    Serial.printf("[ServoControl] applyNamedMovement ch=%d  %s  magnitude=%.2f  jointAngle=%.2f\n",
                  channel, movementName, magnitude, jointAngle);

    _model.setJointAngle(channel, jointAngle, immediate);
}

// =============================================================================
//  POSE MANAGEMENT
// =============================================================================

void ServoControl::saveCurrentPose(String name) {
    // Poses are saved as absolute servo angles (°) — one angle per line.
    // This matches the old format so existing pose files still load correctly.
    String filename = "/pose_" + name + ".txt";
    File file = LittleFS.open(filename, FILE_WRITE);
    if (!file) {
        Serial.printf("[ServoControl] saveCurrentPose: failed to open '%s'\n",
                      filename.c_str());
        return;
    }
    for (int i = 0; i < NUM_JOINTS; i++) {
        file.println(_model.getAbsoluteAngle(i));
    }
    file.close();
    Serial.printf("[ServoControl] Pose '%s' saved.\n", name.c_str());
}

void ServoControl::loadPose(String name) {
    String filename = "/pose_" + name + ".txt";
    File file = LittleFS.open(filename, FILE_READ);
    if (!file) {
        Serial.printf("[ServoControl] loadPose: pose '%s' not found\n", name.c_str());
        return;
    }
    for (int i = 0; i < NUM_JOINTS; i++) {
        if (!file.available()) break;
        float absoluteDeg = file.readStringUntil('\n').toFloat();
        _model.setAbsoluteAngle(i, absoluteDeg, /*immediate=*/false);
    }
    file.close();
    Serial.printf("[ServoControl] Pose '%s' loaded.\n", name.c_str());
}

String ServoControl::listPoses() {
    String list = "POSES:";
    File root = LittleFS.open("/");
    File file = root.openNextFile();
    while (file) {
        String fname = file.name();
        if (fname.startsWith("pose_")) {
            list += fname.substring(5, fname.length() - 4) + ",";
        }
        file = root.openNextFile();
    }
    return list;
}

void ServoControl::resetToNeutral() {
    // immediate=true: writes _currentPulse, _targetPulse, AND hardware registers
    // in one call. This ensures the smooth-stepper has nothing to chase when
    // OE goes LOW immediately after this returns.
    _model.moveToNeutral(/*immediate=*/true);
}

// =============================================================================
//  SPEED CONTROL  —  thin delegation to JointModel
// =============================================================================

void ServoControl::setJointSpeed(uint8_t channel, float speedDegPerSec) {
    _model.setJointSpeed(channel, speedDegPerSec);
}

void ServoControl::setAllJointsSpeed(float speedDegPerSec) {
    _model.setAllJointsSpeed(speedDegPerSec);
}

float ServoControl::getJointSpeed(uint8_t channel) const {
    return _model.getJointSpeed(channel);
}