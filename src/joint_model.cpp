#include "joint_model.h"

// =============================================================================
//  JOINT MODEL — Implementation
// =============================================================================

JointModel::JointModel()
    : _driver(nullptr),
      stepSize(10.0f),
      deadband(4.0f),
      _lastUpdateMs(0)
{
    // Zero-initialize arrays so un-initted channels are safe to read.
    for (int i = 0; i < NUM_JOINTS; i++) {
        _currentPulse[i] = (float)SD_USMIN;
        _targetPulse[i]  = (uint16_t)SD_USMIN;
    }
}

// -----------------------------------------------------------------------------
//  init()  —  Wire up to the hardware driver and move to neutral.
// -----------------------------------------------------------------------------
void JointModel::init(ServoDriver* driver) {
    if (driver == nullptr) {
        Serial.println("[JointModel] ERROR: init() called with null driver!");
        return;
    }
    _driver = driver;
    Serial.println("[JointModel] Initialized.");
}

// =============================================================================
//  PRIMARY API
// =============================================================================

void JointModel::setJointAngle(uint8_t jointId, float angleDeg, bool immediate) {
    if (jointId >= NUM_JOINTS) {
        Serial.printf("[JointModel] ERROR: setJointAngle jointId=%d out of range\n", jointId);
        return;
    }

    // 1. Enforce joint limits (defined in joint_config.h).
    float clamped = clampToLimits(jointId, angleDeg);

    // Log if the requested angle was clamped — useful for catching motion bugs.
    if (clamped != angleDeg) {
        Serial.printf("[JointModel] LIMIT: ch=%d  requested=%.1f°  clamped=%.1f°\n",
                      jointId, angleDeg, clamped);
    }

    // 2. Convert joint angle → servo pulse (applies neutral + direction).
    int pulse = jointAngleToPulse(jointId, clamped);

    // 3. Store as the new target.
    _targetPulse[jointId] = (uint16_t)pulse;

    // 4. Immediate write bypasses the smooth stepping loop.
    if (immediate && _driver != nullptr) {
        _currentPulse[jointId] = (float)pulse;
        _driver->setServoPulse(jointId, pulse);
    }
}

void JointModel::moveToNeutral(bool immediate) {
    // Joint angle 0 = neutral by definition — no math needed.
    //
    // Channels in SKIP_BOOT_NEUTRAL_CHANNELS (joint_config.h) are intentionally
    // skipped here. Their mounting orientation has not been confirmed; sending
    // them to a "neutral" pulse before verification risks mechanical damage.
    // Remove a channel from that list once its neutral position is physically
    // verified.
    for (int i = 0; i < NUM_JOINTS; i++) {
        bool skip = false;
        for (uint8_t k = 0; k < SKIP_BOOT_NEUTRAL_COUNT; k++) {
            if ((uint8_t)i == SKIP_BOOT_NEUTRAL_CHANNELS[k]) {
                skip = true;
                break;
            }
        }
        // prime PCA9685 registers with a safe pulse before OE releases,
// even though these channels are excluded from the motion queue.
// Without this write, the PCA9685 holds its power-on or last-session
// register state; when OE goes LOW after the boot hold, every skipped
// servo snaps to that undefined position simultaneously.
if (skip) {
    int pulse = ServoDriver::degToPulse(JOINT_CONFIG[i].neutralDeg);
    _currentPulse[i] = static_cast<float>(pulse);
    _targetPulse[i]  = static_cast<uint16_t>(pulse);
    if (_driver != nullptr) {
        _driver->setServoPulse(i, pulse);
    }
    continue;
}
        setJointAngle(i, 0.0f, immediate);
    }
}

// =============================================================================
//  COMPATIBILITY API
// =============================================================================

void JointModel::setAbsoluteAngle(uint8_t jointId, float absoluteDeg, bool immediate) {
    // Convert from hardware-frame (absolute servo angle) to motion-frame
    // (joint-relative angle), then use the standard path which enforces limits.
    float jointAngle = absoluteDegToJointAngle(jointId, absoluteDeg);
    setJointAngle(jointId, jointAngle, immediate);
}

// =============================================================================
//  GETTERS
// =============================================================================

float JointModel::getAbsoluteAngle(uint8_t jointId) const {
    if (jointId >= NUM_JOINTS) return 0.0f;
    // Return the TARGET angle (what we want), not the smoothed current.
    // The UI shows where we're going, not where the servo physically is right now.
    return ServoDriver::pulseToDeg((int)_targetPulse[jointId]);
}

float JointModel::getJointAngle(uint8_t jointId) const {
    if (jointId >= NUM_JOINTS) return 0.0f;
    return absoluteDegToJointAngle(jointId, getAbsoluteAngle(jointId));
}

int JointModel::getServoPulse(uint8_t jointId) const {
    if (jointId >= NUM_JOINTS) return SD_USMIN;
    return (int)_targetPulse[jointId];
}

// =============================================================================
//  SMOOTH STEPPING LOOP
// =============================================================================

void JointModel::update() {
    // Self-limit to ~50 Hz. At 50 Hz and stepSize=10µs, max servo speed is
    // 500 µs/s ≈ 67°/s — fast enough for controlled motion, slow enough to
    // feel deliberate.
    if (millis() - _lastUpdateMs < 20) return;
    _lastUpdateMs = millis();

    if (_driver == nullptr) return;

    for (int i = 0; i < NUM_JOINTS; i++) {
        float diff = (float)_targetPulse[i] - _currentPulse[i];

        // Deadband prevents constant tiny hardware writes when the servo is
        // essentially at rest. Reduces I2C bus traffic and servo jitter.
        if (fabsf(diff) > deadband) {
            _currentPulse[i] += (diff > 0.0f) ? stepSize : -stepSize;
            _driver->setServoPulse(i, (int)_currentPulse[i]);
        }
    }
}

// =============================================================================
//  PRIVATE HELPERS
// =============================================================================

/*static*/ float JointModel::clampToLimits(uint8_t jointId, float angleDeg) {
    // Reads limits from the config table. Returns angleDeg clamped to [min, max].
    if (jointId >= NUM_JOINTS) return 0.0f;
    const JointConfig& jc = JOINT_CONFIG[jointId];
    if (angleDeg < jc.minAngle) return jc.minAngle;
    if (angleDeg > jc.maxAngle) return jc.maxAngle;
    return angleDeg;
}

/*static*/ int JointModel::jointAngleToPulse(uint8_t jointId, float angleDeg) {
    // This is the core conversion formula.
    //
    // Step 1: apply direction and offset from neutral to get absolute servo angle
    //   servoAngle = neutralDeg + direction × jointAngle
    //
    // Step 2: convert absolute servo angle to pulse width via ServoDriver math
    //
    // NOTE: angleDeg should already be clamped before this is called.
    if (jointId >= NUM_JOINTS) return SD_USMIN;
    const JointConfig& jc = JOINT_CONFIG[jointId];

    float servoAngle = jc.neutralDeg + (float)jc.direction * angleDeg;

    // Hardware bounds — servoAngle must stay within 0°–270°.
    if (servoAngle < 0.0f)          servoAngle = 0.0f;
    if (servoAngle > SD_ANGLE_RANGE) servoAngle = SD_ANGLE_RANGE;

    return ServoDriver::degToPulse(servoAngle);
}

/*static*/ float JointModel::absoluteDegToJointAngle(uint8_t jointId, float absoluteDeg) {
    // Reverse of jointAngleToPulse (step 1 only):
    //   servoAngle = neutral + direction × jointAngle
    //   jointAngle = (servoAngle − neutral) × direction   [since direction = ±1]
    if (jointId >= NUM_JOINTS) return 0.0f;
    const JointConfig& jc = JOINT_CONFIG[jointId];
    return (absoluteDeg - jc.neutralDeg) * (float)jc.direction;
}