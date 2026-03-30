#include "joint_model.h"
#include "project_wide_defs.h"

// =============================================================================
//  JOINT MODEL — Implementation
// =============================================================================

JointModel::JointModel()
    : _driver(nullptr),
      _lastUpdateMs(0)
{
    for (int i = 0; i < NUM_JOINTS; i++) {
        _currentPulse[i]   = (float)SD_USMIN;
        _targetPulse[i]    = (uint16_t)SD_USMIN;
        _speedDegPerSec[i] = JOINT_SPEED_DEFAULT_DEG_S;
        deadband[i]        = 4.0f;  // µs — 0.54° for 270° servo, 0.36° for 180°
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
    // Channels in SKIP_BOOT_NEUTRAL_CHANNELS (joint_config.h) are handled
    // separately below. Their mounting orientation has not been confirmed;
    // sending them to a "neutral" pulse before verification risks mechanical
    // damage. Remove a channel from that list once its neutral position is
    // physically verified.
    for (int i = 0; i < NUM_JOINTS; i++) {
        bool skip = false;
        for (uint8_t k = 0; k < SKIP_BOOT_NEUTRAL_COUNT; k++) {
            if ((uint8_t)i == SKIP_BOOT_NEUTRAL_CHANNELS[k]) {
                skip = true;
                break;
            }
        }

        if (skip) {
            // Set firmware state to neutral for coherence — when this channel is
            // first explicitly commanded (UI slider or controller), the smooth-stepper
            // steps from neutralPulse rather than SD_USMIN (0°), preventing a violent
            // snap to minimum position on the first write.
            //
            // Do NOT write to hardware. After ServoDriver::init(), the PCA9685 has
            // been reset: all channel registers are 0, which outputs no valid servo
            // pulse (below the 500µs minimum threshold recognised by hobby servos).
            // The servo holds its physical position under no torque until explicitly
            // commanded. This is the correct behaviour for unverified channels.
            //
            // WHY NOT write a neutral pulse here:
            //   OE is HIGH when moveToNeutral() runs, so the servo cannot move yet.
            //   But the PCA9685 register still holds whatever was last written.
            //   When OE releases (1000ms later), the servo snaps to that register
            //   value — even if the register was written while OE was HIGH.
            //   Writing 1500µs here would cause the servo to snap to 135° on OE
            //   release regardless of its current physical position. For unverified
            //   channels this is dangerous. Leave the register at 0 (no pulse).
            int pulse = ServoDriver::degToPulse(JOINT_CONFIG[i].neutralDeg);
            _currentPulse[i] = static_cast<float>(pulse);
            _targetPulse[i]  = static_cast<uint16_t>(pulse);
            // Hardware write intentionally omitted — see comment above.
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
    // Rate-limit to ~50 Hz (20ms minimum interval).
    unsigned long nowMs = millis();
    if (nowMs - _lastUpdateMs < 20) return;

    // Use actual elapsed time for speed-to-µs conversion so accuracy holds
    // even when WiFi or I2C occasionally delays a tick past 20ms.
    float dt_s = (float)(nowMs - _lastUpdateMs) * 1e-3f;
    _lastUpdateMs = nowMs;

    if (_driver == nullptr) return;

    // Pulse range is fixed by the PCA9685 / servo hardware — same for all joints.
    const float fullRangeUs = (float)(SD_USMAX - SD_USMIN);  // 2000 µs

    for (int i = 0; i < NUM_JOINTS; i++) {
        float diff = (float)_targetPulse[i] - _currentPulse[i];

        // Deadband: skip write if already close enough. Reduces I2C bus
        // traffic and prevents audible servo buzz at rest.
       if (fabsf(diff) <= deadband[i]) continue;

        // Convert this joint's deg/s speed to a µs step for this tick.
        // MUST use JOINT_CONFIG[i].rangeDeg, not SD_ANGLE_RANGE:
        //   180° joints (hip yaw, ankle pitch) map the same 2000 µs pulse range
        //   to fewer degrees, so each µs represents more degrees on those joints.
        //   Using SD_ANGLE_RANGE (270°) on a 180° joint underestimates the step
        //   size by 33%, making those joints appear slower than commanded.
        float stepUs = (_speedDegPerSec[i] / JOINT_CONFIG[i].rangeDeg) * fullRangeUs * dt_s;

        // CRITICAL: clamp step to the remaining distance.
        // Without this, stepUs > |diff| causes the servo to overshoot and
        // ping-pong around the target — the root cause of jitter at low speeds
        // or on final approach. This was a bug in the original fixed-stepSize code.
        stepUs = fminf(stepUs, fabsf(diff));

        _currentPulse[i] += (diff > 0.0f) ? stepUs : -stepUs;
        _driver->setServoPulse(i, (int)(_currentPulse[i] + 0.5f));
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
    if (servoAngle < 0.0f)           servoAngle = 0.0f;
    if (servoAngle > SD_ANGLE_RANGE)  servoAngle = SD_ANGLE_RANGE;

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

// =============================================================================
//  SPEED CONTROL
// =============================================================================

void JointModel::setJointSpeed(uint8_t jointId, float speedDegPerSec) {
    if (jointId >= NUM_JOINTS) return;
    // Floor: JOINT_SPEED_MIN_DEG_S prevents a zero/negative speed that would
    // freeze the joint silently or step backward indefinitely.
    // Ceiling: per-joint no-load speed from JointConfig — the servo physically
    // cannot exceed this. Clamping here prevents commanding a slew rate the
    // hardware cannot follow, which would cause the smooth-stepper to fall
    // permanently behind and never reach the target.
    const float maxSpeed = JOINT_CONFIG[jointId].noLoadSpeedDegS;
    if (speedDegPerSec < JOINT_SPEED_MIN_DEG_S) speedDegPerSec = JOINT_SPEED_MIN_DEG_S;
    if (speedDegPerSec > maxSpeed)              speedDegPerSec = maxSpeed;
    _speedDegPerSec[jointId] = speedDegPerSec;
}

void JointModel::setAllJointsSpeed(float speedDegPerSec) {
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        setJointSpeed(i, speedDegPerSec);
    }
}

float JointModel::getJointSpeed(uint8_t jointId) const {
    if (jointId >= NUM_JOINTS) return JOINT_SPEED_DEFAULT_DEG_S;
    return _speedDegPerSec[jointId];
}

void JointModel::setDeadband(uint8_t jointId, float deadbandUs) {
    if (jointId >= NUM_JOINTS) return;
    if (deadbandUs < 1.0f) deadbandUs = 1.0f;  // floor prevents silent zero-deadband bug
    deadband[jointId] = deadbandUs;
}