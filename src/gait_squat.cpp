#include "gait_squat.h"
#include "project_wide_defs.h"

SquatGait::SquatGait(ServoControl* servoCtrl) : _servoCtrl(servoCtrl) {
    _state = IDLE;
    
    // Initialize Target Array with Neutral values first
    for(int i=0; i<NUM_SERVOS; i++) _targetSquatAngles[i] = JOINT_MAP[i].neutralAngle;

    // --- DEFINE BOTTOM SQUAT POSE ---
    _targetSquatAngles[0]  = 131.0f; // R Ankle Roll
    _targetSquatAngles[1]  = 65.0f;  // R Ankle Pitch
    _targetSquatAngles[2]  = 27.0f;  // R Knee Pitch
    _targetSquatAngles[3]  = 130.0f; // R Hip Roll
    _targetSquatAngles[4]  = 130.0f; // R Hip Yaw
    _targetSquatAngles[5]  = 225.0f; // R Hip Pitch

    _targetSquatAngles[10] = 65.0f;  // L Hip Pitch
    _targetSquatAngles[11] = 130.0f; // L Hip Yaw
    _targetSquatAngles[12] = 134.0f; // L Hip Roll
    _targetSquatAngles[13] = 243.0f; // L Knee Pitch
    _targetSquatAngles[14] = 185.0f; // L Ankle Pitch
    _targetSquatAngles[15] = 127.0f; // L Ankle Roll
}

void SquatGait::startSequence() {
    Serial.println("Gait: Starting Full Sequence");
    _state = DESCENDING;
    _stateStartTime = millis();
}

void SquatGait::squatDown() {
    Serial.println("Gait: Squatting Down (Hold)");
    _state = DESCENDING;
    _stateStartTime = millis();
}

void SquatGait::standUp() {
    Serial.println("Gait: Standing Up");
    _state = ASCENDING;
    _stateStartTime = millis();
}

bool SquatGait::isActive() {
    return _state != IDLE;
}

float SquatGait::easeInOutCubic(float t) {
    return t < 0.5f ? 4.0f * t * t * t : 1.0f - pow(-2.0f * t + 2.0f, 3.0f) / 2.0f;
}

void SquatGait::update() {
    if (_state == IDLE) return;

    unsigned long now = millis();
    unsigned long elapsed = now - _stateStartTime;
    float t = 0.0f;
    float phaseProgress = 0.0f;

    switch (_state) {
        case DESCENDING:
            if (elapsed >= DURATION_DESCENT) {
                // If this was triggered by startSequence(), go to HOLD_AUTO
                // If triggered by squatDown(), go to HOLD_MANUAL
                // For simplicity, we default to HOLD_MANUAL logic unless sequence logic enforced.
                // Here we simply check context or default to manual hold if we reached bottom.
                _state = HOLD_MANUAL; 
                _stateStartTime = now;
                Serial.println("Gait: Reached Bottom (Holding)");
            } else {
                t = (float)elapsed / (float)DURATION_DESCENT;
                phaseProgress = easeInOutCubic(t);
                
                for (int i = 0; i < NUM_SERVOS; i++) {
                    float start = JOINT_MAP[i].neutralAngle;
                    float end = _targetSquatAngles[i];
                    float current = start + (end - start) * phaseProgress;
                    _servoCtrl->setTargetAngle(i, current, false); 
                }
            }
            break;

        case HOLD_MANUAL:
            // Stay here indefinitely until standUp() is called
            for (int i = 0; i < NUM_SERVOS; i++) {
                _servoCtrl->setTargetAngle(i, _targetSquatAngles[i], false);
            }
            break;

        case ASCENDING:
            if (elapsed >= DURATION_ASCENT) {
                _state = IDLE;
                Serial.println("Gait: Back to Neutral");
                for (int i = 0; i < NUM_SERVOS; i++) {
                    _servoCtrl->setTargetAngle(i, JOINT_MAP[i].neutralAngle, false);
                }
            } else {
                t = (float)elapsed / (float)DURATION_ASCENT;
                phaseProgress = easeInOutCubic(t);
                
                for (int i = 0; i < NUM_SERVOS; i++) {
                    float start = _targetSquatAngles[i];
                    float end = JOINT_MAP[i].neutralAngle;
                    float current = start + (end - start) * phaseProgress;
                    _servoCtrl->setTargetAngle(i, current, false);
                }
            }
            break;
            
        default:
            _state = IDLE;
            break;
    }
}