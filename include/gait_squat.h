#ifndef GAIT_SQUAT_H
#define GAIT_SQUAT_H

#include <Arduino.h>
#include "servo_control.h"

class SquatGait {
public:
    SquatGait(ServoControl* servoCtrl);
    
    // Original full sequence (Down -> Wait -> Up)
    void startSequence();

    // Manual Controls
    void squatDown(); // Go down and stay there
    void standUp();   // Go back to neutral

    void update();
    bool isActive();

private:
    ServoControl* _servoCtrl;
    
    enum State {
        IDLE,
        DESCENDING,     // Neutral -> Squat
        HOLD_AUTO,      // Wait for timer (Sequence mode)
        HOLD_MANUAL,    // Wait indefinitely (Manual mode)
        ASCENDING       // Squat -> Neutral
    };
    
    State _state;
    unsigned long _stateStartTime;
    
    // Tuning (ms)
    const unsigned long DURATION_DESCENT = 2000;
    const unsigned long DURATION_HOLD    = 1000; // For auto sequence only
    const unsigned long DURATION_ASCENT  = 2000;

    float easeInOutCubic(float t);
    float _targetSquatAngles[NUM_SERVOS];
};

#endif