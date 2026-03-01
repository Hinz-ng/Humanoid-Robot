#include <Arduino.h>
#include "project_wide_defs.h"
#include "servo_control.h"
#include "WebComm.h"
#include "gait_squat.h"

// Global Objects
ServoControl servoController;
SquatGait squatGait(&servoController);

// Pass both controllers to WebComm
WebComm webComm(&servoController, &squatGait);

void setup() {
    Serial.begin(115200);
    
    Serial.println("Booting Robot...");
    servoController.init(); // Moves to Neutral
    webComm.init();
    
    Serial.println("System Ready.");
}

void loop() {
    // 1. Network housekeeping
    webComm.cleanupClients();
    
    // 2. Physical Servo Update (SSOT enforcement)
    servoController.update();
    
    // 3. Gait State Machine Update
    squatGait.update();
}