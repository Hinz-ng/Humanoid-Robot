#include <Arduino.h>
#include "project_wide_defs.h"
#include "servo_control.h"
#include "WebComm.h"
#include "gait_squat.h"
#include "oe_control.h"   // --- ADD ---

// =============================================================================
//  GLOBAL OBJECTS
// =============================================================================

ServoControl servoController;
SquatGait    squatGait(&servoController);
WebComm      webComm(&servoController, &squatGait);

// =============================================================================
//  --- ADD START: OE CONTROL IMPLEMENTATION ---
//
//  All OE state lives here. Functions are declared extern in oe_control.h
//  so WebComm.cpp can call oe_estop() / oe_clear() without including main.cpp.
//
//  Non-blocking boot hold:
//    oe_begin()  → immediately drives OE HIGH (disable)
//    oe_loop()   → called every loop(); drives OE LOW after OE_BOOT_HOLD_MS
//
//  Once the boot hold completes, oe_loop() does nothing (cheap check).
// =============================================================================

static unsigned long _oe_boot_start = 0;
static bool          _oe_released   = false;   // true once boot hold is done
static bool          _oe_estopped   = false;

void oe_begin() {
    pinMode(OE_PIN, OUTPUT);
    digitalWrite(OE_PIN, HIGH);   // HIGH = DISABLED (active-low)
    _oe_boot_start = millis();
    _oe_released   = false;
    _oe_estopped   = false;
    Serial.printf("[OE] Boot hold: outputs DISABLED (OE=HIGH). "
                  "Releasing in %u ms.\n", (unsigned)OE_BOOT_HOLD_MS);
}

void oe_loop() {
    // Skip if already released or manually e-stopped
    if (_oe_released || _oe_estopped) return;

    if (millis() - _oe_boot_start >= OE_BOOT_HOLD_MS) {
        digitalWrite(OE_PIN, LOW);  // LOW = ENABLED
        _oe_released = true;
        Serial.println("[OE] Boot hold complete. Outputs ENABLED (OE=LOW).");
    }
}

void oe_estop() {
    _oe_estopped = true;
    digitalWrite(OE_PIN, HIGH);   // HIGH = DISABLED
    Serial.println("[OE] *** EMERGENCY STOP *** Outputs DISABLED (OE=HIGH).");
}

void oe_clear() {
    _oe_estopped = false;
    _oe_released = true;          // Don't re-enter the boot hold path
    digitalWrite(OE_PIN, LOW);   // LOW = ENABLED
    Serial.println("[OE] E-stop cleared. Outputs ENABLED (OE=LOW).");
}

bool oe_is_estopped() {
    // True if actively e-stopped OR if boot hold has not yet released
    return _oe_estopped || !_oe_released;
}

// =============================================================================
//  --- ADD END ---
// =============================================================================

void setup() {
    Serial.begin(115200);
    Serial.println("Booting Robot...");

    oe_begin();
    servoController.init();   // <-- Wire.begin(8, 9) happens here
    if (!IMU_init()) {        // <-- safe to call now; Wire is already up
        Serial.println("[WARN] Continuing without IMU.");
    }
    webComm.init();
    Serial.println("System Ready.");
}

void loop() {
    webComm.cleanupClients();
    oe_loop();              // --- ADD: non-blocking boot hold release ---
    servoController.update();
    squatGait.update();
    webComm.broadcastIMU(IMU_update());
}