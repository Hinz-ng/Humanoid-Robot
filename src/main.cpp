#include <Arduino.h>
#include "project_wide_defs.h"
#include "servo_control.h"
#include "WebComm.h"
#include "oe_control.h"
#include "state_estimator.h"    // --- ADD: complementary filter


// =============================================================================
//  GLOBAL OBJECTS
// =============================================================================

ServoControl servoController;
// legacy gait pointer removed; WebComm now only needs the servo controller.
WebComm      webComm(&servoController);
StateEstimator stateEstimator;   // default FilterConfig (alpha=0.995)

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
// Balance loop: runs every iteration, dt measured in seconds.
// Target: 200-400 Hz. ServoControl.update() self-limits to 50 Hz internally.
{
    static uint32_t _lastEstimatorMicros = micros();
    uint32_t _now = micros();
    float dt = (_now - _lastEstimatorMicros) * 1e-6f;
    _lastEstimatorMicros = _now;

    RawIMUData raw = IMU_update();
    IMUState   state = stateEstimator.update(raw, dt);

    webComm.broadcastIMU(raw);        // raw values → UI IMU panel
    webComm.broadcastEstimate(state); // filtered angles → UI IMU panel
}
}