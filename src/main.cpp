#include <Arduino.h>
#include "project_wide_defs.h"
#include "servo_control.h"
#include "WebComm.h"
#include "oe_control.h"
#include "state_estimator.h"
#include "balance_controller.h"
#include "motion_manager.h"        // joint authority layer

// =============================================================================
//  GLOBAL OBJECTS
//
//  Construction order note:
//  MotionManager is constructed before setup() runs — this is safe because
//  its constructor only zero-initialises an array (no hardware access).
//  The ServoControl pointer is wired in setup() via motionManager.init().
// =============================================================================

ServoControl      servoController;
WebComm           webComm(&servoController);
StateEstimator    stateEstimator;
BalanceController balanceController(&servoController);
MotionManager     motionManager;   // joint authority layer — wired in setup()
// =============================================================================
//  OE CONTROL IMPLEMENTATION
//  Declared extern in oe_control.h so WebComm.cpp can call oe_estop/oe_clear.
// =============================================================================

static unsigned long _oe_boot_start = 0;
static bool          _oe_released   = false;
static bool          _oe_estopped   = false;

void oe_begin() {
    pinMode(OE_PIN, OUTPUT);
    digitalWrite(OE_PIN, HIGH);
    _oe_boot_start = millis();
    _oe_released   = false;
    _oe_estopped   = false;
    Serial.printf("[OE] Boot hold: outputs DISABLED (OE=HIGH). Releasing in %u ms.\n",
                  (unsigned)OE_BOOT_HOLD_MS);
}

void oe_loop() {
    if (_oe_released || _oe_estopped) return;
    if (millis() - _oe_boot_start >= OE_BOOT_HOLD_MS) {
        digitalWrite(OE_PIN, LOW);
        _oe_released = true;
        Serial.println("[OE] Boot hold complete. Outputs ENABLED (OE=LOW).");
    }
}

void oe_estop() {
    _oe_estopped = true;
    digitalWrite(OE_PIN, HIGH);
    Serial.println("[OE] *** EMERGENCY STOP *** Outputs DISABLED (OE=HIGH).");
}

void oe_clear() {
    _oe_estopped = false;
    _oe_released = true;
    digitalWrite(OE_PIN, LOW);
    Serial.println("[OE] E-stop cleared. Outputs ENABLED (OE=LOW).");
}

bool oe_is_estopped() {
    return _oe_estopped || !_oe_released;
}

// =============================================================================
//  SETUP
// =============================================================================

void setup() {
    Serial.begin(115200);
    Serial.println("Booting Robot...");

    oe_begin();
    servoController.init();       // Wire.begin(8, 9) happens here
    if (!IMU_init()) {
        Serial.println("[WARN] Continuing without IMU.");
    }
    stateEstimator.reset();       // starts calibration countdown
    webComm.setStateEstimator(&stateEstimator);
    balanceController.init();
    // Wire MotionManager — must come after servoController.init() because
    // init() stores a pointer to servoController (no hardware calls yet).
    motionManager.init(&servoController);
    balanceController.setMotionManager(&motionManager);
    webComm.setBalanceController(&balanceController);
    webComm.setMotionManager(&motionManager);
    webComm.init();
    Serial.println("System Ready.");
}

// =============================================================================
//  LOOP
// =============================================================================

void loop() {
    webComm.cleanupClients();
    oe_loop();
    servoController.update();

    // AFTER — 400 Hz gate: only reads IMU and runs estimator every 2500 µs.
    // WebSocket/servo tasks above still run every loop() iteration (no rate limit).
    {
        // 2500 µs = 400 Hz. Must match BMI160 ODR set in IMU_init().
        static const uint32_t ESTIMATOR_INTERVAL_US = 2500;
        static uint32_t _lastEstimatorMicros = micros();

        uint32_t _now = micros();
        if ((_now - _lastEstimatorMicros) >= ESTIMATOR_INTERVAL_US) {
            float dt = (_now - _lastEstimatorMicros) * 1e-6f;
            _lastEstimatorMicros = _now;

            RawIMUData raw   = IMU_update();
            IMUState   state = stateEstimator.update(raw, dt);

            BalanceState balState = balanceController.update(state);
            // Flush after ALL sources have submitted for this tick:
            //   balanceController.update() → submit(SOURCE_BALANCE, ...)
            //   WebSocket slider events   → submit(SOURCE_UI, ...)  [already queued]
            // flush() dispatches the highest-priority command per joint, then clears slots.
            motionManager.flush();

            webComm.broadcastIMU(raw);
            webComm.broadcastEstimate(state);
            webComm.broadcastBalanceState(balState);

            // Broadcast calibration progress at 10 Hz (rate-limited inside broadcastCalibStatus).
            // _calibDoneAnnounced ensures one final "done" packet is sent, then stops.
            static bool _calibDoneAnnounced = false;
            if (!_calibDoneAnnounced) {
                webComm.broadcastCalibStatus(
                    stateEstimator.getCalibState(),
                    stateEstimator.getCalibProgress()
                );
                if (stateEstimator.isCalibrated()) _calibDoneAnnounced = true;
            }
        }
    }
}