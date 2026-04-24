#include <Arduino.h>
#include "project_wide_defs.h"
#include "servo_control.h"
#include "webComm.h"
#include "oe_control.h"
#include "state_estimator.h"
#include "balance_controller.h"
#include "motion_manager.h"        // joint authority layer
#include "weight_shift.h"          // gait: CoM lateral shift
#include "leg_ik.h"
#include "gait_controller.h"
#include <math.h>

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
WeightShift       weightShift;     // gait: CoM lateral shift — wired in setup()
GaitController    gaitController;   // open-loop walking gait

// =============================================================================
//  OE CONTROL IMPLEMENTATION
//  Declared extern in oe_control.h so WebComm.cpp can call oe_estop/oe_clear.
// =============================================================================

static unsigned long _oe_boot_start = 0;

// volatile required: these flags cross FreeRTOS task boundaries.
// The WiFi task reads _oe_estopped; the main loop writes it (and vice versa).
// Without volatile the compiler is permitted to cache the value and never
// re-read memory, making the flag update invisible to the other task.
static volatile bool _oe_released   = false;
static volatile bool _oe_estopped   = false;

// Single definition of oe_is_estopped(). Cast from volatile bool to bool is
// explicit so the compiler cannot optimise away the memory read.
bool oe_is_estopped() {
    return static_cast<bool>(_oe_estopped) || !static_cast<bool>(_oe_released);
}

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
    // Push neutral targets before disabling OE so the last register state is
    // benign if signal lines float while OE is high.
    servoController.resetToNeutral();   // pre-zero to prevent violent snap
    balanceController.resetOutputState();  // was resetOutputState() — method renamed

    digitalWrite(OE_PIN, HIGH);
    Serial.println("[OE] *** EMERGENCY STOP *** Outputs DISABLED (OE=HIGH).");
}

void oe_clear() {
    servoController.resetToNeutral();
    balanceController.resetState();
    // Reset weight shift to center so no stale lean is applied on re-enable.
    // Must run before OE goes LOW to prevent the injected setpoint from triggering
    // an immediate balance correction toward the pre-estop lean target.
    weightShift.trigger(ShiftDirection::NONE);
    _oe_estopped = false;
    _oe_released = true;
    digitalWrite(OE_PIN, LOW);
    Serial.println("[OE] E-stop cleared — targets reset to neutral. Outputs ENABLED (OE=LOW).");
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

    // Validate IK roundtrip - create FootTarget objects explicitly
    FootTarget target1;
    target1.x_mm = 0.0f;
    target1.h_sagittal_mm = 160.0f;
    target1.y_mm = 0.0f;
    target1.h_frontal_mm = 0.0f;
    target1.isRightLeg = true;
    LegIK::validateRoundtrip(target1, 0.5f);

    FootTarget target2;
    target2.x_mm = 20.0f;
    target2.h_sagittal_mm = 155.0f;
    target2.y_mm = 15.0f;
    target2.h_frontal_mm = 0.0f;
    target2.isRightLeg = true;
    LegIK::validateRoundtrip(target2, 0.5f);
    // Replace with measured physical angles once you have them:
    // LegIK::validateNeutral(hip_meas, knee_meas, ankle_meas, 1.5f);

    // Wire MotionManager — must come after servoController.init() because
    // init() stores a pointer to servoController (no hardware calls yet).
    motionManager.init(&servoController);
    balanceController.setMotionManager(&motionManager);
    webComm.setBalanceController(&balanceController);
    webComm.setMotionManager(&motionManager);
    weightShift.init(&balanceController, &motionManager);
    webComm.setWeightShift(&weightShift);
    gaitController.init(&motionManager, &weightShift);
    webComm.setGaitController(&gaitController);
    webComm.init();
    Serial.println("System Ready.");
}

// =============================================================================
//  LOOP
// =============================================================================

void loop() {
    oe_loop();
    servoController.update();

    // Periodic joint-position state broadcast — 10 Hz.
    // Keeps the UI sliders in sync with the balance controller's live output
    // without requiring user interaction. Rate-limited to reduce WebSocket load.
    {
        static uint32_t _lastStateBroadcastMs = 0;
        const uint32_t  STATE_BROADCAST_INTERVAL_MS = 100;  // 10 Hz
        if (millis() - _lastStateBroadcastMs >= STATE_BROADCAST_INTERVAL_MS) {
            _lastStateBroadcastMs = millis();
            webComm.broadcastState();
        }
    }

    // 400 Hz gate: only reads IMU and runs estimator every 2500 µs.
    // WebSocket/servo tasks above still run every loop() iteration (no rate limit).
    {
        // 2500 µs = 400 Hz. Must match BMI160 ODR set in IMU_init().
        static const uint32_t ESTIMATOR_INTERVAL_US = 2500;
        static uint32_t _lastEstimatorMicros = micros();

        // _now is used for the gate check only.
        // _readNow is sampled immediately before IMU_update() so that dt
        // reflects actual sensor integration time, not WiFi/servo processing.
        uint32_t _now = micros();
        if ((_now - _lastEstimatorMicros) >= ESTIMATOR_INTERVAL_US) {
            uint32_t _readNow = micros();          // capture just before sensor read
            float dt = (_readNow - _lastEstimatorMicros) * 1e-6f;
            _lastEstimatorMicros = _readNow;       // anchor to sensor-read timestamp

            RawIMUData raw   = IMU_update();
            IMUState   state = stateEstimator.update(raw, dt);

            // WeightShift must run BEFORE balanceController so the injected
            // roll_setpoint_rad is used by balance on this same tick.
            weightShift.update(dt);

            gaitController.update(dt);

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
