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

// ── Phase F1: foot IMUs (read + filter + telemetry, no closed-loop) ───────
#if FOOT_IMU_ENABLED
#include <Wire.h>
#include "foot_contact.h"
#include "lsm6dsv_driver.h"

// Build a FilterConfig tuned for the foot LSM6DSV. Mirrors torso defaults
// where defensible; deviates where the chip differs from BMI160 or where
// the foot's mechanical environment justifies it. See project_wide_defs.h
// for rationale on each value.
static FilterConfig _makeFootFilterConfig() {
    FilterConfig c;
    c.alpha               = FOOT_IMU_COMP_FILTER_ALPHA;
    c.accel_scale         = FOOT_IMU_ACCEL_SCALE_LSB_PER_G;
    c.gyro_scale          = FOOT_IMU_GYRO_SCALE_LSB_PER_DPS;
    c.accel_lpf_beta      = FOOT_IMU_ACCEL_LPF_BETA;
    c.accel_sanity_margin = FOOT_IMU_ACCEL_SANITY_MARGIN;
    // Axis signs default +1 (same as torso). If F1 verification shows pitch
    // or roll inverted on a foot, flip the matching sign here AND put a
    // dated comment in this function explaining which foot and why.
    c.pitch_gyro_sign     = 1.0f;
    c.roll_gyro_sign      = 1.0f;
    return c;
}
#endif

// Foot IMU axis remap — applied per tick to RawIMUData before estimator.
//
// CHIP MOUNTING (verified 2026-05-13 with foot flat on the floor):
//   chip X axis points robot-right        → robot Y = -chip X
//   chip Y axis points robot-backward     → robot X = -chip Y
//   chip Z axis points DOWN through sole  → robot Z = -chip Z
//
// Result: (robot_x, robot_y, robot_z) = (-chip_y, -chip_x, -chip_z).
// Same matrix applied to gyro rates (they transform identically).
//
// If you re-mount the IMU, EDIT THIS FUNCTION — don't add per-axis sign
// fields to FilterConfig. The remap is a property of the physical mounting,
// not the filter. Keeping it here keeps the source of truth in one place.
static void _remapFootRawToRobotFrame(RawIMUData& r) {
    if (!r.valid) return;
    const int16_t ax = r.accel_x, ay = r.accel_y, az = r.accel_z;
    const int16_t gx = r.gyro_x,  gy = r.gyro_y,  gz = r.gyro_z;
    r.accel_x = -ay;   // robot X (forward)
    r.accel_y =  ax;   // robot Y (left)
    r.accel_z =  az;   // robot Z (up)
    r.gyro_x  = -gy;
    r.gyro_y  =  gx;
    r.gyro_z  =  gz;
}

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

#if FOOT_IMU_ENABLED
// Wire1 — separate I2C bus from Wire0 (PCA9685 + BMI160 on SDA=8, SCL=9).
// Bus split rationale:
//   1. Foot reads do not compete with servo writes or torso IMU reads for
//      bus time on Wire0 (which the PCA9685 already saturates near 200 Hz).
//   2. Each chip's hardwired SA0 puts it at a unique 7-bit address via
//      Wire1 (0x6A / 0x6B), distinct from BMI160's 0x68/0x69. No collision
//      risk remains even if a future revision merges buses.
//   3. Wire1 clock (200 kHz) is set below the LSM6DSV's I2C maximum to
//      survive the ~40 cm flex-cable run to the feet. Do NOT raise without
//      re-running the F1 loop-budget telemetry; long unshielded cables on a
//      faster bus produce silent corruption, not error returns.
//
// SA0 is hardwired on the breakout. Pass -1 for sa0_pin so the driver only
// uses sa0_high to compute the address and never writes to a GPIO.
//   Left  breakout: SA0 to GND -> 0x6A
//   Right breakout: SA0 to 3V3 -> 0x6B
// VERIFY the physical wiring on the bench before flashing — mis-mapping
// left/right addresses causes the left-foot estimator to consume right-foot
// data with no error indication.
LSM6DSVDriver   leftFootIMU (&Wire1, /*sa0_high=*/false);   // SA0 → GND → 0x6A
LSM6DSVDriver   rightFootIMU(&Wire1, /*sa0_high=*/true );   // SA0 → 3V3 → 0x6B

// One estimator per foot. FilterConfig differs per-instance — must be passed
// at construction time. Each estimator runs its OWN calibration cycle
// (CALIB_WAITING → CALIB_COLLECTING → CALIB_DONE), independently from the
// torso's calibration. All three calibrations run in parallel during boot.
StateEstimator leftFootEstimator (_makeFootFilterConfig());
StateEstimator rightFootEstimator(_makeFootFilterConfig());
FootContactConfig footContactCfg;

#endif
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
    balanceController.setAnkleRollBias(0.0f, 0.0f);
    balanceController.setRollSetpointRad(0.0f);
    // Reset weight shift to center so no stale lean is applied on re-enable.
    // Must run before OE goes LOW to prevent the injected setpoint from triggering
    // an immediate balance correction toward the pre-estop lean target.
    weightShift.forceCenterImmediate();
    // Force gait to IDLE so any in-progress RECOVERY_STEP (or gait mode) is
    // terminated immediately. start(IDLE) clears recovery private state and
    // re-centers WeightShift (harmless after forceCenterImmediate above).
    gaitController.start(GaitMode::IDLE);
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
    gaitController.init(&motionManager, &weightShift, &balanceController);
    webComm.setGaitController(&gaitController);
    webComm.init();

#if FOOT_IMU_ENABLED
    // Wire1 — foot IMU bus. SDA=11, SCL=12 @ 200 kHz. Both drivers share
    // this bus; addresses come from the breakouts' hardwired SA0 wiring.
    //
    // Pull-ups: ESP32 internal pull-ups are weak (~45 kΩ). On long foot-cable
    // runs, add 4.7 kΩ external pull-ups to 3.3V on both lines if your
    // breakouts don't already provide them. That margin matters even more if
    // you later try to raise Wire1 above the verified 200 kHz baseline.
    Wire1.begin(/*SDA=*/11, /*SCL=*/12, /*clock_hz=*/200000);
    Serial.println("[FootIMU/F1] Wire1 started on SDA=11 SCL=12 @ 200 kHz");

    const bool lOk = leftFootIMU.begin("FootIMU/L");
    const bool rOk = rightFootIMU.begin("FootIMU/R");
    if (!lOk) Serial.println("[FootIMU/F1] LEFT  init FAILED — estimator will hold invalid");
    if (!rOk) Serial.println("[FootIMU/F1] RIGHT init FAILED — estimator will hold invalid");

    leftFootEstimator.reset();
    rightFootEstimator.reset();
    Serial.println("[FootIMU/F1] Foot estimators armed; running boot calibration in parallel.");
    webComm.setFootIMUs(&leftFootIMU, &rightFootIMU);
    webComm.setFootContactConfig(&footContactCfg);
    webComm.setFootNeutralPitch(0.0f, 0.0f);   // F4 will source session neutrals
#endif

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

#if FOOT_IMU_ENABLED
            // Foot IMU read + filter. Per the F1 contract, NO consumer in
            // the control stack reads these values yet. They flow to the
            // UI only. F2 will add foot_contact predicates that consume
            // these states; F3 will wire them into liftGateOK.
            const uint32_t _footStartUs = micros();
            RawIMUData lf_raw   = leftFootIMU.read();
            _remapFootRawToRobotFrame(lf_raw);
            const IMUState   lf_state = leftFootEstimator.update(lf_raw, dt);
            RawIMUData rf_raw   = rightFootIMU.read();
            _remapFootRawToRobotFrame(rf_raw);
            const IMUState   rf_state = rightFootEstimator.update(rf_raw, dt);
            const uint32_t _footDurUs = micros() - _footStartUs;

            // Loop budget — accumulates inside the 400 Hz gate so timing is
            // sample-aligned with the actual reads.
            static uint32_t _budgetMaxUs    = 0;
            static uint64_t _budgetSumUs    = 0;
            static uint32_t _budgetCount    = 0;
            static uint32_t _budgetReportMs = 0;
            if (_footDurUs > _budgetMaxUs) _budgetMaxUs = _footDurUs;
            _budgetSumUs += _footDurUs;
            _budgetCount++;
            if (millis() - _budgetReportMs >= 5000) {
                _budgetReportMs = millis();
                const uint32_t avg = (_budgetCount > 0)
                    ? (uint32_t)(_budgetSumUs / _budgetCount) : 0;
                Serial.printf("[FootIMU/Budget] peak=%u us  avg=%u us  "
                              "(window=5s, n=%u)\n",
                              (unsigned)_budgetMaxUs, (unsigned)avg,
                              (unsigned)_budgetCount);
                _budgetMaxUs = 0;
                _budgetSumUs = 0;
                _budgetCount = 0;
            }
#endif

#if FOOT_IMU_ENABLED
            // Phase F3: precompute per-foot contact predicates once per tick.
            // Same snapshot drives the lift gate and the UI telemetry - there
            // is no path by which they can disagree.
            FootContactSnapshot footSnap;
            const IMUState* foot_states[SIDE_COUNT] = { &lf_state, &rf_state };
            for (uint8_t i = 0; i < SIDE_COUNT; ++i) {
                footSnap.flat[i]   = isFootFlat(*foot_states[i], footContactCfg);
                footSnap.loaded[i] = isFootLoaded(*foot_states[i], /*neutral=*/0.0f, footContactCfg);
                footSnap.valid[i]  = foot_states[i]->valid;
            }
            footSnap.allValid = footSnap.valid[sideIdx(Side::LEFT)]
                             && footSnap.valid[sideIdx(Side::RIGHT)];
            const FootContactSnapshot* footSnapPtr = &footSnap;
#else
            const FootContactSnapshot* footSnapPtr = nullptr;
#endif

            // WeightShift must run BEFORE balanceController so any state it
            // sets (ankle bias, roll setpoint) is visible to balance same tick.
            weightShift.update(dt);
            // Note: swing-leg bias masking is now internal to WeightShift,
            // driven by GaitController::_publishSwingLegMaskToWS() each tick.

            // Stage 3: when task_space_output=true, balance runs FIRST so its
            // correction is available when gait builds FootTargets. Legacy order
            // preserved when task_space_output=false.
            //
            // Recovery flag wiring: both paths pass a BalanceState* to
            // gaitController.update() so catch-step detection can observe the flag.
            // Task-space: current-tick balState (fresh).
            // Legacy: previous-tick balState via getLastState() — at most 2.5 ms stale,
            // which is well within the recovery_confirm_ticks debounce window.
            const bool taskSpace = balanceController.getConfig().task_space_output;
            BalanceState balState;
            if (taskSpace) {
                balState = balanceController.update(state);
                const BalanceCorrection bc = balanceController.getLastCorrection();
                gaitController.update(dt, state, &bc, &balState, footSnapPtr);
            } else {
                const BalanceState lastBal = balanceController.getLastState();
                gaitController.update(dt, state, nullptr, &lastBal, footSnapPtr);
                balState = balanceController.update(state);
            }

            // Flush after ALL sources have submitted for this tick:
            //   balanceController.update() → submit(SOURCE_BALANCE, ...)
            //   WebSocket slider events   → submit(SOURCE_UI, ...)  [already queued]
            // flush() dispatches the highest-priority command per joint, then clears slots.
            motionManager.flush();

            webComm.broadcastIMU(raw);
            webComm.broadcastEstimate(state);
#if FOOT_IMU_ENABLED
            webComm.broadcastFootIMU(lf_raw, rf_raw);
            webComm.broadcastFootEstimate(lf_state, rf_state);
            webComm.broadcastFootContact(footSnapPtr);
#endif
            webComm.broadcastBalanceState(balState);
            webComm.broadcastGaitState();

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

#if FOOT_IMU_ENABLED && FOOT_IMU_BENCH_PRINT
    // Bench-mode raw count print. Disabled by default in F1 (telemetry
    // replaces it). Re-enable temporarily by setting FOOT_IMU_BENCH_PRINT=1
    // in project_wide_defs.h if you need to debug a specific axis without
    // a browser open.
    {
        static uint32_t _lastFootImuPrintMs = 0;
        if (millis() - _lastFootImuPrintMs >= 200) {
            _lastFootImuPrintMs = millis();
            const RawIMUData lf = leftFootIMU.read();
            const RawIMUData rf = rightFootIMU.read();
            Serial.printf("[FootIMU/Bench] L ax=%6d az=%6d gx=%6d %s | "
                          "R ax=%6d az=%6d gx=%6d %s\n",
                          lf.accel_x, lf.accel_z, lf.gyro_x, lf.valid ? "OK" : "BAD",
                          rf.accel_x, rf.accel_z, rf.gyro_x, rf.valid ? "OK" : "BAD");
        }
    }
#endif
}
