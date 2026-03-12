// =============================================================================
// state_estimator.h
// Purpose : Complementary filter — fuses BMI160 accel + gyro into orientation.
// Scope   : Pitch, roll, and their rates. No yaw (unobservable without mag).
//
// Architecture position (from balancing framework guide):
//   IMU_update() → RawIMUData → StateEstimator::update(dt) → IMUState
//                                                               ↓
//                                                       Balance Controller
//
// Upgrade path:
//   Replace ComplementaryFilter internals with Madgwick — IMUState interface
//   stays identical so nothing above this layer changes.
//   Add a second IMU: average RawIMUData before passing in, or subclass.
// =============================================================================

#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include <Arduino.h>
#include "imu.h"    // for RawIMUData

// ---------------------------------------------------------------------------
// IMUState — output of the filter. Consumed by the balance controller.
// All angles in RADIANS. All rates in RAD/S.
// ---------------------------------------------------------------------------
struct IMUState {
    float pitch;      // Body tilt forward/back (rad). Positive = leaning forward.
    float roll;       // Body tilt left/right  (rad). Positive = leaning right.
    float pitchRate;  // Angular velocity about pitch axis (rad/s).
    float rollRate;   // Angular velocity about roll  axis (rad/s).
    bool  valid;      // false until calibration finishes + first valid tick.
};

// ---------------------------------------------------------------------------
// FilterConfig — all tuneable parameters in one place.
// ---------------------------------------------------------------------------
struct FilterConfig {
    // --- Complementary filter blend ---
    //   α = 0.995 → recommended starting point at 400 Hz.
    //   τ = α·dt / (1-α). At 400 Hz, α=0.995 → τ ≈ 0.5 s.
    float alpha = 0.990f;

    // --- Gyro bias (raw LSB). Auto-filled by calibrate(). ---
    // Leave at 0 if you want to skip calibration (not recommended).
    float gyro_bias_x = 0.0f;
    float gyro_bias_y = 0.0f;
    float gyro_bias_z = 0.0f;

    // --- Sensor scaling (must match BMI160 range config) ---
    //   Accel ±2g    → 16384.0 LSB/g
    //   Gyro  ±2000°/s → 16.4  LSB/°/s
    float accel_scale = 16384.0f;
    float gyro_scale  = 16.4f;

    // --- Gyro deadband (rad/s after bias subtraction) ---
    // Signals below this are clamped to zero.
    // Default 0.01 rad/s ≈ 0.57 deg/s — below BMI160 noise floor at ±2000°/s.
    // Increase if you see slow angle creep at rest; decrease if fast moves feel sluggish.
    float gyro_deadband_rs = 0.01f;

    // --- Gyro axis signs ---
    float pitch_gyro_sign = 1.0f;
    float roll_gyro_sign  =  1.0f;

    // --- Accelerometer IIR low-pass filter coefficient ---
    //   filtered = accel_lpf_beta * filtered + (1 - accel_lpf_beta) * raw
    //   0.85 at 400 Hz → time constant ≈ 2 ms.
    //   Increase toward 0.95 for noisier/vibration-heavy setups.
    float accel_lpf_beta = 0.70f;

    // --- Accelerometer sanity band (g-units) ---
    // Accel is only trusted for tilt correction when its magnitude is within
    // [1g - margin, 1g + margin]. Outside this band (e.g. footstrike impulse),
    // the accel blend weight drops to zero and the gyro integrates alone.
    float accel_sanity_margin = 0.20f;  // ±0.20g → ~±11.4°

    // --- Still detection (calibration gate) ---
    // Calibration only starts once ALL three gyro axes are below this raw-LSB
    // threshold for STILL_WINDOW_SAMPLES consecutive samples.
    // At ±2000°/s range: 100 LSB ≈ 6 deg/s — clearly "not being carried/shaken".
    // Increase if calibration never starts on a noisy bench; decrease for faster gate.
    static const int16_t STILL_THRESHOLD_LSB   = 100;   // per axis, absolute
    static const int     STILL_WINDOW_SAMPLES  = 50;    // 50 * 2.5ms = 125ms quiet window
    
    // --- Calibration sample count ---
    // Samples collected at boot before integration starts.
    static const int CALIB_SAMPLES = 2000;
};

// ---------------------------------------------------------------------------
// CalibState — tracks where we are in the boot calibration sequence.
// Readable via getCalibState() for UI status display.
// ---------------------------------------------------------------------------
enum CalibState {
    CALIB_WAITING,     // Waiting for robot to be still before accumulating.
    CALIB_COLLECTING,  // Accumulating gyro samples for bias estimation.
    CALIB_DONE         // Bias computed — estimator is live.
};

// ---------------------------------------------------------------------------
// StateEstimator
// ---------------------------------------------------------------------------
class StateEstimator {
public:
    explicit StateEstimator(FilterConfig cfg = FilterConfig());

    // Call once in setup() after IMU_init(). Resets filter + restarts calibration.
    void reset();

    // Feed one raw sample. Returns valid=false while calibrating.
    // dt: seconds since last call (compute from micros() in the caller).
    IMUState update(const RawIMUData& raw, float dt);

    IMUState          getState()     const { return _state; }
    CalibState        getCalibState()const { return _calibState; }
    bool              isCalibrated() const { return _calibState == CALIB_DONE; }

    // Returns 0.0 → 1.0 progress through the calibration window.
    float             getCalibProgress() const {
        if (_calibState == CALIB_DONE) return 1.0f;
        return (float)_calibCount / (float)FilterConfig::CALIB_SAMPLES;
    }

    void              setConfig(const FilterConfig& cfg) { _cfg = cfg; }
    const FilterConfig& getConfig() const { return _cfg; }

private:
    FilterConfig _cfg;

    // --- Filter state ---
    float _pitch   = 0.0f;
    float _roll    = 0.0f;
    bool  _seeded  = false;

    // --- Accel IIR state (seeded on first sample) ---
    float _ax_filt = 0.0f;
    float _ay_filt = 0.0f;
    float _az_filt = 0.0f;
    bool  _accelSeeded = false;

    // --- Calibration state ---
    CalibState _calibState = CALIB_COLLECTING;
    int        _stillCount = 0;   // consecutive still samples seen; resets on motion
    int        _calibCount = 0;
    int32_t    _gyroSumX   = 0;  // int32_t: safe for 2000 * int16_t without overflow
    int32_t    _gyroSumY   = 0;
    int32_t    _gyroSumZ   = 0;

    IMUState _state = {};

    // Pipeline stages — each does exactly one job.
    void _scaleRaw   (const RawIMUData& raw,
                      float& ax_g,  float& ay_g,  float& az_g,
                      float& gx_rs, float& gy_rs, float& gz_rs) const;

    void _applyAccelLPF (float ax_g,  float ay_g,  float az_g,
                         float& ax_f, float& ay_f, float& az_f);

    void _accelAngles(float ax_g, float ay_g, float az_g,
                      float& accelPitch, float& accelRoll) const;

    // Returns the blend weight for accel correction: 0.0 if sanity fails, 1.0 if ok.
    float _accelSanityWeight(float ax_g, float ay_g, float az_g) const;
};

#endif // STATE_ESTIMATOR_H 