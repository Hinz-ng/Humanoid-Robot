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
    bool  valid;      // false if the estimator has not been seeded yet (first tick).
};

// ---------------------------------------------------------------------------
// FilterConfig — all tuneable parameters in one place.
// Change alpha here; change bias offsets after a calibration run.
// ---------------------------------------------------------------------------
struct FilterConfig {
    // Complementary filter blend coefficient.
    //   α = 0.995 → recommended starting point at 400 Hz.
    //   Increase toward 1.0 → trust gyro more (smoother, drifts more).
    //   Decrease toward 0.9 → trust accel more (less drift, noisier).
    //   Rule of thumb: τ = α·dt / (1-α).  At 400Hz, α=0.995 → τ ≈ 0.5 s.
    float alpha = 0.995f;

    // Gyroscope bias offsets in raw LSB counts.
    // Measure by averaging 1000 samples while the robot is perfectly still.
    // Default 0 = no correction. Tune these before relying on pitchRate.
    float gyro_bias_x = 0.0f;  // raw LSB
    float gyro_bias_y = 0.0f;  // raw LSB
    float gyro_bias_z = 0.0f;  // raw LSB

    // Sensor scaling — must match BMI160 range configuration.
    // Default ranges (set by BMI160Gen library on init):
    //   Accel ±2g    → 16384.0 LSB/g
    //   Gyro  ±2000°/s → 16.4  LSB/°/s
    // If you change the range via BMI160.setGyroRange() etc., update these.
    float accel_scale = 16384.0f;  // LSB per g
    float gyro_scale  = 16.4f;     // LSB per deg/s
};

// ---------------------------------------------------------------------------
// StateEstimator — owns the filter state and config.
// Usage:
//   StateEstimator estimator;                // uses default FilterConfig
//   estimator.reset();                       // call once after IMU_init()
//   IMUState s = estimator.update(raw, dt);  // call every loop tick
// ---------------------------------------------------------------------------
class StateEstimator {
public:
    // Construct with optional custom config.
    explicit StateEstimator(FilterConfig cfg = FilterConfig());

    // Reset internal state to zero. Call once in setup() after IMU_init().
    void reset();

    // Update the filter with one new IMU sample.
    //   raw : output of IMU_update()
    //   dt  : elapsed time since last call, in SECONDS (compute from micros())
    // Returns the latest estimated orientation.
    // If raw.valid is false, returns the last known state with valid=false.
    IMUState update(const RawIMUData& raw, float dt);

    // Read the last computed state without triggering a new update.
    IMUState getState() const { return _state; }

    // Allow runtime config changes (e.g., alpha tuning from UI later).
    void setConfig(const FilterConfig& cfg) { _cfg = cfg; }
    const FilterConfig& getConfig() const   { return _cfg; }

private:
    FilterConfig _cfg;

    // Internal filter state (radians)
    float _pitch = 0.0f;
    float _roll  = 0.0f;
    bool  _seeded = false;   // false until first valid sample arrives

    IMUState _state = {};

    // --- Stage 1: Scale raw LSB counts to physical units ---
    // ax_g, ay_g, az_g  → units of g  (1g = 9.81 m/s²)
    // gx_rs, gy_rs, gz_rs → rad/s
    void _scaleRaw(const RawIMUData& raw,
                   float& ax_g,  float& ay_g,  float& az_g,
                   float& gx_rs, float& gy_rs, float& gz_rs) const;

    // --- Stage 2: Compute tilt angle from accelerometer alone ---
    // Valid only when robot is in quasi-static motion (no large linear accel).
    // Used as the long-term correction reference.
    void _accelAngles(float ax_g, float ay_g, float az_g,
                      float& accelPitch, float& accelRoll) const;
};

#endif // STATE_ESTIMATOR_H