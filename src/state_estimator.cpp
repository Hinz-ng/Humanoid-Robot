// =============================================================================
// state_estimator.cpp
// Purpose : Complementary filter implementation.
//
// AXIS CONVENTION (sensor frame → robot frame transform applied in update()):
//   Pitch  : forward/back tilt. Positive = leaning forward.
//   Roll   : left/right tilt.   Positive = leaning right.
//
// PIPELINE (one call to update()):
//   RawIMUData
//     → [Calibration accumulate / block]
//     → _scaleRaw()       : LSB → g and rad/s, bias subtract, deadband
//     → _applyAccelLPF()  : IIR smooth accel before tilt estimate
//     → _accelAngles()    : atan2 tilt reference
//     → _accelSanityWeight(): disable accel during high-g events
//     → Complementary blend
//     → IMUState
//
// COMMON MISTAKES:
//   1. Passing raw LSB directly to atan2 — must divide by accel_scale first.
//   2. Forgetting to convert gyro from deg/s to rad/s before integrating.
//   3. dt in milliseconds instead of seconds — angles grow 1000× too fast.
//   4. Wrong axis for pitch vs roll — swap gx_rs/gy_rs in blend if needed.
//   5. Skipping bias calibration — causes slow drift even at rest.
// =============================================================================

#include "state_estimator.h"
#include <math.h>   // atan2f, sqrtf, fabsf

StateEstimator::StateEstimator(FilterConfig cfg) : _cfg(cfg) {}

// ---------------------------------------------------------------------------
void StateEstimator::reset() {
    _pitch        = 0.0f;
    _roll         = 0.0f;
    _seeded       = false;
    _accelSeeded  = false;
    _ax_filt = _ay_filt = _az_filt = 0.0f;
    _calibState   = CALIB_COLLECTING;
    _calibCount   = 0;
    _gyroSumX = _gyroSumY = _gyroSumZ = 0;
    _state        = {};
}

// ---------------------------------------------------------------------------
// Stage 1 — Scale raw LSB → physical units + bias subtract + deadband
// ---------------------------------------------------------------------------
void StateEstimator::_scaleRaw(const RawIMUData& raw,
                                float& ax_g,  float& ay_g,  float& az_g,
                                float& gx_rs, float& gy_rs, float& gz_rs) const {
    // Accelerometer: counts → g
    ax_g = (float)raw.accel_x / _cfg.accel_scale;
    ay_g = (float)raw.accel_y / _cfg.accel_scale;
    az_g = (float)raw.accel_z / _cfg.accel_scale;

    // Gyroscope: counts → deg/s → rad/s, with bias subtraction
    // DEG_TO_RAD is defined in Arduino.h
    gx_rs = ((float)raw.gyro_x - _cfg.gyro_bias_x) / _cfg.gyro_scale * DEG_TO_RAD;
    gy_rs = ((float)raw.gyro_y - _cfg.gyro_bias_y) / _cfg.gyro_scale * DEG_TO_RAD;
    gz_rs = ((float)raw.gyro_z - _cfg.gyro_bias_z) / _cfg.gyro_scale * DEG_TO_RAD;

    // Deadband: clamp near-zero gyro noise to exactly zero.
    // This prevents the integrator from accumulating sub-threshold jitter at rest.
    const float db = _cfg.gyro_deadband_rs;
    if (fabsf(gx_rs) < db) gx_rs = 0.0f;
    if (fabsf(gy_rs) < db) gy_rs = 0.0f;
    if (fabsf(gz_rs) < db) gz_rs = 0.0f;
}

// ---------------------------------------------------------------------------
// Stage 2a — Accelerometer IIR low-pass filter
// Smooths accel before it feeds atan2. Suppresses servo/footstrike vibration.
// _accelSeeded ensures the first sample doesn't get blended with a zero state.
// ---------------------------------------------------------------------------
void StateEstimator::_applyAccelLPF(float ax_g,  float ay_g,  float az_g,
                                     float& ax_f, float& ay_f, float& az_f) {
    if (!_accelSeeded) {
        // First call: seed with raw value to avoid startup transient.
        _ax_filt = ax_g;
        _ay_filt = ay_g;
        _az_filt = az_g;
        _accelSeeded = true;
    } else {
        const float b = _cfg.accel_lpf_beta;
        _ax_filt = b * _ax_filt + (1.0f - b) * ax_g;
        _ay_filt = b * _ay_filt + (1.0f - b) * ay_g;
        _az_filt = b * _az_filt + (1.0f - b) * az_g;
    }
    ax_f = _ax_filt;
    ay_f = _ay_filt;
    az_f = _az_filt;
}

// ---------------------------------------------------------------------------
// Stage 2b — Accel-only tilt estimate (long-term correction reference)
// ---------------------------------------------------------------------------
void StateEstimator::_accelAngles(float ax_g, float ay_g, float az_g,
                                   float& accelPitch, float& accelRoll) const {
    accelPitch = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g));
    accelRoll  = atan2f( ay_g, az_g);
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Stage 2c — Accel sanity weight (soft)
//
// Returns 1.0 when accel magnitude is within the inner trust band (quasi-static).
// Linearly decreases to 0.0 at the outer rejection band (high-g event).
// This prevents the sharp transition that allowed wrong-sign gyro to integrate
// unchecked when the binary version dropped weight to 0.0 instantly.
//
//   |weight
// 1 |------\
//   |       \
// 0 |________\___  → deviation from 1g
//   0  inner outer
// ---------------------------------------------------------------------------
float StateEstimator::_accelSanityWeight(float ax_g, float ay_g, float az_g) const {
    float mag       = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
    float deviation = fabsf(mag - 1.0f);

    // inner: full trust. outer: zero trust. Linear ramp between.
    const float inner = _cfg.accel_sanity_margin * 0.5f;  // 0.075g by default
    const float outer = _cfg.accel_sanity_margin;          // 0.150g by default

    if (deviation <= inner) return 1.0f;
    if (deviation >= outer) return 0.0f;
    return 1.0f - (deviation - inner) / (outer - inner);
}

// ---------------------------------------------------------------------------
// update() — main entry point, called every loop tick
// ---------------------------------------------------------------------------
IMUState StateEstimator::update(const RawIMUData& raw, float dt) {

    if (!raw.valid) {
        _state.valid = false;
        return _state;
    }

    // -----------------------------------------------------------------------
    // CALIBRATION PHASE
    // Accumulate raw gyro LSB counts into integer sums (no float error).
    // Integration is blocked until CALIB_SAMPLES are collected.
    // -----------------------------------------------------------------------
    if (_calibState == CALIB_COLLECTING) {
        _gyroSumX += raw.gyro_x;
        _gyroSumY += raw.gyro_y;
        _gyroSumZ += raw.gyro_z;
        _calibCount++;

        if (_calibCount >= FilterConfig::CALIB_SAMPLES) {
            // Compute mean bias in raw LSB and store into config.
            _cfg.gyro_bias_x = (float)_gyroSumX / (float)FilterConfig::CALIB_SAMPLES;
            _cfg.gyro_bias_y = (float)_gyroSumY / (float)FilterConfig::CALIB_SAMPLES;
            _cfg.gyro_bias_z = (float)_gyroSumZ / (float)FilterConfig::CALIB_SAMPLES;
            _calibState = CALIB_DONE;
            // Note: Serial.printf intentionally not called here.
            // Caller can check isCalibrated() and log if desired.
        }

        // Return invalid state — do not feed un-calibrated data to the filter.
        _state.valid = false;
        return _state;
    }

    // -----------------------------------------------------------------------
    // ESTIMATION PHASE
    // -----------------------------------------------------------------------

    // dt sanity guard: reject stale or implausible timesteps.
    if (dt <= 0.0f || dt > 0.05f) {
        _state.valid = false;
        return _state;
    }

    // --- Stage 1: Scale + bias subtract + deadband ---
    float ax_g, ay_g, az_g, gx_rs, gy_rs, gz_rs;
    _scaleRaw(raw, ax_g, ay_g, az_g, gx_rs, gy_rs, gz_rs);

    // --- Sensor frame → Robot frame ---
    // Axis rename: sensor (Zs=fwd, Xs=up, Ys=left) → robot (Xr=fwd, Zr=up, Yr=left).
    // Sign corrections applied here — do NOT move them into _scaleRaw() because
    // that would also flip the reported rates in IMUState (balance controller uses them).
    // AFTER — standard Z=up, X=forward: no axis permutation, sign correction only.
    // IMU mounted with Z=up, X=forward (standard orientation).
    // Accel: az≈+1g upright — sanity check and atan2 formulas are correct as-is.
    // Gyro:  gx = roll rate (rotation about X), gy = pitch rate (rotation about Y).
    // If the estimate overshoots/inverts, flip the corresponding sign in FilterConfig.
    gx_rs *= _cfg.roll_gyro_sign;    // ±1: rotation about forward(X) → roll
    gy_rs *= _cfg.pitch_gyro_sign;   // ±1: rotation about lateral(Y) → pitch
    // gz_rs (yaw rate) is not used by this filter.
   
   
    // --- Stage 2a: Accel IIR low-pass ---
    float ax_f, ay_f, az_f;
    _applyAccelLPF(ax_g, ay_g, az_g, ax_f, ay_f, az_f);

    // --- Stage 2b: Accel tilt angles ---
    float accelPitch, accelRoll;
    _accelAngles(ax_f, ay_f, az_f, accelPitch, accelRoll);

    // --- Stage 2c: Sanity weight (0.0 or 1.0) ---
    float w = _accelSanityWeight(ax_f, ay_f, az_f);
    // Scale the accel correction term by the sanity weight.
    // w=1: normal blend. w=0: pure gyro, no accel correction this tick.
    float accelBlend = (1.0f - _cfg.alpha) * w;

    // --- Stage 3: Complementary blend ---
    if (!_seeded) {
        _pitch  = accelPitch;
        _roll   = accelRoll;
        _seeded = true;
    } else {
        float gyroPitch = _pitch + gy_rs * dt;
        float gyroRoll  = _roll  + gx_rs * dt;
        _pitch = _cfg.alpha * gyroPitch + accelBlend * accelPitch;
        _roll  = _cfg.alpha * gyroRoll  + accelBlend * accelRoll;
    }

    _state.pitch     = _pitch;
    _state.roll      = _roll;
    _state.pitchRate = gy_rs;
    _state.rollRate  = gx_rs;
    _state.valid     = true;

    return _state;
}