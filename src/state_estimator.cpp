// =============================================================================
// state_estimator.cpp
// Purpose : Complementary filter implementation.
//
// AXIS CONVENTION (BMI160 mounted flat, Z pointing up, X pointing forward):
//   Pitch  : rotation about Y axis. Leaning forward → positive pitch.
//            Gyro axis: gy. Accel formula: atan2(-ax, az).
//   Roll   : rotation about X axis. Leaning right  → positive roll.
//            Gyro axis: gx. Accel formula: atan2(ay,  az).
//
// If your IMU is mounted differently (rotated or flipped), adjust the signs
// in _accelAngles() and the gyro assignments in update(). The filter math
// itself does not change — only the axis mapping.
//
// COMMON MISTAKES:
//   1. Passing raw LSB directly to atan2 — must divide by accel_scale first.
//   2. Forgetting to convert gyro from deg/s to rad/s before integrating.
//   3. dt in milliseconds instead of seconds — angles grow 1000x too fast.
//   4. Wrong axis for pitch vs roll (swap gx/gy if pitch and roll are inverted).
//   5. Not subtracting gyro bias — causes slow angle drift even when still.
// =============================================================================

#include "state_estimator.h"
#include <math.h>   // atan2f, sqrtf

// ---------------------------------------------------------------------------
StateEstimator::StateEstimator(FilterConfig cfg) : _cfg(cfg) {}

// ---------------------------------------------------------------------------
void StateEstimator::reset() {
    _pitch   = 0.0f;
    _roll    = 0.0f;
    _seeded  = false;
    _state   = {};
}

// ---------------------------------------------------------------------------
// Stage 1 — Scale raw LSB → physical units
// ---------------------------------------------------------------------------
void StateEstimator::_scaleRaw(const RawIMUData& raw,
                                float& ax_g,  float& ay_g,  float& az_g,
                                float& gx_rs, float& gy_rs, float& gz_rs) const {
    // Convert accelerometer counts to g-units.
    ax_g = (float)(raw.accel_x - 0) / _cfg.accel_scale;   // bias handled in FilterConfig
    ay_g = (float)(raw.accel_y - 0) / _cfg.accel_scale;
    az_g = (float)(raw.accel_z - 0) / _cfg.accel_scale;

    // Convert gyroscope counts to rad/s.
    // Step 1: subtract bias (in raw LSB).
    // Step 2: divide by scale to get deg/s.
    // Step 3: multiply by PI/180 to get rad/s.
    // DEG_TO_RAD is defined in Arduino.h — no local declaration needed.
    gx_rs = ((float)raw.gyro_x - _cfg.gyro_bias_x) / _cfg.gyro_scale * DEG_TO_RAD;
    gy_rs = ((float)raw.gyro_y - _cfg.gyro_bias_y) / _cfg.gyro_scale * DEG_TO_RAD;
    gz_rs = ((float)raw.gyro_z - _cfg.gyro_bias_z) / _cfg.gyro_scale * DEG_TO_RAD;
}

// ---------------------------------------------------------------------------
// Stage 2 — Accelerometer-only tilt estimate
// ---------------------------------------------------------------------------
void StateEstimator::_accelAngles(float ax_g, float ay_g, float az_g,
                                   float& accelPitch, float& accelRoll) const {
    // atan2 is used (not atan) so the result is correct in all quadrants.
    //
    // Pitch: tilt about Y axis.
    //   When robot leans forward, ax becomes more negative (gravity pulls back).
    //   atan2(-ax, az) gives positive angle for forward lean.
    //   If your robot pitches the wrong direction, negate ax here.
    accelPitch = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g));

    // Roll: tilt about X axis.
    //   When robot leans right, ay becomes positive.
    //   atan2(ay, az) gives positive angle for right lean.
    //   If your robot rolls the wrong direction, negate ay here.
    accelRoll  = atan2f( ay_g, az_g);
}

// ---------------------------------------------------------------------------
// Stage 3 — Complementary filter blend
// ---------------------------------------------------------------------------
IMUState StateEstimator::update(const RawIMUData& raw, float dt) {

    // Guard: propagate invalid data cleanly without corrupting state.
    if (!raw.valid) {
        _state.valid = false;
        return _state;
    }

    // Guard: clamp dt to a sane range.
    // If loop stalls (e.g. I2C blocking) dt could be huge → angle jumps.
    // 0.05s cap means even at 20 Hz the filter won't blow up.
    if (dt <= 0.0f || dt > 0.05f) {
        _state.valid = false;
        return _state;
    }

    // --- Stage 1: Scale ---
    float ax_g, ay_g, az_g, gx_rs, gy_rs, gz_rs;
    _scaleRaw(raw, ax_g, ay_g, az_g, gx_rs, gy_rs, gz_rs);

    // ---------------------------------------------------------------------------
// Sensor frame → Robot frame transform
//
// Sensor mounting:
//   Zs → forward
//   Xs → up
//   Ys → left
//
// Robot frame expected by filter:
//   Xr → forward
//   Yr → left
//   Zr → up
//
// Mapping:
//   Xr = Zs
//   Yr = Ys
//   Zr = Xs
// ---------------------------------------------------------------------------

// Save original sensor-frame values
float ax_s = ax_g;
float ay_s = ay_g;
float az_s = az_g;

float gx_s = gx_rs;
float gy_s = gy_rs;
float gz_s = gz_rs;

// Transform to robot frame

ax_g = az_s;     // forward
ay_g = -ay_s;    // left (sign flipped)
az_g = ax_s;     // up

gx_rs = gz_s;
gy_rs = -gy_s;
gz_rs = gx_s;
    // --- Stage 2: Accel angles ---
    float accelPitch, accelRoll;
    _accelAngles(ax_g, ay_g, az_g, accelPitch, accelRoll);

    // --- Stage 3: Complementary blend ---
    if (!_seeded) {
        // First valid sample — seed the filter with accel angle so it
        // doesn't start at zero and take many seconds to converge.
        _pitch  = accelPitch;
        _roll   = accelRoll;
        _seeded = true;
    } else {
        // Gyro integration: extrapolate angle forward in time.
        // gy drives pitch (forward/back). gx drives roll (left/right).
        // If pitch and roll appear swapped, exchange gx_rs and gy_rs here.
        float gyroPitch = _pitch + gy_rs * dt;
        float gyroRoll  = _roll  + gx_rs * dt;

        // Blend: trust gyro for fast dynamics, trust accel for slow correction.
        _pitch = _cfg.alpha * gyroPitch + (1.0f - _cfg.alpha) * accelPitch;
        _roll  = _cfg.alpha * gyroRoll  + (1.0f - _cfg.alpha) * accelRoll;
    }

    // --- Populate output struct ---
    _state.pitch     = _pitch;
    _state.roll      = _roll;
    _state.pitchRate = gy_rs;   // already in rad/s, bias corrected
    _state.rollRate  = gx_rs;
    _state.valid     = true;

    return _state;
}