// =============================================================================
// imu.h
// Purpose : Declare the raw IMU data struct and the two public functions.
// Scope   : Raw read only. No filtering, no fusion, no orientation math.
// Next step: Pass RawIMUData into a ComplementaryFilter (separate file).
// =============================================================================

#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

// ---------------------------------------------------------------------------
// RawIMUData
// Holds one snapshot of accelerometer + gyroscope raw 16-bit ADC counts.
// Units are NOT converted here — conversion happens in the filter layer.
//   Accelerometer LSB sensitivity : 16384 LSB/g  (±2g default range)
//   Gyroscope LSB sensitivity     : 16.4  LSB/°/s (±2000°/s default range)
// ---------------------------------------------------------------------------
struct RawIMUData {
    int16_t accel_x;  // Accelerometer X axis (raw ADC count)
    int16_t accel_y;  // Accelerometer Y axis
    int16_t accel_z;  // Accelerometer Z axis

    int16_t gyro_x;   // Gyroscope X axis (raw ADC count)
    int16_t gyro_y;   // Gyroscope Y axis
    int16_t gyro_z;   // Gyroscope Z axis

    bool valid;        // false if the last read failed (I2C error, not init'd, etc.)
};

// ---------------------------------------------------------------------------
// IMU_init()
// Call once in setup().
// Both the IMU and PCA9685 share the Wire bus on GPIO 8/9.
// Wire.begin(8,9) is called by ServoControl::init() before IMU_init().
// The BMI160Gen library re-calls Wire.begin() internally; on ESP32 this
// is harmless when the bus is already running on the correct pins.

// Returns true if the BMI160 was found and configured successfully.
// Returns false if the sensor did not respond (check wiring / SA0 pin).
// ---------------------------------------------------------------------------
bool IMU_init();

// ---------------------------------------------------------------------------
// IMU_update()
// Call every loop iteration. Internally rate-limits prints to ~100 Hz.
// Reads the sensor and, when the print interval has elapsed, writes one
// formatted line to Serial.
//
// Returns the latest RawIMUData snapshot (valid=false if sensor not ready).
// The caller can ignore the return value at this stage — it is provided so
// the complementary filter step can simply call IMU_update() and pass the
// result straight into the filter without refactoring this function.
// ---------------------------------------------------------------------------
RawIMUData IMU_update();

#endif // IMU_H