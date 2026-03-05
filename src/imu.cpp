// =============================================================================
// imu.cpp
// Purpose : Initialize and read the BMI160 IMU over I2C.
// Scope   : Raw data only. No filtering. No orientation math.
//
// Hardware assumptions:
//   - BMI160 connected via I2C on GPIO 17 (SDA) / GPIO 18 (SCL)
//   - SA0 pin tied to GND  →  I2C address = 0x68
//
// Output: Raw values are returned from IMU_update() for the caller to
//         broadcast. Serial printing removed — UI panel handles display.
// =============================================================================

#include "imu.h"
#include <BMI160Gen.h>

static const int IMU_SDA_PIN = 8;   // shared I2C bus with PCA9685
static const int IMU_SCL_PIN = 9;   // servoController.init() calls Wire.begin(8,9)
static const uint8_t BMI160_I2C_ADDR  = 0x68;

static bool       _initialized = false;
static RawIMUData _latestData  = {};

// ---------------------------------------------------------------------------
// IMU_init()
// Call once in setup(). Order relative to servoController.init() does not
// matter — this library manages its own Wire bus using the SDA pin number.
// ---------------------------------------------------------------------------
bool IMU_init() {
    // Wire.begin(8, 9) is already called by servoController.init() before
    // IMU_init(). The library reuses that bus via the SDA pin number arg.
    bool ok = BMI160.begin(BMI160GenClass::I2C_MODE, BMI160_I2C_ADDR, IMU_SDA_PIN);
    if (!ok) {
        Serial.println("[IMU] ERROR: BMI160 not found. Check wiring and SA0 pin.");
        Serial.printf("[IMU]   SDA: GPIO %d  SCL: GPIO %d\n", IMU_SDA_PIN, IMU_SCL_PIN);
        Serial.printf("[IMU]   Expected I2C address: 0x%02X\n", BMI160_I2C_ADDR);
        _initialized = false;
        return false;
    }

    _initialized = true;
    _latestData.valid = false;

    Serial.println("[IMU] BMI160 initialized successfully.");
    Serial.printf("[IMU]   SDA: GPIO %d  SCL: GPIO %d  Addr: 0x%02X\n",
                  IMU_SDA_PIN, IMU_SCL_PIN, BMI160_I2C_ADDR);
    return true;
}

// ---------------------------------------------------------------------------
// IMU_update()
// Call every loop(). Reads the sensor and returns the latest snapshot.
// Display/broadcast is handled by the caller (WebComm::broadcastIMU).
// ---------------------------------------------------------------------------
RawIMUData IMU_update() {
    if (!_initialized) {
        _latestData.valid = false;
        return _latestData;
    }

    int ax, ay, az, gx, gy, gz;
    BMI160.readAccelerometer(ax, ay, az);
    BMI160.readGyro(gx, gy, gz);

    _latestData.accel_x = (int16_t)ax;
    _latestData.accel_y = (int16_t)ay;
    _latestData.accel_z = (int16_t)az;
    _latestData.gyro_x  = (int16_t)gx;
    _latestData.gyro_y  = (int16_t)gy;
    _latestData.gyro_z  = (int16_t)gz;
    _latestData.valid   = true;

    return _latestData;
}