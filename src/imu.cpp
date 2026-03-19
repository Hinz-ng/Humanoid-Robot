// =============================================================================
// imu.cpp
// Purpose : Initialize and read the BMI160 IMU over I2C.
// Scope   : Raw data only. No filtering. No orientation math.
//
// Hardware assumptions:
//   - BMI160 connected via I2C on GPIO 8 (SDA) / GPIO 9 (SCL)
//   - SA0 pin tied to GND  →  I2C address = 0x68
//
// Output: Raw values are returned from IMU_update() for the caller to
//         broadcast. Serial printing removed — UI panel handles display.
// =============================================================================

#include "imu.h"
#include <BMI160Gen.h>

// Both PCA9685 and BMI160 share the SAME Wire bus on GPIO 8 (SDA) / 9 (SCL).
// Wire.begin(8,9) is called once in ServoControl::init(). The BMI160Gen
// library calls Wire.begin() internally — on ESP32 this is a no-op when the
// bus is already initialized, so the pin configuration is preserved.
static const int IMU_SDA_PIN = 8;
static const int IMU_SCL_PIN = 9;
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
    // pass -1 as interrupt pin: BMI160Gen checks (0 <= arg2) before
// calling digitalPinToInterrupt(), so -1 suppresses registration entirely.
// The BMI160 is polled at 400 Hz via getMotion6(); no data-ready interrupt is used.
    bool ok = BMI160.begin(BMI160GenClass::I2C_MODE, BMI160_I2C_ADDR, /*intr_pin=*/-1);
    if (!ok) {
        Serial.println("[IMU] ERROR: BMI160 not found. Check wiring and SA0 pin.");
        Serial.printf("[IMU]   SDA: GPIO %d  SCL: GPIO %d\n", IMU_SDA_PIN, IMU_SCL_PIN);
        Serial.printf("[IMU]   Expected I2C address: 0x%02X\n", BMI160_I2C_ADDR);
        _initialized = false;
        return false;
    }

    // AFTER:
    // Configure sensor ODR and range explicitly.
    // BMI160 default after begin() is 100 Hz — we need 400 Hz.
    // Range defaults match FilterConfig: ±2g / ±2000 deg/s.
    BMI160.setFullScaleAccelRange(BMI160_ACCEL_RANGE_2G);    // 16384 LSB/g
    BMI160.setFullScaleGyroRange(BMI160_GYRO_RANGE_2000);    // 16.4 LSB/(deg/s)
    BMI160.setAccelRate(BMI160_ACCEL_RATE_400HZ);            // 400 Hz ODR
    BMI160.setGyroRate(BMI160_GYRO_RATE_400HZ);              // 400 Hz ODR

    _initialized = true;
    _latestData.valid = false;

    Serial.println("[IMU] BMI160 initialized successfully.");
    Serial.printf("[IMU]   SDA: GPIO %d  SCL: GPIO %d  Addr: 0x%02X\n",
                  IMU_SDA_PIN, IMU_SCL_PIN, BMI160_I2C_ADDR);
    Serial.println("[IMU]   ODR: 400 Hz | Accel: +-2g | Gyro: +-2000 deg/s");
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

    // AFTER — single I2C burst, halves bus traffic vs two separate reads:
    int16_t ax, ay, az, gx, gy, gz;
    BMI160.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    _latestData.accel_x = ax;
    _latestData.accel_y = ay;
    _latestData.accel_z = az;
    _latestData.gyro_x  = gx;
    _latestData.gyro_y  = gy;
    _latestData.gyro_z  = gz;
    _latestData.valid   = true;

    return _latestData;
}