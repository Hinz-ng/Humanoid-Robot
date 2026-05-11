// =============================================================================
// FILE:    lsm6dsv_driver.cpp
// LAYER:   1 — raw SPI I/O only
//
// LAST CHANGED: 2026-05-09 | Hinz | Phase F0 driver bench-up
// =============================================================================

#include "lsm6dsv_driver.h"

LSM6DSVDriver::LSM6DSVDriver(uint8_t cs_pin, SPIClass* spi, uint32_t spi_clock_hz)
    : _cs(cs_pin), _spi(spi), _clk_hz(spi_clock_hz) {}

// ---------------------------------------------------------------------------
//  begin() — software reset, WHO_AM_I check, configure registers.
// ---------------------------------------------------------------------------
bool LSM6DSVDriver::begin(const char* label) {
    if (_spi == nullptr) {
        Serial.printf("[%s] ERROR: null SPIClass pointer\n", label);
        return false;
    }

    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);   // deselected by default

    // Allow the chip to power up before first SPI transaction.
    // Datasheet §3.3: 35 ms boot time after VDD reaches valid level.
    delay(40);

    // Identify the chip. If WHO_AM_I returns 0xFF or 0x00, SPI wiring is wrong.
    const uint8_t whoami = readWhoAmI();
    if (whoami != LSM6DSV_WHOAMI_DSV && whoami != LSM6DSV_WHOAMI_DSVBX) {
        Serial.printf("[%s] ERROR: WHO_AM_I=0x%02X (expected 0x70 or 0x71). "
                      "Check CS/SCK/MOSI/MISO wiring and 3.3V supply.\n",
                      label, whoami);
        return false;
    }
    Serial.printf("[%s] WHO_AM_I=0x%02X — chip detected.\n", label, whoami);

    _writeReg(LSM6DSV_REG_CTRL3, 0x01);   // SW_RESET
    delay(50);                            // matches BB-8 — 50 ms is datasheet max
                                          // (previous 10 ms is on the edge)

    // Verify reset cleared. If not, fail loudly — this is a strong signal of
    // SPI write failure (wrong mode, broken MOSI, wrong CS).
    const uint8_t ctrl3_post = _readReg(LSM6DSV_REG_CTRL3);
    if (ctrl3_post & 0x01) {
        Serial.printf("[%s] ERROR: SW_RESET did not clear (CTRL3=0x%02X). "
                      "SPI write path is broken — check MODE/CS/MOSI.\n",
                      label, ctrl3_post);
        return false;
    }

    _writeReg(LSM6DSV_REG_CTRL3, 0x44);   // IF_INC + BDU
    _writeReg(LSM6DSV_REG_CTRL1, LSM6DSV_ODR_416HZ | LSM6DSV_ACCEL_FS_2G);
    _writeReg(LSM6DSV_REG_CTRL2, LSM6DSV_ODR_416HZ | LSM6DSV_GYRO_FS_2000);
    delay(20);

    // Readback verification — exactly the BB-8 pattern. Refuse to proceed if
    // CTRL1/CTRL2 don't match what we wrote: it means writes are silently
    // failing, and read() will just return zeros.
    const uint8_t ctrl1v = _readReg(LSM6DSV_REG_CTRL1);
    const uint8_t ctrl2v = _readReg(LSM6DSV_REG_CTRL2);
    const uint8_t want1  = LSM6DSV_ODR_416HZ | LSM6DSV_ACCEL_FS_2G;
    const uint8_t want2  = LSM6DSV_ODR_416HZ | LSM6DSV_GYRO_FS_2000;
    Serial.printf("[%s] CTRL1=0x%02X (want 0x%02X)  CTRL2=0x%02X (want 0x%02X)\n",
                  label, ctrl1v, want1, ctrl2v, want2);
    if (ctrl1v != want1 || ctrl2v != want2) {
        Serial.printf("[%s] ERROR: control register writes did not stick. "
                      "SPI mode or wiring is wrong.\n", label);
        return false;
    }

    _initialized = true;
    Serial.printf("[%s] Configured: ±2g, ±2000 dps, 416 Hz HP, SPI %lu Hz.\n",
                  label, (unsigned long)_clk_hz);

    // Sanity: read a sample and confirm it's not all-zero. This is a hard
    // failure — if the first sample reads as all-zero or invalid, it means
    // the SPI read path is broken (or the part has no power). Do not proceed.
    // Future-you with no serial monitor will appreciate this diagnostic.
    RawIMUData sample = read();
    if (!sample.valid ||
        (sample.accel_x == 0 && sample.accel_y == 0 && sample.accel_z == 0 &&
         sample.gyro_x  == 0 && sample.gyro_y  == 0 && sample.gyro_z  == 0)) {
        Serial.printf("[%s] ERROR: first sample is all-zero. "
                      "Accel should read ~16384 LSB on the gravity axis at rest.\n",
                      label);
        _initialized = false;
        return false;
    }

    return true;
}

// ---------------------------------------------------------------------------
//  read() — burst-read accel + gyro into RawIMUData.
//
//  Two separate 6-byte bursts (gyro at 0x22, accel at 0x28) — auto-increment
//  inside each burst. Reading both in one 12-byte burst would also work since
//  the registers are contiguous (gyro then accel), but doing them separately
//  matches the BMI160 driver pattern and keeps the code easy to skim.
// ---------------------------------------------------------------------------
RawIMUData LSM6DSVDriver::read() {
    RawIMUData data{};
    if (!_initialized) {
        data.valid = false;
        return data;
    }

    uint8_t buf[6];

    _readBurst(LSM6DSV_REG_OUTX_L_G, buf, 6);
    data.gyro_x = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
    data.gyro_y = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
    data.gyro_z = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));

    _readBurst(LSM6DSV_REG_OUTX_L_A, buf, 6);
    data.accel_x = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
    data.accel_y = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
    data.accel_z = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));

    data.valid = true;
    return data;
}

// ---------------------------------------------------------------------------
//  readWhoAmI() — direct register read, no init guard.
//  Used by begin() for connectivity check; safe to call any time.
// ---------------------------------------------------------------------------
uint8_t LSM6DSVDriver::readWhoAmI() {
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
    delayMicroseconds(10);
    return _readReg(LSM6DSV_REG_WHO_AM_I);
}

// ---------------------------------------------------------------------------
//  SPI helpers
// ---------------------------------------------------------------------------
void LSM6DSVDriver::_select() {
    _spi->beginTransaction(SPISettings(_clk_hz, MSBFIRST, SPI_MODE3));
    digitalWrite(_cs, LOW);
}

void LSM6DSVDriver::_deselect() {
    digitalWrite(_cs, HIGH);
    _spi->endTransaction();
}

void LSM6DSVDriver::_writeReg(uint8_t reg, uint8_t val) {
    _select();
    _spi->transfer(reg & 0x7F);   // MSB=0 → write
    _spi->transfer(val);
    _deselect();
}

uint8_t LSM6DSVDriver::_readReg(uint8_t reg) {
    _select();
    _spi->transfer(reg | LSM6DSV_SPI_READ_BIT);
    const uint8_t val = _spi->transfer(0x00);
    _deselect();
    return val;
}

void LSM6DSVDriver::_readBurst(uint8_t reg, uint8_t* buf, size_t len) {
    _select();
    _spi->transfer(reg | LSM6DSV_SPI_READ_BIT);
    for (size_t i = 0; i < len; i++) buf[i] = _spi->transfer(0x00);
    _deselect();
}