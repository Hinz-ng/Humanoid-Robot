// =============================================================================
// FILE:    lsm6dsv_driver.h
// MODULE:  imu_driver
// LAYER:   1 — raw I/O only
//
// PURPOSE:
//   Bare-metal SPI driver for one LSM6DSV / LSM6DSV16X / LSM6DSV16BX IMU.
//   Returns RawIMUData (shared struct with BMI160; defined in imu.h).
//   Knows nothing about feet, robot frames, filtering, or fusion — those
//   live in StateEstimator (Layer 2) and foot_contact (Layer 3.5, F2+).
//
// USAGE (per-instance):
//   1. Construct with a chip-select pin and a pre-initialized SPIClass*.
//   2. Call begin() once — verifies WHO_AM_I, configures ODR + range.
//   3. Call read() each tick — returns RawIMUData with valid=false on failure.
//
// DEPENDENCIES:
//   imu.h — RawIMUData struct.
//   SPI library — caller owns SPIClass and has called .begin(SCK, MISO, MOSI).
//
// LAST CHANGED: 2026-05-09 | Hinz | Phase F0 driver bench-up
// =============================================================================

#ifndef LSM6DSV_DRIVER_H
#define LSM6DSV_DRIVER_H

#include <Arduino.h>
#include <SPI.h>
#include "imu.h"   // RawIMUData

// ---------------------------------------------------------------------------
//  Register map (subset — only what F0 needs)
//  Source: ST AN5763 (LSM6DSV16X application note), §6 register description.
// ---------------------------------------------------------------------------
static constexpr uint8_t LSM6DSV_REG_WHO_AM_I = 0x0F;
static constexpr uint8_t LSM6DSV_REG_CTRL1    = 0x10;  // accel ODR + FS + power mode
static constexpr uint8_t LSM6DSV_REG_CTRL2    = 0x11;  // gyro ODR + FS + power mode
static constexpr uint8_t LSM6DSV_REG_CTRL3    = 0x12;  // BDU, IF_INC, sw_reset
static constexpr uint8_t LSM6DSV_REG_OUTX_L_G = 0x22;  // gyro X low byte (start of 6-byte burst)
static constexpr uint8_t LSM6DSV_REG_OUTX_L_A = 0x28;  // accel X low byte (start of 6-byte burst)

// WHO_AM_I expected values. The driver accepts either — both match the chips
// we may receive from the same supplier. If you see a different value, the
// part is a different variant and the register map below may differ in the
// fusion-block region (CTRL10+) — but the basic accel/gyro registers used
// here are stable across the whole DSV family.
static constexpr uint8_t LSM6DSV_WHOAMI_DSV   = 0x70;  // LSM6DSV / LSM6DSV16X
static constexpr uint8_t LSM6DSV_WHOAMI_DSVBX = 0x71;  // LSM6DSV16BX

// CTRL1 / CTRL2 — combined ODR + FS in a single write.
// Bits[7:4] = ODR, bits[3:2] = FS, bits[1:0] = power mode (00 = HP).
static constexpr uint8_t LSM6DSV_ODR_416HZ      = 0x60;  // bits[7:4] = 0110
static constexpr uint8_t LSM6DSV_ACCEL_FS_2G    = 0x00;  // bits[3:2] = 00
static constexpr uint8_t LSM6DSV_GYRO_FS_2000   = 0x0C;  // bits[3:2] = 11

// SPI read/write address bit. LSM6DSV uses MSB=1 for read, 0 for write.
static constexpr uint8_t LSM6DSV_SPI_READ_BIT = 0x80;

// ---------------------------------------------------------------------------
class LSM6DSVDriver {
public:
    // cs_pin       : chip-select GPIO. Software-toggled by the driver.
    // spi          : pre-initialized SPIClass instance (caller owns + .begin()s).
    // spi_clock_hz : SPI clock. Start at 1 MHz, raise after cable-noise check.
    LSM6DSVDriver(uint8_t cs_pin, SPIClass* spi, uint32_t spi_clock_hz = 1000000);

    // Reset, verify WHO_AM_I, configure ODR + range. Returns true on success.
    // On failure, prints the reason and leaves _initialized=false.
    bool begin(const char* label = "lsm6dsv");

    // One-shot raw read. Returns RawIMUData with valid=false on any error.
    RawIMUData read();

    bool isInitialized() const { return _initialized; }

    // Diagnostic: read WHO_AM_I directly. Used for connectivity test.
    // Returns 0xFF if SPI is not responding (typical for wrong CS / no CLK).
    uint8_t readWhoAmI();

private:
    uint8_t   _cs;
    SPIClass* _spi;
    uint32_t  _clk_hz;
    bool      _initialized = false;

    void    _select();                          // CS LOW + beginTransaction
    void    _deselect();                        // endTransaction + CS HIGH
    void    _writeReg(uint8_t reg, uint8_t val);
    uint8_t _readReg(uint8_t reg);
    void    _readBurst(uint8_t reg, uint8_t* buf, size_t len);
};

#endif // LSM6DSV_DRIVER_H