// =============================================================================
// FILE:    lsm6dsv_driver.h
// MODULE:  imu_driver
// LAYER:   1 — raw I/O only
//
// PURPOSE:
//   I2C driver for one LSM6DSV / LSM6DSV16X / LSM6DSV16BX IMU.
//   Returns RawIMUData (shared struct with BMI160; defined in imu.h).
//   Knows nothing about feet, robot frames, filtering, or fusion — those
//   live in StateEstimator (Layer 2).
//
// PROTOCOL CHOICE (history):
//   F0 used SPI to avoid I2C address collision with BMI160. That concern only
//   applies on a SHARED bus. By putting the foot IMUs on Wire1 (separate from
//   BMI160/PCA9685 on Wire0), each chip gets a unique 7-bit address via SA0
//   and there is no collision with anything on Wire0.
//
// USAGE (per-instance):
//   1. Caller calls Wire1.begin(SDA, SCL, clock_hz) ONCE before any driver.
//   2. Construct with (Wire1 pointer, SA0 GPIO, desired SA0 level).
//      - If sa0_pin >= 0, the driver drives that GPIO to the requested level
//        in begin(), giving full code control over the I2C address.
//      - If sa0_pin < 0, the caller has hardwired SA0 on the breakout
//        (to VDD or GND); the driver only uses the level to compute the addr.
//   3. Call begin() once per instance — verifies WHO_AM_I, configures
//      ODR + range. Returns false on any I2C/identity failure.
//   4. Call read() each tick — returns RawIMUData with valid=false on failure.
//
// ADDRESSING:
//   Datasheet Table 27. The 7-bit I2C address is 0b1101011x, where x = SA0.
//     SA0 = 0 → 0x6A
//     SA0 = 1 → 0x6B
//   Both addresses are distinct from BMI160 (0x68 / 0x69), so even if you
//   moved everything to Wire0 later there is still no collision.
//
// DEPENDENCIES:
//   Wire.h — TwoWire (Arduino I2C library).
//   imu.h  — RawIMUData struct.
//
// LAST CHANGED: 2026-05-12 | Hinz | F0→F1 protocol switch (SPI → I2C)
// =============================================================================

#ifndef LSM6DSV_DRIVER_H
#define LSM6DSV_DRIVER_H

#include <Arduino.h>
#include <Wire.h>
#include "imu.h"   // RawIMUData

// ---------------------------------------------------------------------------
//  I2C address constants — derived from SA0 pin (datasheet Table 27).
// ---------------------------------------------------------------------------
static constexpr uint8_t LSM6DSV_I2C_ADDR_SA0_LOW  = 0x6A;
static constexpr uint8_t LSM6DSV_I2C_ADDR_SA0_HIGH = 0x6B;

// ---------------------------------------------------------------------------
//  Register map — same chip, same registers. Unchanged from SPI driver.
//  Source: ST AN5763 (LSM6DSV16X application note), §6.
// ---------------------------------------------------------------------------
static constexpr uint8_t LSM6DSV_REG_WHO_AM_I = 0x0F;
static constexpr uint8_t LSM6DSV_REG_CTRL1    = 0x10;  // accel ODR + FS
static constexpr uint8_t LSM6DSV_REG_CTRL2    = 0x11;  // gyro  ODR + FS
static constexpr uint8_t LSM6DSV_REG_CTRL3    = 0x12;  // BDU, IF_INC, sw_reset
static constexpr uint8_t LSM6DSV_REG_OUTX_L_G = 0x22;  // gyro X low byte
static constexpr uint8_t LSM6DSV_REG_OUTX_L_A = 0x28;  // accel X low byte

// WHO_AM_I expected values. Either is acceptable — both come from the same
// supplier family and share the register map used in this driver.
static constexpr uint8_t LSM6DSV_WHOAMI_DSV   = 0x70;  // LSM6DSV / LSM6DSV16X
static constexpr uint8_t LSM6DSV_WHOAMI_DSVBX = 0x71;  // LSM6DSV16BX

// CTRL1 (10h) / CTRL2 (11h) — same layout for accel and gyro:
//   Bits[7:4]: OP_MODE_XL/G[3:0]  — operating mode (0000 = HP)
//   Bits[3:0]: ODR_XL/G[3:0]      — output data rate (0000 = power-down!)
// Full-scale lives in SEPARATE registers — not in CTRL1/CTRL2:
//   FS_XL → CTRL8 (17h) bits[1:0]
//   FS_G  → CTRL6 (15h) bits[2:0]
// NOTE: LSM6DSV16X has no exact 416 Hz ODR — 480 Hz is the nearest.
// We use 480 Hz so the chip generates ≥ 1 fresh sample per 400 Hz main tick.

// OP_MODE nibble (OR with ODR):
static constexpr uint8_t LSM6DSV_OPMODE_HP    = 0x00;  // High-performance, both axes

// ODR nibble:
static constexpr uint8_t LSM6DSV_ODR_120HZ    = 0x06;
static constexpr uint8_t LSM6DSV_ODR_240HZ    = 0x07;
static constexpr uint8_t LSM6DSV_ODR_480HZ    = 0x08;
static constexpr uint8_t LSM6DSV_ODR_960HZ    = 0x09;

// Full-scale registers and values:
static constexpr uint8_t LSM6DSV_REG_CTRL6    = 0x15;  // gyro full-scale
static constexpr uint8_t LSM6DSV_REG_CTRL8    = 0x17;  // accel full-scale
static constexpr uint8_t LSM6DSV_FS_XL_2G     = 0x00;  // CTRL8 bits[1:0] = 00
static constexpr uint8_t LSM6DSV_FS_G_2000    = 0x04;  // CTRL6 bits[2:0] = 100

// ---------------------------------------------------------------------------
class LSM6DSVDriver {
public:
    // wire        : pre-initialized TwoWire instance. Caller MUST have already
    //               called wire->begin(SDA, SCL, clock_hz) before begin().
    // sa0_pin     : GPIO to drive the IMU's SA0/SDO line. Pass -1 if the
    //               breakout has SA0 hardwired to GND or VDD.
    // sa0_high    : Desired SA0 logic level. The driver:
    //                 - drives sa0_pin to this level (if sa0_pin >= 0)
    //                 - infers the I2C address from this level
    //                       false → 0x6A,  true → 0x6B
    //               If sa0_pin < 0, only the address is derived; no GPIO write.
    LSM6DSVDriver(TwoWire* wire, int sa0_pin, bool sa0_high);

    // Reset, verify WHO_AM_I, configure ODR + range. Returns true on success.
    // On failure, prints the reason and leaves _initialized = false.
    bool begin(const char* label = "lsm6dsv");

    // One-shot raw read. Returns RawIMUData with valid=false on any I2C error.
    RawIMUData read();

    bool isInitialized() const { return _initialized; }

    // Diagnostic: direct WHO_AM_I read. No init guard — safe any time.
    // Returns 0xFF if the chip NACKs (typical for wrong address / no power).
    uint8_t readWhoAmI();

    // Diagnostic snapshot for the UI probe button. Reads WHO_AM_I + CTRL state
    // without modifying anything. Safe to call even if begin() failed.
    //
    // NOTE: addr field is new in the I2C version. webcomm.cpp's probe broadcast
    // does not currently include it — extend the FOOT_PROBE format string to
    // expose it to the UI if you want address visibility during diagnostics.
    struct ProbeResult {
        uint8_t addr;          // resolved 7-bit I2C address used by this instance
        uint8_t whoami;        // 0xFF if NACK
        uint8_t ctrl1;
        uint8_t ctrl2;
        uint8_t ctrl3;
        bool    initialized;
    };
    ProbeResult probe();

private:
    TwoWire* _wire;
    int      _sa0Pin;          // -1 → SA0 hardwired on breakout, no GPIO drive
    bool     _sa0High;         // requested level → resolved address
    uint8_t  _addr;             // computed from _sa0High in constructor
    bool     _initialized = false;

    // ------- I2C primitives — all return false on NACK / short read -------
    // Pattern: beginTransmission → write(reg [+ val]) → endTransmission.
    // For reads, endTransmission(false) issues a repeated START so the bus
    // stays owned by us through requestFrom().
    bool    _writeReg(uint8_t reg, uint8_t val);
    uint8_t _readReg (uint8_t reg);                            // 0xFF on error
    bool    _readBurst(uint8_t reg, uint8_t* buf, size_t len);
};

#endif // LSM6DSV_DRIVER_H