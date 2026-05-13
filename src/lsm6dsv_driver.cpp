// =============================================================================
// FILE:    lsm6dsv_driver.cpp
// LAYER:   1 — raw I2C I/O only
//
// LAST CHANGED: 2026-05-12 | Hinz | F0→F1 protocol switch (SPI → I2C)
// =============================================================================

#include "lsm6dsv_driver.h"

// ---------------------------------------------------------------------------
//  Constructor — resolves the I2C address from the requested SA0 level.
//  No I2C transactions here; chip may not even be powered yet at construction
//  time (this is a global-scope object in main.cpp).
// ---------------------------------------------------------------------------
LSM6DSVDriver::LSM6DSVDriver(TwoWire* wire, int sa0_pin, bool sa0_high)
    : _wire(wire),
      _sa0Pin(sa0_pin),
      _sa0High(sa0_high),
      _addr(sa0_high ? LSM6DSV_I2C_ADDR_SA0_HIGH : LSM6DSV_I2C_ADDR_SA0_LOW) {}

// ---------------------------------------------------------------------------
//  begin() — drive SA0, verify WHO_AM_I, software reset, configure registers.
//
//  ORDER MATTERS:
//    1. Drive SA0 BEFORE any I2C transaction — the chip latches its address
//       on every START condition based on the current SA0 level.
//    2. Wait for boot (datasheet §3.3 — 35 ms max).
//    3. WHO_AM_I check — early fail-fast if wiring/address is wrong.
//    4. Software reset, then verify reset cleared.
//    5. Configure CTRL3 (BDU + auto-increment), then CTRL1/CTRL2 (ODR + FS).
//    6. Readback-verify CTRL1/CTRL2 — proves I2C writes are sticking.
//    7. Sanity-read one sample — all-zero means SPI/I2C mode mismatch
//       (CS pin not tied HIGH on the breakout) or chip is dead.
// ---------------------------------------------------------------------------
bool LSM6DSVDriver::begin(const char* label) {
    if (_wire == nullptr) {
        Serial.printf("[%s] ERROR: null TwoWire pointer.\n", label);
        return false;
    }

    // --- Step 1: SA0 setup ------------------------------------------------
    // If the caller is driving SA0 from a GPIO, configure it BEFORE the first
    // I2C transaction. Even one START with SA0 in the wrong state will NACK
    // (or worse, ACK the wrong chip on a shared bus).
    if (_sa0Pin >= 0) {
        pinMode(_sa0Pin, OUTPUT);
        digitalWrite(_sa0Pin, _sa0High ? HIGH : LOW);
        // Brief settle. The datasheet does not specify a minimum SA0 setup
        // time but the chip needs SA0 stable before the next address compare;
        // 2 ms is generous and harmless at boot.
        delay(2);
    }

    // --- Step 2: chip power-up delay --------------------------------------
    delay(40);   // ≥ 35 ms boot per datasheet §3.3

    // --- Step 3: identity check -------------------------------------------
    const uint8_t whoami = readWhoAmI();
    if (whoami != LSM6DSV_WHOAMI_DSV && whoami != LSM6DSV_WHOAMI_DSVBX) {
        Serial.printf("[%s] ERROR: WHO_AM_I=0x%02X at I2C 0x%02X "
                      "(expected 0x70 or 0x71).\n"
                      "  Check: SDA/SCL wiring, 3.3V supply, 4.7k pull-ups on SDA+SCL,\n"
                      "         SA0 level, CS pin tied HIGH on breakout (I2C-mode select).\n",
                      label, whoami, _addr);
        return false;
    }
    Serial.printf("[%s] WHO_AM_I=0x%02X at I2C 0x%02X — chip detected.\n",
                  label, whoami, _addr);

    // --- Step 4: software reset + verify ----------------------------------
    if (!_writeReg(LSM6DSV_REG_CTRL3, 0x01)) {
        Serial.printf("[%s] ERROR: SW_RESET write NACKed. "
                      "I2C write path broken.\n", label);
        return false;
    }
    delay(50);   // datasheet max reset time

    const uint8_t ctrl3_post = _readReg(LSM6DSV_REG_CTRL3);
    if (ctrl3_post & 0x01) {
        Serial.printf("[%s] ERROR: SW_RESET did not clear (CTRL3=0x%02X). "
                      "Writes may be silently failing.\n",
                      label, ctrl3_post);
        return false;
    }

    // --- Step 5: configure ------------------------------------------------
    // CTRL3 = 0x44 → BDU + IF_INC (block data update + auto-increment).
    // CTRL1/CTRL2 → HP mode in high nibble, ODR in low nibble.
    // CTRL6/CTRL8 → set full-scale separately (otherwise chip stays at
    // power-on defaults: ±2g accel, ±125 dps gyro — gyro scale would be 16x
    // off from what FOOT_IMU_GYRO_SCALE_LSB_PER_DPS assumes).
    _writeReg(LSM6DSV_REG_CTRL3, 0x44);
    _writeReg(LSM6DSV_REG_CTRL1, LSM6DSV_OPMODE_HP | LSM6DSV_ODR_480HZ);
    _writeReg(LSM6DSV_REG_CTRL2, LSM6DSV_OPMODE_HP | LSM6DSV_ODR_480HZ);
    _writeReg(LSM6DSV_REG_CTRL6, LSM6DSV_FS_G_2000);
    _writeReg(LSM6DSV_REG_CTRL8, LSM6DSV_FS_XL_2G);
    delay(20);

    // --- Step 6: readback-verify ------------------------------------------
    const uint8_t ctrl1v = _readReg(LSM6DSV_REG_CTRL1);
    const uint8_t ctrl2v = _readReg(LSM6DSV_REG_CTRL2);
    const uint8_t ctrl6v = _readReg(LSM6DSV_REG_CTRL6);
    const uint8_t ctrl8v = _readReg(LSM6DSV_REG_CTRL8);
    const uint8_t want1  = LSM6DSV_OPMODE_HP | LSM6DSV_ODR_480HZ;
    const uint8_t want2  = LSM6DSV_OPMODE_HP | LSM6DSV_ODR_480HZ;
    const uint8_t want6  = LSM6DSV_FS_G_2000;
    const uint8_t want8  = LSM6DSV_FS_XL_2G;
    Serial.printf("[%s] CTRL1=0x%02X (want 0x%02X)  CTRL2=0x%02X (want 0x%02X)  "
                  "CTRL6=0x%02X (want 0x%02X)  CTRL8=0x%02X (want 0x%02X)\n",
                  label, ctrl1v, want1, ctrl2v, want2, ctrl6v, want6, ctrl8v, want8);
    if (ctrl1v != want1 || ctrl2v != want2 || ctrl6v != want6 || ctrl8v != want8) {
        Serial.printf("[%s] ERROR: control register writes did not stick.\n", label);
        return false;
    }

    _initialized = true;
    Serial.printf("[%s] Configured: ±2g, ±2000 dps, 416 Hz HP, I2C 0x%02X.\n",
                  label, _addr);

    // --- Step 7: sanity sample --------------------------------------------
    // All-zero on the first read means I2C succeeded but the chip never
    // produced a sample. Most common cause: breakout's CS pin is not tied
    // HIGH, so the chip is sitting in SPI mode and ignoring I2C writes
    // EXCEPT for WHO_AM_I (which is a quirk on some parts).
    RawIMUData sample = read();
    if (!sample.valid ||
        (sample.accel_x == 0 && sample.accel_y == 0 && sample.accel_z == 0 &&
         sample.gyro_x  == 0 && sample.gyro_y  == 0 && sample.gyro_z  == 0)) {
        Serial.printf("[%s] ERROR: first sample is all-zero. "
                      "Tie breakout CS to 3.3V to force I2C mode. "
                      "Accel should read ~16384 LSB on the gravity axis at rest.\n",
                      label);
        _initialized = false;
        return false;
    }

    return true;
}

// ---------------------------------------------------------------------------
//  read() — two separate 6-byte bursts (gyro then accel).
//  Total time on a 400 kHz bus: ~310 µs per IMU (incl. address + repeated
//  starts). At 400 Hz tick rate (2500 µs budget), two IMUs use ~25% of the
//  loop — comfortably inside budget alongside everything else.
// ---------------------------------------------------------------------------
RawIMUData LSM6DSVDriver::read() {
    RawIMUData data{};
    if (!_initialized) {
        data.valid = false;
        return data;
    }

    uint8_t buf[6];

    if (!_readBurst(LSM6DSV_REG_OUTX_L_G, buf, 6)) {
        data.valid = false;
        return data;
    }
    data.gyro_x = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
    data.gyro_y = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
    data.gyro_z = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));

    if (!_readBurst(LSM6DSV_REG_OUTX_L_A, buf, 6)) {
        data.valid = false;
        return data;
    }
    data.accel_x = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
    data.accel_y = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
    data.accel_z = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));

    data.valid = true;
    return data;
}

// ---------------------------------------------------------------------------
uint8_t LSM6DSVDriver::readWhoAmI() {
    return _readReg(LSM6DSV_REG_WHO_AM_I);
}

// ---------------------------------------------------------------------------
//  I2C primitives.
//  endTransmission() return codes (Arduino-ESP32):
//    0 = success, 1 = data too long, 2 = NACK on address, 3 = NACK on data,
//    4 = other error, 5 = timeout.
//  We collapse all non-zero into "failure" — the caller treats it uniformly.
// ---------------------------------------------------------------------------
bool LSM6DSVDriver::_writeReg(uint8_t reg, uint8_t val) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(val);
    return (_wire->endTransmission() == 0);
}

uint8_t LSM6DSVDriver::_readReg(uint8_t reg) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    // endTransmission(false) → repeated START, no STOP. Required so the
    // chip's internal register pointer stays at `reg` for the following read.
    if (_wire->endTransmission(false) != 0) return 0xFF;
    if (_wire->requestFrom((uint8_t)_addr, (uint8_t)1) != 1) return 0xFF;
    return _wire->read();
}

bool LSM6DSVDriver::_readBurst(uint8_t reg, uint8_t* buf, size_t len) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0) return false;
    const size_t got = _wire->requestFrom((uint8_t)_addr, (uint8_t)len);
    if (got != len) return false;
    for (size_t i = 0; i < len; i++) buf[i] = _wire->read();
    return true;
}

// ---------------------------------------------------------------------------
LSM6DSVDriver::ProbeResult LSM6DSVDriver::probe() {
    ProbeResult r{};
    r.addr        = _addr;
    r.whoami      = readWhoAmI();
    r.ctrl1       = _readReg(LSM6DSV_REG_CTRL1);
    r.ctrl2       = _readReg(LSM6DSV_REG_CTRL2);
    r.ctrl3       = _readReg(LSM6DSV_REG_CTRL3);
    r.initialized = _initialized;
    return r;
}