#include "servo_driver.h"
#include "project_wide_defs.h"   // I2C_SDA_PIN, I2C_SCL_PIN

// =============================================================================
//  SERVO DRIVER — Implementation
// =============================================================================
//  All PCA9685 communication is in this file. Nothing else touches the hardware.
// =============================================================================

ServoDriver::ServoDriver() : _pwm(Adafruit_PWMServoDriver()) {
    // No init work here — hardware is not ready until init() is called.
}

// -----------------------------------------------------------------------------
//  init()
//  Wire.begin() must already be called before this (done by ServoControl::init).
// -----------------------------------------------------------------------------
void ServoDriver::init() {
    _pwm.begin();
    _pwm.setOscillatorFrequency(SD_OSCILLATOR_HZ);
    _pwm.setPWMFreq(SD_PWM_FREQ_HZ);
    Serial.printf("[ServoDriver] PCA9685 ready. freq=%d Hz  osc=%d Hz\n",
                  SD_PWM_FREQ_HZ, SD_OSCILLATOR_HZ);
}

// -----------------------------------------------------------------------------
//  setServoPulse() — the single hardware write point
// -----------------------------------------------------------------------------
void ServoDriver::setServoPulse(uint8_t channel, int pulse_us) {
    // Bounds check: don't write to channels outside hardware range.
    if (channel > 15) {
        Serial.printf("[ServoDriver] ERROR: channel %d > 15, skipping write\n", channel);
        return;
    }
    // Clamp to safe pulse range before writing.
    int p = clampPulse(pulse_us);
    _pwm.writeMicroseconds(channel, (uint16_t)p);
}

// -----------------------------------------------------------------------------
//  Static math helpers
// -----------------------------------------------------------------------------

/*static*/ int ServoDriver::degToPulse(float absoluteDeg) {
    // Formula: pulse = (angle / totalRange) * pulseRange + pulseMin
    // Derived from: 0° → SD_USMIN, SD_ANGLE_RANGE → SD_USMAX
    if (absoluteDeg < 0.0f)          absoluteDeg = 0.0f;
    if (absoluteDeg > SD_ANGLE_RANGE) absoluteDeg = SD_ANGLE_RANGE;
    float pulse = (absoluteDeg / SD_ANGLE_RANGE) * (float)(SD_USMAX - SD_USMIN)
                  + (float)SD_USMIN;
    return (int)(pulse + 0.5f);  // round to nearest integer
}

/*static*/ float ServoDriver::pulseToDeg(int pulse_us) {
    // Inverse of degToPulse
    int p = clampPulse(pulse_us);
    return ((float)(p - SD_USMIN) / (float)(SD_USMAX - SD_USMIN)) * SD_ANGLE_RANGE;
}

/*static*/ int ServoDriver::clampPulse(int pulse_us) {
    if (pulse_us < SD_USMIN) return SD_USMIN;
    if (pulse_us > SD_USMAX) return SD_USMAX;
    return pulse_us;
}