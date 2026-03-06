#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

// =============================================================================
//  SERVO DRIVER  —  Layer 1: Hardware Communication Only
// =============================================================================
//
//  This class has exactly ONE job: talk to the PCA9685 chip.
//
//  It knows nothing about joints, neutral positions, angle limits, or robot
//  geometry. It only accepts a channel number and a pulse width in µs.
//
//  If you ever swap the PCA9685 for a different PWM board, this is the only
//  class you need to rewrite.
//
//  HARDWARE CONSTANTS
//  ──────────────────
//  USMIN / USMAX    : pulse range of your servos (500–2500 µs for 270° servos)
//  ANGLE_RANGE_DEG  : total servo travel in degrees
//  PWM_FREQ_HZ      : PWM signal frequency (50 Hz is standard for hobby servos)
//  OSCILLATOR_HZ    : PCA9685 crystal frequency — adjust if servos jitter
// =============================================================================

// Servo hardware limits — change these if your servo model differs.
#define SD_USMIN          500       // Pulse at 0°    (µs)
#define SD_USMAX          2500      // Pulse at 270°  (µs)
#define SD_ANGLE_RANGE    270.0f    // Total servo travel (degrees)
#define SD_PWM_FREQ_HZ    50        // PWM frequency (Hz)
#define SD_OSCILLATOR_HZ  27000000  // PCA9685 crystal frequency (Hz)

class ServoDriver {
public:
    // -------------------------------------------------------------------------
    //  Constructor
    // -------------------------------------------------------------------------
    ServoDriver();

    // -------------------------------------------------------------------------
    //  init()
    //
    //  Configures the PCA9685. Must be called after Wire.begin().
    //  Only call this once during startup.
    // -------------------------------------------------------------------------
    void init();

    // -------------------------------------------------------------------------
    //  setServoPulse(channel, pulse_us)
    //
    //  Sends a PWM pulse to one PCA9685 channel.
    //
    //  Parameters:
    //    channel  : PCA9685 channel, 0–15
    //    pulse_us : pulse width in microseconds
    //
    //  Out-of-range pulse values are silently clamped to [SD_USMIN, SD_USMAX].
    //  Out-of-range channel values log an error and return without hardware write.
    //
    //  This is the ONLY method in the codebase that calls pwm.writeMicroseconds().
    // -------------------------------------------------------------------------
    void setServoPulse(uint8_t channel, int pulse_us);

    // -------------------------------------------------------------------------
    //  Static math helpers — unit-testable pulse/degree conversions.
    //  These live here because they depend only on hardware constants.
    // -------------------------------------------------------------------------

    // Convert absolute servo angle (0°–270°) to pulse width (µs).
    static int   degToPulse(float absoluteDeg);

    // Convert pulse width (µs) to absolute servo angle (0°–270°).
    static float pulseToDeg(int pulse_us);

    // Clamp pulse to hardware-safe range [SD_USMIN, SD_USMAX].
    static int   clampPulse(int pulse_us);

private:
    Adafruit_PWMServoDriver _pwm;  // Owns the PCA9685 instance — not shared
};

#endif // SERVO_DRIVER_H