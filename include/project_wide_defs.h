#ifndef PROJECT_WIDE_DEFS_H
#define PROJECT_WIDE_DEFS_H

#include <Arduino.h>

// =============================================================================
//  PROJECT-WIDE DEFINITIONS
// =============================================================================
//
//  Joint metadata and limits live in joint_config.h.
//  Servo hardware limits live in servo_driver.h.
// =============================================================================

// --- Hardware pins ---
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9

// --- WiFi ---
#define AP_SSID "HumanoidRobot"
#define AP_PASS "robot1234"

// --- Joint smooth-stepping speed limits (deg/s) ---
// Applied to all smooth-stepped (immediate=false) writes: UI sliders, pose loads, gait.
// Immediate writes (balance controller) bypass the smooth-stepper entirely.
//
// DEFAULT_DEG_S: comfortable default for UI interaction — not too sluggish, not snappy.
// Recalculate MAX_DEG_S if SD_ANGLE_RANGE changes (servo_driver.h):
//   max_us_per_tick = (MAX_DEG_S / SD_ANGLE_RANGE) * (SD_USMAX - SD_USMIN) * UPDATE_INTERVAL_S
#define JOINT_SPEED_DEFAULT_DEG_S   50.0f   // All joints at boot
#define JOINT_SPEED_MIN_DEG_S        1.0f   // Floor — prevents silent frozen-joint bugs
// No global MAX: upper clamp is now per-joint (JointConfig::noLoadSpeedDegS).

// --- Single source of truth for joint metadata is joint_config.h ---

// No global MAX: upper clamp is now per-joint (JointConfig::noLoadSpeedDegS).

// --- Balance controller safe gain limits ---
// Hard ceiling enforced by WebComm when parsing BALANCE_TUNE commands.
// Derived from phase-margin analysis: at ~50ms servo lag and ω_n ≈ 6.3 rad/s,
// Kp above 20 deg/rad eliminates phase margin entirely. Kd above 1.0 amplifies
// gyro shot noise faster than it damps the pendulum.
// Recalculate if CoM height or servo model changes.
#define BALANCE_KP_MAX   30.0f   // deg/rad — do not raise without re-measuring servo lag
#define BALANCE_KD_MAX    1.0f   // deg/(rad/s)

// --- Weight shift ankle pitch forward tilt ---
// Joint-relative ankle pitch offset applied during weight shift (degrees).
// Positive = forward lean direction. Negate to reverse. Set to 0 to disable.
// Both ankles receive this value; JointModel direction fields produce symmetry.
// #define ANKLE_PITCH_FORWARD_TILT_DEG  -0.0f
 
// ============================================================
// GAIT PARAMETERS — GaitController (weight_shift evolution)
// ============================================================
constexpr float GAIT_PHASE_RATE_HZ          = 0.4f;   // start slow; cycles/sec
constexpr float GAIT_STEP_HEIGHT_MM         = 20.0f;  // swing foot clearance
constexpr float GAIT_STANCE_WIDTH_MM        = 40.0f;  // lateral CoM amplitude
constexpr float GAIT_STEP_LENGTH_MM         = 35.0f;  // Phase 3+; forward stride
constexpr float GAIT_WEIGHT_SHIFT_THRESHOLD_RAD = 0.15f; // ~8.5° — min roll before lift

// Stance leg's nominal hip→ankle vertical drop in mm. Used by GaitController
// to construct FootTargets. Must stay inside LegIK's safe range (130–189 mm).
// Lower = deeper crouch = more knee flexion at neutral.
constexpr float GAIT_STANCE_HEIGHT_MM           = 186.0f;

// Static fore-aft offset applied to every FootTarget.x_mm built by GaitController.
// Compensates for CoM ≠ hip-pitch axis: with x_mm=0 the foot lands directly under
// the hip pitch axis, but the body CoM sits forward of that, so the robot leans
// back during stance and crouching. Positive = foot shifts FORWARD relative to
// hip (body shifts BACK over the foot). Negative = foot shifts BACKWARD (body
// leans forward). Same role as UVC's adjFR (uses -2.04mm).
//
// Tune from 0 in ±1mm increments until ankle-pitch IK output sits near each
// joint's neutralDeg in standing pose. Typical magnitude: ≤ 10 mm.
constexpr float GAIT_STANCE_X_OFFSET_MM         = -20.0f;

// Hold duration (ms) GaitController inserts between half-cycles. Weight shift
// is centered (trigger NONE) and held for this duration so mechanical/IIR
// transients damp before the next direction is commanded. Should be ≥ ~5×
// the IIR time constant from WeightShiftConfig::ankle_tilt_smooth_alpha.
// At alpha=0.90 / 400 Hz, τ ≈ 25 ms → 150 ms gives ~6τ.
constexpr uint32_t GAIT_STABILIZE_MS            = 400;
constexpr uint32_t GAIT_STOP_RAMP_MS            = 600;

// ============================================================
// RECOVERY STEP PARAMETERS — secondary balance system
// ============================================================
//
// Stage 1: lateral-only fixed magnitude.
// Stage 2: 2D directional, magnitude-scaled, soft-return.
//
// Per-trigger magnitude is computed inside GaitController::triggerRecovery()
// from the pitch/roll passed by BalanceController, then clamped to these caps.
// ============================================================

// --- Step magnitude caps (hard physical limits) ---
constexpr float RECOVERY_STEP_LATERAL_MAX_MM = 35.0f;  // was 30 fixed — now ceiling
constexpr float RECOVERY_STEP_FORWARD_MAX_MM = 35.0f;  // matches lateral cap
constexpr float RECOVERY_STEP_HEIGHT_MM      = 15.0f;  

// --- Magnitude scaling: simplified capture-point ---
// step_mm = clamp(MAGNITUDE_GAIN_MM_PER_RAD * |tilt|, MIN, MAX)
// Initial value: 30 mm step at 0.20 rad tilt → 150 mm/rad. Tune after T2.3.
// Floor prevents trivial steps from triggering motion (less than the IMU noise floor).
constexpr float RECOVERY_MAGNITUDE_GAIN_MM_PER_RAD = 150.0f;
constexpr float RECOVERY_STEP_MIN_MAGNITUDE_MM     = 10.0f;

// --- Stance counter-shift fraction ---
// stance_offset = -counterShift * swing_offset (opposite direction, smaller magnitude)
// 0.0 disables. 0.3 = 30% counter-shift. UVC equivalent ~0.25.
constexpr float RECOVERY_STANCE_COUNTERSHIFT_FRAC  = 0.3f;

// --- Sub-state durations (ms) ---
// LIFT and LAND unchanged from Stage 1.
// DAMP_HOLD: brief hold at planted offset before soft return begins.
// SOFT_RETURN: smoothstep decay of step offset back to centered stance.
//   DAMP_HOLD + SOFT_RETURN ≈ original RECOVERY_DAMP_MS (600).
constexpr uint32_t RECOVERY_LIFT_MS         = 200;     // unchanged
constexpr uint32_t RECOVERY_LAND_MS         = 200;     // unchanged
constexpr uint32_t RECOVERY_DAMP_HOLD_MS    = 200;     // was 600 (combined)
constexpr uint32_t RECOVERY_SOFT_RETURN_MS  = 400;     // new

// --- Refractory lockout (ms) ---
// After RecoverySubState::COMPLETE, recovery_flag from BalanceController
// is ignored for this many ms. Prevents post-step transient retrigger.
// Set ≥ output_iir time-constant of balance × 5 (~150 ms) plus mechanical
// settling margin. Tune after T2.5.
constexpr uint32_t RECOVERY_LOCKOUT_MS      = 500;

#endif // PROJECT_WIDE_DEFS_H