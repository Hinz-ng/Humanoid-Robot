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
#define JOINT_SPEED_DEFAULT_DEG_S   70.0f   // All joints at boot
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
// TUNING NOTE (2026-04-xx): Reduced from 0.4 → 0.25 Hz to give the IMU roll
// gate (wsThresholdRad) more time to satisfy before the half-cycle advances.
// Return to 0.4 once stance loading is stable at the lower rate.
constexpr float GAIT_PHASE_RATE_HZ          = 0.45f;  // cycles/sec

// TUNING NOTE: Reduced from 20 → 15 mm. The IMU gate now prevents lifting
// before the stance leg is loaded, so less clearance is needed to avoid drag.
// If drag returns at 15 mm, the root cause is the gate, not step height.
constexpr float GAIT_STEP_HEIGHT_MM         = 14.0f;  // swing foot clearance
constexpr float GAIT_STANCE_WIDTH_MM        = 40.0f;  // lateral CoM amplitude
constexpr float GAIT_STEP_LENGTH_MM         = 35.0f;  // Phase 3+; forward stride

// TUNING NOTE: Reduced from 0.15 (~8.5°) → 0.10 (~5.7°). The previous value
// was chosen conservatively for Phase 2 diagnostics. At 0.10 rad the gate opens
// when the CoM is meaningfully over the stance foot without requiring a large
// visible lean, which the QY3242BLS ankle servos cannot reliably produce under
// full load against a 500ms ramp. Tighten toward 0.12 if gate opens too easily.
constexpr float GAIT_WEIGHT_SHIFT_THRESHOLD_RAD = 0.10f; // ~5.7° — min roll before lift

// Stance leg's nominal hip→ankle vertical drop in mm. Used by GaitController
// to construct FootTargets. Must stay inside LegIK's safe range (130–189 mm).
// Lower = deeper crouch = more knee flexion at neutral.
constexpr float GAIT_STANCE_HEIGHT_MM           = 185.0f;

// Static fore-aft offset applied to every FootTarget.x_mm built by GaitController.
// Compensates for CoM ≠ hip-pitch axis: with x_mm=0 the foot lands directly under
// the hip pitch axis, but the body CoM sits forward of that, so the robot leans
// back during stance and crouching. Positive = foot shifts FORWARD relative to
// hip (body shifts BACK over the foot). Negative = foot shifts BACKWARD (body
// leans forward). Same role as UVC's adjFR (uses -2.04mm).
//
// Tune from 0 in ±1mm increments until ankle-pitch IK output sits near each
// joint's neutralDeg in standing pose. Typical magnitude: ≤ 10 mm.
constexpr float GAIT_STANCE_X_OFFSET_MM         = -10.0f;

// Hold duration (ms) GaitController inserts between half-cycles. Weight shift
// is centered (trigger NONE) and held for this duration so mechanical/IIR
// transients damp before the next direction is commanded. Should be ≥ ~5×
// the IIR time constant from WeightShiftConfig::ankle_tilt_smooth_alpha.
// At alpha=0.90 / 400 Hz, τ ≈ 25 ms → 150 ms gives ~6τ.
constexpr uint32_t GAIT_STABILIZE_MS            = 400;
constexpr uint32_t GAIT_STOP_RAMP_MS            = 600;

// ============================================================
// CAPTURE-POINT — Stage 2.2 (sagittal only)
// ============================================================
// LIP natural frequency at H_COM_MM = 220 mm:
//   omega0 = sqrt(g / h) = sqrt(9.81 / 0.22) ≈ 6.74 rad/s
// Stored as 1/omega0 to avoid recomputing sqrt each tick.
// Recompute if H_COM_MM changes:
//   1.0f / sqrtf(9810.0f / H_COM_MM)   (using mm units consistently)
constexpr float GAIT_CAPTURE_INV_OMEGA0_S = 0.1498f;

// Capture-point default gain. 1.0 = full LIP correction.
// Reduce toward 0.5 if hardware capture-point is hot. Increase only after
// confirming the LIP model is conservative on this platform.
constexpr float GAIT_CAPTURE_GAIN_DEFAULT = 0.25f;

// Hard cap on the capture offset's contribution to a single step (mm).
// Must remain inside the IK reach budget after combining with stepLengthMm.
// At stanceHeightMm=185 and stepLengthMm=30, the swing foot already reaches
// ~98% at peak. This 15 mm cap leaves ~2% IK headroom before clamping.
// Re-tune after first hardware run; do not raise blindly.
constexpr float GAIT_CAPTURE_OFFSET_MAX_MM = 15.0f;

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
constexpr float RECOVERY_STEP_LATERAL_MAX_MM     = 35.0f;  // was 30 fixed — now ceiling
constexpr float RECOVERY_STEP_FORWARD_MAX_MM     = 35.0f;  // matches lateral cap
constexpr float RECOVERY_STEP_HEIGHT_LATERAL_MM  = 0.0f;  // tune 18–25
constexpr float RECOVERY_STEP_HEIGHT_SAGITTAL_MM = 10.0f;  // tune 28–38

// --- Magnitude scaling: simplified capture-point ---
// step_mm = clamp(MAGNITUDE_GAIN_MM_PER_RAD * |tilt|, MIN, MAX)
// Initial value: 30 mm step at 0.20 rad tilt → 150 mm/rad. Tune after T2.3.
// Floor prevents trivial steps from triggering motion (less than the IMU noise floor).
constexpr float RECOVERY_MAGNITUDE_GAIN_MM_PER_RAD = 150.0f;
constexpr float RECOVERY_STEP_MIN_MAGNITUDE_MM     = 10.0f;

// --- Stance counter-shift fraction ---
// stance_offset = +counterShift * |swing_offset| (abducts away from fall).
// 0.0 disables. UVC reference: ~0.25.
// TUNING NOTE 2026-05-04: Increased from 0.30 → 0.85.
//   At 0.30 with swingY=30mm, stance abducts only 9mm → hip_roll≈3.2°. Below
//   visual perception. At 0.85, abduction is 25.5mm → hip_roll≈9.0°, still inside
//   the 20° IK limit and much closer to transferring CoM over the stance edge.
constexpr float RECOVERY_STANCE_COUNTERSHIFT_FRAC  = 0.85f;

// --- Sub-state durations (ms) ---
// LIFT and LAND unchanged from Stage 1.
// DAMP_HOLD: brief hold at planted offset before soft return begins.
// SOFT_RETURN: smoothstep decay of step offset back to centered stance.
//   DAMP_HOLD + SOFT_RETURN ≈ original RECOVERY_DAMP_MS (600).
constexpr uint32_t RECOVERY_PRE_LIFT_MS     = 120;      // new: load stance before swing lifts
constexpr uint32_t RECOVERY_LIFT_MS         = 250;     // unchanged
constexpr uint32_t RECOVERY_LAND_MS         = 200;     // unchanged
constexpr uint32_t RECOVERY_DAMP_HOLD_MS    = 200;     // was 600 (combined)
constexpr uint32_t RECOVERY_SOFT_RETURN_MS  = 400;     // new

// --- Refractory lockout (ms) ---
// After RecoverySubState::COMPLETE, recovery_flag from BalanceController
// is ignored for this many ms. Prevents post-step transient retrigger.
// Set ≥ output_iir time-constant of balance × 5 (~150 ms) plus mechanical
// settling margin. Tune after T2.5.
constexpr uint32_t RECOVERY_LOCKOUT_MS      = 800;

// --- Recovery step chaining threshold (rad) ---
// After RecoverySubState::COMPLETE, if |roll| or |pitch| still exceeds this
// value, a second catch step is triggered immediately instead of entering
// the refractory lockout. Prevents falls from large disturbances that a
// single step cannot fully catch.
//
// Set below recovery_tilt_roll_rad (~0.18 rad) so it fires on residual lean
// that the first step reduced but didn't fully recover. Set above normal
// walking roll (~0.05 rad) so it never fires after a clean step landing.
// Default 0.07 rad ≈ 4°. Tune after T2.5 if second steps are spurious.
constexpr float RECOVERY_CHAIN_TILT_THRESHOLD_RAD = 0.12f;

// ============================================================
// FOOT IMU — Phase F1 (on-robot passive read)
// ============================================================
// Sensor scales for LSM6DSV (configured in lsm6dsv_driver.cpp at ±2g, ±2000 dps).
// Both differ from the BMI160 torso unit and MUST be set per StateEstimator
// instance via FilterConfig — there is no global default.
//
//   Accel ±2g     : 0.061 mg/LSB → 16384 LSB/g  (numerically same as BMI160)
//   Gyro  ±2000   : 70 mdps/LSB  → 14.286 LSB/(deg/s)  (BMI160 is 16.4)
constexpr float FOOT_IMU_ACCEL_SCALE_LSB_PER_G  = 16384.0f;
constexpr float FOOT_IMU_GYRO_SCALE_LSB_PER_DPS = 14.286f;

// Foot IMU sees larger short-duration accel transients than torso (footstrike,
// ankle servo vibration). A torso-tight ±0.20g sanity band would zero out the
// accel correction during normal swing motion. Start at ±0.40g; F1 telemetry
// will show whether this needs tightening or further loosening.
constexpr float FOOT_IMU_ACCEL_SANITY_MARGIN = 0.40f;

// Complementary filter alpha. Higher = trust gyro more, accel less (slower
// drift correction, more vibration immunity). Foot frame is rigid to the
// chip, so accel reference is in principle as trustworthy as torso — but
// foot accel is noisier. 0.985 splits the difference; tune from telemetry.
constexpr float FOOT_IMU_COMP_FILTER_ALPHA = 0.985f;

// Same accel IIR beta as torso default. Foot has more vibration so this is
// a candidate for tightening (toward 0.85+) after F1 telemetry shows the
// noise envelope. Keeping torso's value initially eliminates one variable.
constexpr float FOOT_IMU_ACCEL_LPF_BETA = 0.70f;

// Compile-time enable — leave enabled for F1 bring-up. After F2 ships, this
// macro can be retired (foot IMUs become structural). Set to 0 to revert to
// pre-F1 behavior in one line if you need to bisect a regression.
#define FOOT_IMU_ENABLED  1

// Phase F0 raw-count print loop — disabled in F1 because telemetry replaces it.
// Kept as a separate flag in case bench-style debugging is needed later.
#define FOOT_IMU_BENCH_PRINT  0

// ============================================================
// SWING-FOOT LEVELER (Section 14)
// Closed-loop ankle pitch/roll correction during swing phase.
// Reads swing-leg foot IMU; emits additive bias on top of IK angles.
// Enabled=false by default; opt-in via CMD:SWING_LEVELER_TUNE:enabled=1.
// ============================================================

// P gain: deg of ankle correction per rad of foot tilt error.
// 1 rad ~= 57 deg, so kp=25 -> ~0.44 deg per deg of foot tilt. Conservative
// start; tune toward 0.7 on hardware during L1.
constexpr float SWING_LEVELER_KP_PITCH_DEG_PER_RAD = 25.0f;
constexpr float SWING_LEVELER_KP_ROLL_DEG_PER_RAD  = 25.0f;

// Bias clamps. Must remain inside IK ankle soft-limits with margin for the
// IK-computed flat-foot value (which already consumes most of the range
// during deep swing arcs).
constexpr float SWING_LEVELER_MAX_PITCH_BIAS_DEG = 8.0f;
constexpr float SWING_LEVELER_MAX_ROLL_BIAS_DEG  = 6.0f;

// Input deadband. ~1 deg below the noise floor of the foot LPF at rest.
constexpr float SWING_LEVELER_DEADBAND_RAD = 0.0175f;

// IIR alphas. 0 = no smoothing; 0.9 = heavy.
constexpr float SWING_LEVELER_INPUT_LPF_ALPHA  = 0.6f;
constexpr float SWING_LEVELER_OUTPUT_IIR_ALPHA = 0.5f;

// Lift-frac envelope corners.
// liftFrac peaks at 1.0 at the swing apex (sin(pi/2)). The previous
// peak=0.55, disengage=0.80 defaults notched the envelope to zero through
// the entire apex window because liftFrac > disengage there. Setting
// peak=1.0 places full bias at the apex; disengage=1.01 is effectively
// "never" on the ascent path, and the envelope's natural return to zero
// happens via liftFrac sinusoidally falling below engage on descent.
constexpr float SWING_LEVELER_ENGAGE_LIFT_FRAC    = 0.20f;
constexpr float SWING_LEVELER_PEAK_LIFT_FRAC      = 1.0f;
constexpr float SWING_LEVELER_DISENGAGE_LIFT_FRAC = 1.01f;

#endif // PROJECT_WIDE_DEFS_H
