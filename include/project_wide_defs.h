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
#define JOINT_SPEED_DEFAULT_DEG_S   140.0f   // All joints at boot
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
constexpr float GAIT_PHASE_RATE_HZ          = 0.65f;  // cycles/sec

// TUNING NOTE: Reduced from 20 → 15 mm. The IMU gate now prevents lifting
// before the stance leg is loaded, so less clearance is needed to avoid drag.
// If drag returns at 15 mm, the root cause is the gate, not step height.
constexpr float GAIT_STEP_HEIGHT_MM         = 7.0f;  // swing foot clearance
constexpr float GAIT_STANCE_WIDTH_MM        = 40.0f;  // lateral CoM amplitude
constexpr float GAIT_STEP_LENGTH_MM         = 30.0f;  // Phase 3+; forward stride

// Session neutral-trim safety gate (deg). JointModel::captureSessionNeutral()
// rejects the whole capture if any joint is further than this from its factory
// neutral — a guard against snapshotting a pose that was never posed near
// neutral. Sized generously above real servo-neutral drift (~1–2° backlash)
// plus a few degrees of operator correction. This bounds the runtime trim,
// so the joint-limit soft window (neutral-relative) can only ride a few
// degrees in servo space; the hard [0,270] clamp in jointAngleToPulse is the
// final backstop. See JointModel — factory vs. runtime neutral.
constexpr float NEUTRAL_TRIM_MAX_DEG = 8.0f;

// W1 — swing-envelope apex fraction. _swingFrac() time-warps localPhase so
// the vertical lift apex lands at this fraction of the swing instead of the
// midpoint. 0.5 = symmetric raised cosine (legacy behaviour). Below 0.5 =
// faster lift, slower descent — the foot spends more of the swing in the
// descending half, lowering vertical speed through the touchdown approach.
// Zero vertical velocity at lift-off / apex / touchdown is preserved for any
// value. Clamped to [0.05, 0.95] in _swingFrac().
//
// TUNING NOTE 2026-05-20: hardware testing found a strongly front-loaded
// apex (0.1) noticeably better than the symmetric 0.5 — the long, slow
// descent visibly reduces touchdown disturbance. Default lowered 0.5 → 0.15.
constexpr float GAIT_SWING_APEX_FRAC_DEFAULT = 0.1f;

// Horizontal stride completion fraction (W-stride-frontload). _buildFootTarget()
// time-warps localPhase into [0, strideCompleteFrac] for the SWING leg's
// horizontal stride only: lpw = min(localPhase / frac, 1); s = lpw*lpw*(3-2*lpw)
// then held at 1.0. Properties: zero horizontal foot velocity at lift-off, at
// completion (lpw=1), and throughout the post-completion hold (smoothstep has
// zero derivative at both ends). Endpoints unchanged (s(0)=0, s>=frac → 1.0).
// Stance retreat is NOT warped (left on original timing to keep this experiment
// isolated from a "shift CoM forward earlier" change).
//
// 1.0 = no warp; reduces EXACTLY to the unwarped smoothstep (lpw == localPhase).
// Ship at 1.0 — flashing must change nothing until tuned down at runtime.
//
// TUNING: for the front-loading to act before the touchdown latch can fire,
// keep strideCompleteFrac <= touchdownGatePhase (currently 0.7 — see GaitConfig).
// Recommended experimental range 0.5–0.7. At 0.6 the swing foot reaches +L/2
// at localPhase=0.6 and holds horizontally stationary until touchdown at ~0.7+.
// Clamped to (0.05, 1.0] in GAIT_TUNE.
constexpr float GAIT_STRIDE_COMPLETE_FRAC_DEFAULT = 0.1f;

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
constexpr uint32_t GAIT_STABILIZE_MS            = 0;
constexpr uint32_t GAIT_STOP_RAMP_MS            = 600;

// ============================================================
// LIFT GATE DWELL & TIMEOUT  (Stage 2 -- support-transfer robustness)
// ============================================================
// Required continuous-OK duration before WAIT_SHIFT exits to SWINGING.
// Converts the single-tick edge trigger into a persistence-confirmed
// transition. Filters transient predicate satisfactions that don't
// represent a stably-completed support transfer.
//
// 80 ms ~= 32 ticks at 400 Hz -- long enough to debounce ramp-edge
// transients and brief predicate flickers, short enough that the
// added per-step latency is mechanically invisible.
//
// 0 disables dwell (single-tick gate -- pre-Stage-2 behavior).
constexpr uint32_t GAIT_LIFT_DWELL_MS_DEFAULT = 40;

// Maximum WAIT_SHIFT duration before declaring transfer failure.
// On expiry, GaitController calls stop() (clean ramp to IDLE).
// Sized to give the WeightShift ramp (~400 ms) + phase delay (~500 ms)
// + dwell (80 ms) + 2x margin to complete. Tune up if your gait
// genuinely needs longer to settle; tune down to fail-fast in testing.
//
// 0 disables the timeout (wait indefinitely -- pre-Stage-2 behavior).
constexpr uint32_t GAIT_WAIT_SHIFT_MAX_MS_DEFAULT = 1500;

// ============================================================
// LATERAL CoM OBSERVATION GATE  (Stage 3)
// ============================================================
// Threshold (mm) on neutral-corrected observed lateral CoM displacement,
// computed from torso IMU roll via the inverted-pendulum approximation:
//
//   com_lateral_mm = H_COM_MM * sin(imuRoll - torsoRollNeutralRad)
//                  ~= H_COM_MM * (imuRoll - torsoRollNeutralRad)
//
// for small angles. Gate satisfies when the signed observation projected
// onto the expected shift direction exceeds this threshold.
//
// Tuning: set to 60-80% of |lateral_shift_mm|. With lateral_shift_mm=-20,
// a threshold of 14-16 mm confirms the body has substantially shifted
// before lift, with margin for IIR settling and sensor noise. Setting
// too close to lateral_shift_mm makes the gate brittle to drift; setting
// too low defeats the point of observation.
constexpr float GAIT_COM_LATERAL_GATE_MM_DEFAULT = 8.0f;

// ============================================================
// LATERAL CoM TRIM  (W2 — continuous foot-placement correction)
// ============================================================
// UVC-style continuous lateral trim. Every tick, the observed lateral CoM
// displacement (H_COM_MM * (imuRoll - torsoRollNeutralRad)) is scaled by this
// gain and added to BOTH feet's y_mm so the support polygon tracks under the
// CoM. This is the continuous analog of the binary COM_LATERAL lift gate:
// the gate is a one-shot permission check; this trim runs every tick.
//
// Gain is dimensionless — mm of foot trim per mm of observed CoM offset.
// 0.30 corrects ~1/3 of a steady observed displacement. Start conservative;
// raise toward 0.6 only after confirming the IK clamp rate does not rise.
// The cap bounds a noisy-IMU spike from over-driving the feet in one tick.
constexpr float GAIT_COM_LAT_TRIM_GAIN_DEFAULT   = 0.20f;
constexpr float GAIT_COM_LAT_TRIM_MAX_MM_DEFAULT = 20.0f;
 
// ------------------------------------------------------------
// W5 — velocity-damping extension of the lateral CoM trim.
// ------------------------------------------------------------
// The W2 trim above is displacement-only (a position term, Kp). W5 adds a
// velocity-damping term (Kd): the filtered torso roll RATE is converted to a
// lateral CoM velocity (H_COM_MM * rollRate_filt, units mm/s) and scaled by
// comLatTrimKd to produce extra foot trim. Final trim, before the cap:
//
//   trim = comLatDisp * comLatTrimGain      (W2 — displacement / Kp)
//        + comLatVel  * comLatTrimKd        (W5 — velocity     / Kd)
//
// comLatTrimKd units: mm of foot trim per (mm/s) of CoM velocity = seconds.
// DEFAULT 0.0 — W5 is INERT until tuned, so out of the box the trim is
// bit-identical to W2. Raise in small steps (try 0.02 .. 0.15 s) and stop
// at the first sign of servo buzz or new oscillation.
//
// The roll rate is low-pass filtered before the Kd term uses it: raw BMI160
// rollRate carries high-frequency gyro hiss that a Kd term would amplify
// straight into the servos. Alpha is the IIR retention factor:
//
//   rollRate_filt = alpha * rollRate_filt + (1 - alpha) * rollRate
//
// alpha = 0.0  → no filtering (raw rate, noisy).
// alpha → 1.0  → heavy smoothing (quiet, but more phase lag → less damping).
// DEFAULT 0.70 — moderate smoothing: kills hiss while keeping enough
// bandwidth for the damping to act at stepping cadence.
constexpr float GAIT_COM_LAT_TRIM_KD_DEFAULT     = 0.01f;
constexpr float GAIT_COM_LAT_TRIM_RATE_LPF_ALPHA = 0.70f;

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
constexpr float GAIT_CAPTURE_GAIN_DEFAULT = 0.15f;

// Hard cap on the capture offset's contribution to a single step (mm).
// Must remain inside the IK reach budget after combining with stepLengthMm.
// At stanceHeightMm=185 and stepLengthMm=30, the swing foot already reaches
// ~98% at peak. This 15 mm cap leaves ~2% IK headroom before clamping.
// Re-tune after first hardware run; do not raise blindly.
constexpr float GAIT_CAPTURE_OFFSET_MAX_MM = 15.0f;

// ------------------------------------------------------------
// LATERAL (FRONTAL-PLANE) CAPTURE-POINT — swing-foot placement.
// ------------------------------------------------------------
// Frontal analogue of the sagittal capture term above. Applied to the SWING
// leg's landing y only (stance leg untouched). Mechanism:
//
//   vy_CoM_mm_s    = H_COM_MM * rollRate_filt          (mm/s, LIP)
//   latCaptureRaw  = vy_CoM_mm_s * GAIT_CAPTURE_INV_OMEGA0_S
//                                 * latCaptureGain * latCaptureSign
//   applied        = clamp(raw, ±latCaptureMaxMm) * smoothstep(localPhase)
//                                                  * strideScale
//
// The lateral LIP natural frequency omega0 is identical to the sagittal
// case (same CoM height H_COM_MM), so GAIT_CAPTURE_INV_OMEGA0_S is shared.
//
// DEFAULT GAIN: 0.15 — same starting-conservative rationale as the sagittal
// capture (well below full LIP correction; raise only after hardware shows
// the swing-foot placement is responsive but not chasing noise).
//
// DEFAULT OFFSET CAP: 15 mm — derived from the frontal reach budget.
// L_FRONTAL_MM = 166 mm, CHAIN_HEIGHT_DELTA_MM ≈ 25.4 mm → at stanceH=185
// the frontal chain length is ~159.6 mm, and IK_MAX_REACH_FRACTION (0.99)
// gives ~158 mm of total |y| budget. After deducting the realistic
// frontal-plane consumers — WS lateral shift up to ~40 mm
// (GAIT_STANCE_WIDTH_MM), comLatTrimMaxMm up to 20 mm, and balance
// correction dy of a few mm — roughly 95 mm of headroom remains. A 15 mm
// hard cap therefore stays well inside frontal IK reach in any realistic
// combination. Matches the sagittal cap so both axes scale together.
// Re-tune after first hardware run; do not raise blindly.
constexpr bool  GAIT_LAT_CAPTURE_ENABLED_DEFAULT = true;
constexpr float GAIT_LAT_CAPTURE_GAIN_DEFAULT    = 0.5f;
constexpr float GAIT_LAT_CAPTURE_OFFSET_MAX_MM   = 30.0f;

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
constexpr float RECOVERY_STEP_HEIGHT_LATERAL_MM  = 5.0f;  // tune 18–25
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
// FOOT IMPACT DETECTION
// L3.5 stateful detector. Telemetry-only at L0; no control consumer.
// See include/foot_impact.h for signal definition.
// ============================================================

// Master enable at boot. True so the telemetry pipeline is live as soon as
// the operator opens the UI panel. Detector does not act on detections at
// L0 even when enabled — actuation is a future-phase concern (L1+).
constexpr bool FOOT_IMPACT_ENABLED_DEFAULT = true;

// Threshold on |first-difference of LPF-smoothed |a||. Default is a
// placeholder pending empirical characterization on real hardware.
// Units: g.
constexpr float FOOT_IMPACT_THRESHOLD_G_DEFAULT = 0.05f;

// First-order LPF coefficient on the magnitude stream before differencing.
// alpha=1.0 disables smoothing; alpha<1.0 attenuates both noise and the
// transient of interest.
constexpr float FOOT_IMPACT_LPF_ALPHA_DEFAULT = 0.5f;

// Per-side refractory after a detection. Prevents one physical event
// from being counted as multiple events while the sensor rings. Units: ms.
constexpr uint32_t FOOT_IMPACT_REFRACTORY_MS_DEFAULT = 75;

// ============================================================
// TOUCHDOWN COMPLIANCE  (W4 — event-triggered geometric yield)
// ============================================================
// The early-contact / swing-truncation event (the touchdown latch in
// GaitController::update()) also fires a brief stance-height yield: the
// effective stance height is shortened by complianceDepthMm so both legs
// flex a few mm and absorb the landing impulse, then the offset smoothsteps
// back to nominal over complianceDecayMs. Transient and self-clearing — the
// offset always returns to zero, so there is no persistent posture bias.
//
// Depth is intentionally small — a few mm of knee flex, not a squat. Start
// conservative; raise only after confirming no oscillation. The decay window
// smears the impulse across tens of ms instead of one tick; too short and
// the recovery itself becomes a second transient, too long and it bleeds
// into the next half-cycle's load transfer.
constexpr bool     GAIT_TD_COMPLIANCE_ENABLED_DEFAULT  = true;
constexpr float    GAIT_TD_COMPLIANCE_DEPTH_MM_DEFAULT = 4.0f;   // mm of yield
constexpr uint32_t GAIT_TD_COMPLIANCE_DECAY_MS_DEFAULT = 200;    // recovery time

// ============================================================
// TOUCHDOWN TRIM SUPPRESSION  (phase-gate of the W2/W5 lateral trim)
// ============================================================
// The W2/W5 lateral CoM trim (_comLateralTrimMm) is driven by torso roll. At
// touchdown the foot-strike injects a short roll transient into the IMU that
// is NOT a real CoM excursion — feeding it into the trim makes the support
// polygon chase a phantom. This suppression freezes (or zeroes) the trim while
// the touchdown latch is held, and for a short hold window afterward, so the
// transient never reaches the foot-placement correction.
//
//   ENABLED_DEFAULT — master enable. Default ON: the transient leak is real
//                     and the suppression is conservative (it only ever
//                     reduces trim authority during the touchdown window).
//   HOLD_MS_DEFAULT — ms the suppression extends past the moment the latch
//                     clears, so the post-strike roll ring-down is also
//                     rejected. 60 ms ≈ a few control ticks past _advanceHalf.
//   FREEZE_DEFAULT  — suppression mode. true = freeze the trim at its last
//                     clean (pre-touchdown) value; false = force trim to 0.
//                     Default true (freeze): holds the correction the robot
//                     already needs instead of introducing a step to zero.
constexpr bool     GAIT_TD_TRIM_SUPPRESS_ENABLED_DEFAULT = true;
constexpr uint32_t GAIT_TD_TRIM_HOLD_MS_DEFAULT          = 80;   // post-latch hold
constexpr bool     GAIT_TD_TRIM_FREEZE_DEFAULT           = true; // freeze vs. zero

// ============================================================
// FORWARD CoM PROGRESSION  (transfer-shaping, body-side complement of change A)
// ============================================================
// Swing-phase-timed forward CoM offset. Applied symmetrically to BOTH feet's
// x_mm so the body advances forward relative to the feet over the course of
// the swing — i.e. the CoM is progressing forward over the incoming support
// foot AT touchdown instead of arriving late and forcing a post-landing
// catch-up. Targets the residual sagittal transfer problem (body arriving
// late relative to the support exchange) that remains after change A fixed
// the swing FOOT's horizontal contact velocity.
//
// SIGN CONVENTION: matches WeightShift::getStanceShift's forward_lean
// contribution, which applies x_offset = -smoothedForwardLean_mm on BOTH
// legs. A forward body shift means the feet sit further BEHIND the hips in
// body frame → negative x_mm on both feet. _buildFootTarget therefore
// subtracts the (non-negative) progression magnitude from t.x_mm on both
// legs. A wrong sign would pull the body backward and worsen the late-
// arrival failure mode; do not flip without re-confirming against
// weight_shift.cpp:getStanceShift().
//
// PHASE SHAPING: shape(localPhase) ramps 0 → 1 across the swing using a
// front-loaded smoothstep (same lpw-compression pattern as W-stride-front-
// load, but applied to a body offset instead of foot placement). shape(0)
// is zero so lift-off is unaffected; shape reaches 1.0 at fwdComLeadFrac
// and holds at 1.0 until touchdown, so the CoM is at its full forward
// offset BEFORE contact (i.e. it leads touchdown). Default lead = 1.0 =
// pure smoothstep (no front-load) — lower toward 0.5 to push the
// progression earlier in the swing.
//
// COMPOSITION (the double-count guard):
//   1. Stance-leg retreat already drives forward body translation. This
//      term is ADDITIVE on top of that. Tune with the existing
//      forward_lean_mm set to 0 so progression sources don't stack.
//   2. The touchdown latch (_touchdownLockedX_mm) freezes only the
//      SWING LEG'S stride x contribution. This term is a BOTH-LEGS body
//      offset and is intentionally NOT latched — it keeps ramping on
//      localPhase exactly like the stance retreat does, so a small forward
//      progression continues to accrue between latch and _advanceHalf.
//      The two are independent and never fight on the same x component.
//
// TUNING (off-by-default; tune up cautiously):
//   - Ship at 0.0 mm: bit-identical to current gait, fwdComProgressionMm
//     reads 0 in telemetry.
//   - Raise in ~1 mm steps. Failure mode B (rebound / "body arrives late")
//     should drop FIRST. STOP / back off as soon as failure mode A
//     (forward tip) starts rising — that is the optimum.
//   - Watch lastReachPctL/R: a forward CoM bias adds to the SWING leg's
//     reach near +L/2 (the term subtracts from x, so swing-foot x at
//     touchdown becomes more negative — increasing |x| at +L/2).
//   - If raising never reduces mode B, the late-arrival diagnosis is
//     wrong: stop and reconsider before building double-support blending.
constexpr float GAIT_FWD_COM_PROGRESSION_MAX_MM_DEFAULT   = 0.0f;
constexpr float GAIT_FWD_COM_PROGRESSION_LEAD_FRAC_DEFAULT = 1.0f;

// ============================================================
// SWING-FOOT LEVELER (Section 14)
// Closed-loop ankle pitch/roll correction during swing phase.
// Reads swing-leg foot IMU; emits additive bias on top of IK angles.
// Enabled=false by default; opt-in via CMD:SWING_LEVELER_TUNE:enabled=1.
// ============================================================

// P gain: deg of ankle correction per rad of foot tilt error.
// 1 rad ~= 57 deg, so kp=25 -> ~0.44 deg per deg of foot tilt. Conservative
// start; tune toward 0.7 on hardware during L1.
constexpr float SWING_LEVELER_KP_PITCH_DEG_PER_RAD = 50.0f;
constexpr float SWING_LEVELER_KP_ROLL_DEG_PER_RAD  = 50.0f;

// Bias clamps. Must remain inside IK ankle soft-limits with margin for the
// IK-computed flat-foot value (which already consumes most of the range
// during deep swing arcs).
constexpr float SWING_LEVELER_MAX_PITCH_BIAS_DEG = 12.0f;
constexpr float SWING_LEVELER_MAX_ROLL_BIAS_DEG  = 20.0f;

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
constexpr float SWING_LEVELER_ENGAGE_LIFT_FRAC    = 0.6f;
constexpr float SWING_LEVELER_PEAK_LIFT_FRAC      = 1.0f;
constexpr float SWING_LEVELER_DISENGAGE_LIFT_FRAC = 1.01f;

#endif // PROJECT_WIDE_DEFS_H
