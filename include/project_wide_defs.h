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

// --- Control-loop rate (B2) ---
// Single source of truth for the 400 Hz IMU/estimator/control gate.
// The main-loop gate period and every UI-facing nominal-dt derivation come
// from this constant. MUST match the BMI160 ODR configured in IMU_init() —
// change both together, then re-verify the foot-IMU loop budget telemetry.
//
// NOTE: runtime control dt is still MEASURED (micros() delta inside the
// gate, capped in the estimator). This nominal value is for gate scheduling
// and display math only — do NOT use it as a hardcoded dt in control code
// (CODING_STANDARD dt rule).
constexpr uint32_t IMU_LOOP_HZ          = 400;
constexpr uint32_t IMU_LOOP_INTERVAL_US = 1000000UL / IMU_LOOP_HZ;  // 2500 µs at 400

// --- Servo PWM frame rate (B6 — default-inert readiness) ---
// PCA9685 output frame rate. 50 Hz is the shipped default and MUST stay 50
// until the hardware validation ladder below is run — the constant exists so
// the raise is a one-line change, not a code hunt.
//
// The pulse math is already frequency-parametric: degToPulse() produces
// microseconds and the single hardware write is pwm.writeMicroseconds(),
// which converts µs → counts from the chip's LIVE prescale. Changing this
// constant alone therefore keeps every commanded pulse width correct.
//
// Why raise it (later): resolution. The PCA9685 has 4096 counts per frame:
//   50 Hz  → 20000 µs frame → ~4.88 µs/count (≈0.66° on a 270°/2000 µs servo)
//   300 Hz →  3333 µs frame → ~0.81 µs/count (≈0.11°)
// Note servo backlash (~1–2°) still dominates precision — treat a raise as
// a latency/refresh improvement, not an accuracy one.
//
// HARD CEILING: the frame period must contain the widest pulse. SD_USMAX =
// 2500 µs → ceiling ≈ 1e6/2500 = 400 Hz with ZERO margin; with margin, stay
// ≤ ~333 Hz. A static_assert in servo_driver.cpp enforces this at compile
// time.
//
// VALIDATION LADDER (bench, robot on a stand, before any raise ships):
//   1. Confirm every installed servo model is rated for the target frame
//      rate (all current servos are rated ≥330 Hz).
//   2. Raise in steps: 50 → 100 → 200 → 250/300. At each step, flash and
//      check: no jitter at rest, no servo heating after 5 min holding
//      stance, no brown-out under a weight-shift cycle (higher frame rate
//      raises average servo current draw).
//   3. Verify joint calibration is unchanged (it must be — pulses are
//      µs-based; any shift means a servo is misbehaving at that rate).
//   4. Re-run a 60 s stepping session and compare GAIT: telemetry (IK reach,
//      flat-foot error, touchdown timing) against the 50 Hz baseline.
// DO NOT raise the default without completing the ladder (handoff B6 rule).
constexpr uint32_t SERVO_FRAME_HZ = 50;

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
#define JOINT_SPEED_DEFAULT_DEG_S   300.0f   // All joints at boot
#define JOINT_SPEED_MIN_DEG_S        1.0f   // Floor — prevents silent frozen-joint bugs
// No global MAX: upper clamp is now per-joint (JointConfig::noLoadSpeedDegS).

// Dedicated speed (deg/s) for deliberate pose transitions (Neutral / Stance
// buttons), separate from the global per-joint cap above so gait/IK responsiveness
// and pose-move smoothness are tuned independently. Armed one-shot per move via
// JointModel::armTransition(); runtime-tunable via setTransitionSpeed(). Lower =
// gentler. Reboot restores this default.
#define JOINT_TRANSITION_SPEED_DEFAULT_DEG_S   20.0f

// --- Single source of truth for joint metadata is joint_config.h ---

// No global MAX: upper clamp is now per-joint (JointConfig::noLoadSpeedDegS).

// --- Balance controller safe gain limits ---
// Hard ceiling enforced by WebComm when parsing BALANCE_TUNE commands.
// Derived from phase-margin analysis: at ~50ms servo lag and
// ω₀ = sqrt(g/h) ≈ 6.68 rad/s (H_COM_MM = 220 — see robot_geometry.h),
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
// TUNING HISTORY: 0.4 → 0.25 (2026-04, IMU-roll-gate era) → raised back up as
// stance loading stabilised; shipped value below is the current tuned rate.
// (B7b comment sync 2026-07: the old note still claimed 0.25 was current.)
constexpr float GAIT_PHASE_RATE_HZ          = 0.7f;  // cycles/sec

// TUNING HISTORY: 20 → 15 → 12 mm. The lift gate prevents lifting before the
// stance leg is loaded, so less clearance is needed to avoid drag.
// If drag returns at 12 mm, the root cause is the gate, not step height.
constexpr float GAIT_STEP_HEIGHT_MM         = 12.0f;  // swing foot clearance
constexpr float GAIT_STANCE_WIDTH_MM        = 40.0f;  // lateral CoM amplitude
constexpr float GAIT_STEP_LENGTH_MM         = 37.0f;  // Phase 3+; forward stride

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
// descent visibly reduces touchdown disturbance. Default lowered 0.5 → 0.15,
// then settled at the shipped 0.2 (matches GAIT_DESIGN §3.6).
constexpr float GAIT_SWING_APEX_FRAC_DEFAULT = 0.2f;

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
// SHIPPED AT 0.2 — this is a tuned operating default, NOT the inert 1.0 the
// original experiment shipped with (B7b comment sync 2026-07). The stride is
// strongly front-loaded: the swing foot completes its horizontal travel by
// localPhase 0.2 and holds until touchdown.
//
// TUNING: for the front-loading to act before the touchdown latch can fire,
// keep strideCompleteFrac <= touchdownGatePhase (shipped 0.45 — see
// GaitConfig); 0.2 <= 0.45 holds. Clamped to (0.05, 1.0] in GAIT_TUNE.
constexpr float GAIT_STRIDE_COMPLETE_FRAC_DEFAULT = 0.2f;

// WeightShift swing-bias preload completion fraction. The swing-side ankle-roll
// bias is masked by (1 - liftFrac); that mask only reaches full magnitude (the
// commanded ankle_shift) at peak swing — i.e. AT contact. The slow ankle servos
// cannot reach the commanded attitude that late, so the swing foot lands canted
// inward. This front-loads the mask: the swing scale becomes
//   scaleSwing = clamp((1 - liftFrac) / swingPreloadCompleteFrac, 0, 1)
// so full bias is reached by the time liftFrac has fallen to
// (1 - swingPreloadCompleteFrac) — earlier in the descent — then held, giving
// the servo time to achieve the attitude before touchdown.
//
// 1.0 = no front-load; reduces EXACTLY to the unmodified (1 - liftFrac) mask
// (since 1 - liftFrac is already in [0, 1], the clamp is identity). Ship at 1.0 —
// flashing must change nothing until tuned down at runtime. Lower toward ~0.5 to
// reach full swing bias halfway through the descent. Clamped to (0.05, 1.0] in
// WEIGHT_SHIFT_TUNE.
constexpr float GAIT_WS_SWING_PRELOAD_COMPLETE_FRAC_DEFAULT = 1.0f;

// Swing-foot ankle-roll PREPARATION (support-transition prep). Supersedes the
// WeightShift swing-bias front-load above: instead of masking the CoM-shift
// bias on the airborne foot, GaitController drives the swing foot's ankle roll
// toward the orientation it will need as the NEXT stance foot, reaching it
// before touchdown. Composed in the single ankle-roll summation in
// _submitFootTargetIK; see GaitController::_swingPrepRollBias.
//
//   target = swingPrepGain * ankle_shift_deg * futureStanceSign
//   d      = clamp((1 - liftFrac) / swingPrepCompleteFrac, 0, 1)
//   bias   = target * (d*d*(3 - 2*d))            // smoothstep, zero-slope ends
//
// SHIPPED ACTIVE: ENABLED=true with GAIN=0.3 — the term is LIVE by default
// (B7b comment sync 2026-07: an earlier note claimed gain 0.0 / no-op; the
// gain has since been tuned up on hardware). The swing-prep envelope is 0
// through lift-off and the rise, then ramps monotonically across the DESCENT
// (apex → touchdown). COMPLETE_FRAC 1.0 = completes at touchdown; lower
// toward 0.5 to complete the outward rotation earlier in the descent.
// Clamped (0.05, 1.0].
constexpr bool  GAIT_SWING_PREP_ENABLED_DEFAULT       = true;
constexpr float GAIT_SWING_PREP_GAIN_DEFAULT          = 0.3f;
constexpr float GAIT_SWING_PREP_COMPLETE_FRAC_DEFAULT = 0.5f;

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

// Idle/standing stance-width inset (mm). Positive = both feet pulled
// symmetrically INWARD toward the body midline (adduction). The lateral
// posture baseline — sagittal analog is GAIT_STANCE_HEIGHT_MM above.
//
// Applied as the y_mm baseline in _buildFootTarget. FootTarget.y_mm uses
// outward(+) / inward(-) in each leg's own frame (gait_types.h), so an
// inset of +N becomes t.y_mm = -N on BOTH legs — symmetric adduction.
// LegIK's frontal solve enforces the flat-foot constraint for any y_mm
// (returned as ankle_roll = +hip_roll on this hardware — see
// leg_ik.cpp solveFrontal, verified 2026-05-30), so hip-roll and
// ankle-roll coordinate automatically and the sole stays flat. Do NOT
// add a hand-coordinated ankle-roll bias.
//
// SHIPPED AT 7.0 mm — a tuned operating default (B7b comment sync 2026-07:
// the original note described the inert 0.0 bring-up value and an
// unverified sign; the inset has since been tuned on hardware). Tune in
// ~2 mm steps at runtime via CMD:GAIT_TUNE:stanceInset_mm=...
constexpr float GAIT_STANCE_INSET_MM            = 7.0f;

// Static fore-aft offset applied to every FootTarget.x_mm built by GaitController.
// Compensates for CoM ≠ hip-pitch axis: with x_mm=0 the foot lands directly under
// the hip pitch axis, but the body CoM sits forward of that, so the robot leans
// back during stance and crouching. Positive = foot shifts FORWARD relative to
// hip (body shifts BACK over the foot). Negative = foot shifts BACKWARD (body
// leans forward). Same role as UVC's adjFR (uses -2.04mm).
//
// Tune from 0 in ±1mm increments until ankle-pitch IK output sits near each
// joint's neutralDeg in standing pose. Typical magnitude: ≤ 10 mm.
constexpr float GAIT_STANCE_X_OFFSET_MM         = -1.0f;

// Hold duration (ms) GaitController inserts between half-cycles. Weight shift
// is centered (trigger NONE) and held for this duration so mechanical/IIR
// transients damp before the next direction is commanded. Should be ≥ ~5×
// the IIR time constant from WeightShiftConfig::shift_smooth_alpha.
// At alpha=0.90 / 400 Hz, τ ≈ 25 ms → the shipped 180 ms gives ~7τ.
constexpr uint32_t GAIT_STABILIZE_MS            = 180;
constexpr uint32_t GAIT_STOP_RAMP_MS            = 600;

// ============================================================
// LIFT GATE DWELL & TIMEOUT  (Stage 2 -- support-transfer robustness)
// ============================================================
// Required continuous-OK duration before WAIT_SHIFT exits to SWINGING.
// Converts the single-tick edge trigger into a persistence-confirmed
// transition. Filters transient predicate satisfactions that don't
// represent a stably-completed support transfer.
//
// Shipped 40 ms ~= 16 ticks at 400 Hz -- long enough to debounce ramp-edge
// transients and brief predicate flickers, short enough that the
// added per-step latency is mechanically invisible. (History: introduced
// at 80 ms, halved after hardware sessions.)
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
constexpr float GAIT_COM_LATERAL_GATE_MM_DEFAULT = 5.0f;

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
constexpr float GAIT_COM_LAT_TRIM_GAIN_DEFAULT   = 0.0f;
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
// SHIPPED AT 0.01 — a small live damping term (B7b comment sync 2026-07:
// an earlier note claimed 0.0/inert). Note the W2 master enable
// (comLatTrimEnabled) ships FALSE, so the whole trim — Kp and Kd — is
// inactive until the trim is enabled at runtime; this Kd then applies.
// Raise in small steps (0.02 .. 0.15 s) and stop at the first sign of
// servo buzz or new oscillation.
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
//   omega0 = sqrt(g / h) = sqrt(9.81 / 0.220) ≈ 6.68 rad/s → 1/omega0 ≈ 0.1498 s
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
// SHIPPED GAIN: 1.0 — FULL LIP correction (B7b comment sync 2026-07: the
// original conservative 0.15 note was stale; the gain has been tuned to
// full on hardware). Back off toward 0.15 if placement starts chasing
// gyro noise.
//
// SHIPPED OFFSET CAP: 30 mm (raised from the original 15) — still inside
// the frontal reach budget derived below.
// L_FRONTAL_MM = 166 mm, CHAIN_HEIGHT_DELTA_MM ≈ 25.4 mm → at stanceH=185
// the frontal chain length is ~159.6 mm, and IK_MAX_REACH_FRACTION (0.99)
// gives ~158 mm of total |y| budget. After deducting the realistic
// frontal-plane consumers — WS lateral shift up to ~40 mm
// (GAIT_STANCE_WIDTH_MM), comLatTrimMaxMm up to 20 mm, and balance
// correction dy of a few mm — roughly 95 mm of headroom remains. The
// shipped 30 mm hard cap therefore stays inside frontal IK reach in any
// realistic combination. Do not raise blindly.
constexpr bool  GAIT_LAT_CAPTURE_ENABLED_DEFAULT = true;
constexpr float GAIT_LAT_CAPTURE_GAIN_DEFAULT    = 1.0f;
constexpr float GAIT_LAT_CAPTURE_OFFSET_MAX_MM   = 30.0f;

// ============================================================
// PITCH-ANGLE SWING-FOOT FORWARD PLACEMENT  (UVC-style, ANGLE-based)
// ============================================================
// DISTINCT from the rate-based capture point above. This authority is driven by
// the clean torso pitch ANGLE (pitch - torsoPitchNeutralRad), NOT CoM velocity,
// and shares no state, filter, or control logic with _captureOffsetMm(). It is
// applied to the SWING leg's stride x only (stance untouched), additively
// alongside the sagittal capture offset.
//
// HYPOTHESIS: forward torso pitch indicates insufficient forward support
// placement, so placing the swing foot further forward in proportion to clean
// forward pitch should reduce scrub, double-touchdowns, and falls.
//
// MECHANISM (see GaitController::_pitchPlacementOffsetMm):
//   pitch    = _lastImuState.pitch - torsoPitchNeutralRad
//   effPitch = pitch >  db ? pitch - db : pitch < -db ? pitch + db : 0   (deadband)
//   raw      = effPitch * gain * sign
//   clamped  = constrain(raw, ±max)
//   applied  = clamped * smoothstep(localPhase)
//
// SIGN: +1 is correct and proven by inspection — positive torso pitch = leaning
// forward (state_estimator.h), positive FootTarget.x_mm = foot placed forward
// (GAIT_STANCE_X_OFFSET_MM block + _captureOffsetMm doc), so forward lean ->
// foot forward. The runtime sign knob is retained only as a hardware safety
// valve, mirroring latCaptureSign.
//
// DEFAULTS: GAIN ships at 0.0f -> the helper short-circuits to 0 and the term is
// a bit-identical no-op (ENABLED defaults true, but the gain gate makes it inert
// until tuned up at runtime). At a first test gain of ~100 mm/rad with the
// defaults below the ±25 mm clamp is first reached near 16.6° of forward pitch
// (5°->5.4mm, 10°->14.2mm, 15°->22.9mm) — measurable yet non-saturating across
// the 5–15° band.
constexpr float GAIT_PITCH_PLACEMENT_GAIN_MM_PER_RAD = 0.0f;   // ship OFF
constexpr float GAIT_PITCH_PLACEMENT_MAX_MM          = 25.0f;
constexpr float GAIT_PITCH_PLACEMENT_DEADBAND_RAD    = 0.033f; // ~1.9°
constexpr float GAIT_PITCH_PLACEMENT_SIGN            = +1.0f;
constexpr bool  GAIT_PITCH_PLACEMENT_ENABLED         = true;

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
// Set below recovery_tilt_roll_rad (0.13 rad shipped) so it fires on residual
// lean that the first step reduced but didn't fully recover. Set above normal
// walking roll (~0.05 rad) so it never fires after a clean step landing.
// Shipped 0.12 rad ≈ 6.9° (raised from the initial 0.07 after spurious
// second steps). Tune after T2.5 if chaining is still spurious.
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
constexpr bool     GAIT_TD_COMPLIANCE_ENABLED_DEFAULT  = false;
constexpr float    GAIT_TD_COMPLIANCE_DEPTH_MM_DEFAULT = 2.0f;   // mm of yield
constexpr uint32_t GAIT_TD_COMPLIANCE_DECAY_MS_DEFAULT = 200;    // recovery time

// ============================================================
// H-RESTORE  (deferred swing-leg stance-height restoration)
// ============================================================
// After an early touchdown the swing leg's h is frozen at the touchdown
// snapshot (no in-swing re-extension — re-extending a planted leg forward of
// the CoM produces a backward braking impulse). The deficit between that
// frozen h and stanceHeightMm is then restored across the NEXT stance phase,
// gated on the new stance leg's _halfPhase, surviving the role swap with no h
// discontinuity. This removes the instantaneous h snap at _advanceHalf that
// otherwise causes a backward impulse.
//
// hRestoreCompleteFrac is the new-half phase at which the restore completes —
// the foot is ~under the hip here, so re-extending it no longer brakes the
// body. Clamped (0.05, 1.0] at use.
constexpr bool  GAIT_H_RESTORE_ENABLED_DEFAULT       = true;
constexpr float GAIT_H_RESTORE_COMPLETE_FRAC_DEFAULT = 0.5f;

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

// ============================================================
// HEEL-STRIKE ROLLOVER  (off-by-default, hybrid phase+timer)
// ============================================================
// Replaces the rigid forefoot-edge vault (land flat → vault forefoot → PE→KE
// → backward rebound) with a translating rocker contact. The bias is
// intrinsically DORSIFLEXIVE: applied as a NEGATIVE ankle-pitch offset
// (positive ankle_pitch is plantar-flexion in this joint convention; heel-up
// = negative). The sign is baked into _heelStrikeBiasDeg() and is NOT user-
// configurable — confirmed on hardware, not exposed in the UI.
//
// Bias is ADDITIVE AFTER IK — the flat-foot guard is unaffected (it checks
// the pure IK solution, not the final submitted angle).
//
// Hybrid timing (post-redesign — modelled on W4 compliance decay):
//   ENGAGE  — phase-based: ramp 0→1 across [heelStrikeGatePhase, touchdown-
//             GatePhase], then HOLD at 1.0 until contact occurs. Full peak
//             is reached BEFORE touchdown; no jump at the engage→release
//             transition.
//   RELEASE — timer-based, decoupled from swing-side ownership: starts with
//             a snapshot of the engage frac (continuous, no upward jump),
//             decays to 0 over heelStrikeReleaseMs via smoothstep. Survives
//             the half-cycle role swap exactly as W4 compliance does.
//   FALLBACK — if touchdown is never detected during the swing, _advanceHalf
//             starts the release from the current peak instead of snapping.
// ============================================================

// Master enable at boot. false = true hardware no-op: _heelStrikeBiasDeg()
// returns 0 unconditionally, ankle-pitch submit is bit-identical to pre-feature.
constexpr bool GAIT_HEELSTRIKE_ENABLED_DEFAULT = false;

// Peak dorsiflexion magnitude (deg). The actual bias is the negative of this
// times the engage/release fraction. Shipped 3° — a gentler rocker than the
// originally-recommended 5–7° band (feature ships DISABLED regardless).
constexpr float GAIT_HEELSTRIKE_DORSIFLEX_DEG_DEFAULT = 3.0f;

// Half-phase fraction at which the engage ramp starts [0..1). Below this the
// bias is 0; above it the bias ramps 0 → peak via smoothstep over the window
// [heelStrikeGatePhase, touchdownGatePhase], then HOLDS at peak until contact.
// Shipped 0.2, pairing with the shipped touchdownGatePhase 0.45 (full peak
// at 0.45). INVARIANT: must remain strictly less than touchdownGatePhase so
// the engage window has nonzero width — 0.2 < 0.45 holds.
constexpr float GAIT_HEELSTRIKE_GATE_PHASE_DEFAULT = 0.2f;

// Time (ms) for the bias to decay from its release-snapshot value to zero
// after touchdown (or _advanceHalf fallback). Shipped 500 ms — raised from
// the original 200–280 ms band; at the shipped 0.7 Hz cadence a half-cycle
// is ~714 ms, so the release still completes before the next lift-off, but
// re-check this if phaseRate is raised. Independent of FSM state — runs to
// completion across the role swap, mirroring W4 compliance.
constexpr uint32_t GAIT_HEELSTRIKE_RELEASE_MS_DEFAULT = 500;

// Hard ceiling on the applied bias magnitude regardless of heelStrike-
// DorsiflexDeg. Safety backstop: combined with pitch_bias from the swing
// leveler (also default OFF), the total cannot exceed this even if both
// features are on. The actual bias is always negative (dorsiflex); clamp
// is symmetric so the magnitude check is unambiguous.
constexpr float GAIT_HEELSTRIKE_MAX_BIAS_DEG = 12.0f;

// ============================================================
// POST-CONTACT ANKLE YIELD  (** SHIPS ACTIVE — see A-09 note below **)
// ============================================================
// A post-contact RELEASE: it relaxes the held-flat ankle command on the
// just-planted leg so the foot can roll forward under the vaulting body. It is
// NOT a driven dorsiflexion and is NEVER present before contact.
//
// CRITICAL — why this is gated 1:1 to detected touchdowns, single-site, no
// fallback: pre-contact dorsiflexion shortens the catch lever and reproduces
// the already-FALSIFIED heel-strike failure mode. The yield therefore arms
// ONLY at the touchdown latch (which already gates on _halfPhase >
// touchdownGatePhase) and has NO _advanceHalf fallback — a missed detection
// yields nothing that step, by design. The swing foot carries zero yield at
// any pre-contact phase (there is no swing-leg branch in _yieldBiasDeg()).
//
// Sign is intrinsically NEGATIVE (dorsiflexion), baked into _yieldBiasDeg();
// the config holds magnitude only and is not user-exposed as a sign.
// Bias is ADDITIVE AFTER IK — the flat-foot guard (which checks the pure IK
// solution) is unaffected, same contract as the heel-strike bias.
//
// Profile: smoothstep ramp-IN over yieldEngageMs (so the release does not itself
// step-impulse the ankle the instant after contact), then smoothstep decay over
// yieldReleaseMs. Timer-based + side-locked: survives the _advanceHalf role swap
// and runs on the same physical (now-stance) foot to completion.
// yieldEngageMs + yieldReleaseMs together stay well inside one half-cycle so the
// bias is back to 0 before the next lift-off.
// ============================================================

// Master enable at boot.
// *** AUDIT A-09 (2026-07): the yield SHIPS ACTIVE — enabled=true AND
// dorsiflexDeg=2.0 — so every detected touchdown arms a 2° dorsiflex
// release. This contradicts the original "ships inert / gain 0" design
// note (retired below) and the companion docs' "specified, not
// implemented" status. Presumed a deliberate in-progress experiment
// (commit "Ankle yield"); PENDING HUMAN DECISION: either bless the active
// default here or restore 0.0. Any A/B run assuming "release off by
// default" is confounded until then. ***
constexpr bool GAIT_YIELD_ENABLED_DEFAULT = true;

// Peak dorsiflexion MAGNITUDE (deg). Shipped 2.0 → ACTIVE (see A-09 note
// above; the original design shipped 0 = no-op). The applied bias is the
// negative of this times the engage/release fraction.
constexpr float GAIT_YIELD_DORSIFLEX_DEG_DEFAULT = 2.0f;

// Ramp-IN time (ms) for the bias to smoothstep 0 → peak after the latch. This is
// NOT a gating stage (the latch already enforced the phase gate) — it only eases
// the bias in so the post-contact release is not a one-tick step-impulse.
constexpr uint32_t GAIT_YIELD_ENGAGE_MS_DEFAULT = 10;

// Decay time (ms) for the bias to smoothstep peak → 0 after the engage ramp.
// Shipped engage(10) + release(400) = 410 ms total — this is NOT "well
// inside a half-cycle": at the shipped 0.7 Hz a half-cycle is ~714 ms minus
// WAIT_SHIFT/STABILIZING, so the release frequently spans into the next
// half (by design it survives the role swap), and at higher cadence a new
// touchdown can find the previous release still active → that step gets NO
// yield and yieldSkipCount increments (see _startYield early-return).
// Watch yieldSkipCount when sweeping cadence or release duration.
constexpr uint32_t GAIT_YIELD_RELEASE_MS_DEFAULT = 400;

// Hard ceiling on the applied bias magnitude regardless of yieldDorsiflexDeg.
// Safety backstop; symmetric so the magnitude check is unambiguous. A few deg
// above expected use.
constexpr float GAIT_YIELD_MAX_BIAS_DEG = 10.0f;

// ============================================================
//  STEERING  (Layer 3.5 hip-yaw turning — Stage 1, open-loop)
// ------------------------------------------------------------
// Drives the two otherwise-undriven hip-yaw joints (ch4 right dir+1,
// ch7 left dir-1, both ±50°) at SOURCE_GAIT to turn the body during
// GaitMode::RUNNING. See STEERING_PLAN.md for the full law + decision
// record. The per-tick output is:
//   yaw = turnEff * yawStepDeg * turnSign * halfSign * (s - 0.5)
// where s = smoothstep(halfPhase) and halfSign flips with the stance leg.
// ============================================================

// Master enable at boot. false → SteeringController::update() submits NOTHING
// (the yaw channels stay an uncontested SOURCE_GAIT slot the UI can still own),
// so the firmware is bit-identical to the pre-steering build. Turn on only after
// the inert + unit-test gates pass (STEERING_PLAN §9).
constexpr bool GAIT_STEER_ENABLED_DEFAULT = false;

// Body-yaw accumulated per footfall at |turnCmd| = 1, in degrees. This is the
// quantity measured on the floor and the clean seam to a future heading-rate
// command (rate ÷ cadence = yaw/step). Start small: smoothstep bounds the yaw
// acceleration but there is no arm counter-swing to cancel the reaction torque.
constexpr float GAIT_STEER_YAW_STEP_DEG_DEFAULT = 2.5f;

// Hard clamp on the per-tick |hip-yaw output| (deg). Far below the ±50° joint
// limit so even a mis-tuned yawStepDeg/turnCmd can never command near the stop.
// JointModel re-clamps to ±50° downstream regardless — this is the first guard.
// Stage 1.2: raised 15 → 16 so it stays >= STEERING_YAW_STEP_MAX_DEG / 2 (= 15):
// at full authority the sweep-window hold endpoints sit at ±A/2 = ±15°, and a
// clamp below that would flat-top them. Behavior-neutral — no current shipped or
// tuned setting reaches the old 15° clamp (prior yawStep max 15 → endpoints ±7.5).
constexpr float GAIT_STEER_MAX_YAW_DEG = 16.0f;

// Max rate of change of the internal effective command _turnEff (units/s, where
// 1 unit = full turnCmd). 2.0 → a full -1→+1 slider slam takes ~1 s. Makes the
// yaw output C¹ no matter when the operator moves the joystick.
constexpr float GAIT_STEER_TURN_SLEW_PER_SEC = 2.0f;

// Hardware-verified turn direction (Stage 1.1). Positive turnCmd must map to the
// observed physical turn direction; -1 is the confirmed mapping on this build.
// This is a HARDWARE PROPERTY, not a tuning parameter — the runtime sign control
// was removed. Re-verify on a hip-yaw servo swap or re-cabling, then edit here
// and reflash. SteeringConfig.turnSign initialises from this.
constexpr float STEERING_TURN_SIGN = -1.0f;

// --- Stage 1.1 characterization telemetry (per-step yaw budget) ---
// Sign that maps each measured gyro-Z (IMUState::yawRate, world z-up CCW+) onto
// the steering body-yaw convention so realizationRatio reads positive for a
// realized turn. Bench-verify once (rotate the robot CCW from above → all three
// yaw rates must read positive); flip here if not. Applied at ingestion.
constexpr float STEERING_YAW_RATE_SIGN = +1.0f;

// IIR retention for the live torso yaw-rate DISPLAY only (yawRateDps). Rejects
// footstrike spikes for a readable number; does not affect the per-step budget
// integrals (those integrate the unsmoothed rate * dt).
constexpr float STEERING_YAW_RATE_LPF_ALPHA = 0.90f;

// Rolling-mean window (footfalls) for the budget telemetry — mirrors the
// GaitState step-timing ring sizing.
constexpr uint8_t STEERING_STEP_RING_SIZE = 6;

// realizationRatio = bodyYawStepDeg / cmdYawStepDeg is guarded to 0 when the
// commanded step yaw magnitude is below this (deg), so a near-straight step does
// not divide measurement noise into a meaningless ratio.
constexpr float STEERING_RATIO_MIN_CMD_DEG = 1.0f;

// --- Stage 1.2 sweep windowing ---
// The yaw smoothstep runs over [sweepStartFrac, sweepEndFrac] of the half-cycle
// and HOLDS at the ±A/2 endpoints outside that interval.
//   (a) Defaults 0/1 reduce the windowed law EXACTLY to the legacy smoothstep —
//       a bit-identical no-op. Ship inert; tune on hardware.
//   (b) Purpose: relocate the yaw sweep into the phase interval where the swing
//       foot is reliably AIRBORNE. Yaw commanded while both feet are grounded
//       resolves into symmetric foot splay (zero net heading), not turning — so
//       sweeping during double support spends authority on scrub, not rotation.
//   (c) Peak-rate note: qdotPeak = 1.5 * yawStepDeg / ((wEnd - wStart) * T_half),
//       with T_half = 1 / (2 * phaseRate). A narrow window, max yawStepDeg, and a
//       high phaseRate together spike the hip-yaw rate — do NOT combine all three
//       in one trial.
//   (d) Suggested first operating window (tune on hardware): start in
//       [0.15 .. 0.30], end in [0.70 .. 0.85].
constexpr float STEERING_SWEEP_START_DEFAULT = 0.0f;
constexpr float STEERING_SWEEP_END_DEFAULT   = 1.0f;
// Minimum sweep width: the law (and the STEER_TUNE parser) keep
// sweepEnd >= sweepStart + this, so the smoothstep denominator never degenerates.
constexpr float STEERING_SWEEP_MIN_WIDTH     = 0.2f;

// --- Stage 1.2 authority ceiling ---
// Parse-time ceiling for the yawStep= STEER_TUNE key (deg of body-yaw/footfall).
// Chosen ABOVE the expected physical limit: double-support splay is A/2 per foot
// at the boundary hold (15° at A=30), which already couples ~26% of the lateral
// weight-shift ramp into foot-frame fore-aft. Raising past 30 is a design review,
// not a retune. Does NOT change the shipped yawStepDeg default (still 2.5).
constexpr float STEERING_YAW_STEP_MAX_DEG = 30.0f;

#endif // PROJECT_WIDE_DEFS_H
