// =============================================================================
// FILE:    balance_controller.cpp
// MODULE:  control
// LAYER:   3 — Balance Controller
//
// LAST CHANGED: 2026-04-08 | Hinz | Task-1 gain boost, Task-2 ankle bias
// =============================================================================

#include "balance_controller.h"
#include "oe_control.h"
#include <math.h>
#include "motion_manager.h"

// ---------------------------------------------------------------------------
BalanceController::BalanceController(ServoControl* servo)
    : _servo(servo), _cfg(), _lastState() {}

// ---------------------------------------------------------------------------
void BalanceController::init() {
    _prevPitchEnabled = _cfg.pitch_enabled;
    _prevRollEnabled  = _cfg.roll_enabled;
    resetOutputState();
}

// ---------------------------------------------------------------------------
//  resetOutputState() — zero all transient state, preserve config.
//  Call from oe_clear() in main.cpp after ESTOP to prevent command spikes.
// ---------------------------------------------------------------------------
void BalanceController::resetOutputState() {
    _pitchRateFiltered = 0.0f;
    _rollRateFiltered  = 0.0f;
    _fallTickCount     = 0;
    for (uint8_t i = 0; i < BJI_COUNT; i++) {
        _jointState[i] = {};
    }
    // Reset transition flags to current enabled state so BUG-04 detector
    // starts clean without a spurious disable-on-first-tick event.
    _prevPitchEnabled = _cfg.pitch_enabled;
    _prevRollEnabled  = _cfg.roll_enabled;

    // Stage 3: zero task-space filter state.
    _dx_filtered_mm      = 0.0f;
    _dy_filtered_mm      = 0.0f;
    _ts_pitch_u_clamped  = 0.0f;
    _ts_roll_u_clamped   = 0.0f;
    _ts_pitch_error      = 0.0f;
    _ts_roll_error       = 0.0f;
    _lastCorrection      = {};

    Serial.printf("[BalanceController] Reset. pitch_en=%s roll_en=%s "
                  "Kp=%.1f(boost×%.1f@%.3f) Kd=%.2f iir=%.2f rate=%.2f\n",
                  _cfg.pitch_enabled ? "Y" : "N",
                  _cfg.roll_enabled  ? "Y" : "N",
                  _cfg.Kp, _cfg.gain_boost_factor, _cfg.gain_boost_threshold_rad,
                  _cfg.Kd, _cfg.output_iir_alpha,
                  _cfg.max_output_rate_deg_per_tick);
}

// ---------------------------------------------------------------------------
//  _computeEffectiveKp() — TASK-1: nonlinear gain boost.
//
//  Returns a linearly ramped Kp between baseKp and baseKp × boostFactor:
//    absError < threshold              → baseKp (no boost)
//    threshold <= absError < 2×thresh  → linear ramp baseKp → baseKp × factor
//    absError >= 2 × threshold         → baseKp × boostFactor (full boost)
//
//  Physical rationale:
//    Oscillations occur near equilibrium where |error| < ~0.05 rad. Setting
//    threshold=0.10 keeps those errors in the flat Kp zone. Disturbances
//    (pushes, weight shift steps) produce |error| > 0.15 rad, triggering
//    the boost and providing faster recovery without oscillation risk.
//
//    The linear ramp (rather than a step) prevents a discontinuity in the
//    control law at the threshold crossing, which would itself excite the servo.
// ---------------------------------------------------------------------------
/*static*/ float BalanceController::_computeEffectiveKp(float absError,
                                                          float baseKp,
                                                          float threshold,
                                                          float boostFactor) {
    // If boost is disabled (factor <= 1) or threshold is zero, return flat Kp.
    if (boostFactor <= 1.0f || threshold < 1e-4f) return baseKp;

    float excess = absError - threshold;
    if (excess <= 0.0f) return baseKp;  // below threshold — stable zone

    // Normalise: ramp completes over one additional threshold width.
    float t = constrain(excess / threshold, 0.0f, 1.0f);
    return baseKp * (1.0f + t * (boostFactor - 1.0f));
}

// ---------------------------------------------------------------------------
//  _shapeOutput() — universal output shaping pipeline.
//
//  Pipeline: rawCmd → [zero bypass | delta clamp] → IIR → damp → clamp
//
//  BUG-01 FIX — zero-case bypass:
//    When rawCmd=0 (deadband or axis disabled), clamped is set to 0 directly.
//    Without this, constrain(0, filtered−δ, filtered+δ) returns a non-zero
//    lower bound, preventing the IIR from ever reaching zero. Result: servos
//    keep moving smoothly after the controller is "off" until the IIR decays.
//    With the bypass, the IIR decays at rate (1−alpha)/tick:
//      alpha=0.60: from 15°, reaches ≈0 in ~30ms at 400Hz.
//
//  BUG-05 FIX — clamp after velocity damping:
//    Without clamp: kv=2.0, rate=−5 rad/s, smoothed=6° → 6+10 = 16° output.
//    This exceeds max_correction_deg. The final constrain prevents this.
// ---------------------------------------------------------------------------
float BalanceController::_shapeOutput(uint8_t jIdx, float rawCmd,
                                       float angularRate_rs) {
    JointOutputState& s = _jointState[jIdx];

    // ── Step 1: Delta clamp ───────────────────────────────────────────────────
    float clamped;
    if (fabsf(rawCmd) < 1e-4f) {
        // BUG-01 FIX: zero input → free IIR decay, no spike prevention needed.
        clamped = 0.0f;
    } else {
        // Non-zero: limit per-tick rate-of-change to prevent step spikes.
        // Bounds are relative to current smoothed trajectory (not raw cmd)
        // so the clamp correctly tracks where the servo actually is.
        clamped = constrain(rawCmd,
                            s.filtered_cmd_deg - _cfg.max_output_rate_deg_per_tick,
                            s.filtered_cmd_deg + _cfg.max_output_rate_deg_per_tick);
    }

    // ── Step 2: IIR smoothing ─────────────────────────────────────────────────
    // f_c ≈ (1−alpha) × loop_hz / (2π). alpha=0.60 → f_c ≈ 38 Hz.
    // filtered_cmd_deg is the IIR state — tracks the smoothed trajectory.
    float smoothed = _cfg.output_iir_alpha * s.filtered_cmd_deg
                   + (1.0f - _cfg.output_iir_alpha) * clamped;
    s.prev_cmd_deg     = clamped;   // pre-IIR, for diagnostics
    s.filtered_cmd_deg = smoothed;  // IIR state for next tick

    // ── Step 3: Velocity damping ───────────────────────────────────────────────
    // Mirrors UVC footCont(): A0W[s] = k0 − x0 − 0.003 * fbAV
    // Applied after IIR so it does not perturb the smoothed trajectory state.
    // kv=0 (default) disables entirely.
    float damped = smoothed - _cfg.output_damping_kv * angularRate_rs;

    // ── Step 4 (BUG-05 FIX): Clamp to correction budget ──────────────────────
    // Both pitch and roll joints pass through here; use the larger limit.
    float maxDeg = fmaxf(_cfg.max_correction_deg, _cfg.max_roll_correction_deg);
    return constrain(damped, -maxDeg, maxDeg);
}

// ---------------------------------------------------------------------------
BalanceState BalanceController::update(const IMUState& state) {
    BalanceState out = {};

    // =========================================================================
    //  FALL DETECTION
    //  Gated by fall_detection_enabled (default: false).
    //  Enable via CMD:BALANCE_TUNE:fall_det=1 once the robot stands stably.
    //  Kept before the enabled/disabled gate so it fires even when both
    //  controllers are off — but only when explicitly armed.
    // =========================================================================
    if (_cfg.fall_detection_enabled && state.valid && !oe_is_estopped()) {
        if (fabsf(state.pitch) > _cfg.fall_threshold_rad ||
            fabsf(state.roll)  > _cfg.fall_threshold_rad) {
            _fallTickCount++;
            if (_fallTickCount >= _cfg.fall_confirm_ticks) {
                _fallTickCount = 0;
                oe_estop();
                if (_servo != nullptr) _servo->resetToNeutral();
                out.fell = true;
                Serial.printf("[BalanceController] FALL: pitch=%.3f roll=%.3f "
                              "thr=%.3f → ESTOP\n",
                              state.pitch, state.roll, _cfg.fall_threshold_rad);
                _lastState = out;
                return out;
            }
        } else {
            _fallTickCount = 0;
        }
    } else if (!_cfg.fall_detection_enabled) {
        // Keep counter clean so it doesn't carry state when detection is re-armed.
        _fallTickCount = 0;
    }

    // =========================================================================
    //  BUG-04: Axis disable transition — zero IIR before Gate 1.
    //  Must precede Gate 1 so the "both disabled" case is handled:
    //  if pitch transitions true→false while roll is already false, Gate 1
    //  exits early, never reaching a post-gate check.
    // =========================================================================
    if (_prevPitchEnabled && !_cfg.pitch_enabled) {
        for (uint8_t i = BJI_R_ANKLE_PITCH; i <= BJI_TORSO_PITCH; i++) {
            _jointState[i] = {};
        }
        _pitchRateFiltered = 0.0f;
        Serial.println("[BalanceController] Pitch disabled — IIR/filter zeroed.");
    }
    if (_prevRollEnabled && !_cfg.roll_enabled) {
        for (uint8_t i = BJI_R_ANKLE_ROLL; i <= BJI_TORSO_ROLL; i++) {
            _jointState[i] = {};
        }
        _rollRateFiltered = 0.0f;
        Serial.println("[BalanceController] Roll disabled — IIR/filter zeroed.");
    }
    _prevPitchEnabled = _cfg.pitch_enabled;
    _prevRollEnabled  = _cfg.roll_enabled;

    // ── Gate 1: enabled ───────────────────────────────────────────────────────
    if (!_cfg.pitch_enabled && !_cfg.roll_enabled) {
        // C-02 ext: still apply ankle roll bias when both controllers are off.
        // Without this, WeightShift bias has no effect during gait testing with
        // balance fully disabled — the most common test configuration.
        if (!oe_is_estopped() &&
            (fabsf(_cfg.ankle_roll_bias_l_deg) > 0.01f ||
             fabsf(_cfg.ankle_roll_bias_r_deg) > 0.01f)) {
            _applyRollCorrection(0.0f, 0.0f, 0.0f, 0.0f);
        }
        _lastState = out; return out;
    }
    // ── Gate 2: valid IMU ─────────────────────────────────────────────────────
    if (!state.valid)     { _lastState = out; return out; }
    // ── Gate 3: hardware live ─────────────────────────────────────────────────
    if (oe_is_estopped()) { _lastState = out; return out; }

    // =========================================================================
    //  STAGE 3: TASK-SPACE OUTPUT BRANCH
    //  When task_space_output=true, balance PD outputs dx_mm/dy_mm that the
    //  GaitController merges into FootTargets before LegIK::solve(). Leg joints
    //  (ankle pitch, knee, hip pitch, hip roll, ankle roll) are NOT submitted here.
    //  Torso pitch and torso roll remain joint-space (not in the IK chain).
    // =========================================================================
    if (_cfg.task_space_output) {
        _lastCorrection = computeCorrection(state);

        // Torso pitch — still joint-space; uses cached _ts_pitch_u_clamped.
        if (_motionManager && _cfg.pitch_enabled && _cfg.torso_ratio > 0.001f) {
            float torso_cmd = _ts_pitch_u_clamped * _cfg.torso_ratio;
            float shaped    = _shapeOutput(BJI_TORSO_PITCH, -torso_cmd,
                                           _pitchRateFiltered);
            _motionManager->submit(SOURCE_BALANCE, IDX_TORSO_PITCH, shaped);
        }
        // Torso roll — still joint-space; uses cached _ts_roll_u_clamped.
        if (_motionManager && _cfg.roll_enabled && _cfg.torso_roll_ratio > 0.001f) {
            float torso_cmd = _ts_roll_u_clamped * _cfg.torso_roll_ratio;
            float shaped    = _shapeOutput(BJI_TORSO_ROLL, torso_cmd,
                                           _rollRateFiltered);
            _motionManager->submit(SOURCE_BALANCE, IDX_TORSO_ROLL, shaped);
        }

        out.active       = _cfg.pitch_enabled || _cfg.roll_enabled;
        out.pitch_active = _cfg.pitch_enabled;
        out.roll_active  = _cfg.roll_enabled;
        // Surface PD telemetry from the last computeCorrection run.
        out.pitch_error    = _ts_pitch_error;
        out.roll_error     = _ts_roll_error;
        out.u_clamped      = _ts_pitch_u_clamped;
        out.u_roll_clamped = _ts_roll_u_clamped;

        // Apply WeightShift ankle bias even in task-space mode.
        // V5 invariant: bias is feedforward shaping, not foot placement —
        // it stays in SOURCE_BALANCE via MotionManager, not in the IK chain.
        // Same path as the C-02 fix in the legacy (!pitch && !roll) gate.
        if (fabsf(_cfg.ankle_roll_bias_l_deg) > 0.01f ||
            fabsf(_cfg.ankle_roll_bias_r_deg) > 0.01f) {
            _applyRollCorrection(0.0f, 0.0f, 0.0f, 0.0f);
        }

        _lastState = out;
        return out;
    }

    // =========================================================================
    //  PITCH PD (legacy joint-space path)
    // =========================================================================
    if (_cfg.pitch_enabled) {
        // Derivative filter — updated every tick to keep IIR warm.
        // A cold state causes a Kd spike on deadband exit.
        _pitchRateFiltered = _cfg.derivative_lpf_alpha * _pitchRateFiltered
                           + (1.0f - _cfg.derivative_lpf_alpha) * state.pitchRate;

        // Smith predictor: predict future pitch to partially cancel servo lag.
        float predictedPitch = state.pitch
                             + state.pitchRate * _cfg.servo_lag_compensation_s;
        float error = predictedPitch - _cfg.pitch_setpoint_rad;

        out.pitch_active = true;
        out.pitch_error  = error;

        float u_raw = 0.0f, u_clamped = 0.0f;
        float effKp = _cfg.Kp;  // default (inside deadband or boost disabled)

        if (fabsf(error) >= _cfg.pitch_deadband_rad) {
            // TASK-1: Compute nonlinear effective Kp.
            effKp  = _computeEffectiveKp(fabsf(error), _cfg.Kp,
                                          _cfg.gain_boost_threshold_rad,
                                          _cfg.gain_boost_factor);
            u_raw  = (effKp * error + _cfg.Kd * _pitchRateFiltered)
                     * _cfg.correction_sign;
            u_clamped = constrain(u_raw,
                                  -_cfg.max_correction_deg,
                                   _cfg.max_correction_deg);
        }

        float ankle_cmd = u_clamped * _cfg.ankle_ratio;
        float hip_cmd   = u_clamped * _cfg.hip_ratio;
        float torso_cmd = u_clamped * _cfg.torso_ratio;

        _applyPitchCorrection(ankle_cmd, hip_cmd, torso_cmd, _pitchRateFiltered);

        // Telemetry: pre-shaping values + effective_kp for UI transparency.
        out.u_raw         = u_raw;
        out.u_clamped     = u_clamped;
        out.effective_kp  = effKp;
        out.ankle_cmd_deg = ankle_cmd;
        out.hip_cmd_deg   = hip_cmd;
        out.torso_cmd_deg = torso_cmd;
    }

    // =========================================================================
    //  ROLL PD
    // =========================================================================
    if (_cfg.roll_enabled) {
        _rollRateFiltered = _cfg.roll_derivative_lpf_alpha * _rollRateFiltered
                         + (1.0f - _cfg.roll_derivative_lpf_alpha) * state.rollRate;

        float predictedRoll = state.roll
                            + state.rollRate * _cfg.servo_lag_compensation_s;
        float roll_error = predictedRoll - _cfg.roll_setpoint_rad;

        out.roll_active = true;
        out.roll_error  = roll_error;

        float u_roll_raw = 0.0f, u_roll_clamped = 0.0f;
        float effKpRoll  = _cfg.Kp_roll;

        if (fabsf(roll_error) >= _cfg.roll_deadband_rad) {
            effKpRoll    = _computeEffectiveKp(fabsf(roll_error), _cfg.Kp_roll,
                                               _cfg.gain_boost_threshold_roll_rad,
                                               _cfg.gain_boost_factor_roll);
            u_roll_raw   = (effKpRoll * roll_error
                            + _cfg.Kd_roll * _rollRateFiltered)
                           * _cfg.roll_correction_sign;
            u_roll_clamped = constrain(u_roll_raw,
                                       -_cfg.max_roll_correction_deg,
                                        _cfg.max_roll_correction_deg);
        }

        float ankle_r = u_roll_clamped * _cfg.ankle_roll_ratio;
        float hip_r   = u_roll_clamped * _cfg.hip_roll_ratio;
        float torso_r = u_roll_clamped * _cfg.torso_roll_ratio;

        _applyRollCorrection(ankle_r, hip_r, torso_r, _rollRateFiltered);

        out.u_roll_raw         = u_roll_raw;
        out.u_roll_clamped     = u_roll_clamped;
        out.effective_kp_roll  = effKpRoll;
        out.ankle_roll_cmd_deg = ankle_r;
        out.hip_roll_cmd_deg   = hip_r;
        out.torso_roll_cmd_deg = torso_r;

    } else if (fabsf(_cfg.ankle_roll_bias_l_deg) > 0.01f ||
               fabsf(_cfg.ankle_roll_bias_r_deg) > 0.01f) {
        // C-02 FIX: Roll PD is disabled, but WeightShift has injected a non-zero
        // ankle roll bias. Without this branch, _applyRollCorrection() is never
        // called, so the bias is never submitted to MotionManager and weight shift
        // has no effect on ankle roll when roll controller is off.
        //
        // We call _applyRollCorrection() with zero PD correction (ankle/hip/torso=0,
        // rate=0) so only the bias is applied. The IIR shaping for roll joints
        // receives rawCmd=0, which the BUG-01 fix handles correctly: IIR decays
        // toward zero, and the bias is added post-shaping at submission time.
        // Velocity damping is also zero (rollRate_rs=0) — correct for bias-only mode.
        _applyRollCorrection(0.0f, 0.0f, 0.0f, 0.0f);
    }

    out.active = out.pitch_active || out.roll_active;
    out.fell   = false;
    _lastState = out;
    return out;
}

// ---------------------------------------------------------------------------
//  computeCorrection — Stage 3 task-space PD computation.
//
//  Runs the pitch and roll PD (including derivative filter, Smith predictor,
//  deadband, nonlinear gain boost, and clamp) identically to the legacy path,
//  but outputs foot-target offsets in mm rather than joint angles in degrees.
//
//  Side effects (intentional):
//    _pitchRateFiltered, _rollRateFiltered — updated (must happen once per tick).
//    _ts_pitch_u_clamped, _ts_roll_u_clamped — cached for torso submission in
//      update()'s task-space branch.
//    _dx_filtered_mm, _dy_filtered_mm — IIR state for the mm outputs.
//
//  Does NOT submit to any joints; does NOT touch MotionManager.
// ---------------------------------------------------------------------------
BalanceCorrection BalanceController::computeCorrection(const IMUState& state) {
    BalanceCorrection bc;
    if (!state.valid || oe_is_estopped()) {
        bc.valid = false;
        return bc;
    }

    // ── Pitch PD → dx_mm ─────────────────────────────────────────────────────
    if (_cfg.pitch_enabled) {
        _pitchRateFiltered = _cfg.derivative_lpf_alpha * _pitchRateFiltered
                           + (1.0f - _cfg.derivative_lpf_alpha) * state.pitchRate;

        const float predictedPitch = state.pitch
                                   + state.pitchRate * _cfg.servo_lag_compensation_s;
        const float error = predictedPitch - _cfg.pitch_setpoint_rad;
        _ts_pitch_error = error;

        float u_clamped = 0.0f;
        if (fabsf(error) >= _cfg.pitch_deadband_rad) {
            const float effKp = _computeEffectiveKp(fabsf(error), _cfg.Kp,
                                                     _cfg.gain_boost_threshold_rad,
                                                     _cfg.gain_boost_factor);
            const float u_raw = (effKp * error
                               + _cfg.Kd * _pitchRateFiltered)
                               * _cfg.correction_sign;
            u_clamped = constrain(u_raw,
                                  -_cfg.max_correction_deg,
                                   _cfg.max_correction_deg);
        }
        _ts_pitch_u_clamped = u_clamped;
        // _ts_pitch_error already set above.

        // Convert to mm with zero-bypass + delta clamp + IIR (mirrors _shapeOutput).
        // Negated: joint-space u_clamped>0 means dorsiflexion (body tilts back).
        // Equivalent task-space response: foot moves forward of hip (x_mm increases).
        // mergeBalanceCorrection subtracts dx_mm, so dx_mm must be negative for x_mm
        // to increase. correction_sign is already folded into u_clamped.
        const float raw_dx_mm = -u_clamped * _cfg.pitch_to_dx_mm_per_deg;
        float clamped_dx;
        if (fabsf(raw_dx_mm) < 1e-4f) {
            clamped_dx = 0.0f;  // zero-bypass: let IIR decay freely
        } else {
            const float maxStep = _cfg.max_output_rate_deg_per_tick
                                * _cfg.pitch_to_dx_mm_per_deg;
            clamped_dx = constrain(raw_dx_mm,
                                   _dx_filtered_mm - maxStep,
                                   _dx_filtered_mm + maxStep);
        }
        _dx_filtered_mm = _cfg.output_iir_alpha * _dx_filtered_mm
                        + (1.0f - _cfg.output_iir_alpha) * clamped_dx;
        bc.dx_mm = constrain(_dx_filtered_mm, -25.0f, 25.0f);
    } else {
        _ts_pitch_u_clamped = 0.0f;
        _ts_pitch_error     = 0.0f;
    }

    // ── Roll PD → dy_mm ──────────────────────────────────────────────────────
    if (_cfg.roll_enabled) {
        _rollRateFiltered = _cfg.roll_derivative_lpf_alpha * _rollRateFiltered
                         + (1.0f - _cfg.roll_derivative_lpf_alpha) * state.rollRate;

        const float predictedRoll = state.roll
                                  + state.rollRate * _cfg.servo_lag_compensation_s;
        const float roll_error = predictedRoll - _cfg.roll_setpoint_rad;
        _ts_roll_error = roll_error;

        float u_roll_clamped = 0.0f;
        if (fabsf(roll_error) >= _cfg.roll_deadband_rad) {
            const float effKpRoll = _computeEffectiveKp(fabsf(roll_error),
                                                         _cfg.Kp_roll,
                                                         _cfg.gain_boost_threshold_roll_rad,
                                                         _cfg.gain_boost_factor_roll);
            const float u_roll_raw = (effKpRoll * roll_error
                                    + _cfg.Kd_roll * _rollRateFiltered)
                                   * _cfg.roll_correction_sign;
            u_roll_clamped = constrain(u_roll_raw,
                                       -_cfg.max_roll_correction_deg,
                                        _cfg.max_roll_correction_deg);
        }
        _ts_roll_u_clamped = u_roll_clamped;

        // Soft clamp: prevent dy_mm from pushing the combined y_mm (lateral_shift +
        // dy) beyond IK reach. ±25 mm leaves headroom for WeightShift's contribution.
        u_roll_clamped = constrain(u_roll_clamped,
                                   -_cfg.max_roll_correction_deg,
                                    _cfg.max_roll_correction_deg);

        const float raw_dy_mm = u_roll_clamped * _cfg.roll_to_dy_mm_per_deg;
        float clamped_dy;
        if (fabsf(raw_dy_mm) < 1e-4f) {
            clamped_dy = 0.0f;
        } else {
            const float maxStep = _cfg.max_output_rate_deg_per_tick
                                * _cfg.roll_to_dy_mm_per_deg;
            clamped_dy = constrain(raw_dy_mm,
                                   _dy_filtered_mm - maxStep,
                                   _dy_filtered_mm + maxStep);
        }
        _dy_filtered_mm = _cfg.output_iir_alpha * _dy_filtered_mm
                        + (1.0f - _cfg.output_iir_alpha) * clamped_dy;
        // Outer soft clamp on dy_mm to protect IK reach.
        bc.dy_mm = constrain(_dy_filtered_mm, -25.0f, 25.0f);
    } else {
        _ts_roll_u_clamped = 0.0f;
        _ts_roll_error     = 0.0f;
    }

    bc.dPitch_deg = 0.0f;  // reserved
    bc.dRoll_deg  = 0.0f;  // reserved
    // valid=false when both axes disabled: correction is a no-op and consumers
    // that check valid (e.g. GaitController idle bypass) correctly skip it.
    bc.valid = (_cfg.pitch_enabled || _cfg.roll_enabled);
    return bc;
}

// ---------------------------------------------------------------------------
//  _applyPitchCorrection
//  All 5 pitch joints shaped every tick (even ratio=0) so the IIR for unused
//  joints stays at zero rather than cold-starting on ratio change.
// ---------------------------------------------------------------------------
void BalanceController::_applyPitchCorrection(float ankle_deg, float hip_deg,
                                               float torso_deg, float pitchRate_rs) {
    if (_motionManager) {
        float r_ank = _shapeOutput(BJI_R_ANKLE_PITCH,  ankle_deg,  pitchRate_rs);
        float l_ank = _shapeOutput(BJI_L_ANKLE_PITCH,  ankle_deg,  pitchRate_rs);
        float r_hip = _shapeOutput(BJI_R_HIP_PITCH,    hip_deg,    pitchRate_rs);
        float l_hip = _shapeOutput(BJI_L_HIP_PITCH,    hip_deg,    pitchRate_rs);
        // Torso pitch negated: pushes CoM opposite direction to ankle/hip.
        // Negation before shaping so IIR tracks actual mechanical output direction.
        float torso = _shapeOutput(BJI_TORSO_PITCH,   -torso_deg,  pitchRate_rs);

        _motionManager->submit(SOURCE_BALANCE, IDX_R_ANKLE_PITCH, r_ank);
        _motionManager->submit(SOURCE_BALANCE, IDX_L_ANKLE_PITCH, l_ank);
        if (_cfg.hip_ratio > 0.001f) {
            _motionManager->submit(SOURCE_BALANCE, IDX_R_HIP_PITCH, r_hip);
            _motionManager->submit(SOURCE_BALANCE, IDX_L_HIP_PITCH, l_hip);
        }
        if (_cfg.torso_ratio > 0.001f) {
            _motionManager->submit(SOURCE_BALANCE, IDX_TORSO_PITCH, torso);
        }
        return;
    }

    if (!_servo) return;
    static bool _warned = false;
    if (!_warned) {
        Serial.println("[BalanceController] WARNING: MotionManager not wired.");
        _warned = true;
    }
    float r_ank = _shapeOutput(BJI_R_ANKLE_PITCH,  ankle_deg,  pitchRate_rs);
    float l_ank = _shapeOutput(BJI_L_ANKLE_PITCH,  ankle_deg,  pitchRate_rs);
    float r_hip = _shapeOutput(BJI_R_HIP_PITCH,    hip_deg,    pitchRate_rs);
    float l_hip = _shapeOutput(BJI_L_HIP_PITCH,    hip_deg,    pitchRate_rs);
    float torso = _shapeOutput(BJI_TORSO_PITCH,   -torso_deg,  pitchRate_rs);
    _servo->setJointAngleDirect(IDX_R_ANKLE_PITCH, r_ank);
    _servo->setJointAngleDirect(IDX_L_ANKLE_PITCH, l_ank);
    if (_cfg.hip_ratio   > 0.001f) {
        _servo->setJointAngleDirect(IDX_R_HIP_PITCH, r_hip);
        _servo->setJointAngleDirect(IDX_L_HIP_PITCH, l_hip);
    }
    if (_cfg.torso_ratio > 0.001f) {
        _servo->setJointAngleDirect(IDX_TORSO_PITCH, torso);
    }
}

// ---------------------------------------------------------------------------
//  _applyRollCorrection — TASK-2: adds ankle roll bias AFTER shaping.
//
//  The shaped balance correction alone would completely override the weight
//  shift ankle position (SOURCE_BALANCE > SOURCE_GAIT in MotionManager).
//  Instead, WeightShift no longer submits ankle roll via MotionManager.
//  It sets ankle_roll_bias_l/r_deg in BalanceConfig each tick. Here we add
//  those baseline offsets after the IIR smoother so:
//    final_l = shaped_correction_l + bias_l
//    final_r = shaped_correction_r + bias_r
//
//  The balance controller corrections are now additive ON TOP of the shift.
//  When the roll error equals the setpoint (robot achieved target lean),
//  u_roll ≈ 0, and the only submission is the bias — robot holds the lean.
//  When roll error diverges, correction is added/subtracted around the bias.
//
//  Bias sign convention (matches the negation applied to right ankle):
//    For leftward tilt: bias_l > 0, bias_r < 0
//    Both produce the same physical tilt direction (verified via balance sign).
// ---------------------------------------------------------------------------
void BalanceController::_applyRollCorrection(float ankle_deg, float hip_deg,
                                              float torso_deg, float rollRate_rs) {
    if (_motionManager) {
        // Shape correction (right negated for consistent physical direction).
        float shaped_r = _shapeOutput(BJI_R_ANKLE_ROLL, -ankle_deg, rollRate_rs);
        float shaped_l = _shapeOutput(BJI_L_ANKLE_ROLL,  ankle_deg, rollRate_rs);
        float r_hip    = _shapeOutput(BJI_R_HIP_ROLL,    hip_deg,   rollRate_rs);
        float l_hip    = _shapeOutput(BJI_L_HIP_ROLL,    hip_deg,   rollRate_rs);
        float torso    = _shapeOutput(BJI_TORSO_ROLL,    torso_deg, rollRate_rs);

        // TASK-2: Add weight shift baseline AFTER shaping.
        // IIR history (filtered_cmd_deg) still tracks only the balance correction;
        // the bias is applied at submission time so it does not drift the IIR state.
        float final_r = shaped_r + _cfg.ankle_roll_bias_r_deg;
        float final_l = shaped_l + _cfg.ankle_roll_bias_l_deg;

        // Final clamp: combined bias + correction must not exceed joint limits.
        // JointModel enforces hard limits, but this prevents unnecessary chatter
        // against those limits from balance vs. weight shift contention.
        // M-04 FIX: headroom must accommodate the LARGER of the two biases.
        // The previous code only used left bias; if right is larger (asymmetric
        // shift), the right clamp was too tight and could fight the weight shift.
        float maxBias  = fmaxf(fabsf(_cfg.ankle_roll_bias_l_deg),
                                fabsf(_cfg.ankle_roll_bias_r_deg));
        float maxAnkle = _cfg.max_roll_correction_deg + maxBias;
        maxAnkle = constrain(maxAnkle, 1.0f, 30.0f);
        final_r  = constrain(final_r, -maxAnkle, maxAnkle);
        final_l  = constrain(final_l, -maxAnkle, maxAnkle);

        _motionManager->submit(SOURCE_BALANCE, IDX_R_ANKLE_ROLL, final_r);
        _motionManager->submit(SOURCE_BALANCE, IDX_L_ANKLE_ROLL, final_l);
        if (_cfg.hip_roll_ratio > 0.001f) {
            _motionManager->submit(SOURCE_BALANCE, IDX_R_HIP_ROLL, r_hip);
            _motionManager->submit(SOURCE_BALANCE, IDX_L_HIP_ROLL, l_hip);
        }
        if (_cfg.torso_roll_ratio > 0.001f) {
            _motionManager->submit(SOURCE_BALANCE, IDX_TORSO_ROLL, torso);
        }
        return;
    }

    if (!_servo) return;
    float shaped_r = _shapeOutput(BJI_R_ANKLE_ROLL, -ankle_deg, rollRate_rs);
    float shaped_l = _shapeOutput(BJI_L_ANKLE_ROLL,  ankle_deg, rollRate_rs);
    float r_hip    = _shapeOutput(BJI_R_HIP_ROLL,    hip_deg,   rollRate_rs);
    float l_hip    = _shapeOutput(BJI_L_HIP_ROLL,    hip_deg,   rollRate_rs);
    float torso    = _shapeOutput(BJI_TORSO_ROLL,    torso_deg, rollRate_rs);

    float final_r = shaped_r + _cfg.ankle_roll_bias_r_deg;
    float final_l = shaped_l + _cfg.ankle_roll_bias_l_deg;
        float maxBias  = fmaxf(fabsf(_cfg.ankle_roll_bias_l_deg),
                                fabsf(_cfg.ankle_roll_bias_r_deg));
        float maxAnkle = _cfg.max_roll_correction_deg + maxBias;
        maxAnkle = constrain(maxAnkle, 1.0f, 30.0f);
    final_r = constrain(final_r, -maxAnkle, maxAnkle);
    final_l = constrain(final_l, -maxAnkle, maxAnkle);

    _servo->setJointAngleDirect(IDX_R_ANKLE_ROLL, final_r);
    _servo->setJointAngleDirect(IDX_L_ANKLE_ROLL, final_l);
    if (_cfg.hip_roll_ratio   > 0.001f) {
        _servo->setJointAngleDirect(IDX_R_HIP_ROLL, r_hip);
        _servo->setJointAngleDirect(IDX_L_HIP_ROLL, l_hip);
    }
    if (_cfg.torso_roll_ratio > 0.001f) {
        _servo->setJointAngleDirect(IDX_TORSO_ROLL, torso);
    }
}