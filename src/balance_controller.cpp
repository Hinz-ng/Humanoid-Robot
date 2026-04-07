// =============================================================================
// FILE:    balance_controller.cpp
// MODULE:  control
// LAYER:   3 — Balance Controller
//
// PURPOSE:
//   Implements the PD pitch + roll balance control laws and the output shaping
//   pipeline. Every balance joint command passes through _shapeOutput() before
//   reaching MotionManager:
//
//     control law → magnitude clamp → [_shapeOutput per joint]:
//       delta_clamp → IIR_smooth → velocity_damp → submit
//
//   This decouples command bandwidth from the loop rate, preventing raw 400 Hz
//   PD outputs from overdriving servos with backlash-amplified chatter.
//
// KEY DESIGN CHOICES (justify before changing):
//   - _shapeOutput() is called every tick even inside the deadband (rawCmd=0).
//     This lets the IIR decay toward zero smoothly instead of freezing at the
//     last active command and jumping on deadband exit.
//   - The filtered angular rate (_pitchRateFiltered, _rollRateFiltered) feeds
//     both the Kd term AND velocity damping. Using the same filtered signal
//     avoids the two terms fighting against different noise levels.
//   - MotionManager is the only submit path. No direct hardware writes.
//
// LAST CHANGED: 2026-04-05 | Hinz | Add output shaping pipeline + roll parity
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
    resetState();

    Serial.printf("[BalanceController] Init.\n"
                  "  pitch_en=%s  Kp=%.1f  Kd=%.2f  sp=%.3f rad\n"
                  "  roll_en=%s   Kp_r=%.1f Kd_r=%.2f\n"
                  "  iir_alpha=%.2f  max_delta=%.2f deg/tick  damping_kv=%.2f\n",
                  _cfg.pitch_enabled ? "true" : "false",
                  _cfg.Kp, _cfg.Kd, _cfg.pitch_setpoint_rad,
                  _cfg.roll_enabled ? "true" : "false",
                  _cfg.Kp_roll, _cfg.Kd_roll,
                  _cfg.output_iir_alpha,
                  _cfg.max_output_rate_deg_per_tick,
                  _cfg.output_damping_kv);
}

// ---------------------------------------------------------------------------
void BalanceController::resetState() {
    _lastState         = {};
    _pitchRateFiltered = 0.0f;
    _rollRateFiltered  = 0.0f;
    _fallTickCount     = 0;
    _prevPitchEnabled  = false;
    _prevRollEnabled   = false;

    // Zero all per-joint output shaping state.
    // This ensures IIR seeds at zero on startup and after any ESTOP clear,
    // preventing the smoother from starting with a stale pre-ESTOP command.
    for (uint8_t i = 0; i < BJI_COUNT; i++) {
        _jointState[i] = {};
    }
}

// ---------------------------------------------------------------------------
void BalanceController::_resetJointStateRange(uint8_t first, uint8_t last) {
    if (first >= BJI_COUNT) return;
    if (last >= BJI_COUNT) last = BJI_COUNT - 1;
    if (first > last) return;
    for (uint8_t i = first; i <= last; i++) {
        _jointState[i] = {};
    }
}

// ---------------------------------------------------------------------------
//  _shapeOutput() — universal output shaping pipeline for every balance joint.
//
//  This is the core of the oscillation suppression refactor. The three stages:
//
//  1. Delta clamp: prevents step-command spikes larger than max_output_rate_deg_per_tick
//     from reaching the servo in a single tick. Mirrors UVC footCont():
//       if (2*k0 - HW[s] > 100) k0 = (HW[s] + 100) / 2;
//     Here we reference filtered_cmd_deg (the smoothed trajectory) so the
//     clamp bounds track the actual servo position, not the raw PD output.
//
//  2. IIR smoother: low-pass filters the delta-clamped command trajectory.
//     Cuts effective bandwidth to ~38 Hz at alpha=0.60 (400 Hz loop).
//     Formula: f_c ≈ (1 - alpha) * loop_hz / (2π)
//     The smoothed state persists across ticks — when rawCmd=0 (inside deadband)
//     the IIR decays toward zero gracefully rather than freezing.
//
//  3. Velocity damping: counteracts angular velocity at the output stage,
//     AFTER smoothing. Mirrors UVC footCont():
//       A0W[s] = k0 - x0 - 0.003 * fbAV
//     Injecting damping here is intentionally separate from the PD Kd term —
//     Kd acts on filtered rate in the error space; damping acts on filtered rate
//     in the output space. They are complementary, not redundant.
// ---------------------------------------------------------------------------
float BalanceController::_shapeOutput(uint8_t jIdx, float rawCmd, float angularRate_rs) {
    JointOutputState& s = _jointState[jIdx];

    // Step 1: Delta clamp — limit per-tick command change to avoid step inputs.
    // Reference is filtered_cmd_deg (smoothed trajectory), not rawCmd from last tick,
    // so the clamp bounds follow the actual servo position estimate.
    float clamped = 0.0f;
    if (fabsf(rawCmd) > 1e-4f) {
        clamped = constrain(rawCmd,
            s.filtered_cmd_deg - _cfg.max_output_rate_deg_per_tick,
            s.filtered_cmd_deg + _cfg.max_output_rate_deg_per_tick);
    }

    // Step 2: IIR smoothing — low-pass filter the clamped command.
    // alpha=0 → no smoothing.  alpha→1 → command frozen.  Default 0.60.
    float smoothed = _cfg.output_iir_alpha * s.filtered_cmd_deg
                   + (1.0f - _cfg.output_iir_alpha) * clamped;

    // Persist state for next tick.
    // filtered_cmd_deg is the IIR state — it must track smoothed, not clamped,
    // so the IIR history correctly represents the output trajectory.
    s.prev_cmd_deg     = clamped;   // delta-clamped value (for diagnostics)
    s.filtered_cmd_deg = smoothed;  // IIR state (for next tick's clamp reference + blend)

    // Step 3: Velocity damping — subtract angular-rate term AFTER smoothing.
    // This does not perturb the IIR state, so damping does not cause drift in
    // the smoothed trajectory on the next tick. Kv=0 disables this entirely.
    float maxCmdDeg = fmaxf(_cfg.max_correction_deg, _cfg.max_roll_correction_deg);
    float damped    = smoothed - _cfg.output_damping_kv * angularRate_rs;
    return constrain(damped, -maxCmdDeg, maxCmdDeg);
}

// ---------------------------------------------------------------------------
BalanceState BalanceController::update(const IMUState& state) {
    BalanceState out = {};

    // Fall detection must run whenever state is valid and outputs are live,
    // even if both balance axes are currently disabled.
    if (state.valid && !oe_is_estopped()) {
        if (fabsf(state.pitch) > _cfg.fall_threshold_rad ||
            fabsf(state.roll)  > _cfg.fall_threshold_rad) {
            _fallTickCount++;
            if (_fallTickCount >= _cfg.fall_confirm_ticks) {
                _fallTickCount = 0;
                oe_estop();
                if (_servo != nullptr) _servo->resetToNeutral();
                out.fell = true;
                Serial.printf("[BalanceController] FALL: pitch=%.3f roll=%.3f "
                              "threshold=%.3f → ESTOP\n",
                              state.pitch, state.roll, _cfg.fall_threshold_rad);
                _lastState = out;
                return out;
            }
        } else {
            _fallTickCount = 0;
        }
    }

    // On enabled->disabled transitions, clear stale shaping/derivative states so
    // re-enabling an axis starts cleanly from zero.
    if (_prevPitchEnabled && !_cfg.pitch_enabled) {
        _resetJointStateRange(BJI_R_ANKLE_PITCH, BJI_TORSO_PITCH);
        _pitchRateFiltered = 0.0f;
    }
    if (_prevRollEnabled && !_cfg.roll_enabled) {
        _resetJointStateRange(BJI_R_ANKLE_ROLL, BJI_TORSO_ROLL);
        _rollRateFiltered = 0.0f;
    }
    _prevPitchEnabled = _cfg.pitch_enabled;
    _prevRollEnabled  = _cfg.roll_enabled;

    // Gate 1: at least one controller must be enabled.
    if (!_cfg.pitch_enabled && !_cfg.roll_enabled) {
        _lastState = out;
        return out;
    }

    // Gate 2: estimator must be valid (calibration done, no I²C error).
    if (!state.valid) {
        _lastState = out;
        return out;
    }

    // Gate 3: hardware must not be in ESTOP.
    if (oe_is_estopped()) {
        _lastState = out;
        return out;
    }

    // =========================================================================
    //  PITCH PD — sagittal plane (forward/back)
    //
    //  Pipeline:
    //    predictedPitch (Smith) → error → deadband gate → PD law →
    //    magnitude clamp → ratio split → _applyPitchCorrection (shaping + submit)
    //
    //  _shapeOutput() is called via _applyPitchCorrection for every pitch joint
    //  on EVERY tick, including when inside the deadband (rawCmd=0). This lets
    //  the IIR decay toward zero smoothly on deadband entry.
    // =========================================================================
    if (_cfg.pitch_enabled) {
        // Derivative filter — updated every tick to keep IIR state warm.
        // A cold IIR state causes a Kd spike on deadband exit; staying warm avoids it.
        _pitchRateFiltered = _cfg.derivative_lpf_alpha * _pitchRateFiltered
                           + (1.0f - _cfg.derivative_lpf_alpha) * state.pitchRate;

        // Smith predictor: advance pitch estimate by one servo lag period.
        // Partially cancels servo mechanical delay in the phase response.
        // tau=0 disables prediction safely.
        float predictedPitch = state.pitch
                               + state.pitchRate * _cfg.servo_lag_compensation_s;
        float error = predictedPitch - _cfg.pitch_setpoint_rad;

        out.pitch_active = true;
        out.pitch_error  = error;

        // Control law — zero inside deadband so IIR decays rather than holding.
        float u_raw     = 0.0f;
        float u_clamped = 0.0f;
        if (fabsf(error) >= _cfg.pitch_deadband_rad) {
            u_raw     = (_cfg.Kp * error + _cfg.Kd * _pitchRateFiltered)
                        * _cfg.correction_sign;
            u_clamped = constrain(u_raw,
                                  -_cfg.max_correction_deg,
                                   _cfg.max_correction_deg);
        }

        // Ratio split — zero commands flow through shaping on deadband (IIR decay).
        float ankle_cmd = u_clamped * _cfg.ankle_ratio;
        float hip_cmd   = u_clamped * _cfg.hip_ratio;
        float torso_cmd = u_clamped * _cfg.torso_ratio;

        // Shape and submit all pitch joints.
        // _pitchRateFiltered passed as angular rate for velocity damping stage.
        _applyPitchCorrection(ankle_cmd, hip_cmd, torso_cmd, _pitchRateFiltered);

        // Telemetry reports pre-shaping values — reflects raw PD math for tuning.
        out.u_raw         = u_raw;
        out.u_clamped     = u_clamped;
        out.ankle_cmd_deg = ankle_cmd;
        out.hip_cmd_deg   = hip_cmd;
        out.torso_cmd_deg = torso_cmd;
    }

    // =========================================================================
    //  ROLL PD — frontal plane (left/right)
    //
    //  Feature parity with pitch: Smith predictor, derivative LPF, deadband,
    //  magnitude clamp, ratio split, same _shapeOutput pipeline.
    //  Roll uses separate gains (Kp_roll, Kd_roll) and separate IIR state
    //  (_rollRateFiltered, BJI_R_ANKLE_ROLL ... BJI_TORSO_ROLL).
    // =========================================================================
    if (_cfg.roll_enabled) {
        // Derivative filter — same pattern as pitch; updated every tick.
        // roll_derivative_lpf_alpha defaults to 0.85 (matches pitch LPF).
        _rollRateFiltered = _cfg.roll_derivative_lpf_alpha * _rollRateFiltered
                          + (1.0f - _cfg.roll_derivative_lpf_alpha) * state.rollRate;

        // Smith predictor for roll — same servo lag constant as pitch.
        // Both axes share the same servo hardware delay.
        float predictedRoll = state.roll
                              + state.rollRate * _cfg.servo_lag_compensation_s;
        float roll_error = predictedRoll - _cfg.roll_setpoint_rad;

        out.roll_active = true;
        out.roll_error  = roll_error;

        float u_roll_raw     = 0.0f;
        float u_roll_clamped = 0.0f;
        if (fabsf(roll_error) >= _cfg.roll_deadband_rad) {
            u_roll_raw     = (_cfg.Kp_roll * roll_error + _cfg.Kd_roll * _rollRateFiltered)
                             * _cfg.roll_correction_sign;
            u_roll_clamped = constrain(u_roll_raw,
                                       -_cfg.max_roll_correction_deg,
                                        _cfg.max_roll_correction_deg);
        }

        float ankle_roll_cmd = u_roll_clamped * _cfg.ankle_roll_ratio;
        float hip_roll_cmd   = u_roll_clamped * _cfg.hip_roll_ratio;
        float torso_roll_cmd = u_roll_clamped * _cfg.torso_roll_ratio;

        // Shape and submit all roll joints.
        _applyRollCorrection(ankle_roll_cmd, hip_roll_cmd, torso_roll_cmd,
                             _rollRateFiltered);

        out.u_roll_raw         = u_roll_raw;
        out.u_roll_clamped     = u_roll_clamped;
        out.ankle_roll_cmd_deg = ankle_roll_cmd;
        out.hip_roll_cmd_deg   = hip_roll_cmd;
        out.torso_roll_cmd_deg = torso_roll_cmd;
    }

    out.active = out.pitch_active || out.roll_active;
    out.fell   = false;
    _lastState = out;
    return out;
}

// ---------------------------------------------------------------------------
//  _applyPitchCorrection
//
//  Shapes and submits all pitch balance joints through _shapeOutput().
//  All 5 joints are shaped every tick — even joints with ratio=0 receive a
//  rawCmd=0, which lets their IIR decay gracefully instead of cold-starting
//  when a ratio is enabled mid-session via WebSocket.
//
//  Torso pitch negation: the torso pitch servo's mechanical direction is
//  opposite to ankle/hip for the same correction intent. The negation is
//  applied before _shapeOutput so the IIR state tracks the actual mechanical
//  output (not the inverted PD command).
// ---------------------------------------------------------------------------
void BalanceController::_applyPitchCorrection(float ankle_deg, float hip_deg,
                                               float torso_deg, float pitchRate_rs) {
    // ── Path A: MotionManager wired (normal runtime path) ────────────────────
    if (_motionManager) {
        // Shape all 5 pitch joints unconditionally.
        float r_ankle = _shapeOutput(BJI_R_ANKLE_PITCH,  ankle_deg,  pitchRate_rs);
        float l_ankle = _shapeOutput(BJI_L_ANKLE_PITCH,  ankle_deg,  pitchRate_rs);
        float r_hip   = _shapeOutput(BJI_R_HIP_PITCH,    hip_deg,    pitchRate_rs);
        float l_hip   = _shapeOutput(BJI_L_HIP_PITCH,    hip_deg,    pitchRate_rs);
        // Torso negated before shaping — IIR tracks actual mechanical output.
        float torso   = _shapeOutput(BJI_TORSO_PITCH,   -torso_deg,  pitchRate_rs);

        // Always submit ankle — primary balance joint.
        _motionManager->submit(SOURCE_BALANCE, IDX_R_ANKLE_PITCH, r_ankle);
        _motionManager->submit(SOURCE_BALANCE, IDX_L_ANKLE_PITCH, l_ankle);

        // Submit hip and torso only if ratio is configured.
        // IIR is still updated above (ratio=0 → zero rawCmd → IIR decays).
        if (_cfg.hip_ratio > 0.001f) {
            _motionManager->submit(SOURCE_BALANCE, IDX_R_HIP_PITCH, r_hip);
            _motionManager->submit(SOURCE_BALANCE, IDX_L_HIP_PITCH, l_hip);
        }
        if (_cfg.torso_ratio > 0.001f) {
            _motionManager->submit(SOURCE_BALANCE, IDX_TORSO_PITCH, torso);
        }
        return;
    }

    // ── Path B: MotionManager absent — direct fallback (log warning once) ────
    if (!_servo) return;
    static bool _fallbackWarned = false;
    if (!_fallbackWarned) {
        Serial.println("[BalanceController] WARNING: MotionManager not wired — "
                       "direct servo fallback active. Wire setMotionManager() in setup().");
        _fallbackWarned = true;
    }

    float r_ankle = _shapeOutput(BJI_R_ANKLE_PITCH,  ankle_deg,  pitchRate_rs);
    float l_ankle = _shapeOutput(BJI_L_ANKLE_PITCH,  ankle_deg,  pitchRate_rs);
    float r_hip   = _shapeOutput(BJI_R_HIP_PITCH,    hip_deg,    pitchRate_rs);
    float l_hip   = _shapeOutput(BJI_L_HIP_PITCH,    hip_deg,    pitchRate_rs);
    float torso   = _shapeOutput(BJI_TORSO_PITCH,   -torso_deg,  pitchRate_rs);

    _servo->setJointAngleDirect(IDX_R_ANKLE_PITCH, r_ankle);
    _servo->setJointAngleDirect(IDX_L_ANKLE_PITCH, l_ankle);
    if (_cfg.hip_ratio > 0.001f) {
        _servo->setJointAngleDirect(IDX_R_HIP_PITCH, r_hip);
        _servo->setJointAngleDirect(IDX_L_HIP_PITCH, l_hip);
    }
    if (_cfg.torso_ratio > 0.001f) {
        _servo->setJointAngleDirect(IDX_TORSO_PITCH, torso);
    }
}

// ---------------------------------------------------------------------------
//  _applyRollCorrection
//
//  Mirrors _applyPitchCorrection exactly — same shaping, same submit pattern.
//
//  Right ankle roll negation: IDX_R_ANKLE_ROLL's mechanical correction direction
//  is opposite to IDX_L_ANKLE_ROLL for the same roll error sign. The JointModel
//  direction field partially compensates, but the roll correction axis requires
//  an explicit negation for the right side. Applied before _shapeOutput so the
//  IIR state tracks the actual mechanical command.
//
//  Bug fix vs original: the original fallback path referenced _motionManager
//  instead of _servo for hip commands. That is corrected here.
// ---------------------------------------------------------------------------
void BalanceController::_applyRollCorrection(float ankle_deg, float hip_deg,
                                              float torso_deg, float rollRate_rs) {
    // ── Path A: MotionManager wired ───────────────────────────────────────────
    if (_motionManager) {
        // Right ankle is negated — see comment above.
        float r_ankle = _shapeOutput(BJI_R_ANKLE_ROLL, -ankle_deg, rollRate_rs);
        float l_ankle = _shapeOutput(BJI_L_ANKLE_ROLL,  ankle_deg, rollRate_rs);
        float r_hip   = _shapeOutput(BJI_R_HIP_ROLL,    hip_deg,   rollRate_rs);
        float l_hip   = _shapeOutput(BJI_L_HIP_ROLL,    hip_deg,   rollRate_rs);
        // Torso roll: direction currently assumed +1 (not yet verified on hardware).
        // If correction worsens the lean, set roll_correction_sign = -1 from UI.
        float torso   = _shapeOutput(BJI_TORSO_ROLL,    torso_deg, rollRate_rs);

        _motionManager->submit(SOURCE_BALANCE, IDX_R_ANKLE_ROLL, r_ankle);
        _motionManager->submit(SOURCE_BALANCE, IDX_L_ANKLE_ROLL, l_ankle);

        if (_cfg.hip_roll_ratio > 0.001f) {
            _motionManager->submit(SOURCE_BALANCE, IDX_R_HIP_ROLL, r_hip);
            _motionManager->submit(SOURCE_BALANCE, IDX_L_HIP_ROLL, l_hip);
        }
        if (_cfg.torso_roll_ratio > 0.001f) {
            _motionManager->submit(SOURCE_BALANCE, IDX_TORSO_ROLL, torso);
        }
        return;
    }

    // ── Path B: MotionManager absent — direct fallback ────────────────────────
    if (!_servo) return;
    // (Warning already printed by _applyPitchCorrection fallback path.)

    float r_ankle = _shapeOutput(BJI_R_ANKLE_ROLL, -ankle_deg, rollRate_rs);
    float l_ankle = _shapeOutput(BJI_L_ANKLE_ROLL,  ankle_deg, rollRate_rs);
    float r_hip   = _shapeOutput(BJI_R_HIP_ROLL,    hip_deg,   rollRate_rs);
    float l_hip   = _shapeOutput(BJI_L_HIP_ROLL,    hip_deg,   rollRate_rs);
    float torso   = _shapeOutput(BJI_TORSO_ROLL,    torso_deg, rollRate_rs);

    _servo->setJointAngleDirect(IDX_R_ANKLE_ROLL, r_ankle);
    _servo->setJointAngleDirect(IDX_L_ANKLE_ROLL, l_ankle);
    if (_cfg.hip_roll_ratio > 0.001f) {
        _servo->setJointAngleDirect(IDX_R_HIP_ROLL, r_hip);
        _servo->setJointAngleDirect(IDX_L_HIP_ROLL, l_hip);
    }
    if (_cfg.torso_roll_ratio > 0.001f) {
        _servo->setJointAngleDirect(IDX_TORSO_ROLL, torso);
    }
}
