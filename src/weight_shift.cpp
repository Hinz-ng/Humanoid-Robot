// =============================================================================
// FILE:    weight_shift.cpp  
// LAST CHANGED: 2026-04-08 | Hinz | Task-2 phased shift + ankle bias
// =============================================================================

#include "weight_shift.h"
#include "balance_controller.h"
#include "oe_control.h"
#include <math.h>

void WeightShift::init(BalanceController* bal, MotionManager* mm) {
    if (bal == nullptr || mm == nullptr) {
        Serial.println("[WeightShift] ERROR: null pointer — update() will no-op.");
    }
    _bal  = bal;
    _mm   = mm;
    _state = {};
    _rightAnkleProgress = _leftAnkleProgress = 0.0f;
    _targetRight = _targetLeft = 0.0f;
    _rightDelayRemaining_ms = _leftDelayRemaining_ms = 0.0f;
    _lastInjectedSetpointRad = 0.0f;
    _ankleTiltCmdSmoothed = 0.0f;
    Serial.println("[WeightShift] Initialized.");
}

// ---------------------------------------------------------------------------
void WeightShift::trigger(ShiftDirection dir) {
    _state.direction = dir;

    if (dir == ShiftDirection::NONE) {
        _targetRight = _targetLeft = 0.0f;
        _rightDelayRemaining_ms = _leftDelayRemaining_ms = 0.0f;
        Serial.println("[WeightShift] CENTER — both ankles ramping simultaneously.");
        return;
    }

    if (dir == ShiftDirection::LEFT) {
        // LEFT shift: right=swing (immediate), left=stance (delayed).
        // Both targets = +1.0f for leftward tilt direction.
        _targetRight = 1.0f;
        _targetLeft  = 1.0f;
        _rightDelayRemaining_ms = 0.0f;
        _leftDelayRemaining_ms  = _cfg.shift_phase_delay_ms;
        Serial.printf("[WeightShift] LEFT — right(swing) immediate, "
                      "left(stance) +%.0fms.\n", _cfg.shift_phase_delay_ms);
    } else {
        // RIGHT shift: left=swing (immediate), right=stance (delayed).
        // Both targets = -1.0f for rightward tilt direction.
        _targetRight = -1.0f;
        _targetLeft  = -1.0f;
        _leftDelayRemaining_ms  = 0.0f;
        _rightDelayRemaining_ms = _cfg.shift_phase_delay_ms;
        Serial.printf("[WeightShift] RIGHT — left(swing) immediate, "
                      "right(stance) +%.0fms.\n", _cfg.shift_phase_delay_ms);
    }
}

// ---------------------------------------------------------------------------
void WeightShift::update(float dt_s) {
    if (_bal == nullptr || _mm == nullptr) return;
    if (oe_is_estopped()) return;

    const float dt_ms  = dt_s * 1000.0f;
    const float ramp_s = fmaxf(_cfg.ramp_ms, 10.0f) * 1e-3f;
    const float step   = dt_s / ramp_s;

    // ── Advance right ankle ───────────────────────────────────────────────────
    if (_rightDelayRemaining_ms > 0.0f) {
        _rightDelayRemaining_ms = fmaxf(_rightDelayRemaining_ms - dt_ms, 0.0f);
    } else {
        float d = _targetRight - _rightAnkleProgress;
        _rightAnkleProgress = (fabsf(d) <= step)
                              ? _targetRight
                              : _rightAnkleProgress + copysignf(step, d);
    }

    // ── Advance left ankle ────────────────────────────────────────────────────
    if (_leftDelayRemaining_ms > 0.0f) {
        _leftDelayRemaining_ms = fmaxf(_leftDelayRemaining_ms - dt_ms, 0.0f);
    } else {
        float d = _targetLeft - _leftAnkleProgress;
        _leftAnkleProgress = (fabsf(d) <= step)
                             ? _targetLeft
                             : _leftAnkleProgress + copysignf(step, d);
    }

    // ── Update state for telemetry ────────────────────────────────────────────
    _state.right_progress = _rightAnkleProgress;
    _state.left_progress  = _leftAnkleProgress;
    _state.progress       = (_rightAnkleProgress + _leftAnkleProgress) * 0.5f;
    _state.ramping        = (fabsf(_rightAnkleProgress - _targetRight) > 1e-3f ||
                              fabsf(_leftAnkleProgress  - _targetLeft)  > 1e-3f ||
                              _rightDelayRemaining_ms > 0.0f ||
                              _leftDelayRemaining_ms  > 0.0f);

    // ── Inject ankle bias + roll setpoint into BalanceConfig ─────────────────
    //
    // TASK-2 CORE: WeightShift no longer submits ankle roll via SOURCE_GAIT.
    //
    // Bias sign convention (must match _applyRollCorrection):
    //   bias_r = −rightProgress × amp  →  for LEFT(right=+1): bias_r = −5°
    //   bias_l = +leftProgress  × amp  →  for LEFT(left=+1):  bias_l = +5°
    //   Both produce the same physical tilt (body tilts left for left shift).
    //   RIGHT shift: rightProgress=−1 → bias_r=+5°, leftProgress=−1 → bias_l=−5°
    //   Both tilt body right ✓
    //
    // The balance controller reads these from _cfg each tick and adds them
    // to the shaped correction AFTER the IIR, so correction is additive.
    BalanceConfig cfg = _bal->getConfig();

    cfg.ankle_roll_bias_r_deg = (fabsf(_cfg.ankle_shift_deg) > 0.01f)
                                 ? (-_rightAnkleProgress * _cfg.ankle_shift_deg)
                                 : 0.0f;
    cfg.ankle_roll_bias_l_deg = (fabsf(_cfg.ankle_shift_deg) > 0.01f)
                                 ? (+_leftAnkleProgress  * _cfg.ankle_shift_deg)
                                 : 0.0f;

    // Roll setpoint: use average progress. Update only if changed > 0.5 mrad.
    const float newSetpoint = -_state.progress * fabsf(_cfg.setpoint_shift_rad);
    if (fabsf(newSetpoint - _lastInjectedSetpointRad) > 0.0005f) {
        cfg.roll_setpoint_rad    = newSetpoint;
        _lastInjectedSetpointRad = newSetpoint;
    }
    _bal->setConfig(cfg);

    // ── Stance hip roll (SOURCE_GAIT) ─────────────────────────────────────────
    // V4 — Stage 2: fold this into stance FootTarget.y_mm so IK derives hip roll.
    // Still submitted via MotionManager — does not conflict with balance controller
    // (hip_roll_ratio=0 by default keeps balance off hip roll joints).
    if (fabsf(_cfg.hip_shift_deg) > 0.01f && fabsf(_state.progress) > 0.01f) {
        if (_state.progress > 0.01f) {
            _mm->submit(SOURCE_GAIT, IDX_L_HIP_ROLL,
                        _state.progress * _cfg.hip_shift_deg);
        } else {
            _mm->submit(SOURCE_GAIT, IDX_R_HIP_ROLL,
                        -_state.progress * _cfg.hip_shift_deg);
        }
    }

    // ── Ankle pitch tilt (SOURCE_GAIT) ────────────────────────────────────────
    // V3 — Stage 2: fold this into stance FootTarget.x_mm so IK derives ankle pitch.
    // Sagittal plane only — no conflict with roll controller.
    //
    // IIR-smoothed: the raw value `|progress| * tilt_deg` formed a V-shape with a
    // derivative discontinuity at every direction reversal (progress crosses 0),
    // and a step on/off at the historic |progress| > 0.01 gate. Both manifested
    // as an abrupt plantar/dorsi flexion that destabilised the stance leg.
    //
    // Always submit the smoothed value (no progress gate) so it can settle to 0
    // continuously when shift centers — no edge from "submitted" → "not submitted".
    if (fabsf(_cfg.ankle_pitch_tilt_deg) > 0.01f) {
        const float rawTilt = fabsf(_state.progress) * _cfg.ankle_pitch_tilt_deg;
        const float a = _cfg.ankle_tilt_smooth_alpha;
        _ankleTiltCmdSmoothed = a * _ankleTiltCmdSmoothed + (1.0f - a) * rawTilt;
        _mm->submit(SOURCE_GAIT, IDX_R_ANKLE_PITCH, _ankleTiltCmdSmoothed);
        _mm->submit(SOURCE_GAIT, IDX_L_ANKLE_PITCH, _ankleTiltCmdSmoothed);
    }

    // ── Torso roll (SOURCE_GAIT) ──────────────────────────────────────────────
    if (fabsf(_cfg.torso_shift_deg) > 0.01f && fabsf(_state.progress) > 0.01f) {
        _mm->submit(SOURCE_GAIT, IDX_TORSO_ROLL,
                    _state.progress * _cfg.torso_shift_deg);
    }

    // ── Swing leg lift — REMOVED in Stage 1 refactor ──────────────────────────
    // Foot lift is now owned by GaitController, which constructs a FootTarget
    // (h_sagittal_mm = stanceHeight − swingFrac × stepHeight) and submits hip,
    // knee, and ankle angles via LegIK::solve() → SOURCE_GAIT.
    //
    // WeightShift's role is now: lateral CoM ramp + ankle-roll bias (V5) +
    // V3/V4 (ankle pitch tilt, stance hip roll) which remain direct submits
    // pending Stage 2.
    //
    // The swing_hip_extension_deg / swing_knee_flexion_deg config fields are
    // now dormant — see weight_shift.h "DEPRECATED in Stage 1" markers.
}