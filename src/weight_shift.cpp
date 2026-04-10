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
    // Sagittal plane only — no conflict with roll controller.
    if (fabsf(_cfg.ankle_pitch_tilt_deg) > 0.01f && fabsf(_state.progress) > 0.01f) {
        float tilt = fabsf(_state.progress) * _cfg.ankle_pitch_tilt_deg;
        _mm->submit(SOURCE_GAIT, IDX_R_ANKLE_PITCH, tilt);
        _mm->submit(SOURCE_GAIT, IDX_L_ANKLE_PITCH, tilt);
    }

    // ── Torso roll (SOURCE_GAIT) ──────────────────────────────────────────────
    if (fabsf(_cfg.torso_shift_deg) > 0.01f && fabsf(_state.progress) > 0.01f) {
        _mm->submit(SOURCE_GAIT, IDX_TORSO_ROLL,
                    _state.progress * _cfg.torso_shift_deg);
    }

    // ── Swing leg lift (SOURCE_GAIT) ──────────────────────────────────────────
    // Uses individual swing ankle progress (not overall average) for lift timing.
    // This means lift begins when the swing ankle starts moving — before stance
    // ankle has started — which is the correct physical sequence.
    {
        const float th = constrain(_cfg.swing_lift_threshold, 0.0f, 0.95f);

        // Swing progress = the ankle that starts first (no delay).
        // LEFT shift → right is swing → use _rightAnkleProgress
        // RIGHT shift → left is swing → use |_leftAnkleProgress|
        float swingAbs = 0.0f;
        uint8_t swingHipCh  = IDX_R_HIP_PITCH;
        uint8_t swingKneeCh = IDX_R_KNEE_PITCH;

        if (_state.direction == ShiftDirection::LEFT) {
            swingAbs    = fabsf(_rightAnkleProgress);
            swingHipCh  = IDX_R_HIP_PITCH;
            swingKneeCh = IDX_R_KNEE_PITCH;
        } else if (_state.direction == ShiftDirection::RIGHT) {
            swingAbs    = fabsf(_leftAnkleProgress);
            swingHipCh  = IDX_L_HIP_PITCH;
            swingKneeCh = IDX_L_KNEE_PITCH;
        }

        if (swingAbs > 0.01f) {
            float liftScale = 0.0f;
            if (swingAbs > th) {
                liftScale = constrain((swingAbs - th) / (1.0f - th), 0.0f, 1.0f);
            }
            float hipCmd  = -liftScale * _cfg.swing_hip_extension_deg;
            float kneeCmd =  liftScale * _cfg.swing_knee_flexion_deg;
            _mm->submit(SOURCE_GAIT, swingHipCh,  hipCmd);
            _mm->submit(SOURCE_GAIT, swingKneeCh, kneeCmd);
        }
    }
}