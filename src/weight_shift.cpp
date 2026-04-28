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
    forceCenterImmediate();
    Serial.println("[WeightShift] Initialized.");
}

// ---------------------------------------------------------------------------
void WeightShift::forceCenterImmediate() {
    _state = {};
    _state.direction = ShiftDirection::NONE;
    _rightAnkleProgress     = 0.0f;
    _leftAnkleProgress      = 0.0f;
    _targetRight            = 0.0f;
    _targetLeft             = 0.0f;
    _rightDelayRemaining_ms = 0.0f;
    _leftDelayRemaining_ms  = 0.0f;
    _lastInjectedSetpointRad = 0.0f;
    _settledAtMs = 0;
    _smoothedForwardLean_mm  = 0.0f;
}

// ---------------------------------------------------------------------------
void WeightShift::trigger(ShiftDirection dir) {
    if (dir != ShiftDirection::NONE
        && _state.direction != ShiftDirection::NONE
        && dir != _state.direction
        && _cfg.min_dwell_ms > 0.0f
        && _settledAtMs > 0
        && (millis() - _settledAtMs) < (uint32_t)_cfg.min_dwell_ms) {
        Serial.printf("[WeightShift] trigger %d ignored - dwell %lu/%.0f ms\n",
                      (int)dir,
                      (unsigned long)(millis() - _settledAtMs),
                      _cfg.min_dwell_ms);
        return;
    }
    _settledAtMs = 0;
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
    if (!_state.ramping && _settledAtMs == 0
        && _state.direction != ShiftDirection::NONE) {
        _settledAtMs = millis();
    }

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
    const float biasRightDeg = (fabsf(_cfg.ankle_shift_deg) > 0.01f)
                               ? (-_rightAnkleProgress * _cfg.ankle_shift_deg)
                               : 0.0f;
    const float biasLeftDeg  = (fabsf(_cfg.ankle_shift_deg) > 0.01f)
                               ? (+_leftAnkleProgress  * _cfg.ankle_shift_deg)
                               : 0.0f;
    _bal->setAnkleRollBias(biasLeftDeg, biasRightDeg);

    // Roll setpoint: use average progress. Update only if changed > 0.5 mrad.
    const float newSetpoint = -_state.progress * fabsf(_cfg.setpoint_shift_rad);
    if (fabsf(newSetpoint - _lastInjectedSetpointRad) > 0.0005f) {
        _bal->setRollSetpointRad(newSetpoint);
        _lastInjectedSetpointRad = newSetpoint;
    }

    // ── Forward-lean IIR smoothing (mm-space) ────────────────────────────────
    // Stage 2: the |progress|-driven body-forward offset has a derivative
    // discontinuity at zero crossing. IIR-smooth it here so callers of
    // getStanceShift() see a continuous x_mm contribution. Lateral shift uses
    // progress directly (already ramp-smoothed) — no second-stage IIR needed.
    {
        const float rawFwd = fabsf(_state.progress) * _cfg.forward_lean_mm;
        const float a      = _cfg.shift_smooth_alpha;
        _smoothedForwardLean_mm = a * _smoothedForwardLean_mm + (1.0f - a) * rawFwd;
    }

    // ── V3 / V4 — folded into FootTarget via getStanceShift() in Stage 2 ─────
    // Ankle pitch tilt → FootTarget.x_mm (forward_lean_mm, smoothed above).
    // Stance hip roll  → FootTarget.y_mm (lateral_shift_mm, signed per leg).
    // GaitController calls _ws->getStanceShift() inside _buildFootTarget.

    // ── Torso roll (SOURCE_GAIT) ──────────────────────────────────────────────
    if (fabsf(_cfg.torso_shift_deg) > 0.01f && fabsf(_state.progress) > 0.01f) {
        _mm->submit(SOURCE_GAIT, IDX_TORSO_ROLL,
                    _state.progress * _cfg.torso_shift_deg);
    }
}

// ---------------------------------------------------------------------------
//  getStanceShift — Stage 2 task-space contribution to FootTarget.
//
//  Composes two terms:
//    Forward lean: x_offset = -smoothedForwardLean_mm  (both legs symmetric)
//    Lateral shift: y_offset = ±(progress × lateral_shift_mm), signed per leg
//
//  Sign for lateral, with progress = +1 (LEFT shift):
//    right leg: foot more outward of right hip → y_mm += progress*lat
//    left  leg: foot more inward  of left  hip → y_mm -= progress*lat
// ---------------------------------------------------------------------------
void WeightShift::getStanceShift(bool isRightLeg,
                                  float& x_offset_mm,
                                  float& y_offset_mm) const {
    x_offset_mm = 0.0f;
    y_offset_mm = 0.0f;
    if (oe_is_estopped()) return;

    x_offset_mm = -_smoothedForwardLean_mm;

    const float bodyShift = _state.progress * _cfg.lateral_shift_mm;
    y_offset_mm = isRightLeg ? +bodyShift : -bodyShift;
}
