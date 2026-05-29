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
    for (uint8_t i = 0; i < SIDE_COUNT; ++i) {
        _ankleProgress[i]     = 0.0f;
        _targetProgress[i]    = 0.0f;
        _delayRemaining_ms[i] = 0.0f;
    }
    _settledAtMs = 0;
    _smoothedForwardLean_mm  = 0.0f;
    _swingMask     = SwingLeg::NONE;
    _swingLiftFrac = 0.0f;
}

// ---------------------------------------------------------------------------
//  setSwingLegMask  -  see header. Pure setter, no side effects.
// ---------------------------------------------------------------------------
void WeightShift::setSwingLegMask(SwingLeg swing, float liftFrac) {
    _swingMask     = swing;
    _swingLiftFrac = constrain(liftFrac, 0.0f, 1.0f);
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
        for (uint8_t i = 0; i < SIDE_COUNT; ++i) {
            _targetProgress[i]    = 0.0f;
            _delayRemaining_ms[i] = 0.0f;
        }
        Serial.println("[WeightShift] CENTER — both ankles ramping simultaneously.");
        return;
    }

    // Both ankles ramp to the same magnitude target. The stance side starts
    // immediately; the swing side waits shift_phase_delay_ms. Sign and the
    // stance/swing assignment depend on direction — same physical effect as
    // before the Phase 2 refactor.
    if (dir == ShiftDirection::LEFT) {
        // CoM moves onto RIGHT foot: stance = RIGHT (immediate), swing = LEFT (delayed).
        const Side stance = Side::RIGHT;
        const Side swing  = Side::LEFT;
        _targetProgress[sideIdx(Side::LEFT)]  = 1.0f;
        _targetProgress[sideIdx(Side::RIGHT)] = 1.0f;
        _delayRemaining_ms[sideIdx(stance)] = 0.0f;
        _delayRemaining_ms[sideIdx(swing)]  = _cfg.shift_phase_delay_ms;
        Serial.printf("[WeightShift] LEFT (CoM→right) — right(stance) immediate, "
                      "left(swing) +%.0fms.\n", _cfg.shift_phase_delay_ms);
    } else {  // RIGHT
        // CoM moves onto LEFT foot: stance = LEFT (immediate), swing = RIGHT (delayed).
        const Side stance = Side::LEFT;
        const Side swing  = Side::RIGHT;
        _targetProgress[sideIdx(Side::LEFT)]  = -1.0f;
        _targetProgress[sideIdx(Side::RIGHT)] = -1.0f;
        _delayRemaining_ms[sideIdx(stance)] = 0.0f;
        _delayRemaining_ms[sideIdx(swing)]  = _cfg.shift_phase_delay_ms;
        Serial.printf("[WeightShift] RIGHT (CoM→left) — left(stance) immediate, "
                      "right(swing) +%.0fms.\n", _cfg.shift_phase_delay_ms);
    }
}

// ---------------------------------------------------------------------------
//  _swingBiasScale  -  front-loaded swing-side ankle-roll bias scale. See
//  header + GAIT_WS_SWING_PRELOAD_COMPLETE_FRAC_DEFAULT. At completeFrac == 1.0
//  this is exactly (1 - _swingLiftFrac).
// ---------------------------------------------------------------------------
float WeightShift::_swingBiasScale() const {
    const float completeFrac = constrain(_cfg.swingPreloadCompleteFrac, 0.05f, 1.0f);
    return constrain((1.0f - _swingLiftFrac) / completeFrac, 0.0f, 1.0f);
}

// ---------------------------------------------------------------------------
void WeightShift::update(float dt_s) {
    if (_bal == nullptr || _mm == nullptr) return;
    if (oe_is_estopped()) return;

    const float dt_ms  = dt_s * 1000.0f;
    const float ramp_s = fmaxf(_cfg.ramp_ms, 10.0f) * 1e-3f;
    const float step   = dt_s / ramp_s;

    // ── Advance both ankles (Side-indexed loop, post-Phase-2) ────────────────
    for (uint8_t i = 0; i < SIDE_COUNT; ++i) {
        if (_delayRemaining_ms[i] > 0.0f) {
            _delayRemaining_ms[i] = fmaxf(_delayRemaining_ms[i] - dt_ms, 0.0f);
        } else {
            const float d = _targetProgress[i] - _ankleProgress[i];
            _ankleProgress[i] = (fabsf(d) <= step)
                                ? _targetProgress[i]
                                : _ankleProgress[i] + copysignf(step, d);
        }
    }

    // ── Update state for telemetry ────────────────────────────────────────────
    // Public field names (right_progress / left_progress) preserved — wire
    // protocol and UI parsers depend on them. Reads come from the Side-indexed
    // canonical store.
    _state.right_progress = _ankleProgress[sideIdx(Side::RIGHT)];
    _state.left_progress  = _ankleProgress[sideIdx(Side::LEFT)];
    _state.progress       = (_ankleProgress[sideIdx(Side::RIGHT)]
                            + _ankleProgress[sideIdx(Side::LEFT)]) * 0.5f;
    _state.ramping        = (fabsf(_ankleProgress[sideIdx(Side::RIGHT)]
                                   - _targetProgress[sideIdx(Side::RIGHT)]) > 1e-3f ||
                              fabsf(_ankleProgress[sideIdx(Side::LEFT)]
                                    - _targetProgress[sideIdx(Side::LEFT)])  > 1e-3f ||
                              _delayRemaining_ms[sideIdx(Side::RIGHT)] > 0.0f ||
                              _delayRemaining_ms[sideIdx(Side::LEFT)]  > 0.0f);
    if (!_state.ramping && _settledAtMs == 0
        && _state.direction != ShiftDirection::NONE) {
        _settledAtMs = millis();
    }

    // ── Ankle-roll bias ownership (consolidation refactor) ───────────────────
    // WeightShift no longer pushes an ankle-roll bias into BalanceController.
    // GaitController reads this module's progress (getState()) + ankle_shift_deg
    // and composes the CoM-shift ankle-roll bias onto the IK flat-foot base in
    // its single SOURCE_GAIT ankle-roll summation (see
    // GaitController::_comShiftRollBias / _swingPrepRollBias). The swing-leg mask
    // machinery here (_swingBiasScale / _swingMask) is now unused and will be
    // removed once the consolidated path is hardware-validated.

    // Roll setpoint is intentionally NOT injected into BalanceConfig: lateral
    // CoM control flows through FootTarget.y_mm (task-space dy). Injecting a
    // roll setpoint would make balance chase a lean target that conflicts with
    // dy_mm foot placement.

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
