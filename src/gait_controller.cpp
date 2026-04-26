// =============================================================================
// FILE:    gait_controller.cpp
// MODULE:  gait
// LAYER:   3.5 — Gait / Motion
// LAST CHANGED: 2026-04-25 | Hinz | Phase 1: pose-by-pose mode, leg inversion
//
// IMPLEMENTATION NOTES
// ────────────────────
// Each stepping half-cycle has two sub-states:
//
//   WAIT_SHIFT — WeightShift is ramping; phase holds; no foot lift.
//                Gate: wsProgress in the correct signed direction >= liftGateThreshold.
//                This prevents the swing foot from leaving the ground before the
//                stance leg is loaded — the root cause of the previous "shuffle".
//
//   SWINGING   — Gate is open; _halfPhase advances [0→1); foot follows
//                sin(_halfPhase × π): 0 at start, peak at midpoint, 0 at end.
//                Once _halfPhase >= 1.0, _advanceHalf() flips the half-cycle
//                and re-enters WAIT_SHIFT for the next leg.
//
// Weight shift direction (hardware-verified):
//   _currentHalf == 0 → LEFT shift (body over left leg, LEFT foot swings)
//   _currentHalf == 1 → RIGHT shift (body over right leg, RIGHT foot swings)
//
// POSE_STEP mode pause points:
//   1 = gate-open   : weight shifted, foot still on ground (verify CoM)
//   2 = peak swing  : foot at max height (verify lift height & direction)
//   3 = foot landed : before advancing half (verify stable landing)
//   Call nextPose() to advance through each hold point.
//
// Priority note: SOURCE_GAIT (priority 1) < SOURCE_BALANCE (priority 2).
// BalanceController will partially oppose the hip/knee commands on shared
// joints. This is expected and acceptable in Phase 1.
// =============================================================================

#include "gait_controller.h"
#include "weight_shift.h"
#include "oe_control.h"
#include <math.h>

// ---------------------------------------------------------------------------
void GaitController::init(MotionManager* mm, WeightShift* ws) {
    if (mm == nullptr) {
        Serial.println("[GaitCtrl] ERROR: MotionManager null — update() will no-op.");
    }
    _mm           = mm;
    _ws           = ws;
    _state        = {};
    _currentHalf  = 0;
    _halfPhase    = 0.0f;
    _subState     = StepSubState::WAIT_SHIFT;
    _lastShiftDir = 0;
    _poseHoldPt   = 0;
    Serial.println("[GaitCtrl] Initialized.");
}

// ---------------------------------------------------------------------------
void GaitController::start(GaitMode mode) {
    _state.mode    = mode;
    _state.running = (mode != GaitMode::IDLE);

    if (mode == GaitMode::IDLE) {
        _state.phase      = 0.0f;
        _state.liftL_deg  = 0.0f;
        _state.liftR_deg  = 0.0f;
        _state.wsProgress = 0.0f;
        _state.liftGateOK = false;
        _state.poseHold   = 0;
        _currentHalf      = 0;
        _halfPhase        = 0.0f;
        _subState         = StepSubState::WAIT_SHIFT;
        _lastShiftDir     = 0;
        _poseHoldPt       = 0;
        if (_ws) _ws->trigger(ShiftDirection::NONE);
        Serial.println("[GaitCtrl] IDLE — weight shift centered.");
        return;
    }

    // RUNNING, SINGLE_STEP, WSHIFT_ONLY, POSE_STEP all start from half=0.
    _currentHalf  = 0;
    _halfPhase    = 0.0f;
    _subState     = StepSubState::WAIT_SHIFT;
    _lastShiftDir = 0;
    _poseHoldPt   = 0;
    _state.poseHold = 0;
    _triggerShiftForHalf(0);  // begin LEFT shift immediately

    Serial.printf("[GaitCtrl] start mode=%d\n", static_cast<int>(mode));
}

void GaitController::stop()       { start(GaitMode::IDLE); }
void GaitController::singleStep() { start(GaitMode::SINGLE_STEP); }

// ---------------------------------------------------------------------------
//  nextPose  —  advance one step in POSE_STEP mode.
//  At hold 1 (gate-open) and hold 2 (peak): clear the pause, resume freely.
//  At hold 3 (landing): clear pause AND call _advanceHalf() to flip half-cycle.
// ---------------------------------------------------------------------------
void GaitController::nextPose() {
    if (_state.mode != GaitMode::POSE_STEP || _poseHoldPt == 0) return;

    const uint8_t hold = _poseHoldPt;
    _poseHoldPt       = 0;
    _state.poseHold   = 0;

    if (hold == 3) {
        // At landing: transition to next half now.
        _halfPhase = 0.0f;
        _advanceHalf();
    }
    // For holds 1 and 2: clear pause; update() will resume from current phase.
    Serial.printf("[GaitCtrl] POSE_STEP: nextPose (was hold=%d)\n", hold);
}

// ---------------------------------------------------------------------------
//  _triggerShiftForHalf  —  send the correct WeightShift direction for the
//  given half-cycle. Idempotent: skipped if direction has not changed.
// ---------------------------------------------------------------------------
void GaitController::_triggerShiftForHalf(uint8_t half) {
    if (!_ws) return;
    // half=0 → body weight over LEFT leg → trigger LEFT shift (+1)
    // half=1 → body weight over RIGHT leg → trigger RIGHT shift (-1)
    const int8_t wantDir = (half == 0) ? +1 : -1;
    if (wantDir == _lastShiftDir) return;
    _lastShiftDir = wantDir;
    _ws->trigger(wantDir > 0 ? ShiftDirection::LEFT : ShiftDirection::RIGHT);
    Serial.printf("[GaitCtrl] half=%d  shift=%s\n",
                  half, wantDir > 0 ? "LEFT" : "RIGHT");
}

// ---------------------------------------------------------------------------
//  _swingFrac  —  sin-shaped lift fraction.
//  localPhase ∈ [0, 1) → [0, 1]: 0 at start, 1.0 at midpoint, 0 at end.
// ---------------------------------------------------------------------------
float GaitController::_swingFrac(float localPhase) const {
    return sinf(localPhase * static_cast<float>(M_PI));
}

// ---------------------------------------------------------------------------
//  _submitSwingLift  —  SOURCE_GAIT hip flexion + knee flexion for one leg.
//  frac=0 → grounded (no command); frac=1 → peak lift.
// ---------------------------------------------------------------------------
void GaitController::_submitSwingLift(uint8_t hipCh, uint8_t kneeCh, float frac) {
    if (!_mm) return;
    // Joint convention: positive = anatomical flexion (from joint_config.h).
    // Hip flexion (positive) raises the thigh forward/upward — correct for a step.
    // Hip extension (negative) would push the leg rearward — wrong.
    // Knee flexion (positive) bends the knee for ground clearance — correct.
    _mm->submit(SOURCE_GAIT, hipCh,   frac * _cfg.stepHeightHipDeg);
    _mm->submit(SOURCE_GAIT, kneeCh,  frac * _cfg.stepHeightKneeDeg);
}

// ---------------------------------------------------------------------------
//  _advanceHalf  —  called when _halfPhase >= 1.0 (foot back on ground).
//  Flips the half-cycle and re-enters WAIT_SHIFT for the next leg.
//  SINGLE_STEP stops after completing half=1 (both legs have taken one step).
// ---------------------------------------------------------------------------
void GaitController::_advanceHalf() {
    if (_state.mode == GaitMode::SINGLE_STEP && _currentHalf == 1) {
        stop();
        Serial.println("[GaitCtrl] SINGLE_STEP complete — IDLE.");
        return;
    }

    _currentHalf = 1 - _currentHalf;   // 0→1 or 1→0
    _halfPhase   = 0.0f;
    _subState    = StepSubState::WAIT_SHIFT;
    _triggerShiftForHalf(_currentHalf);
}

// ---------------------------------------------------------------------------
//  update  —  main control loop, every tick from main.cpp.
// ---------------------------------------------------------------------------
void GaitController::update(float dt_s, const IMUState& imuState) {
    if (_mm == nullptr)   return;
    if (oe_is_estopped()) return;

    // Clear per-tick lift outputs (re-set below when swinging).
    _state.liftL_deg  = 0.0f;
    _state.liftR_deg  = 0.0f;
    _state.liftGateOK = false;

    if (_state.mode == GaitMode::IDLE) {
        _state.phase      = 0.0f;
        _state.wsProgress = 0.0f;
        _state.poseHold   = 0;
        return;
    }

    // ── Weight shift state ────────────────────────────────────────────────────
    const float wsProgress = _ws ? _ws->getState().progress : 1.0f;
    _state.wsProgress = wsProgress;

    // Gate direction check (signed, not just magnitude):
    //   half=0 (LEFT shift) → wsProgress ramps toward +1 → gate when >= +threshold
    //   half=1 (RIGHT shift) → wsProgress ramps toward -1 → gate when <= -threshold
    if (_currentHalf == 0) {
        _state.liftGateOK = (wsProgress >= _cfg.liftGateThreshold);
    } else {
        _state.liftGateOK = (wsProgress <= -_cfg.liftGateThreshold);
    }

    _state.subState = _subState;
    _state.phase    = _currentHalf * 0.5f + _halfPhase * 0.5f;
    _state.poseHold = _poseHoldPt;

    // ── POSE_STEP: if currently paused, re-submit lift to hold pose ───────────
    if (_state.mode == GaitMode::POSE_STEP && _poseHoldPt > 0) {
        if (_subState == StepSubState::SWINGING) {
            const float liftFrac = _swingFrac(_halfPhase);
            if (_currentHalf == 0) {
                _submitSwingLift(IDX_L_HIP_PITCH, IDX_L_KNEE_PITCH, liftFrac);
                _state.liftL_deg = liftFrac * _cfg.stepHeightHipDeg;
            } else {
                _submitSwingLift(IDX_R_HIP_PITCH, IDX_R_KNEE_PITCH, liftFrac);
                _state.liftR_deg = liftFrac * _cfg.stepHeightHipDeg;
            }
        }
        return;
    }

    // ── WSHIFT_ONLY mode ──────────────────────────────────────────────────────
    if (_state.mode == GaitMode::WSHIFT_ONLY) {
        if (_cfg.phaseRateHz > 0.0f) {
            const float halfPeriod_s = 1.0f / (2.0f * _cfg.phaseRateHz);
            _halfPhase += dt_s / halfPeriod_s;
            if (_halfPhase >= 1.0f) {
                _halfPhase   = 0.0f;
                _currentHalf = 1 - _currentHalf;
                _triggerShiftForHalf(_currentHalf);
            }
        }
        return;
    }

    // ── RUNNING / SINGLE_STEP / POSE_STEP: weight-shift-gated stepping ────────

    if (_subState == StepSubState::WAIT_SHIFT) {
        if (!_state.liftGateOK) return;  // still waiting for CoM to shift
        // Gate opened — begin the swing arc from zero for a clean sin profile.
        _subState  = StepSubState::SWINGING;
        _halfPhase = 0.0f;
        _state.subState = _subState;
        Serial.printf("[GaitCtrl] Gate open  half=%d  ws=%.3f  roll=%.4f rad\n",
                      _currentHalf, wsProgress, imuState.roll);

        // POSE_STEP: pause here so user can verify weight shift before lift.
        if (_state.mode == GaitMode::POSE_STEP) {
            _poseHoldPt     = 1;
            _state.poseHold = 1;
            Serial.println("[GaitCtrl] POSE_STEP: hold=1 gate-open");
            return;
        }
    }

    // SWINGING: compute lift, submit, advance phase.
    const float liftFrac = _swingFrac(_halfPhase);

    if (_currentHalf == 0) {
        _submitSwingLift(IDX_L_HIP_PITCH, IDX_L_KNEE_PITCH, liftFrac);
        _state.liftL_deg = liftFrac * _cfg.stepHeightHipDeg;
    } else {
        _submitSwingLift(IDX_R_HIP_PITCH, IDX_R_KNEE_PITCH, liftFrac);
        _state.liftR_deg = liftFrac * _cfg.stepHeightHipDeg;
    }

    if (_cfg.phaseRateHz > 0.0f) {
        const float halfPeriod_s = 1.0f / (2.0f * _cfg.phaseRateHz);
        const float newPhase     = _halfPhase + dt_s / halfPeriod_s;

        // POSE_STEP: pause at peak (halfPhase first crosses 0.5).
        if (_state.mode == GaitMode::POSE_STEP &&
            _halfPhase < 0.5f && newPhase >= 0.5f) {
            _halfPhase      = 0.5f;
            _poseHoldPt     = 2;
            _state.poseHold = 2;
            _state.phase    = _currentHalf * 0.5f + _halfPhase * 0.5f;
            Serial.println("[GaitCtrl] POSE_STEP: hold=2 peak");
            return;
        }

        _halfPhase = newPhase;
    }

    if (_halfPhase >= 1.0f) {
        // POSE_STEP: pause at landing before advancing to next half-cycle.
        if (_state.mode == GaitMode::POSE_STEP) {
            _halfPhase      = 0.999f;  // hold near end; nextPose() will advance
            _poseHoldPt     = 3;
            _state.poseHold = 3;
            _state.phase    = _currentHalf * 0.5f + _halfPhase * 0.5f;
            Serial.println("[GaitCtrl] POSE_STEP: hold=3 landing");
            return;
        }
        _halfPhase = 0.0f;
        _advanceHalf();
    }
}
