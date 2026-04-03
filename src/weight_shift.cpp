// ============================================================
// FILE:    weight_shift.cpp
// MODULE:  gait
// LAYER:   3.5 — Gait / Motion
//
// PURPOSE:  See weight_shift.h.
//
// SIGN CONVENTIONS:
//   state.roll: positive = lean RIGHT (from state_estimator.h).
//   roll_setpoint_rad:
//     negative → balance controller tries to reach lean-left equilibrium.
//     positive → balance controller tries to reach lean-right equilibrium.
//   progress:
//     +1.0 = full LEFT shift → roll_setpoint = -setpoint_shift_rad (lean left)
//     -1.0 = full RIGHT shift → roll_setpoint = +setpoint_shift_rad (lean right)
//   Formula: roll_setpoint = −progress × setpoint_shift_rad
//
// LAST CHANGED: 2026-03-26  |  Hinz  |  Initial implementation.
// ============================================================

#include "weight_shift.h"
#include "balance_controller.h"
#include "oe_control.h"
#include <math.h>  // fabsf

// ---------------------------------------------------------------------------
void WeightShift::init(BalanceController* bal, MotionManager* mm) {
    if (bal == nullptr || mm == nullptr) {
        Serial.println("[WeightShift] ERROR: init() received null pointer. "
                       "update() will be a no-op.");
    }
    _bal   = bal;
    _mm    = mm;
    _state = {};
    _targetProgress             = 0.0f;
    _lastInjectedSetpointRad    = 0.0f;
    Serial.println("[WeightShift] Initialized.");
}

// ---------------------------------------------------------------------------
void WeightShift::trigger(ShiftDirection dir) {
    _state.direction = dir;
    // _targetProgress is the ramp destination:
    //   LEFT  → +1.0   RIGHT → -1.0   NONE → 0.0
    _targetProgress = static_cast<float>(static_cast<int8_t>(dir));
    Serial.printf("[WeightShift] Trigger: %s  target_progress=%.2f\n",
                  (dir == ShiftDirection::LEFT)  ? "LEFT"   :
                  (dir == ShiftDirection::RIGHT) ? "RIGHT"  : "CENTER",
                  _targetProgress);
}

// ---------------------------------------------------------------------------
void WeightShift::update(float dt_s) {
    if (_bal == nullptr || _mm == nullptr) return;
    if (oe_is_estopped()) return;

    // ── Advance ramp ──────────────────────────────────────────────────────
    // Step size = dt / ramp_duration. Linear ramp is sufficient at this stage.
    // A minimum ramp floor prevents divide-by-zero; 10 ms is physically unreachable
    // at 400 Hz (2.5 ms per tick), so this guard never fires in normal operation.
    const float ramp_s = (_cfg.ramp_ms > 10.0f) ? _cfg.ramp_ms * 1e-3f : 0.01f;
    const float step   = dt_s / ramp_s;

    const float diff = _targetProgress - _state.progress;
    if (fabsf(diff) <= step) {
        _state.progress = _targetProgress;
        _state.ramping  = false;
    } else {
        _state.progress += (diff > 0.0f) ? step : -step;
        _state.ramping   = true;
    }

    // ── 1. Roll setpoint injection ────────────────────────────────────────
    // Only write when the setpoint actually changes — prevents WeightShift from
    // overwriting any manual trim the user applies via the UI roll setpoint slider
    // while progress = 0 (fully centered).
    //
    // Magnitude is clamped to [0, ∞) here: a negative setpoint_shift_rad would
    // reverse LEFT/RIGHT semantics, which is user error. Clamp enforces the
    // convention that setpoint_shift_rad is always a magnitude.
    //
    // Sign convention: progress = +1 (full LEFT shift) → lean left → negative setpoint.
    //   roll_setpoint = −progress × |setpoint_shift_rad|
    const float shiftMag       = fabsf(_cfg.setpoint_shift_rad);  // enforce magnitude
    const float newSetpointRad = -_state.progress * shiftMag;

    // Threshold: only write if offset changed by more than 0.5 mrad.
    // This suppresses the every-tick overwrite while still tracking the ramp accurately.
    if (fabsf(newSetpointRad - _lastInjectedSetpointRad) > 0.0005f) {
        BalanceConfig cfg     = _bal->getConfig();
        cfg.roll_setpoint_rad = newSetpointRad;
        _bal->setConfig(cfg);
        _lastInjectedSetpointRad = newSetpointRad;
    }

    // ── 2. Stance-leg hip roll command (SOURCE_GAIT) ──────────────────────
    // Only the STANCE leg receives this command. The other leg floats.
    // SOURCE_GAIT (priority 1) loses to SOURCE_BALANCE (priority 2) on shared
    // joints. The balance controller defaults to hip_roll_ratio=0, so there is
    // no priority conflict unless the user explicitly increases that ratio.
    if (fabsf(_cfg.hip_shift_deg) > 0.01f && fabsf(_state.progress) > 0.01f) {
        if (_state.progress > 0.01f) {
            // Shifting LEFT: left foot is stance. Command left hip roll.
            // Positive joint angle = anatomical flexion direction for this joint.
            // If motion is wrong, negate hip_shift_deg in the UI.
            _mm->submit(SOURCE_GAIT, IDX_L_HIP_ROLL,
                         _state.progress * _cfg.hip_shift_deg);
        } else {
            // Shifting RIGHT: right foot is stance. Command right hip roll.
            // Negated because the right side is mirrored relative to left.
            _mm->submit(SOURCE_GAIT, IDX_R_HIP_ROLL,
                        -_state.progress * _cfg.hip_shift_deg);
        }
    }

    // ── 2b. Ankle roll feedforward (SOURCE_GAIT) ─────────────────────────
    // Primary CoM shift driver. Commands both ankle roll joints directly,
    // proportional to progress. The balance controller must have
    // ankle_roll_ratio=0 or it will override these via SOURCE_BALANCE (priority 2).
    //
    // Sign: mirrors balance_controller._applyRollCorrection() negate convention.
    //   progress > 0 (LEFT): lean left → L_ankle = +cmd, R_ankle = -cmd.
    //   This is the same command the balance controller issues to correct a right lean.
    if (fabsf(_cfg.ankle_shift_deg) > 0.01f && fabsf(_state.progress) > 0.01f) {
        float cmd = _state.progress * _cfg.ankle_shift_deg;
        _mm->submit(SOURCE_GAIT, IDX_L_ANKLE_ROLL, -cmd);
        _mm->submit(SOURCE_GAIT, IDX_R_ANKLE_ROLL,  cmd);
    }

    // ── 2c. Ankle pitch forward tilt (SOURCE_GAIT) ───────────────────────
    // Tilts both ankles forward proportionally to |progress| during weight shift.
    // Uses |progress| (not signed progress) because the forward lean preparation
    // applies equally for left and right shifts — it is a sagittal-plane effect
    // independent of the frontal-plane CoM translation.
    //
    // JointModel maps the same joint-relative angle to symmetric physical motion
    // on both sides via the direction field (right=+1, left=-1). No manual
    // sign flip per channel is needed or correct here.
    //
    // SOURCE_GAIT (1) < SOURCE_BALANCE (2): if balance pitch correction is active
    // on ankle pitch joints, SOURCE_BALANCE wins this tick and the tilt is not
    // applied. To use tilt independently, reduce ankle_ratio in BalanceConfig.
    if (fabsf(_cfg.ankle_pitch_tilt_deg) > 0.01f && fabsf(_state.progress) > 0.01f) {
        // Scale smoothly with |progress|: 0 at center, full at ±1.0.
        float tiltCmd = fabsf(_state.progress) * _cfg.ankle_pitch_tilt_deg;
        _mm->submit(SOURCE_GAIT, IDX_R_ANKLE_PITCH, tiltCmd);
        _mm->submit(SOURCE_GAIT, IDX_L_ANKLE_PITCH, tiltCmd);
    }

    // ── 3. Torso roll command (SOURCE_GAIT) ───────────────────────────────
    // Direction unverified on hardware — keep torso_shift_deg = 0 until confirmed.
    // If torso moves wrong way, negate torso_shift_deg in the UI.
    // TODO(FSR): when foot contact is available, condition torso command on
    //            confirmed single-support phase rather than motion command.
    
    if (fabsf(_cfg.torso_shift_deg) > 0.01f && fabsf(_state.progress) > 0.01f) {
        _mm->submit(SOURCE_GAIT, IDX_TORSO_ROLL,
                     _state.progress * _cfg.torso_shift_deg);
    }

    // ── 4. Swing leg lift (SOURCE_GAIT) ──────────────────────────────────
    // Raises the swing leg once sufficient weight is on the stance leg.
    //
    // lift_scale: linearly ramps from 0 at swing_lift_threshold to 1 at
    // |progress|=1.0. Submitting 0° (liftScale=0) as progress returns toward
    // zero issues a "go to neutral" command that the smooth-stepper tracks.
    //
    // SOURCE_GAIT (1) < SOURCE_BALANCE (2): documented conflict guard in config.
    {
        // Constrain threshold away from 1.0 to prevent division by zero.
        const float th = constrain(_cfg.swing_lift_threshold, 0.0f, 0.95f);

        float absProgress = fabsf(_state.progress);
        float liftScale   = 0.0f;

        if (absProgress > th) {
            // Linear ramp from 0 at threshold to 1 at full shift.
            liftScale = (absProgress - th) / (1.0f - th);
            liftScale = constrain(liftScale, 0.0f, 1.0f);
        }

        // Submit whenever the shift is active. submitting 0° during the return
        // ramp commands the joints back to neutral via the smooth-stepper.
        if (absProgress > 0.01f) {
            uint8_t swingHipCh, swingKneeCh;
            if (_state.progress > 0.0f) {
                // Shifting LEFT → right leg is swing.
                swingHipCh  = IDX_R_HIP_PITCH;
                swingKneeCh = IDX_R_KNEE_PITCH;
            } else {
                // Shifting RIGHT → left leg is swing.
                swingHipCh  = IDX_L_HIP_PITCH;
                swingKneeCh = IDX_L_KNEE_PITCH;
            }

            // Hip extension: negative joint angle (joint convention: positive = flexion).
            // Knee flexion:  positive joint angle.
            float hipCmd  = -liftScale * _cfg.swing_hip_extension_deg;
            float kneeCmd =  liftScale * _cfg.swing_knee_flexion_deg;

            _mm->submit(SOURCE_GAIT, swingHipCh,  hipCmd);
            _mm->submit(SOURCE_GAIT, swingKneeCh, kneeCmd);
        }
    }
}