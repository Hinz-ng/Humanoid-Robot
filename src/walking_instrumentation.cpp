// =============================================================================
// FILE:    walking_instrumentation.cpp
// MODULE:  diagnostics
// See walking_instrumentation.h for the full contract and proxy notes.
//
// INSTRUMENTATION ONLY — read-only consumer of state already computed this tick.
// No MotionManager, no OE/e-stop, no control feedback.
// =============================================================================
#include "walking_instrumentation.h"
#include "project_wide_defs.h"   // FOOT_IMU_ACCEL_SCALE_LSB_PER_G
#include <math.h>

static constexpr float WI_RAD2DEG = 57.295779513f;

// -----------------------------------------------------------------------------
void WalkingInstrumentation::init() {
    reset();
}

void WalkingInstrumentation::reset() {
    const bool en = _cfg.enabled;          // preserve config across reset
    _state = WalkingInstrState();
    _state.enabled = en;
    _resetStepAccum();
    for (uint8_t i = 0; i < SIDE_COUNT; ++i) _foot[i] = FootDetect();

    _running       = false;
    _prevSubState  = StepSubState::WAIT_SHIFT;
    _prevTdSeen    = false;
    _prevTargValid = false;
    for (uint8_t i = 0; i < SIDE_COUNT; ++i) {
        _prevFx[i] = 0.0f; _prevFy[i] = 0.0f; _prevFh[i] = 0.0f;
        _prevFlatActive[i] = false;
    }
    _prevBodyHValid = false;
    _prevBodyH      = 0.0f;
    _bodyHvelPrev   = 0.0f;
    _rolloverSeqInit = false;   // re-sync to gait's live seq on the next tick
    _prevRolloverSeq = 0;
}

void WalkingInstrumentation::_resetStepAccum() {
    _acc = StepAccum();   // default-construct zeroes every per-step field
}

// -----------------------------------------------------------------------------
// Per-foot onset detector on the already-filtered accel magnitude. Hysteresis +
// refractory turn the noisy spike into discrete rising edges.
bool WalkingInstrumentation::_detectOnset(uint8_t side, float magG, uint32_t now) {
    FootDetect& f = _foot[side];
    const float hi = _cfg.contactBaselineG + _cfg.contactThreshG;
    const float lo = _cfg.contactBaselineG + _cfg.contactThreshG * 0.5f;  // release band
    bool onset = false;
    if (!f.above) {
        if (magG >= hi) {
            f.above = true;
            if ((now - f.lastOnsetMs) >= _cfg.onsetRefractoryMs) {
                f.lastOnsetMs = now;
                onset = true;
            }
        }
    } else if (magG <= lo) {
        f.above = false;
    }
    return onset;
}

// -----------------------------------------------------------------------------
void WalkingInstrumentation::_finalizeStep() {
    WalkingStepMetrics m;
    m.v[WI_TD_ACCEL]       = _acc.peakAccelG;
    m.v[WI_TD_IMPULSE]     = _acc.impulseGs;
    m.v[WI_TD_SWING_SPEED] = _acc.tdSwingSpeed;
    m.v[WI_DOUBLE_TD]      = _acc.doubleTd ? 1.0f : 0.0f;
    m.v[WI_DOUBLE_TD_INT]  = _acc.doubleTdIntMs;
    m.v[WI_SWING_SPEED]    = _acc.peakSwing3D;
    m.v[WI_SWING_HORIZ]    = _acc.peakSwingH;
    m.v[WI_SWING_VERT]     = _acc.peakSwingV;
    m.v[WI_SWING_ENERGY]   = 0.5f * _acc.peakSwing3D * _acc.peakSwing3D;
    m.v[WI_COM_BOB]        = _acc.bodyHseen ? (_acc.bodyHmax - _acc.bodyHmin) : 0.0f;
    m.v[WI_COM_VZ_TD]      = _acc.comVzTd;
    m.v[WI_COM_LAT_SPEED]  = _acc.peakVyCoM;
    m.v[WI_COM_SAG_SPEED]  = _acc.peakVxCoM;
    m.v[WI_TORSO_ROLL_RATE]= _acc.peakRollDps;
    m.v[WI_TORSO_ROLL_TD]  = _acc.torsoRollTdDeg;
    m.v[WI_REACH_PCT]      = _acc.peakReachPct;
    m.v[WI_FLATFOOT_ACT]   = (float)_acc.flatActCount;
    m.v[WI_TD_VERT_PEAK]   = _acc.tdVertPeakG;
    m.v[WI_TD_HORIZ_PEAK]  = _acc.tdHorizPeakG;
    m.v[WI_TD_IMPULSE_DUR] = _acc.impulseDurMs;
    m.v[WI_TD_WIN_ROLL]    = _acc.tdWinRollDps;
    // Signed offset (peak-roll sample time − primary touchdown). Negative = the
    // dominant body kick precedes contact. Only meaningful once primary latched.
    m.v[WI_PEAK_ROLL_TIME] = _acc.primarySet
        ? (float)((int32_t)_acc.peakRollTimeAbsMs - (int32_t)_acc.firstOnsetMs)
        : 0.0f;
    // Rollover aggregates copied in from GaitController on the rolloverSeq edge
    // (0 if no post-contact window completed during this step).
    m.v[WI_ROLL_FP_PEAK]    = _acc.rollFpPeakDeg;
    m.v[WI_ROLL_FP_RANGE]   = _acc.rollFpRangeDeg;
    m.v[WI_ROLL_DIV_REL]    = _acc.rollDivRelDeg;
    m.v[WI_ROLL_DURATION]   = _acc.rollDurationMs;
    m.v[WI_ROLL_BODY_PITCH] = _acc.rollBodyPitchDeg;
    m.v[WI_ROLL_COM_VSIGN]  = _acc.rollComVelSign;

    // Onset timeline travels with the step (copied into the same ring entry).
    m.onsetN = _acc.onsetN;
    for (uint8_t i = 0; i < WI_ONSET_MAX; ++i) m.onsetMs[i] = _acc.onsetMs[i];

    // Push newest-first into the ring (shift down by one). 10 structs once per
    // ~1-2 s — negligible vs the 400 Hz hot path.
    for (int i = WI_HISTORY_LEN - 1; i > 0; --i) _state.hist[i] = _state.hist[i - 1];
    _state.hist[0] = m;
    if (_state.histCount < WI_HISTORY_LEN) _state.histCount++;

    // Averages over the valid history window.
    for (uint8_t k = 0; k < WI_METRIC_COUNT; ++k) {
        float s = 0.0f;
        for (uint8_t i = 0; i < _state.histCount; ++i) s += _state.hist[i].v[k];
        _state.avg[k] = (_state.histCount > 0) ? (s / (float)_state.histCount) : 0.0f;
    }

    _state.current        = m;
    _state.completedSteps++;
    _state.lastOnsetCount = _acc.onsetCount;
}

// -----------------------------------------------------------------------------
void WalkingInstrumentation::update(float dt_s,
                                    const IMUState&           torso,
                                    const IMUState&           leftFoot,
                                    const IMUState&           rightFoot,
                                    const RawIMUData&         leftFootRaw,
                                    const RawIMUData&         rightFootRaw,
                                    const FootImpactDetector& impact,
                                    const GaitState&          gait) {
    _state.enabled = _cfg.enabled;

    // Commanded foot targets (per leg) and body-height proxy = stance hip height.
    const float fhL = gait.lastFootTargetHL_mm;
    const float fhR = gait.lastFootTargetHR_mm;
    const float bodyH = (fhL > fhR) ? fhL : fhR;
    const float fx[SIDE_COUNT] = { gait.lastFootTargetXL_mm, gait.lastFootTargetXR_mm };
    const float fy[SIDE_COUNT] = { gait.lastFootTargetYL_mm, gait.lastFootTargetYR_mm };
    const float fh[SIDE_COUNT] = { fhL, fhR };

    const uint32_t now = millis();
    const bool dtOk = (dt_s > 1e-6f);

    // Rollover consumer: sync the seq edge-detector to gait's live value on the
    // first tick after a reset so a pre-existing (stale) result is never folded
    // into the first step. After this, a change in rolloverSeq means one fresh
    // post-contact result is ready to record.
    if (!_rolloverSeqInit) { _prevRolloverSeq = gait.rolloverSeq; _rolloverSeqInit = true; }

    // Disabled: hold metrics, only keep edge trackers fresh (incl. rolloverSeq,
    // so re-enabling does not retro-record a result that completed while off).
    if (!_cfg.enabled) {
        _running       = gait.running;
        _prevSubState  = gait.subState;
        _prevTdSeen    = gait.touchdownSeen;
        _prevTargValid = true;
        for (uint8_t i = 0; i < SIDE_COUNT; ++i) { _prevFx[i] = fx[i]; _prevFy[i] = fy[i]; _prevFh[i] = fh[i]; }
        _prevBodyHValid = true; _prevBodyH = bodyH;
        _prevRolloverSeq = gait.rolloverSeq;
        return;
    }

    // Per-foot commanded velocities from foot-target finite differences.
    float vH[SIDE_COUNT] = {0.0f, 0.0f};
    float vV[SIDE_COUNT] = {0.0f, 0.0f};
    float v3[SIDE_COUNT] = {0.0f, 0.0f};
    if (_prevTargValid && dtOk) {
        const float inv = 1.0f / dt_s;
        for (uint8_t i = 0; i < SIDE_COUNT; ++i) {
            const float dx = fx[i] - _prevFx[i];
            const float dy = fy[i] - _prevFy[i];
            const float dh = fh[i] - _prevFh[i];
            vH[i] = sqrtf(dx * dx + dy * dy) * inv;
            vV[i] = fabsf(dh) * inv;
            v3[i] = sqrtf(vH[i] * vH[i] + vV[i] * vV[i]);
        }
    }

    // Commanded body-height velocity; 1-tick lag retained for CoM-vz @ touchdown.
    float bodyHvel = 0.0f;
    if (_prevBodyHValid && dtOk) bodyHvel = (bodyH - _prevBodyH) / dt_s;

    // Only measure while walking. Otherwise keep trackers continuous (small
    // deltas at the next start) and discard any partial step.
    if (!gait.running) {
        _resetStepAccum();
        _running       = false;
        _prevSubState  = gait.subState;
        _prevTdSeen    = gait.touchdownSeen;
        _prevTargValid = true;
        for (uint8_t i = 0; i < SIDE_COUNT; ++i) {
            _prevFx[i] = fx[i]; _prevFy[i] = fy[i]; _prevFh[i] = fh[i];
            _prevFlatActive[i] = false;
        }
        _prevBodyHValid = true; _prevBodyH = bodyH; _bodyHvelPrev = bodyHvel;
        _prevRolloverSeq = gait.rolloverSeq;   // sync; don't record while stopped
        return;
    }

    // Step boundary: rising edge of leaving SWINGING = one completed step.
    if (_running &&
        _prevSubState == StepSubState::SWINGING &&
        gait.subState != StepSubState::SWINGING) {
        _finalizeStep();
        _resetStepAccum();
    }

    // ---- rollover (post-contact): fold one completed window into the CURRENT
    // step on the rising edge of gait.rolloverSeq. Placed after the step-boundary
    // reset so a result that completes is attributed to the step in progress
    // (the step in which the just-landed foot is now stance). One result per
    // step in steady walking; a step with none keeps zeros (matching the other
    // metrics' "0 when the event did not occur" convention). ----
    if (gait.rolloverSeq != _prevRolloverSeq) {
        _prevRolloverSeq      = gait.rolloverSeq;
        _acc.rollFpPeakDeg    = gait.rolloverFootPitchPeakDeg;
        _acc.rollFpRangeDeg   = gait.rolloverFootPitchRangeDeg;
        _acc.rollDivRelDeg    = gait.rolloverDivergencePeakRelDeg;
        _acc.rollDurationMs   = (float)gait.rolloverDurationMs;
        _acc.rollBodyPitchDeg = gait.postContactBodyPitchPeakDeg;
        _acc.rollComVelSign   = (float)gait.postContactCoMVelSign;
    }

    // ---- swing-foot speed peaks (max of both feet; swing foot dominates) ----
    for (uint8_t i = 0; i < SIDE_COUNT; ++i) {
        if (v3[i] > _acc.peakSwing3D) _acc.peakSwing3D = v3[i];
        if (vH[i] > _acc.peakSwingH)  _acc.peakSwingH  = vH[i];
        if (vV[i] > _acc.peakSwingV)  _acc.peakSwingV  = vV[i];
    }

    // ---- commanded CoM bob (peak-to-peak of stance hip height) ----
    if (!_acc.bodyHseen) { _acc.bodyHseen = true; _acc.bodyHmin = bodyH; _acc.bodyHmax = bodyH; }
    else {
        if (bodyH < _acc.bodyHmin) _acc.bodyHmin = bodyH;
        if (bodyH > _acc.bodyHmax) _acc.bodyHmax = bodyH;
    }

    // ---- measured body response (torso estimator) ----
    if (torso.valid) {
        const float avy  = fabsf(torso.vy_CoM_mm_s);
        const float avx  = fabsf(torso.vx_CoM_mm_s);
        const float adps = fabsf(torso.rollRate) * WI_RAD2DEG;
        if (avy  > _acc.peakVyCoM)   _acc.peakVyCoM   = avy;
        if (avx  > _acc.peakVxCoM)   _acc.peakVxCoM   = avx;
        // Track the peak roll rate AND the time it occurred (for the offset-to-TD
        // metric). _finalizeStep() differences this against firstOnsetMs.
        if (adps > _acc.peakRollDps) { _acc.peakRollDps = adps; _acc.peakRollTimeAbsMs = now; }
        // Touchdown-response window: max roll rate inside [start,end] ms after the
        // primary touchdown only — isolates the touchdown-specific roll kick.
        if (_acc.primarySet) {
            const uint32_t sinceTd = now - _acc.firstOnsetMs;
            if (sinceTd >= _cfg.tdRollWinStartMs && sinceTd <= _cfg.tdRollWinEndMs &&
                adps > _acc.tdWinRollDps) {
                _acc.tdWinRollDps = adps;
            }
        }
    }

    // ---- peak reach % (max of both legs) ----
    {
        const float r = (gait.lastReachPctL > gait.lastReachPctR)
                            ? gait.lastReachPctL : gait.lastReachPctR;
        if (r > _acc.peakReachPct) _acc.peakReachPct = r;
    }

    // ---- flat-foot guard activations (rising edges, both legs) ----
    {
        const bool aL = fabsf(gait.lastFlatFootErrL_deg) > _cfg.flatFootErrThreshDeg;
        const bool aR = fabsf(gait.lastFlatFootErrR_deg) > _cfg.flatFootErrThreshDeg;
        if (aL && !_prevFlatActive[sideIdx(Side::LEFT)])  _acc.flatActCount++;
        if (aR && !_prevFlatActive[sideIdx(Side::RIGHT)]) _acc.flatActCount++;
        _prevFlatActive[sideIdx(Side::LEFT)]  = aL;
        _prevFlatActive[sideIdx(Side::RIGHT)] = aR;
    }

    // ---- per-foot accel decomposition (sole-normal vs in-sole-plane) ----
    // The remap applied in main.cpp is a constant chip→foot-frame transform, so
    // remapped Z is the foot's local vertical (sole-normal) and X/Y span the
    // sole plane. Same one-pole LPF (alpha) the impact detector runs, applied
    // per-axis — a decomposition of the same conditioned signal, no new filter.
    const RawIMUData* rawFoot[SIDE_COUNT] = { &leftFootRaw, &rightFootRaw };
    const float accAlpha = impact.getConfig().lpf_alpha;
    float aVert[SIDE_COUNT]  = {0.0f, 0.0f};   // |a_z|            [g]
    float aHoriz[SIDE_COUNT] = {0.0f, 0.0f};   // hypot(a_x,a_y)   [g]
    for (uint8_t i = 0; i < SIDE_COUNT; ++i) {
        FootDetect& fd = _foot[i];
        if (rawFoot[i]->valid) {
            const float ax = (float)rawFoot[i]->accel_x / FOOT_IMU_ACCEL_SCALE_LSB_PER_G;
            const float ay = (float)rawFoot[i]->accel_y / FOOT_IMU_ACCEL_SCALE_LSB_PER_G;
            const float az = (float)rawFoot[i]->accel_z / FOOT_IMU_ACCEL_SCALE_LSB_PER_G;
            if (!fd.accSeeded) { fd.axF = ax; fd.ayF = ay; fd.azF = az; fd.accSeeded = true; }
            else {
                fd.axF = accAlpha * ax + (1.0f - accAlpha) * fd.axF;
                fd.ayF = accAlpha * ay + (1.0f - accAlpha) * fd.ayF;
                fd.azF = accAlpha * az + (1.0f - accAlpha) * fd.azF;
            }
        }
        // Hold last value when a read is invalid (footstrike valid-flicker).
        aVert[i]  = fabsf(fd.azF);
        aHoriz[i] = sqrtf(fd.axF * fd.axF + fd.ayF * fd.ayF);
    }

    // ---- touchdown onset detection (both feet) ----
    const float magL = impact.getState(Side::LEFT).magnitude_g;
    const float magR = impact.getState(Side::RIGHT).magnitude_g;
    const float mag[SIDE_COUNT] = { magL, magR };
    bool onset[SIDE_COUNT];
    onset[sideIdx(Side::LEFT)]  = _detectOnset(sideIdx(Side::LEFT),  magL, now);
    onset[sideIdx(Side::RIGHT)] = _detectOnset(sideIdx(Side::RIGHT), magR, now);

    // Peak touchdown accel = max over step of max(both feet).
    {
        const float m = (magL > magR) ? magL : magR;
        if (m > _acc.peakAccelG) _acc.peakAccelG = m;
    }

    // halfPhase ∈ [0,1) inside the current half-cycle (fallback primary gate).
    const float twoPhase = gait.phase * 2.0f;
    const float halfPhase = twoPhase - floorf(twoPhase);

    if (!_acc.primarySet) {
        // Primary touchdown: prefer the gait's own validated, phase-gated latch;
        // fall back to our late-swing onset if the gait latch is unavailable.
        bool    trig = false;
        uint8_t lf   = 0;
        if (gait.touchdownSeen && !_prevTdSeen) {
            trig = true;
            lf   = (gait.touchdownSide < SIDE_COUNT) ? gait.touchdownSide : 0;
        } else if (gait.subState == StepSubState::SWINGING && halfPhase >= 0.4f) {
            if      (onset[sideIdx(Side::LEFT)])  { trig = true; lf = sideIdx(Side::LEFT); }
            else if (onset[sideIdx(Side::RIGHT)]) { trig = true; lf = sideIdx(Side::RIGHT); }
        }
        if (trig) {
            _acc.primarySet      = true;
            _acc.landingFoot     = lf;
            _acc.firstOnsetMs    = now;
            _acc.tdSwingSpeed    = v3[lf];
            _acc.comVzTd         = _bodyHvelPrev;                          // descent just before contact
            _acc.torsoRollTdDeg  = torso.valid ? (torso.roll * WI_RAD2DEG) : 0.0f;
            _acc.onsetCount      = 1;                                       // the strike itself
            _acc.onsetN          = 0;
            _acc.onsetMs[_acc.onsetN++] = 0;                                // primary onset at t=0
            _acc.impulseOpen     = true;
            _acc.impulseStartMs  = now;
            _acc.liftConfirmed   = false;
            _acc.liftTimerActive = false;
        }
    } else {
        // Impulse integration on the landing foot, time-boxed after primary.
        // Same window also bounds the duration timer and the vert/horiz peaks so
        // every touchdown-decomposition metric shares one definition of "impact".
        if (_acc.impulseOpen) {
            if ((now - _acc.impulseStartMs) <= _cfg.impulseWindowMs) {
                const uint8_t lf = _acc.landingFoot;
                const float   lm = mag[lf];
                const float excess = lm - _cfg.contactBaselineG;
                if (excess > 0.0f && dtOk) _acc.impulseGs += excess * dt_s;
                // Duration the landing foot stays at/above the onset threshold
                // (sharp spike → short; soft impact → long).
                const float onsetHi = _cfg.contactBaselineG + _cfg.contactThreshG;
                if (lm >= onsetHi && dtOk) _acc.impulseDurMs += dt_s * 1000.0f;
                // Vertical (sole-normal) vs horizontal (in-sole) impact peaks.
                if (aVert[lf]  > _acc.tdVertPeakG)  _acc.tdVertPeakG  = aVert[lf];
                if (aHoriz[lf] > _acc.tdHorizPeakG) _acc.tdHorizPeakG = aHoriz[lf];
            } else {
                _acc.impulseOpen = false;
            }
        }

        // Double-touchdown: a re-contact only counts if the foot actually lifted
        // (gyro-confirmed) between the primary and the second onset.
        const IMUState& fs = (_acc.landingFoot == sideIdx(Side::LEFT)) ? leftFoot : rightFoot;
        const float gyroDps = sqrtf(fs.pitchRate * fs.pitchRate +
                                    fs.rollRate  * fs.rollRate) * WI_RAD2DEG;
        if (gyroDps > _cfg.bounceGyroThreshDps) {
            if (!_acc.liftTimerActive) { _acc.liftTimerActive = true; _acc.liftTimerStart = now; }
            else if ((now - _acc.liftTimerStart) >= _cfg.releaseDwellMs) _acc.liftConfirmed = true;
        } else {
            _acc.liftTimerActive = false;   // lift must be sustained, not a single tick
        }

        if (onset[_acc.landingFoot]) {
            _acc.onsetCount++;
            // Record the onset offset for the audit timeline (capped length).
            if (_acc.onsetN < WI_ONSET_MAX) {
                const uint32_t off = now - _acc.firstOnsetMs;
                _acc.onsetMs[_acc.onsetN++] = (off > 65535u) ? 65535u : (uint16_t)off;
            }
            if (_acc.liftConfirmed && !_acc.doubleTd) {
                _acc.doubleTd      = true;
                _acc.doubleTdIntMs = (float)(now - _acc.firstOnsetMs);
            }
            _acc.liftConfirmed = false;     // require a fresh lift before counting again
        }
    }

    // ---- advance cross-tick trackers ----
    _running       = true;
    _prevSubState  = gait.subState;
    _prevTdSeen    = gait.touchdownSeen;
    _prevTargValid = true;
    for (uint8_t i = 0; i < SIDE_COUNT; ++i) { _prevFx[i] = fx[i]; _prevFy[i] = fy[i]; _prevFh[i] = fh[i]; }
    _prevBodyHValid = true;
    _prevBodyH      = bodyH;
    _bodyHvelPrev   = bodyHvel;
}
