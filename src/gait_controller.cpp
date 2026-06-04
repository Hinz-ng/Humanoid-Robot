// =============================================================================
// FILE:    gait_controller.cpp
// MODULE:  gait
// LAYER:   3.5 — Gait / Motion
// LAST CHANGED: 2026-05-24 | Hinz | promote GAIT_STABILIZE_MS → runtime stabilizeMs; add skipStabilizeNone
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
// Weight shift direction (hardware-verified, Phase 5 — now type-keyed):
//   _swingSide == Side::LEFT  → LEFT shift (body over left leg, LEFT foot swings)
//   _swingSide == Side::RIGHT → RIGHT shift (body over right leg, RIGHT foot swings)
// _triggerShiftForSwing() owns the wire-protocol mapping.
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
#include "balance_controller.h"  // BalanceState — needed for recovery flag observation
#include "weight_shift.h"
#include "swing_foot_leveler.h"  // SwingFootLeveler — ankle bias during swing
#include "foot_impact.h"         // FootImpactDetector — touchdown awareness (Stage 0/1)
#include "oe_control.h"
#include "leg_ik.h"
#include "joint_config.h"
#include "motion_manager.h"
#include <math.h>

// ---------------------------------------------------------------------------
//  _clearTouchdownLatch — wipe the per-swing latch + telemetry mirror.
//
//  Called from any boundary that ends "the current swing" semantically:
//    - _advanceHalf()  : new half-cycle starts; previous swing is over.
//    - start(mode)     : any mode entry starts a fresh half from LEFT swing.
//    - triggerRecovery(): recovery uses its own trajectory; latch is irrelevant.
//
//  Stage 0 telemetry fields are cleared here too so the UI never observes
//  stale "seen during last swing" values on the new half.
// ---------------------------------------------------------------------------
void GaitController::_clearTouchdownLatch() {
    _touchdownLocked         = false;
    _touchdownLockedX_mm     = 0.0f;
    _touchdownLockedY_mm     = 0.0f;
    _touchdownLockedH_mm     = 0.0f;
    _touchdownLockedPhase    = 0.0f;
    _touchdownMs             = 0;    // reset per-half timestamp
    _state.touchdownSeen      = false;
    _state.touchdownPhase     = 0.0f;
    _state.touchdownSwingX_mm = 0.0f;
    _state.touchdownLocked    = false;
    _state.touchdownSwingVelX_mm_s = 0.0f;
    _state.touchdownSwingFootRollRad = 0.0f;
    _state.touchdownTorsoRollRad     = 0.0f;
    // touchdownSide is not reset — it carries the side that the (now-cleared)
    // event referred to. UI reads it together with touchdownSeen; when seen=0
    // the UI treats side as "n/a". Leaving it intact avoids a flicker between
    // role-swap and the first detection on the new swing leg.
}

// ---------------------------------------------------------------------------
//  _resetCompliance — zero the W4 touchdown-compliance event state.
//
//  Called on mode entry (start), init, and recovery entry. NOT called at
//  half-cycle boundaries: an in-progress compliance decay must be allowed to
//  finish smoothly across the boundary rather than snapping to zero.
// ---------------------------------------------------------------------------
void GaitController::_resetCompliance() {
    _complianceActive         = false;
    _complianceStartMs        = 0;
    _complianceOffsetMm       = 0.0f;
    _state.complianceActive   = false;
    _state.complianceOffsetMm = 0.0f;
}

// ---------------------------------------------------------------------------
//  _resetRolloverTelemetry — clear the post-contact rollover observation
//  window + its telemetry mirror. Called on mode entry (start), init, and
//  recovery entry — same lifecycle as _resetCompliance. NOT called at
//  half-cycle boundaries: the window is anchored to the touchdown latch and is
//  allowed to run across _advanceHalf (the landed foot becomes stance while it
//  is still rolling to flat). TELEMETRY ONLY.
// ---------------------------------------------------------------------------
void GaitController::_resetRolloverTelemetry() {
    _rolloverActive        = false;
    _rolloverStartMs       = 0;
    _rolloverFlatReached   = false;
    _rolloverPeakComVelAbs = 0.0f;
    _state.rolloverActive                   = false;
    _state.rolloverFootPitchRad             = 0.0f;
    _state.ankleCmdVsFootPitchDivergenceDeg = 0.0f;
    _state.rolloverDurationMs               = 0;
    _state.postContactCoMVelSign            = 0;
    _state.postContactBodyPitchPeak         = 0.0f;
}

// ---------------------------------------------------------------------------
//  _updateRolloverTelemetry — refresh the post-contact observation window once
//  per update() tick. TELEMETRY ONLY: reads the just-landed foot's estimator,
//  the last submitted ankle-pitch command, and the torso IMUState; writes only
//  GaitState telemetry fields. Submits no joints and gates nothing, so control
//  behaviour is bit-identical whether or not this runs.
//
//  The window opens at the touchdown latch (where _rolloverActive is set) and
//  auto-closes rolloverWindowMs later. While open it streams the landed foot's
//  neutral-corrected pitch and the ankle-cmd-vs-foot divergence, latches the
//  first-flat time (rolloverDurationMs), and tracks the post-contact sagittal
//  CoM-velocity sign and the peak torso pitch.
// ---------------------------------------------------------------------------
void GaitController::_updateRolloverTelemetry(const IMUState& torso) {
    if (!_rolloverActive) return;

    const uint32_t now     = millis();
    const uint32_t elapsed = now - _rolloverStartMs;
    if (elapsed > _cfg.rolloverWindowMs) {
        // Window closed — stop streaming the live values but KEEP the captured
        // per-step summaries (duration / sign / peak) for the UI to display.
        _rolloverActive             = false;
        _state.rolloverActive       = false;
        _state.rolloverFootPitchRad = 0.0f;
        _state.ankleCmdVsFootPitchDivergenceDeg = 0.0f;
        return;
    }

    const uint8_t side = _rolloverSide;
    const StateEstimator* est = _footEst[side];
    const bool footOk = (est != nullptr) && est->getState().valid;

    // Neutral-corrected pitch of the just-landed foot (robot frame, rad).
    float footPitchCorr = 0.0f;
    if (footOk) footPitchCorr = est->getState().pitch - est->getNeutralPitch();
    _state.rolloverFootPitchRad = footPitchCorr;

    // Scrub proxy: commanded ankle pitch (deg, joint frame) − measured
    // neutral-corrected foot pitch (deg). The two live in different frames
    // (different zeros), so the magnitude is relative — read its CHANGE through
    // the window, which is what the later ankle-yield experiment will reduce.
    const float footPitchDeg = footPitchCorr * 57.29578f;
    _state.ankleCmdVsFootPitchDivergenceDeg = _lastAnkleCmdDeg[side] - footPitchDeg;

    // Rollover duration: first tick the foot reaches the flat band of neutral.
    if (footOk && !_rolloverFlatReached &&
        fabsf(footPitchCorr) <= _cfg.rolloverFlatBandRad) {
        _rolloverFlatReached      = true;
        _state.rolloverDurationMs = elapsed;
    }

    // Post-contact body response (torso): sign of the sagittal CoM velocity at
    // its peak |·| (forward-fall + vs backward-shove −), and the peak
    // (max-|·|, signed) torso pitch over the window.
    if (torso.valid) {
        const float vx = torso.vx_CoM_mm_s;   // sagittal CoM velocity (mm/s)
        if (fabsf(vx) > _rolloverPeakComVelAbs) {
            _rolloverPeakComVelAbs       = fabsf(vx);
            _state.postContactCoMVelSign = (vx > 0.0f) ? 1 : (vx < 0.0f ? -1 : 0);
        }
        if (fabsf(torso.pitch) > fabsf(_state.postContactBodyPitchPeak)) {
            _state.postContactBodyPitchPeak = torso.pitch;
        }
    }
}

// ---------------------------------------------------------------------------
//  _updateCompliance — W4 per-tick touchdown-compliance offset update.
//
//  When a compliance event is active, the stance-height offset decays from
//  -complianceDepthMm back to 0 over complianceDecayMs on a smoothstep
//  profile (full yield at the event, easing recovery — no hard snap-back).
//  The offset self-clears once the decay window elapses, so the behaviour is
//  always transient and never leaves a persistent posture bias.
//
//  Time-based: the event records only a start timestamp; the offset is a
//  pure function of elapsed time. This keeps the decay independent of FSM
//  state, so it continues cleanly through STABILIZING and the next
//  WAIT_SHIFT.
// ---------------------------------------------------------------------------
void GaitController::_updateCompliance() {
    if (!_complianceActive) {
        _complianceOffsetMm       = 0.0f;
        _state.complianceOffsetMm = 0.0f;
        _state.complianceActive   = false;
        return;
    }

    const uint32_t decayMs = _cfg.complianceDecayMs;
    const uint32_t elapsed = millis() - _complianceStartMs;

    if (decayMs == 0 || elapsed >= decayMs) {
        // Decay window elapsed (or decay disabled): event is over.
        _resetCompliance();
        return;
    }

    // Smoothstep recovery: ease = 1 at the event, 0 at the end of the window.
    const float t    = static_cast<float>(elapsed) / static_cast<float>(decayMs);
    const float ease = 1.0f - (t * t * (3.0f - 2.0f * t));
    _complianceOffsetMm       = -_cfg.complianceDepthMm * ease;
    _state.complianceOffsetMm = _complianceOffsetMm;
    _state.complianceActive   = true;
}

// ---------------------------------------------------------------------------
//  _resetHRestore — zero the deferred h-restore state + telemetry mirror.
//
//  Called on init, start (both IDLE and non-IDLE), and recovery entry. NOT
//  called at half-cycle boundaries: the restore is ARMED in _advanceHalf and
//  must survive the role swap to run across the next stance phase (mirrors
//  _resetCompliance, which is likewise never called at the boundary).
// ---------------------------------------------------------------------------
void GaitController::_resetHRestore() {
    _hRestoreActive         = false;
    _hRestoreSide           = _swingSide;
    _hRestoreDeficitMm      = 0.0f;
    _state.hRestoreActive   = false;
    _state.hRestoreOffsetMm = 0.0f;
}

// ---------------------------------------------------------------------------
//  _updateHRestore — per-tick h-restore cleanup + telemetry refresh.
//
//  When a restore is active the just-planted leg's h is held below
//  stanceHeightMm by a deficit that decays to zero over the new stance leg's
//  phase, gated on _halfPhase, completing at hRestoreCompleteFrac (foot ~under
//  the hip). This clears the restore once that phase is reached and refreshes
//  the telemetry mirror with the offset that WILL be applied this tick.
//
//  Phase-based, NOT time-based: the deficit must track the SAME _halfPhase
//  that _buildFootTarget feeds the restoring leg, so the telemetry and the
//  applied offset agree exactly. Runs once per update() beside
//  _updateCompliance(), ahead of the estop early-return so it stays phase-clean
//  on every path.
// ---------------------------------------------------------------------------
void GaitController::_updateHRestore() {
    if (!_hRestoreActive) {
        _state.hRestoreActive   = false;
        _state.hRestoreOffsetMm = 0.0f;
        return;
    }

    const float frac = constrain(_cfg.hRestoreCompleteFrac, 0.05f, 1.0f);

    // Restore completes once the new stance leg's phase reaches frac — the
    // foot is ~under the hip, so the deficit is fully recovered.
    if (_halfPhase >= frac) {
        Serial.printf("[GaitCtrl] H-restore DONE side=%s phase=%.3f (frac=%.2f)\n",
                      sideName(_hRestoreSide), _halfPhase, frac);
        _hRestoreActive         = false;
        _hRestoreDeficitMm      = 0.0f;
        _state.hRestoreActive   = false;
        _state.hRestoreOffsetMm = 0.0f;
        return;
    }

    // Telemetry mirror: the offset that WILL be applied to the restoring leg
    // this tick (<= 0), computed identically to the _buildFootTarget block.
    const float p    = constrain(_halfPhase / frac, 0.0f, 1.0f);
    const float ease = p * p * (3.0f - 2.0f * p);          // smoothstep
    const float restoreOffset = _hRestoreDeficitMm * (1.0f - ease);
    _state.hRestoreActive   = true;
    _state.hRestoreOffsetMm = -restoreOffset;
}

// ---------------------------------------------------------------------------
//  _resetTrimSuppression — zero the touchdown trim-suppression state.
//
//  Called on mode entry (start) and init. NOT called at half-cycle
//  boundaries: the post-latch hold window must be allowed to keep running
//  across _advanceHalf so the post-strike roll ring-down is still rejected.
// ---------------------------------------------------------------------------
void GaitController::_resetTrimSuppression() {
    _trimSuppressUntilMs      = 0;
    _trimSuppressActive       = false;
    _trimLastValidMm          = 0.0f;
    _state.trimSuppressActive = false;
}

// ---------------------------------------------------------------------------
//  _updateTrimSuppression — per-tick phase-gate of the W2/W5 lateral trim.
//
//  The foot-strike at touchdown injects a short torso-roll transient that is
//  not a real CoM excursion. This decides, once per tick, whether the lateral
//  trim should be suppressed so that transient never reaches _buildFootTarget.
//
//  Window: open while the touchdown latch is held, and for touchdownTrimHoldMs
//  after it clears (the deadline is refreshed every latched tick, so the hold
//  always measures from the LAST latched tick). millis() wrap is handled by
//  the signed-difference comparison.
//
//  Freeze snapshot: whenever suppression is NOT active, _trimLastValidMm is
//  refreshed with the current raw trim, so the value held during the next
//  suppression episode is the most recent clean (pre-touchdown) trim.
//
//  Must run once per update(), before the trim telemetry mirror and before
//  any _buildFootTarget() call — _comLateralTrimMm() only reads the result.
//  Reads _touchdownLocked as of the previous tick (the latch is set later in
//  update()); the one-tick lag is covered by the hold window and is the same
//  staleness contract as _publishSwingLegMaskToWS().
// ---------------------------------------------------------------------------
void GaitController::_updateTrimSuppression() {
    const uint32_t now = millis();

    // Refresh the post-latch deadline every tick the latch is held.
    if (_touchdownLocked) {
        _trimSuppressUntilMs = now + _cfg.touchdownTrimHoldMs;
    }

    // Window open while latched OR still inside the post-latch hold span.
    // The signed cast makes the comparison wrap-safe.
    const bool windowOpen = _touchdownLocked
                          || (int32_t)(_trimSuppressUntilMs - now) > 0;

    // Suppression is a no-op unless the trim is actually live this tick —
    // gating it on _comLatTrimActive() keeps the freeze snapshot meaningful.
    _trimSuppressActive = _cfg.touchdownTrimSuppressEnabled
                        && _comLatTrimActive()
                        && windowOpen;

    // Outside the window, keep the freeze snapshot tracking the clean trim.
    if (!_trimSuppressActive) {
        _trimLastValidMm = _comLateralTrimRawMm();
    }

    _state.trimSuppressActive = _trimSuppressActive;
}

// ---------------------------------------------------------------------------
void GaitController::init(MotionManager* mm, WeightShift* ws, BalanceController* bal) {
    if (mm == nullptr) {
        Serial.println("[GaitCtrl] ERROR: MotionManager null — update() will no-op.");
    }
    _mm               = mm;
    _ws               = ws;
    _bal              = bal;
    _state            = {};
    _swingSide        = Side::LEFT;
    _halfPhase        = 0.0f;
    _subState         = StepSubState::WAIT_SHIFT;
    _lastShiftDir     = ShiftDirection::NONE;
    _poseHoldPt       = 0;
    _stabilizeStartMs = 0;
    _stopStartMs      = 0;
    _stopSwingFrac    = 0.0f;
    _stopSwingSide    = Side::LEFT;
    _waitShiftStartMs = 0;
    _gateOkSinceMs    = 0;
    _lastFootfallMs   = 0;
    _swingStartMs     = 0;
    _touchdownMs      = 0;
    _stepRingHead     = 0;
    _stepRingCount    = 0;
    memset(_stepRingPeriodMs, 0, sizeof(_stepRingPeriodMs));
    memset(_stepRingSide,     0, sizeof(_stepRingSide));
    memset(_stepRingSwingMs,  0, sizeof(_stepRingSwingMs));
    _clearTouchdownLatch();
    _resetCompliance();
    _resetHRestore();
    _resetHeelStrike();
    _resetTrimSuppression();
    _resetRolloverTelemetry();
    _state.touchdownSide = 0;   // not cleared by _clearTouchdownLatch; explicit on init
    Serial.println("[GaitCtrl] Initialized.");
}

// ---------------------------------------------------------------------------
void GaitController::start(GaitMode mode) {
    // NOTE: lateral_shift_mm sign is no longer rejected here. Negative is the
    // intended default (adduction strategy). If a future gait phase requires
    // the translation-led convention (positive), gate it inside the relevant
    // mode handler rather than at start().
    _state.mode    = mode;
    _state.running = (mode != GaitMode::IDLE);

    if (mode == GaitMode::IDLE) {
        _state.phase      = 0.0f;
        _state.liftL_deg  = 0.0f;
        _state.liftR_deg  = 0.0f;
        _state.wsProgress = 0.0f;
        _state.liftGateOK = false;
        _state.poseHold   = 0;
        _swingSide        = Side::LEFT;
        _halfPhase        = 0.0f;
        _subState         = StepSubState::WAIT_SHIFT;
        _lastShiftDir     = ShiftDirection::NONE;
        _poseHoldPt       = 0;
        _stabilizeStartMs = 0;
        _waitShiftStartMs = 0;
        _gateOkSinceMs    = 0;
        _stopStartMs      = 0;
        _stopSwingFrac    = 0.0f;
        _stopSwingSide    = Side::LEFT;
        // Stage 1+: clear recovery state so stale telemetry is not broadcast.
        _rcvSub              = RecoverySubState::TRIGGERED;
        _rcvDirX             = 0;
        _rcvDirY             = 0;
        _rcvSubStartMs       = 0;
        _rcvSwingSide        = Side::LEFT;
        _state.recoverySub       = RecoverySubState::TRIGGERED;
        _state.recoveryDirX      = 0;
        _state.recoveryDirY      = 0;
        _state.recoveryElapsedMs = 0;
        _state.rcvStepX_mm   = 0.0f;
        _state.rcvStepY_mm   = 0.0f;
        _state.rcvMagnitude  = 0.0f;
        _state.cycleXDispl_mm = 0.0f;
        _state.fwdComProgressionMm = 0.0f;
        _state.stepPeriodMs    = 0;
        _state.waitMs          = 0;
        _state.swingMs         = 0;
        _state.stabilizeMsMeas = 0;
        _state.postContactMs   = 0;
        _state.lastStepSide    = 0;
        _state.cadenceHz       = 0.0f;
        _state.cadenceStepsMin = 0.0f;
        _state.meanStepPeriodMs = 0.0f;
        _state.meanCadenceHz    = 0.0f;
        _state.meanStepPeriodL  = 0.0f;
        _state.meanStepPeriodR  = 0.0f;
        _state.meanSwingMsL     = 0.0f;
        _state.meanSwingMsR     = 0.0f;
        _clearTouchdownLatch();
        _resetCompliance();
        _resetHRestore();   // symmetric with init() and the RUNNING branch — without
                            // this, _hRestoreActive/_hRestoreDeficitMm and their
                            // telemetry survive a gait stop (non-zero H-Restore panel).
        _resetHeelStrike();
        _resetTrimSuppression();
        _resetRolloverTelemetry();
        if (_ws) _ws->trigger(ShiftDirection::NONE);
        Serial.println("[GaitCtrl] IDLE — weight shift centered.");
        return;
    }

    // RUNNING, SINGLE_STEP, WSHIFT_ONLY, POSE_STEP all start from LEFT-swing
    // (was _currentHalf = 0 — same hardware-verified mapping, now type-keyed).
    _swingSide        = Side::LEFT;
    _halfPhase        = 0.0f;
    _subState         = StepSubState::WAIT_SHIFT;
    _lastShiftDir     = ShiftDirection::NONE;
    _poseHoldPt       = 0;
    _stabilizeStartMs = 0;
    _stopStartMs      = 0;
    _stopSwingFrac    = 0.0f;
    _stopSwingSide    = Side::LEFT;
    _state.poseHold   = 0;
    _waitShiftStartMs = millis();
    _gateOkSinceMs    = 0;
    _lastFootfallMs   = 0;
    _swingStartMs     = 0;
    _touchdownMs      = 0;
    _stepRingHead     = 0;
    _stepRingCount    = 0;
    _state.flatFootGuardCount = 0;   // per-session reach-guard counter
    _clearTouchdownLatch();
    _resetCompliance();
    _resetHRestore();
    _resetHeelStrike();
    _resetTrimSuppression();
    _resetRolloverTelemetry();
    _triggerShiftForSwing(Side::LEFT);  // begin LEFT-swing shift immediately

    Serial.printf("[GaitCtrl] start mode=%d\n", static_cast<int>(mode));
}

void GaitController::stop() {
    if (_state.mode == GaitMode::IDLE) return;
    if (_subState == StepSubState::STOPPING) return;

    _stopSwingFrac = (_subState == StepSubState::SWINGING)
                     ? _swingFrac(_halfPhase)
                     : 0.0f;
    _stopSwingSide  = _swingSide;
    _stopStartMs    = millis();
    _subState       = StepSubState::STOPPING;
    _state.subState = _subState;
    _poseHoldPt     = 0;
    _state.poseHold = 0;
    // Clear deferred swing-leg height restoration at STOPPING entry, not just at
    // IDLE entry. The applied offset is already faded by strideScale through the
    // ramp, but _hRestoreActive/_hRestoreDeficitMm (and the un-faded telemetry
    // mirror in _updateHRestore) otherwise survive the full GAIT_STOP_RAMP_MS
    // window — and indefinitely if WeightShift never settles to reach IDLE. This
    // makes the H-Restore panel read zero the instant Stop is pressed.
    _resetHRestore();
    Serial.printf("[GaitCtrl] STOP requested — ramping swing %.2f→0 over %ums\n",
                  _stopSwingFrac, (unsigned)GAIT_STOP_RAMP_MS);
}

void GaitController::submitStancePose(const BalanceCorrection* bc) {
    // One-shot idle-stance submit. _submitStanceBothLegs uses strideScale=0, so
    // the pose is the standing-ready stance (stride/capture/trim suppressed) with
    // the task-space stanceInset baseline applied — identical to what gait holds
    // when standing ready. Caller (WebComm STANCE) ensures gait is IDLE so this
    // is not immediately overwritten by an active gait submission.
    _submitStanceBothLegs(bc);
}

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
//  _triggerShiftForSwing  —  send the correct WeightShift direction for the
//  given swing side. Idempotent: skipped if direction has not changed.
//
//  This function is the SINGLE place in the codebase where the wire-protocol
//  inversion lives:
//    swing == LEFT  → CoM must go onto RIGHT leg → protocol command LEFT
//    swing == RIGHT → CoM must go onto LEFT  leg → protocol command RIGHT
//  (Yes, the names match — see weight_shift.h ShiftDirection convention block.)
// ---------------------------------------------------------------------------
void GaitController::_triggerShiftForSwing(Side swing) {
    if (!_ws) return;
    const ShiftDirection wantDir = (swing == Side::LEFT)
                                    ? ShiftDirection::LEFT
                                    : ShiftDirection::RIGHT;
    if (wantDir == _lastShiftDir) return;
    _lastShiftDir = wantDir;
    _ws->trigger(wantDir);
    Serial.printf("[GaitCtrl] swing=%s  trigger=%s  CoM→%s\n",
                  sideName(swing),
                  (wantDir == ShiftDirection::LEFT) ? "LEFT"  : "RIGHT",
                  (wantDir == ShiftDirection::LEFT) ? "right" : "left");
}

// ---------------------------------------------------------------------------
//  _swingFrac  —  raised-cosine vertical lift fraction, asymmetric-apex warp.
//  localPhase ∈ [0, 1) → [0, 1]: 0 at start, 1.0 at the apex, 0 at end.
//  Raised cosine (Hann window) gives zero vertical velocity at lift-off,
//  apex, and touchdown, reducing landing shock while preserving peak
//  ascent/descent amplitude.
//
//  W1 — asymmetric apex. localPhase is time-warped through a piecewise-linear
//  remap so the apex lands at swingApexFrac instead of 0.5. apex < 0.5 makes
//  the lift occupy a shorter time slice and the descent a longer one, so the
//  foot moves slower vertically through the touchdown approach. The cosine
//  still has zero derivative at warped {0, 0.5, 1}, so the zero-velocity
//  property at lift-off / apex / touchdown holds for every apex value.
//  apex == 0.5 reproduces the legacy symmetric raised cosine (warp = identity).
// ---------------------------------------------------------------------------
float GaitController::_swingFrac(float localPhase) const {
    const float a = constrain(_cfg.swingApexFrac, 0.05f, 0.95f);
    const float warped = (localPhase < a)
                       ? 0.5f * (localPhase / a)
                       : 0.5f + 0.5f * ((localPhase - a) / (1.0f - a));
    return 0.5f * (1.0f - cosf(warped * 2.0f * static_cast<float>(M_PI)));
}

// ---------------------------------------------------------------------------
//  _warpedStrideFrac — front-loaded horizontal stride smoothstep (swing leg).
//
//  See header for the full mechanism. SINGLE SOURCE OF TRUTH for the warp:
//  called from both _buildFootTarget (swing-leg stride) AND the touchdown
//  latch snapshot in update(). Duplicating the formula in either site would
//  inject a position jump at touchdown whenever strideCompleteFrac < 1.0.
//
//  At frac = 1.0 the formula reduces EXACTLY to the legacy unwarped
//  smoothstep s = lp*lp*(3-2*lp), so the default ships as a bit-identical
//  no-op until tuned down at runtime.
// ---------------------------------------------------------------------------
float GaitController::_warpedStrideFrac(float localPhase) const {
    const float frac = constrain(_cfg.strideCompleteFrac, 0.05f, 1.0f);
    const float lp   = constrain(localPhase, 0.0f, 1.0f);
    const float lpw  = (lp >= frac) ? 1.0f : (lp / frac);
    return lpw * lpw * (3.0f - 2.0f * lpw);
}

// ---------------------------------------------------------------------------
//  _updateRollRateFilter — W5: advance the roll-rate LPF by one tick.
//
//  Single-pole IIR:  filt = alpha*filt + (1-alpha)*rollRate.
//  alpha is clamped to [0, 0.99]: 0 = pass-through, 0.99 = near-frozen. The
//  filter is zeroed on an invalid IMU so a stale rate from before a dropout
//  cannot re-enter the Kd term once the IMU comes back.
//
//  Must run exactly once per update() — see header. The const trim helpers
//  only READ _rollRateFilt; stepping it here keeps that contract intact.
// ---------------------------------------------------------------------------
void GaitController::_updateRollRateFilter(const IMUState& imuState) {
    if (!imuState.valid) { _rollRateFilt = 0.0f; return; }
    const float alpha = constrain(_cfg.comLatTrimRateLpfAlpha, 0.0f, 0.99f);
    _rollRateFilt = alpha * _rollRateFilt + (1.0f - alpha) * imuState.rollRate;
}

// ---------------------------------------------------------------------------
//  _comLatTrimActive — W5: shared gate predicate for the lateral CoM trim.
//  Trim is a gait foot-placement correction — meaningless when no gait is
//  running, so IDLE returns false (keeps the idle pose clean). The STOPPING
//  ramp (mode still RUNNING here) is faded out separately via strideScale in
//  _buildFootTarget().
// ---------------------------------------------------------------------------
bool GaitController::_comLatTrimActive() const {
    return _cfg.comLatTrimEnabled
        && _state.mode != GaitMode::IDLE
        && _lastImuState.valid
        && !oe_is_estopped();
}

// ---------------------------------------------------------------------------
//  _comLatVelTrimMm — W5: velocity (Kd) contribution alone, pre-clamp.
//  Telemetry only. comLatVel = H_COM_MM * rollRate_filt is the lateral CoM
//  velocity (mm/s); times comLatTrimKd (s) gives mm of foot trim.
// ---------------------------------------------------------------------------
float GaitController::_comLatVelTrimMm() const {
    if (!_comLatTrimActive()) return 0.0f;
    const float comLatVel = H_COM_MM * _rollRateFilt;   // mm/s
    return comLatVel * _cfg.comLatTrimKd;
}

// ---------------------------------------------------------------------------
//  _comLateralTrimMm — W2+W5 continuous lateral CoM foot-placement trim.
//  See header for the sign convention and gating rationale.
//
//  W2 (displacement / Kp):  comLatDisp = H_COM_MM * (roll - rollNeutral)
//  W5 (velocity     / Kd):  comLatVel  = H_COM_MM * rollRate_filt
//      trim = comLatDisp*comLatTrimGain + comLatVel*comLatTrimKd, then clamped.
//  comLatTrimKd defaults to 0, so by default this is exactly the W2 trim.
// ---------------------------------------------------------------------------
float GaitController::_comLateralTrimRawMm() const {
    if (!_comLatTrimActive()) return 0.0f;

    const float rollCorrected = _lastImuState.roll - _cfg.torsoRollNeutralRad;
    const float comLatDisp = H_COM_MM * rollCorrected;   // mm  (sin x ≈ x)
    const float comLatVel  = H_COM_MM * _rollRateFilt;   // mm/s

    const float dispTrim = comLatDisp * _cfg.comLatTrimGain;  // W2 — Kp term
    const float velTrim  = comLatVel  * _cfg.comLatTrimKd;    // W5 — Kd term
    const float trim     = -(dispTrim + velTrim);

    return constrain(trim, -_cfg.comLatTrimMaxMm, +_cfg.comLatTrimMaxMm);
}

float GaitController::_comLateralTrimMm() const {
    if (!_comLatTrimActive()) return 0.0f;

    // Touchdown trim suppression. While the suppression window is open (set
    // once per tick by _updateTrimSuppression), reject the live roll signal —
    // it carries the foot-strike transient — and instead either hold the last
    // clean trim (freeze mode) or contribute nothing (zero mode).
    if (_trimSuppressActive) {
        return _cfg.touchdownTrimFreezeHold ? _trimLastValidMm : 0.0f;
    }

    return _comLateralTrimRawMm();
}

// ---------------------------------------------------------------------------
//  _captureOffsetMm — sagittal capture-point step-placement correction.
//
//  Returns the mm offset to add to the swing leg's stride. Zero when
//  capture is disabled, IMU invalid, or estopped. Sign convention:
//    vx_CoM > 0 (body advancing forward) → positive offset
//                                          (foot lands further forward)
//
//  Capping logic (composite, conservative):
//    raw      = vx_CoM × INV_OMEGA0 × captureGain
//    by_field = clamp(raw, ±captureOffsetMaxMm)
//    by_step  = clamp(by_field, ±stepLengthMm)
//
//  by_step is the second cap: the capture offset alone never exceeds the
//  open-loop nominal stride. Without this, a noisy IMU spike could double
//  the stride in one tick.
//
//  IK reach is checked separately at submit time via the existing
//  IK status path — this function is intentionally pure-math.
// ---------------------------------------------------------------------------
float GaitController::_captureOffsetMm() const {
    if (!_cfg.captureEnabled)        return 0.0f;
    if (!_lastImuState.valid)        return 0.0f;
    if (oe_is_estopped())            return 0.0f;
    if (_cfg.stepLengthMm <= 0.01f)  return 0.0f;  // capture meaningless without nominal stride

    const float raw = _lastImuState.vx_CoM_mm_s
                    * GAIT_CAPTURE_INV_OMEGA0_S
                    * _cfg.captureGain;

    const float byField = constrain(raw,
                                    -_cfg.captureOffsetMaxMm,
                                    +_cfg.captureOffsetMaxMm);
    const float byStep  = constrain(byField,
                                    -_cfg.stepLengthMm,
                                    +_cfg.stepLengthMm);
    return byStep;
}

// ---------------------------------------------------------------------------
//  _pitchPlacementOffsetMm — pitch-ANGLE swing-foot forward placement.
//
//  See header for the full mechanism + sign convention. DISTINCT from the
//  rate-based _captureOffsetMm() above: driven by the clean torso pitch ANGLE
//  (pitch - torsoPitchNeutralRad), NOT CoM velocity, and shares no state with
//  it — only its coding/gating/smoothing style. Pure-math; reads _lastImuState
//  / _cfg only. No _touchdownLocked freeze branch (the term is folded into
//  _touchdownLockedX_mm, so the x-freeze in _buildFootTarget covers the latched
//  case — mirrors _captureOffsetMm).
//
//  Default-path no-op gates — gain == 0 (the shipped default) short-circuits to
//  0 so placeRaw and the latch snapshot become "+ 0.0f", a bit-identical no-op.
// ---------------------------------------------------------------------------
float GaitController::_pitchPlacementOffsetMm(float localPhase) const {
    if (!_cfg.pitchPlacementEnabled)             return 0.0f;
    if (_cfg.pitchPlacementGainMmPerRad == 0.0f) return 0.0f;
    if (!_lastImuState.valid)                    return 0.0f;
    if (oe_is_estopped())                        return 0.0f;
    if (_state.mode == GaitMode::IDLE)           return 0.0f;
    if (_cfg.stepLengthMm <= 0.01f)              return 0.0f;  // placement meaningless without nominal stride

    // Clean (neutral-corrected) torso pitch. Positive = leaning forward
    // (state_estimator.h). torsoPitchNeutralRad is set by tareToTorso().
    const float pitch = _lastImuState.pitch - _cfg.torsoPitchNeutralRad;

    // Deadband: reject sensor noise and small standing tilt before any placement.
    const float db = _cfg.pitchPlacementDeadbandRad;
    const float effPitch = (pitch >  db) ? (pitch - db)
                         : (pitch < -db) ? (pitch + db)
                                         : 0.0f;

    // raw = effPitch * gain * sign  → positive forward pitch with sign=+1 yields
    // positive (forward) foot placement, matching the FootTarget.x_mm convention.
    const float raw     = effPitch
                        * _cfg.pitchPlacementGainMmPerRad
                        * _cfg.pitchPlacementSign;
    const float clamped = constrain(raw,
                                    -_cfg.pitchPlacementMaxMm,
                                    +_cfg.pitchPlacementMaxMm);

    // Smoothstep ramp on localPhase — identical shape to the swing stride and
    // the sagittal/lateral capture terms (s = lp²(3 - 2lp)). Zero foot-placement
    // velocity at lift-off and full placement by touchdown.
    const float lp = constrain(localPhase, 0.0f, 1.0f);
    const float s  = lp * lp * (3.0f - 2.0f * lp);
    return clamped * s;
}

// ---------------------------------------------------------------------------
//  _latCaptureOffsetMm — frontal-plane capture-point swing-foot placement.
//  See header for the full mechanism + sign convention. Pure-math; reads
//  _rollRateFilt (advanced once per tick by _updateRollRateFilter() — same
//  W5 filter — never stepped twice), and _touchdownLockedY_mm for freeze.
//  Per-leg sign and strideScale are applied at the call site in
//  _buildFootTarget so this helper stays world-frame and shape-only.
// ---------------------------------------------------------------------------
float GaitController::_latCaptureOffsetMm(float localPhase) const {
    // Freeze first: once latched, return the snapshot verbatim and do not
    // re-touch the live roll signal — keeps the lateral contribution from
    // re-introducing stance-style body-shove on a planted foot. The snapshot
    // is itself bounded by the same cap, so no extra clamp needed.
    if (_touchdownLocked) return _touchdownLockedY_mm;

    // Default-path no-op gates — kept identical in spirit to _captureOffsetMm
    // so a flipped enable produces the smallest possible code-path delta.
    if (!_cfg.latCaptureEnabled)        return 0.0f;
    if (!_lastImuState.valid)           return 0.0f;
    if (oe_is_estopped())               return 0.0f;
    if (_state.mode == GaitMode::IDLE)  return 0.0f;

    // Lateral LIP omega0 == sagittal omega0 (same CoM height H_COM_MM), so
    // GAIT_CAPTURE_INV_OMEGA0_S is reused verbatim — see CAPTURE-POINT block
    // in project_wide_defs.h for the shared derivation.
    const float vy_CoM_mm_s = H_COM_MM * _rollRateFilt;   // mm/s (LIP)
    const float raw         = vy_CoM_mm_s
                            * GAIT_CAPTURE_INV_OMEGA0_S
                            * _cfg.latCaptureGain
                            * _cfg.latCaptureSign;

    const float clamped = constrain(raw,
                                    -_cfg.latCaptureMaxMm,
                                    +_cfg.latCaptureMaxMm);

    // Smoothstep ramp on localPhase — identical shape to the sagittal stride
    // in _buildFootTarget (s = lp²(3 - 2lp)). Zero at lift-off, full at
    // touchdown → zero lateral foot velocity at contact.
    const float lp = constrain(localPhase, 0.0f, 1.0f);
    const float s  = lp * lp * (3.0f - 2.0f * lp);
    return clamped * s;
}

// ---------------------------------------------------------------------------
//  _fwdComProgressionMm — swing-phase-timed forward CoM body offset.
//
//  See header for the full mechanism + composition notes. Returns the
//  non-negative mm magnitude; the per-leg sign flip is performed at the
//  call site in _buildFootTarget (NEGATED before adding to x_mm on BOTH
//  legs, matching WeightShift::getStanceShift's forward_lean convention).
//
//  Default-OFF early-outs (truly bit-identical to the pre-change path):
//    fwdComMaxMm <= 0       → 0   (knob at default; term inert)
//    mode == IDLE           → 0   (term is a gait shaping aid, not a posture
//                                   bias; never layered on the idle pose)
//    estopped               → 0
//
//  Note: this helper is intentionally NOT gated on _touchdownLocked. The
//  latch freezes only the swing leg's STRIDE x contribution; this term is
//  a both-legs body offset and keeps ramping on localPhase right up to
//  _advanceHalf. Stance retreat composes the same way (its formula is
//  evaluated on _halfPhase regardless of the latch).
// ---------------------------------------------------------------------------
float GaitController::_fwdComProgressionMm(float localPhase) const {
    if (_cfg.fwdComMaxMm <= 0.0f)       return 0.0f;
    if (_state.mode == GaitMode::IDLE)  return 0.0f;
    if (oe_is_estopped())               return 0.0f;

    const float lead = constrain(_cfg.fwdComLeadFrac, 0.05f, 1.0f);
    const float lp   = constrain(localPhase, 0.0f, 1.0f);
    const float lpw  = (lp >= lead) ? 1.0f : (lp / lead);
    const float s    = lpw * lpw * (3.0f - 2.0f * lpw);
    return _cfg.fwdComMaxMm * s;
}

// ---------------------------------------------------------------------------
//  _heelStrikeEngageFrac — phase-based engage fraction in [0..1].
//
//  Smoothstep ramp on [g, td] where g = heelStrikeGatePhase and td =
//  touchdownGatePhase. HOLDS at 1.0 for lp >= td so the bias has already
//  reached peak by the time the touchdown latch can fire (which by design
//  cannot happen before _halfPhase > touchdownGatePhase). The hold is the
//  whole point of the redesign — the previous version spread the smoothstep
//  across [g, 1.0] and only reached frac≈0.26 at the earliest touchdown.
//
//  The g < td invariant is enforced by the WS parser (heelGate is clamped
//  above), but this guard is still defensive — a future config path that
//  bypasses the parser must not divide by zero or run a degenerate window.
// ---------------------------------------------------------------------------
float GaitController::_heelStrikeEngageFrac(float lp) const {
    const float g_raw  = _cfg.heelStrikeGatePhase;
    const float td_raw = _cfg.touchdownGatePhase;
    // Defensive clamp: keep at least 0.01 of window width between g and td.
    const float g  = constrain(g_raw, 0.05f, 0.95f);
    const float td = constrain(td_raw, g + 0.01f, 1.0f);
    const float lpc = constrain(lp, 0.0f, 1.0f);
    if (lpc <= g)  return 0.0f;
    if (lpc >= td) return 1.0f;             // <- HOLD at peak until contact
    const float r = (lpc - g) / (td - g);
    return r * r * (3.0f - 2.0f * r);       // smoothstep
}

// ---------------------------------------------------------------------------
//  _startHeelStrikeRelease — snapshot peak frac, lock active side, start timer.
//
//  Called at the touchdown latch fire site and (as a fallback) at the top of
//  _advanceHalf() when no touchdown was detected during the swing. The peak
//  snapshot is the live engage fraction at this exact tick — so the bias is
//  bit-continuous across the engage→release transition (no jump). Re-entry
//  is a no-op: if a release is already in progress, do not overwrite the
//  active-side or restart the timer.
// ---------------------------------------------------------------------------
void GaitController::_startHeelStrikeRelease() {
    if (!_cfg.heelStrikeEnabled)         return;
    if (_heelStrikeReleaseStartMs != 0)  return;   // already releasing
    _heelStrikePeakFrac       = _heelStrikeEngageFrac(_halfPhase);
    _heelStrikeActiveSide     = _swingSide;
    _heelStrikeReleaseStartMs = millis();
    Serial.printf("[GaitCtrl] HEEL release start  side=%s  peakFrac=%.3f  "
                  "lp=%.3f\n",
                  sideName(_heelStrikeActiveSide),
                  _heelStrikePeakFrac,
                  _halfPhase);
}

// ---------------------------------------------------------------------------
//  _updateHeelStrike — per-tick telemetry refresh + release-complete cleanup.
//
//  Modelled on _updateCompliance(). Runs once per update() before any
//  consumer reads. Refreshes the public telemetry mirror so the UI sees the
//  live bias regardless of which leg currently carries it (swing during
//  ENGAGE; the just-planted leg during RELEASE, which becomes the stance leg
//  after _advanceHalf). Clears the release state when its window elapses.
// ---------------------------------------------------------------------------
void GaitController::_updateHeelStrike() {
    // Step 1 — clear release state if the window has elapsed.
    if (_heelStrikeReleaseStartMs != 0) {
        const uint32_t relMs   = _cfg.heelStrikeReleaseMs;
        const uint32_t elapsed = millis() - _heelStrikeReleaseStartMs;
        if (relMs == 0 || elapsed >= relMs) {
            _heelStrikeReleaseStartMs = 0;
            _heelStrikePeakFrac       = 0.0f;
            Serial.println("[GaitCtrl] HEEL release complete.");
        }
    }

    // Step 2 — refresh telemetry mirror. During RELEASE the bias is on the
    // active (just-planted) leg; during ENGAGE the bias is on the swing leg.
    if (_heelStrikeReleaseStartMs != 0) {
        _state.heelStrikeActiveSide    = static_cast<uint8_t>(_heelStrikeActiveSide);
        _state.heelStrikeBiasDeg       = _heelStrikeBiasDeg(_heelStrikeActiveSide);
        _state.heelStrikeReleaseActive = true;
    } else {
        _state.heelStrikeActiveSide    = static_cast<uint8_t>(_swingSide);
        _state.heelStrikeBiasDeg       = _heelStrikeBiasDeg(_swingSide);
        _state.heelStrikeReleaseActive = false;
    }
}

// ---------------------------------------------------------------------------
//  _resetHeelStrike — zero all heel-strike state + telemetry.
//
//  Called on init, start (both IDLE and non-IDLE), and recovery entry. NOT
//  called from _advanceHalf — an in-progress release MUST be allowed to
//  finish smoothly across the role swap.
// ---------------------------------------------------------------------------
void GaitController::_resetHeelStrike() {
    _heelStrikeReleaseStartMs       = 0;
    _heelStrikePeakFrac             = 0.0f;
    _heelStrikeActiveSide           = _swingSide;
    _state.heelStrikeBiasDeg        = 0.0f;
    _state.heelStrikeReleaseActive  = false;
    _state.heelStrikeActiveSide     = static_cast<uint8_t>(_swingSide);
}

// ---------------------------------------------------------------------------
//  _heelStrikeBiasDeg — additive ankle-pitch bias for the requested side.
//
//  Sign is intrinsically NEGATIVE (dorsiflex). Two branches, selected by
//  whether a timer-based release is in progress — NOT by the touchdown
//  latch state. The release branch is the source of truth across the role
//  swap; the engage branch only applies when no release is active.
// ---------------------------------------------------------------------------
float GaitController::_heelStrikeBiasDeg(Side side) const {
    if (!_cfg.heelStrikeEnabled) return 0.0f;
    if (oe_is_estopped())        return 0.0f;

    const float peak = _cfg.heelStrikeDorsiflexDeg;
    const float maxB = _cfg.heelStrikeMaxBiasDeg;
    float frac = 0.0f;

    if (_heelStrikeReleaseStartMs != 0) {
        // RELEASE — timer-based, side-locked. Survives the role swap because
        // _heelStrikeActiveSide is frozen at the moment the release started.
        if (side != _heelStrikeActiveSide) return 0.0f;
        const uint32_t relMs = _cfg.heelStrikeReleaseMs;
        if (relMs == 0) return 0.0f;
        const float t    = constrain(
            (float)(millis() - _heelStrikeReleaseStartMs) / (float)relMs,
            0.0f, 1.0f);
        const float ease = 1.0f - (t * t * (3.0f - 2.0f * t));  // smoothstep
        frac = _heelStrikePeakFrac * ease;
    } else {
        // ENGAGE — phase-based, only on the live swing leg. The IDLE guard
        // is on this branch only (a release that began mid-walk and got
        // interrupted by stop() is cleared via _resetHeelStrike anyway).
        if (_state.mode == GaitMode::IDLE) return 0.0f;
        if (side != _swingSide)            return 0.0f;
        frac = _heelStrikeEngageFrac(_halfPhase);
    }

    // Sign baked in: bias is intrinsically dorsiflex (negative ankle_pitch).
    return constrain(-peak * frac, -maxB, maxB);
}

// ---------------------------------------------------------------------------
//  _buildFootTarget — Stage 1: vertical lift only.
//  Stance leg returns the neutral pose (x=0, y=0, h=stanceHeight).
//  Swing  leg returns same x,y with h reduced by swingFrac × stepHeight.
//  No lateral or fore/aft motion in Stage 1 — those land in Stage 2.
// ---------------------------------------------------------------------------
FootTarget GaitController::_buildFootTarget(bool isRight,
                                            float swingFrac,
                                            bool isSwing,
                                            float localPhase,
                                            const BalanceCorrection* bc,
                                            float strideScale) const {
    FootTarget t;
    t.isRightLeg   = isRight;
    t.x_mm         = GAIT_STANCE_X_OFFSET_MM;
    // Stance-width inset baseline. FootTarget.y_mm is per-leg own-frame with
    // inward(adduction) negative (same convention as the W2 comLatTrim block
    // below), so an inset of +N becomes -N on BOTH legs → symmetric adduction.
    // UNSCALED (not gated by strideScale) — like the x baseline above — so it
    // holds in IDLE/standing-stance submits (strideScale=0) and composes
    // additively with WS / comLatTrim / latCapture / balance during gait. The
    // LegIK frontal solve enforces flat-foot (ankle_roll=-hip_roll) for any y.
    t.y_mm         = -_cfg.stanceInsetMm;
    t.h_frontal_mm = 0.0f;

    // Touchdown-awareness lift freeze (Stage 1).
    //
    // When the latch is held on the swing leg, h_sagittal is frozen at the
    // value snapshotted at the moment of detection. Continuing to drive h
    // toward stanceHeightMm (lift → 0) after ground contact would command
    // foot penetration, creating a downward reaction force that shoves the
    // body backward — the exact failure mode this subsystem prevents.
    //
    // Normal trajectory runs for all other cases (latch not set, or stance leg).
    if (isSwing && _touchdownLocked) {
        // X stays frozen (applied below as locked*strideScale).
        // Freeze h at the touchdown snapshot, expressed as a deficit below
        // stanceHeight and scaled by strideScale so it fades EXACTLY like the
        // x-freeze (t.x_mm += _touchdownLockedX_mm * strideScale) and every
        // other gait-transient offset:
        //   strideScale == 1 (active gait): h == _touchdownLockedH_mm — the foot
        //       is held at the touchdown snapshot; re-extending a planted foot
        //       mid-swing would brake the body backward. In-gait re-extension is
        //       deferred to the next stance phase by the h-restore block below.
        //   strideScale 1→0 (STOP ramp): h eases smoothly back to stanceHeight,
        //       so gait termination produces no h discontinuity on EITHER leg.
        //   strideScale == 0 (IDLE / standing-stance submit): freeze contributes
        //       nothing — the idle pose can never inherit a frozen height.
        const float freezeDeficit = _cfg.stanceHeightMm - _touchdownLockedH_mm;
        t.h_sagittal_mm = _cfg.stanceHeightMm - freezeDeficit * strideScale;
    } else {
        const float liftMm = isSwing ? (swingFrac * _cfg.stepHeightMm) : 0.0f;
        t.h_sagittal_mm = _cfg.stanceHeightMm - liftMm;
    }

    // W4 — touchdown compliance: a brief negative stance-height offset that
    // shortens the leg after an early-contact event, then decays smoothly
    // back to zero (see _updateCompliance()). Applied to BOTH swing and
    // stance legs so the body yields evenly downward — a small symmetric
    // squat — with no lateral asymmetry. _complianceOffsetMm is <= 0 and is
    // exactly 0 whenever no compliance event is active, so this line is a
    // no-op on every normal tick.
    t.h_sagittal_mm += _complianceOffsetMm;

    // H-restore — deferred swing-leg stance-height restoration. The leg that
    // latched a touchdown last swing had its h frozen below stanceHeightMm;
    // _advanceHalf armed a restore on it, and it is now the stance leg. Re-
    // extend it back to full stance height across this stance phase, gated on
    // localPhase, so the planted foot eases down to nominal instead of snapping
    // at the role swap (the snap produced a backward braking impulse). Applied
    // to the restoring leg only — the new swing leg runs its normal trajectory.
    // At localPhase 0 the offset equals the full deficit, so the commanded h is
    // exactly _touchdownLockedH_mm (continuous with the frozen value at the end
    // of the previous half); at localPhase >= frac the offset is 0.
    //
    // strideScale gates the offset exactly like the freeze above and like
    // stride / capture / comLatTrim: full in active gait (strideScale==1), faded
    // through the STOP ramp (1→0), and ZERO in IDLE / standing-stance submits
    // (strideScale==0). The strideScale==0 case is what makes the idle pose
    // structurally immune to a lingering restore — the offset cannot reach the
    // feet when no gait cycle is running, independent of _hRestoreActive.
    if (_hRestoreActive && _hRestoreSide == (isRight ? Side::RIGHT : Side::LEFT)) {
        const float frac = constrain(_cfg.hRestoreCompleteFrac, 0.05f, 1.0f);
        const float p    = constrain(localPhase / frac, 0.0f, 1.0f);
        const float ease = p * p * (3.0f - 2.0f * p);          // smoothstep
        const float restoreOffset = _hRestoreDeficitMm * (1.0f - ease) * strideScale;
        t.h_sagittal_mm -= restoreOffset;   // phase 0: -deficit (==frozen); phase>=frac: 0
    }

    // Stage 3: smoothstep stride applied to BOTH legs.
    //   Swing  leg: x = (s - 0.5) × L + captureReal  → starts at -L/2, lands at +L/2
    //   Stance leg: x = (0.5 - s) × L           → starts at +L/2, retreats to -L/2
    // Smoothstep on localPhase gives zero velocity at the role-swap boundary
    // for both legs, eliminating the x-velocity discontinuity there.
    //
    // Touchdown-awareness stride override (Stage 1):
    //   When the latch is held AND this is the swing leg, replace the
    //   stride contribution with the snapshot captured at lock time. The
    //   STANCE leg's formula is left untouched so its retreat continues
    //   to advance the body forward over the (now planted) swing foot.
    //   This is the central architectural decision of Stage 1: freeze
    //   the SWING-leg progression only, never the half-cycle phase or
    //   the stance leg.
    if (_cfg.stepLengthMm > 0.0f && strideScale > 0.0f) {
        if (isSwing && _touchdownLocked) {
            t.x_mm += _touchdownLockedX_mm * strideScale;
        } else {
            // Stance retreat uses the RAW (unwarped) smoothstep — see
            // _warpedStrideFrac() and the scope note in project_wide_defs.h:
            // stance drives body forward translation; leaving its timing alone
            // keeps the stride-frontload experiment isolated from a separate
            // "shift CoM forward earlier" change.
            //
            // captureRaw is added on top of the warped swing stride exactly as
            // before. NOTE: capture+warp interaction is unreviewed and should
            // be revisited if capture is re-enabled while strideCompleteFrac<1.
            const float sRaw       = localPhase * localPhase * (3.0f - 2.0f * localPhase);
            const float captureRaw = isSwing ? _captureOffsetMm() : 0.0f;
            // Pitch-ANGLE forward placement — SWING leg only, additive on top of
            // the warped stride + capture, exactly like captureRaw. Default gain
            // 0 makes _pitchPlacementOffsetMm() return 0, so placeRaw is "+ 0.0f"
            // here — a bit-identical no-op. The touchdown latch snapshot in
            // update() uses the SAME helper so commanded x and latched x cannot
            // diverge (no touchdown x discontinuity).
            const float placeRaw   = isSwing ? _pitchPlacementOffsetMm(localPhase) : 0.0f;
            const float strideRaw  = isSwing
                ? (_warpedStrideFrac(localPhase) - 0.5f) * _cfg.stepLengthMm + captureRaw + placeRaw
                : (0.5f - sRaw) * _cfg.stepLengthMm;
            t.x_mm += strideRaw * strideScale;
        }
    }

    // Stage 2: WeightShift task-space contribution (forward lean + lateral shift).
    if (_ws != nullptr) {
        float wsX_mm = 0.0f, wsY_mm = 0.0f;
        _ws->getStanceShift(isRight, wsX_mm, wsY_mm);
        t.x_mm += wsX_mm;
        t.y_mm += wsY_mm;
    }

    // Forward CoM progression — swing-phase-timed body offset (transfer-
    // shaping, body-side complement of change A's strideCompleteFrac warp).
    // Applied symmetrically to BOTH legs (this is a body offset, not foot
    // placement). Sign MATCHES WeightShift::getStanceShift's forward_lean
    // (x_offset = -fwd_mm: feet sit further behind the hips when the body
    // has shifted forward), so we SUBTRACT the helper's non-negative magnitude
    // from t.x_mm. A wrong sign would pull the body backward and worsen the
    // late-arrival failure mode.
    //
    // strideScale fades the term through STOPPING and is 0 on IDLE / standing-
    // stance submits (_submitStanceBothLegs uses strideScale=0), exactly like
    // stride / capture / comLatTrim above. Default fwdComMaxMm = 0 makes
    // _fwdComProgressionMm() return 0 unconditionally, so on the default-OFF
    // path this line resolves to t.x_mm += 0.0 — bit-identical to pre-change.
    //
    // DOUBLE-COUNT NOTE: the existing WS forward_lean above is ALSO a forward
    // body offset (driven by |wsProgress|). Both terms stack additively. To
    // isolate the new behaviour during a sweep, set WeightShiftConfig::
    // forward_lean_mm to 0 while tuning fwdComMaxMm up from 0. The stance-
    // leg retreat (computed above in the stride block) is the THIRD source of
    // forward body translation; that one must stay untouched (it is what
    // physically advances the body across the step).
    //
    // LATCH COMPOSITION: independent of _touchdownLocked. The latch only
    // freezes the swing leg's stride x contribution (_touchdownLockedX_mm).
    // This term ramps on localPhase for both legs equally, so it never
    // double-counts with the latch and never fights the swing-x freeze.
    const float fwdComMagMm  = _fwdComProgressionMm(localPhase);
    const float fwdComBoth_mm = -fwdComMagMm * strideScale;
    t.x_mm += fwdComBoth_mm;

    // W2: continuous lateral CoM trim. UVC-style — the observed lateral CoM
    // displacement is fed into BOTH feet's y_mm every tick so the support
    // polygon tracks under the CoM. Sign mirrors mergeBalanceCorrection's dy
    // convention: CoM displaced to robot-right → right foot abducts (+y),
    // left foot adducts (-y) → body translates right over the feet.
    // Applied to swing and stance alike (UVC adds its dyi to the swing foot
    // target too); the touchdown latch freezes only x and h, never y.
    //
    // Scaled by strideScale — identical to the forward stride and capture
    // offset above. strideScale is 1.0 in active gait, ramps to 0 through
    // STOPPING, and is 0 in IDLE / standing-stance submits. Without this
    // factor the trim would survive as a static lateral offset layered over
    // the idle pose after STOP (the W2 regression this fixes).
    const float comLatTrim = _comLateralTrimMm() * strideScale;
    t.y_mm += isRight ? +comLatTrim : -comLatTrim;

    // Lateral (frontal-plane) capture-point swing-foot placement. Parallel,
    // additive A/B candidate to the W2 trim above + roll-PD merge below.
    // Applied to the SWING leg's landing y ONLY — stance leg untouched (that
    // is the entire point: first true swing-foot placement authority during
    // gait). Per-leg sign mirrors the comLatTrim convention (isRight ? + : -)
    // so body falling toward a side → swing foot placed toward that side
    // (capture). Sign flip exposed at runtime via cfg.latCaptureSign.
    // strideScale fades the term through STOPPING and is 0 in IDLE / standing
    // stance submits, exactly like stride / comLatTrim / sagittal capture.
    // _latCaptureOffsetMm() early-returns 0 when disabled, IDLE, IMU invalid,
    // or estopped, so this line is a true no-op on the default-OFF path.
    if (isSwing) {
        const float latCapture = _latCaptureOffsetMm(localPhase) * strideScale;
        t.y_mm += isRight ? +latCapture : -latCapture;
    }

    // Stage 3: balance correction merges onto stance leg only.
    if (bc != nullptr && !isSwing) {
        mergeBalanceCorrection(t, *bc);
    }

    t.valid = (t.h_sagittal_mm >= 20.0f);
    return t;
}

// ---------------------------------------------------------------------------
//  Ankle-roll composition (consolidation refactor + regression fix). One owner,
//  one inspectable summation in _ankleRollDeg, called from _submitFootTargetIK.
//
//  Grounded legs crossfade flat-foot (sole flat) ↔ CoM-shift tilt by the shift
//  authority. The airborne swing leg holds the inward CoM-shift tilt at lift-off
//  and transitions outward toward its future-stance orientation over the
//  descent (swing-prep). See header + GAIT_SWING_PREP_* in project_wide_defs.h.
// ---------------------------------------------------------------------------

// True only for the active normal-gait swing leg (recovery excluded — recovery
// legs use the grounded crossfade path).
bool GaitController::_isNormalSwing(Side side) const {
    return _state.mode != GaitMode::RECOVERY_STEP
        && _subState == StepSubState::SWINGING
        && side == _swingSide;
}

// CoM-shift ankle-roll tilt for `side`, joint-command frame. Pure function of
// WeightShift progress: reproduces the pre-refactor bias exactly
// (right: -rightProgress·amp, left: +leftProgress·amp). This is the deliberate
// inward tilt that acts as the lateral CoM-shift actuator.
float GaitController::_comShiftTilt(Side side) const {
    if (_ws == nullptr) return 0.0f;
    const float amp = _ws->getConfig().ankle_shift_deg;
    if (fabsf(amp) <= 0.01f) return 0.0f;
    const WeightShiftState& s = _ws->getState();
    const float prog    = (side == Side::RIGHT) ? s.right_progress : s.left_progress;
    const float legSign = (side == Side::RIGHT) ? -1.0f : +1.0f;
    return legSign * prog * amp;
}

// Shift authority a∈[0,1] for the grounded crossfade: 0 when the shift is
// centered (sole flat, flat-foot wins), 1 at full shift (sole = comShift tilt).
float GaitController::_shiftAuthority(Side side) const {
    if (_ws == nullptr) return 0.0f;
    const WeightShiftState& s = _ws->getState();
    const float prog = (side == Side::RIGHT) ? s.right_progress : s.left_progress;
    return constrain(fabsf(prog), 0.0f, 1.0f);
}

// The CoM-shift tilt `side` will carry as the NEXT stance foot. The next
// half-cycle swaps the swing leg and selects the opposite ShiftDirection (see
// _triggerShiftForSwing), so the next-stance progress target is the opposite of
// the current swing-half value — i.e. opposite sign to _comShiftTilt's progress.
float GaitController::_futureStanceTilt(Side side) const {
    if (_ws == nullptr) return 0.0f;
    const float amp = _ws->getConfig().ankle_shift_deg;
    if (fabsf(amp) <= 0.01f) return 0.0f;
    const float legSign          = (side == Side::RIGHT) ? -1.0f : +1.0f;
    const float futureProgTarget = (_swingSide == Side::LEFT) ? -1.0f : +1.0f;
    return legSign * futureProgTarget * amp;
}

// Swing-prep envelope: 0 through lift-off and the rise, then monotonic 0→1
// across the descent (apex→touchdown). swingPrepCompleteFrac<1 completes the
// outward rotation earlier in the descent.
float GaitController::_swingPrepEnv() const {
    const float apex    = constrain(_cfg.swingApexFrac, 0.05f, 0.95f);
    const float descent = constrain((_halfPhase - apex) / (1.0f - apex), 0.0f, 1.0f);
    const float frac    = constrain(_cfg.swingPrepCompleteFrac, 0.05f, 1.0f);
    const float d       = constrain(descent / frac, 0.0f, 1.0f);
    return d * d * (3.0f - 2.0f * d);   // smoothstep, zero-slope ends
}

// Applied outward swing-prep delta on the swing leg this tick (telemetry +
// composition). 0 unless enabled, gain>0, and this is the active normal swing
// leg. Positive g·env interpolates the swing ankle from the inward CoM-shift
// tilt toward the future-stance orientation.
float GaitController::_swingPrepDeltaDeg(Side side) const {
    if (!_cfg.swingPrepEnabled || _cfg.swingPrepGain == 0.0f) return 0.0f;
    if (!_isNormalSwing(side)) return 0.0f;
    const float w = constrain(_cfg.swingPrepGain * _swingPrepEnv(), 0.0f, 1.0f);
    return w * (_futureStanceTilt(side) - _comShiftTilt(side));
}

// Single ankle-roll composition site. flatFootDeg = the IK flat-foot solution
// (-hip_roll) for this leg. Returns the joint-command ankle roll, clamped to
// the joint soft limit.
float GaitController::_ankleRollDeg(Side side, float flatFootDeg) const {
    const float comShift = _comShiftTilt(side);
    const float leveler  = _swingLeveler ? _swingLeveler->ankleRollBiasFor(side) : 0.0f;

    float ankle;
    if (_isNormalSwing(side)) {
        // Airborne swing leg: base = inward CoM-shift tilt (continuous with the
        // grounded value at lift-off, where a≈1), plus the outward swing-prep
        // delta that ramps it toward future stance over the descent.
        ankle = comShift + _swingPrepDeltaDeg(side);
    } else {
        // Grounded (incl. stance / WAIT_SHIFT / STABILIZING / recovery):
        // crossfade flat-foot ↔ CoM-shift tilt by shift authority.
        const float a = _shiftAuthority(side);
        ankle = flatFootDeg * (1.0f - a) + comShift * a;
    }
    return constrain(ankle + leveler, -25.0f, 25.0f);
}

// ---------------------------------------------------------------------------
//  _submitFootTargetIK — Resolve FootTarget through LegIK and submit angles.
//
//  Joint indices come from joint_config.h. Submission goes through SOURCE_GAIT
//  so MotionManager priority arbitration is preserved. JointModel direction
//  fields handle left/right mirroring — submit the SAME anatomical angle to
//  both legs (do NOT negate for left here).
//
//  On DOMAIN_ERROR (h too small, NaN guard): no submission. Logs at 1 Hz to
//  avoid serial flood. UNREACHABLE / LIMIT_CLAMPED still submit the (clamped)
//  solution; status is recorded for the UI.
// ---------------------------------------------------------------------------
LegIKResult GaitController::_submitFootTargetIK(Side side,
                                                 const FootTarget& t) {
    LegIKResult r;
    if (_mm == nullptr) { r.status = IKStatus::DOMAIN_ERROR; return r; }

    // Snapshot the final FootTarget components for UI diagnostics, BEFORE the
    // validity early-out so the UI still sees the requested target on
    // DOMAIN_ERROR (helpful for "why did IK refuse" debugging).
    if (side == Side::LEFT) {
        _state.lastFootTargetXL_mm = t.x_mm;
        _state.lastFootTargetYL_mm = t.y_mm;
        _state.lastFootTargetHL_mm = t.h_sagittal_mm;
    } else {
        _state.lastFootTargetXR_mm = t.x_mm;
        _state.lastFootTargetYR_mm = t.y_mm;
        _state.lastFootTargetHR_mm = t.h_sagittal_mm;
    }

    if (!t.valid)       { r.status = IKStatus::DOMAIN_ERROR; return r; }

    r = LegIK::solve(t);

    if (r.status == IKStatus::DOMAIN_ERROR) {
        static uint32_t _lastLogMs = 0;
        const uint32_t now = millis();
        if (now - _lastLogMs > 1000) {
            _lastLogMs = now;
            Serial.printf("[GaitCtrl] IK DOMAIN_ERROR  side=%s  x=%.1f h=%.1f y=%.1f\n",
                          sideName(side),
                          t.x_mm, t.h_sagittal_mm, t.y_mm);
        }
        return r;
    }

    // Flat-foot guard: a clamped ankle pitch indicates the IK ran out of
    // soft-limit headroom (typically when reach is too close to maxReach
    // and the pose can't keep the sole flat). Treat |error| > 2 deg as a tuning
    // red flag - log and skip submission so the previous tick's pose holds.
    // Rationale: gait_development_integration.md section 6.5 - abort step on
    // flat_foot_error > ~2 deg.
    if (fabsf(r.flat_foot_error_deg) > 2.0f) {
        // Activation counter (per-session; reset on start()). Single source of
        // flat-foot-guard instrumentation — surfaced as ffGuard in the GAIT:
        // packet. A rising count while a swing-foot-placement feature is enabled
        // flags placement pushing IK reach into the ankle soft-limit.
        _state.flatFootGuardCount++;
        static uint32_t _lastFFLogMs = 0;
        const uint32_t now = millis();
        if (now - _lastFFLogMs > 1000) {
            _lastFFLogMs = now;
            Serial.printf("[GaitCtrl] FLAT-FOOT GUARD  side=%s  err=%.2f deg  "
                          "x=%.1f h=%.1f reach=%.1f%% - pose held.\n",
                          sideName(side), r.flat_foot_error_deg,
                          t.x_mm, t.h_sagittal_mm, r.sagittal_reach_pct);
        }
        return r;   // status fields populated; caller updates telemetry
    }

    // SOURCE_GAIT (priority 1). SOURCE_BALANCE (priority 2) on shared joints
    // will still override these. Phase 4 refactor: channels resolved through
    // channelFor(side, LegJoint::*) instead of the legacy isRight ternaries.
    //
    // Swing-foot leveler: additive ankle bias for the airborne leg.
    // anklePitchBiasFor() / ankleRollBiasFor() return 0 for the stance leg
    // (activeSide check inside the leveler) and 0 in L0 (enabled=false),
    // so this is safe to apply unconditionally on both legs.
    // Pitch leveler bias added directly at SOURCE_GAIT — pitch balance
    // submission is opt-in (only when pitch_enabled), so this submission
    // typically wins during leveler testing.
    //
    // Ankle ROLL — single owner, single inspectable summation (consolidation
    // refactor). BalanceController no longer submits ankle roll, so SOURCE_GAIT
    // is never overwritten on this joint. The composed value is:
    //   flat-foot base (-hip_roll)            r.ankle_roll_deg
    // + CoM-shift bias (stance/grounded leg)  _comShiftRollBias(side)
    // + swing-prep toward future stance       _swingPrepRollBias(side)
    // + closed-loop foot leveler              ankleRollBiasFor(side)
    // then clamped to the ankle-roll joint soft limit (±25°, joint_config.cpp).
    // Joint-frame submissions. Servo-neutral drift is corrected globally by
    // JointModel's runtime session trim on the joint↔servo conversion — gait
    // submits raw IK angles and inherits the correction downstream.
    const float pitch_bias = _swingLeveler ? _swingLeveler->anklePitchBiasFor(side) : 0.0f;
    // Heel-strike bias is additive AFTER IK. The flat-foot guard above checks
    // r.flat_foot_error_deg from the pure IK solution (bias is not in the IK
    // target), so a deliberate heel-up attitude at late swing cannot trip the
    // guard. pitch_bias (swing leveler) and heel_bias sum additively; both are
    // off-by-default, so there is no interaction in the shipped config.
    //
    // _heelStrikeBiasDeg returns the bias for THIS side: nonzero on the swing
    // leg during ENGAGE, and on the just-planted leg during RELEASE (which is
    // the stance leg after _advanceHalf flips the role). The per-side write
    // happens here; the telemetry mirror is refreshed once per tick by
    // _updateHeelStrike, so this call site no longer writes _state.* fields.
    const float heel_bias = _heelStrikeBiasDeg(side);
    const float ankle_pitch_cmd = r.ankle_pitch_deg + pitch_bias + heel_bias;
    _mm->submit(SOURCE_GAIT, channelFor(side, LegJoint::HIP_PITCH),   r.hip_pitch_deg);
    _mm->submit(SOURCE_GAIT, channelFor(side, LegJoint::KNEE_PITCH),  r.knee_pitch_deg);
    _mm->submit(SOURCE_GAIT, channelFor(side, LegJoint::ANKLE_PITCH), ankle_pitch_cmd);
    _mm->submit(SOURCE_GAIT, channelFor(side, LegJoint::HIP_ROLL),    r.hip_roll_deg);
    // Rollover telemetry: remember the ankle-pitch value actually submitted for
    // this side (joint frame, deg) so _updateRolloverTelemetry can difference it
    // against the measured foot pitch. Telemetry only — the submitted value
    // above is unchanged.
    _lastAnkleCmdDeg[sideIdx(side)] = ankle_pitch_cmd;

    const float ankle_roll = _ankleRollDeg(side, r.ankle_roll_deg);
    _mm->submit(SOURCE_GAIT, channelFor(side, LegJoint::ANKLE_ROLL),  ankle_roll);

    // Telemetry: the swing leg reports its outward prep delta; the grounded leg
    // reports its CoM-shift tilt. Together they make the composition observable.
    if (side == _swingSide) {
        _state.swingPrepRollBiasDeg = _swingPrepDeltaDeg(side);
        _state.swingPrepSide        = static_cast<uint8_t>(side);
    } else {
        _state.comShiftRollBiasDeg  = _comShiftTilt(side);
    }

    return r;
}

// ---------------------------------------------------------------------------
void GaitController::_submitStanceBothLegs(const BalanceCorrection* bc) {
    // strideScale=0: stride and capture suppressed. Yields x=GAIT_STANCE_X_OFFSET_MM,
    // y=0 (plus WS task-space and balance correction). Used as the neutral
    // baseline whenever no leg is in an active swing role.
    const FootTarget tL = _buildFootTarget(false, 0.0f, false, 0.0f, bc, 0.0f);
    const FootTarget tR = _buildFootTarget(true,  0.0f, false, 0.0f, bc, 0.0f);

    const LegIKResult rL = _submitFootTargetIK(false, tL);
    const LegIKResult rR = _submitFootTargetIK(true,  tR);

    _state.lastIKStatusL = static_cast<uint8_t>(rL.status);
    _state.lastIKStatusR = static_cast<uint8_t>(rR.status);
    _state.lastReachPctL = rL.sagittal_reach_pct;
    _state.lastReachPctR = rR.sagittal_reach_pct;
    _state.lastFlatFootErrL_deg = rL.flat_foot_error_deg;
    _state.lastFlatFootErrR_deg = rR.flat_foot_error_deg;
}

// ---------------------------------------------------------------------------
//  _submitCycleStanceBoth — Stage 3: cycle-role aware submission.
//
//  At any localPhase, exactly one leg is in the swing role (foot on or near
//  the planting trajectory) and one is in the stance role (foot retreating
//  through body frame). This helper submits both legs with the correct role
//  flag so _buildFootTarget applies the appropriate stride formula.
//
//  Use during WAIT_SHIFT and STABILIZING active sub-states. SWINGING uses its
//  own explicit submission (with non-zero swingFrac for the lift). IDLE and
//  STOPPING continue to use _submitStanceBothLegs (no stride applied).
//
//  Note: liftFrac=0 here. The lift trajectory is the responsibility of the
//  SWINGING branch, which overrides whatever this helper submitted.
// ---------------------------------------------------------------------------
void GaitController::_submitCycleStanceBoth(float localPhase,
                                              const BalanceCorrection* bc) {
    const bool leftIsSwing = (_swingSide == Side::LEFT);
    const FootTarget tL = _buildFootTarget(/*isRight=*/false, /*liftFrac=*/0.0f,
                                            /*isSwing=*/  leftIsSwing,
                                            localPhase, bc);
    const FootTarget tR = _buildFootTarget(/*isRight=*/true,  /*liftFrac=*/0.0f,
                                            /*isSwing=*/ !leftIsSwing,
                                            localPhase, bc);

    const LegIKResult rL = _submitFootTargetIK(false, tL);
    const LegIKResult rR = _submitFootTargetIK(true,  tR);

    _state.lastIKStatusL = static_cast<uint8_t>(rL.status);
    _state.lastIKStatusR = static_cast<uint8_t>(rR.status);
    _state.lastReachPctL = rL.sagittal_reach_pct;
    _state.lastReachPctR = rR.sagittal_reach_pct;
    _state.lastFlatFootErrL_deg = rL.flat_foot_error_deg;
    _state.lastFlatFootErrR_deg = rR.flat_foot_error_deg;
}

// ---------------------------------------------------------------------------
//  _advanceHalf  —  called when _halfPhase >= 1.0 (foot back on ground).
//  Flips the swing side and re-enters WAIT_SHIFT for the next leg.
//  SINGLE_STEP stops after completing the RIGHT-swing half (both legs have
//  taken one step).
// ---------------------------------------------------------------------------
void GaitController::_advanceHalf() {
    if (_state.mode == GaitMode::SINGLE_STEP && _swingSide == Side::RIGHT) {
        stop();
        Serial.println("[GaitCtrl] SINGLE_STEP complete — IDLE.");
        return;
    }

    // Heel-strike fallback: if no touchdown was detected during this swing
    // (impact detector failed or was disabled), kick off the timer-based
    // release NOW so the bias smooth-decays instead of snapping to 0 when
    // _swingSide flips below. _startHeelStrikeRelease() is a no-op when the
    // feature is off OR a release is already in progress (the normal
    // touchdown-fire path), so this is safe to call unconditionally.
    // Must run BEFORE _swingSide flips: the helper snapshots _swingSide as
    // the active side, and the release belongs to the leg that just landed.
    _startHeelStrikeRelease();

    // H-restore arming: if this swing latched a touchdown with a frozen h
    // below stanceHeightMm, arm a restore on the leg that just planted (the
    // current _swingSide, BEFORE the flip below) so its h re-extends smoothly
    // across the upcoming stance phase instead of snapping at the role swap.
    // Must run before _swingSide flips and before _clearTouchdownLatch() clears
    // _touchdownLocked / _touchdownLockedH_mm. If no touchdown was detected this
    // swing, _touchdownLocked is false → no restore armed; the leg ran its full
    // trajectory and is already at stanceHeight.
    //
    // The deficit is computed (and _hRestoreDeficitMm assigned) HERE, not at the
    // touchdown latch. _hRestoreDeficitMm is owned by the active restore and is
    // read every tick by the CONTRALATERAL restoring leg in _buildFootTarget;
    // assigning it only at the role swap — when the previous restore has already
    // completed (_halfPhase >= frac was reached, clearing it in _updateHRestore)
    // — guarantees a touchdown latch can never overwrite a deficit a still-active
    // restore is consuming. This is the fix for the frac-dependent, leg-mirrored
    // abrupt-extension asymmetry (root cause: shared deficit + early latch).
    if (_cfg.hRestoreEnabled && _touchdownLocked) {
        const float deficit = _cfg.stanceHeightMm - _touchdownLockedH_mm;  // >= 0
        if (deficit > 0.0f) {
            _hRestoreActive    = true;
            _hRestoreSide      = _swingSide;   // the leg that just planted
            _hRestoreDeficitMm = deficit;      // owned by THIS restore until complete
            Serial.printf("[GaitCtrl] H-restore ARM swing=%s deficit=%.2fmm "
                          "(tdH=%.2f stanceH=%.2f frac=%.2f)\n",
                          sideName(_swingSide), deficit, _touchdownLockedH_mm,
                          _cfg.stanceHeightMm, _cfg.hRestoreCompleteFrac);
        }
    }

    _swingSide        = otherSide(_swingSide);
    _halfPhase        = 0.0f;
    _state.cycleXDispl_mm = 0.0f;
    _subState         = StepSubState::STABILIZING;
    _stabilizeStartMs = millis();
    // New swing leg = new half-cycle: previous latch is no longer meaningful.
    // Telemetry mirror also wiped so the UI shows a clean "no detection yet"
    // state until the new swing leg actually triggers.
    // NOTE: heel-strike state is deliberately NOT cleared here — the release
    // timer continues across this boundary so the just-planted foot can roll
    // smoothly to flat. _updateHeelStrike clears the release on its own when
    // the window elapses. This mirrors how _resetCompliance is also NOT called.
    _clearTouchdownLatch();
    // skipStabilizeNone=false (default): center WeightShift so structural/IIR
    // transients damp during the hold. _triggerShiftForSwing() for the new
    // half runs after STABILIZE expires (see update() STABILIZING branch).
    // skipStabilizeNone=true (step-4 probe): skip centering; immediately arm
    // the next half's direction so CoM blends foot-to-foot without returning
    // to center. _triggerShiftForSwing is idempotent at the STABILIZE→WAIT
    // transition (direction already set), so no double-trigger occurs.
    if (_cfg.skipStabilizeNone) {
        _triggerShiftForSwing(_swingSide);
    } else {
        if (_ws) _ws->trigger(ShiftDirection::NONE);
        _lastShiftDir = ShiftDirection::NONE;
    }
    Serial.printf("[GaitCtrl] STABILIZE — swing=%s, %ums hold (skipNone=%d)\n",
                  sideName(_swingSide), (unsigned)_cfg.stabilizeMs,
                  (int)_cfg.skipStabilizeNone);
}

// ---------------------------------------------------------------------------
//  update  —  main control loop, every tick from main.cpp.
// ---------------------------------------------------------------------------
void GaitController::update(float dt_s, const IMUState& imuState,
                            const BalanceCorrection* bc,
                            const BalanceState* balState,
                            const FootContactSnapshot* footSnap) {
    _lastImuState = imuState;

    // W5 — advance the roll-rate LPF exactly once per tick, BEFORE any trim
    // helper reads _rollRateFilt. Runs ahead of the estop early-return below
    // so the filter keeps tracking (and the telemetry stays live) on every
    // path, the same way _updateCompliance() does.
    _updateRollRateFilter(imuState);

    // Touchdown trim suppression — decide ONCE per tick whether the W2/W5
    // lateral trim is gated this tick. Must run after _updateRollRateFilter
    // (the freeze snapshot reads the raw trim, which uses _rollRateFilt) and
    // before the trim telemetry mirror + any _buildFootTarget call, so every
    // _comLateralTrimMm() read this tick sees the same suppression decision.
    _updateTrimSuppression();

    // W2/W5 telemetry mirror — refresh every tick so the UI sees the live trim
    // on all paths (including IDLE stance refresh, which never reaches the
    // active-gait telemetry block below). comLatTrimMm is the final applied
    // trim (Kp+Kd, clamped); the other two break out the W5 damping signal.
    _state.comLatTrimMm       = _comLateralTrimMm();
    _state.comLatRollRateFilt = _rollRateFilt;
    _state.comLatTrimVelMm    = _comLatVelTrimMm();

    // W4 — decay the touchdown-compliance offset every tick. Placed before
    // the estop early-return so the decay is purely time-based and continues
    // through any FSM path; _buildFootTarget reads _complianceOffsetMm.
    _updateCompliance();

    // H-restore — phase-clean cleanup + telemetry refresh once per tick. Placed
    // beside _updateCompliance and before the estop early-return so the restore
    // bookkeeping stays consistent on every path; _buildFootTarget applies the
    // offset to the restoring leg from _hRestoreDeficitMm / _hRestoreSide.
    _updateHRestore();

    // Heel-strike — refresh telemetry mirror and run release-complete cleanup
    // once per tick. Mirrors the placement of _updateCompliance: before the
    // estop early-return so the decay is purely time-based and continues
    // smoothly through any FSM path, including across the role swap.
    _updateHeelStrike();

    // Publish swing-leg hint to WeightShift FIRST. Reflects last tick's
    // substate; consumed on next tick's WS update (2.5 ms staleness).
    // Placing this at the top guarantees it runs on every early-return path.
    _publishSwingLegMaskToWS();

    if (_mm == nullptr)   return;
    if (oe_is_estopped()) return;

    // Clear per-tick lift outputs (re-set below when swinging).
    _state.liftL_deg  = 0.0f;
    _state.liftR_deg  = 0.0f;
    _state.liftGateOK = false;
    _state.gateImuRollOK    = false;
    _state.gateStanceFlat   = false;
    _state.gateStanceLoaded = false;
    _state.gateStanceIsLeft = false;

    // Stage 1: observe recovery flag from BalanceController.
    // Suppressed when already in RECOVERY_STEP (no re-trigger mid-step) and
    // during RUNNING (F6 mitigation — false positives during walking).
    // balState is the previous-tick BalanceState in legacy mode (2.5 ms stale),
    // which is within the recovery_confirm_ticks debounce window.
    if (balState != nullptr
        && balState->recovery_flag
        && _state.mode != GaitMode::RECOVERY_STEP
        && _state.mode != GaitMode::RUNNING) {
        triggerRecovery(imuState.pitch, imuState.roll);
    }

    if (_state.mode == GaitMode::RECOVERY_STEP) {
        _runRecoveryStep(dt_s, imuState);
        return;
    }

    // Swing-foot leveler: update once per non-recovery tick.
    // liftFrac=0 outside SWINGING causes the envelope to return 0, so the
    // IIR decays smoothly without any bias reaching the ankle joints.
    // The leveler's roll bias is read directly into the SOURCE_GAIT ankle-roll
    // sum in _submitFootTargetIK (consolidation refactor) — it is no longer
    // plumbed through BalanceController, which no longer owns ankle roll.
    if (_swingLeveler) {
        const float lf = (_subState == StepSubState::SWINGING)
                         ? _swingFrac(_halfPhase) : 0.0f;
        _swingLeveler->update(_swingSide, lf);
    }

    const WeightShiftState wsState = _ws ? _ws->getState() : WeightShiftState{};
    const bool wsActive = (_ws != nullptr) &&
                          (fabsf(wsState.progress) > 0.005f || wsState.ramping);
    const bool gaitActive = (_state.mode != GaitMode::IDLE);

    if (!wsActive && !gaitActive) {
        _state.phase      = 0.0f;
        _state.wsProgress = 0.0f;
        _state.poseHold   = 0;
        // Task-space mode: BalanceCorrection reaches servos only through IK.
        // GaitController is the sole consumer of bc, so we must not skip it
        // here — the robot must maintain its pose under balance correction even
        // when standing still with no gait or weight-shift active.
        if (bc != nullptr && bc->valid) {
            _submitStanceBothLegs(bc);
        }
        return;
    }

    // ── Weight shift state ────────────────────────────────────────────────────
    const float wsProgress = _ws ? wsState.progress : 1.0f;
    _state.wsProgress = wsProgress;

    // -- Phase F3: composable lift gate ------------------------------------
    //
    // Layer 1 (always required, except when explicitly bypassed):
    //   progressOK - kinematic check: WeightShift has ramped into the
    //                correct signed direction with sufficient magnitude.
    //
    // Layer 2 (extra verification, mode-dependent):
    //   imuRollOK    - torso IMU roll past wsThresholdRad in correct direction
    //   stanceFlat   - stance foot's pitch and roll within the flat-threshold
    //   stanceLoaded - stance foot's pitch shows the loaded-deviation signature
    //
    // POSE_STEP unconditionally bypasses Layer 2 - the operator is in the loop.
    //
    // swing=LEFT  -> RIGHT foot is stance (rollSign = +1)
    // swing=RIGHT -> LEFT  foot is stance (rollSign = -1)
    const bool poseMode     = (_state.mode == GaitMode::POSE_STEP);
    const bool isLeftStance = (_swingSide == Side::RIGHT);
    const float rollSign    = (_swingSide == Side::LEFT) ? +1.0f : -1.0f;

    const bool progressOK = (wsProgress * rollSign >= _cfg.liftGateThreshold);

    // Always evaluate every sub-gate for telemetry consistency. The mode
    // dispatch below picks which ones actually compose into liftGateOK.
    const bool imuRollOK = (imuState.roll * rollSign >= _cfg.wsThresholdRad);

    bool stanceFlat   = false;
    bool stanceLoaded = false;
    if (footSnap != nullptr) {
        if (isLeftStance) {
            stanceFlat   = footSnap->leftValid && footSnap->leftFlat;
            stanceLoaded = footSnap->leftValid && footSnap->leftLoaded;
        } else {
            stanceFlat   = footSnap->rightValid && footSnap->rightFlat;
            stanceLoaded = footSnap->rightValid && footSnap->rightLoaded;
        }
    }

    // Stage 3 -- Lateral CoM observation via inverted-pendulum model.
    // Computed every tick regardless of liftGateMode so telemetry is
    // always available for diagnostic comparison against IMU_ROLL.
    // Invalid IMU leaves comLateralOK=false, keeping the gate closed.
    float comLateralMm = 0.0f;
    bool  comLateralOK = false;
    if (imuState.valid) {
        const float rollCorrected = imuState.roll - _cfg.torsoRollNeutralRad;
        comLateralMm = H_COM_MM * rollCorrected;   // sin(x) ~= x for small x
        // Signed projection. rollSign is +1 for LEFT-swing (CoM should
        // move RIGHT, body roll positive), -1 for RIGHT-swing -- same
        // convention as the IMU_ROLL gate above so both modes are
        // directly comparable in telemetry.
        comLateralOK = (comLateralMm * rollSign >= _cfg.comLateralGateMm);
    }

    bool extraGate = true;
    if (!poseMode) {
        switch (_cfg.liftGateMode) {
            case LiftGateMode::OFF:
                extraGate = true;
                break;
            case LiftGateMode::IMU_ROLL:
                extraGate = imuRollOK;
                break;
            case LiftGateMode::STANCE_FLAT:
                extraGate = stanceFlat;
                break;
            case LiftGateMode::STANCE_LOADED:
                // STANCE_LOADED implicitly requires the foot to be flat — an
                // unloaded foot that happens to be flat is not "loaded", and
                // a loaded-but-tilted foot indicates the stance leg is on the
                // edge. Compose flat AND loaded so the mode means what its name
                // suggests. This is the one place we DO compose, because the
                // two predicates describe the same physical event.
                extraGate = stanceFlat && stanceLoaded;
                break;
            case LiftGateMode::COM_LATERAL:
                extraGate = comLateralOK;
                break;
            case LiftGateMode::STANCE_FLAT_COM_LATERAL:
                extraGate = stanceFlat && comLateralOK;
                break;
        }
    }

    _state.liftGateOK = progressOK && extraGate;
    _state.gateImuRollOK    = imuRollOK;
    _state.gateStanceFlat   = stanceFlat;
    _state.gateStanceLoaded = stanceLoaded;
    _state.gateStanceIsLeft = isLeftStance;
    _state.comLateralMm     = comLateralMm;
    _state.gateComLateralOK = comLateralOK;

    _state.subState = _subState;
    // phase: LEFT-swing maps to [0, 0.5), RIGHT-swing maps to [0.5, 1.0).
    // Underlying Side enum value (LEFT=0, RIGHT=1) preserves the legacy mapping.
    _state.phase    = static_cast<uint8_t>(_swingSide) * 0.5f + _halfPhase * 0.5f;
    _state.poseHold = _poseHoldPt;

    // BUG FIX (weight-shift step artefact): when gait mode is IDLE but
    // WeightShift is active (manual WEIGHT_SHIFT:left/right from UI), submit
    // ONLY the neutral stance pose (no stride). _submitCycleStanceBoth applies
    // the Stage 3 cycle stride formula based on _swingSide and _halfPhase,
    // which produces a ±stepLengthMm/2 x offset per leg even at _halfPhase=0
    // — visible as a fore/aft "step" alongside the requested lateral shift.
    // _submitStanceBothLegs passes strideScale=0 and is the correct call here.
    //
    // Must come BEFORE _submitCycleStanceBoth, not after. The previous fix
    // pinned FSM state to prevent oscillation but did not undo the bad stance
    // submission that had already gone into the MotionManager slot table.
    if (!gaitActive) {
        _submitStanceBothLegs(bc);
        _subState         = StepSubState::WAIT_SHIFT;
        _state.subState   = _subState;
        _halfPhase        = 0.0f;
        _state.poseHold   = 0;
        _state.liftL_deg  = 0.0f;
        _state.liftR_deg  = 0.0f;
        return;
    }

    // -----------------------------------------------------------------------
    // Touchdown-awareness latch (Stage 1).
    //
    // Run BEFORE any foot target is built this tick so the latch state is
    // honored by both _submitCycleStanceBoth (below) and the SWINGING-branch
    // submission later in this function. The detector itself is gait-
    // agnostic: it produces per-side "detected" events; the policy of
    // *when* to consume them lives here.
    //
    // Gating policy (per touchdown-awareness_integration_plan.md §3.1):
    //   - Only while SWINGING (lift trajectory is active).
    //   - Only when _halfPhase > touchdownGatePhase (late swing).
    //   - Only when the latch is not already set (one event per half).
    //   - Only when the detector reported a fresh detection THIS tick
    //     for the side that is currently swinging.
    //
    // Snapshotting:
    //   - The stride x contribution captured here is the full open-loop
    //     value that _buildFootTarget would have produced for the swing
    //     leg this tick (smoothstep stride + capture). After lock, that
    //     value is reused verbatim while the latch is held — so the foot
    //     stops advancing forward in body frame, and the stance leg keeps
    //     retreating per its own (unlocked) formula. The body therefore
    //     advances naturally over the planted foot.
    // -----------------------------------------------------------------------
    if (_subState == StepSubState::SWINGING
        && _cfg.touchdownEnabled
        && !_touchdownLocked
        && _impact != nullptr
        && _halfPhase > _cfg.touchdownGatePhase) {
        const FootImpactSideState& imp = _impact->getState(_swingSide);
        if (imp.detected) {
            // Snapshot stride x at this tick using the shared warped-stride
            // helper — SAME helper _buildFootTarget calls for the swing leg.
            // Mandatory: a divergence between commanded stride and latched x
            // would inject a position jump at touchdown the moment the latch
            // fires whenever strideCompleteFrac < 1.0. captureX is added on
            // top exactly as in _buildFootTarget.
            const float s_w      = _warpedStrideFrac(_halfPhase);
            const float captureX = _captureOffsetMm();
            // Pitch-ANGLE placement is snapshotted with the SAME helper and at
            // the same phase the swing stride uses in _buildFootTarget. Folding
            // it into the locked x (rather than freezing it separately) is what
            // makes the commanded-vs-latched x continuous at the latch tick;
            // after lock it is held frozen with the rest of the stride x.
            _touchdownLockedX_mm  = (s_w - 0.5f) * _cfg.stepLengthMm + captureX
                                  + _pitchPlacementOffsetMm(_halfPhase);

            // Snapshot the swing foot's HORIZONTAL velocity (mm/s) at this
            // detection. Analytic derivative of the warped stride x:
            //   x(lp)  = (s_w - 0.5) * L  + captureX
            //   s_w    = lpw*lpw*(3 - 2*lpw), lpw = min(lp/frac, 1)
            //   ds_w/dlp = 6*lpw*(1-lpw)/frac   for lp < frac, else 0
            //   dlp/dt   = 2 * phaseRateHz
            // captureX is treated as quasi-static for this telemetry — its
            // per-tick velocity contribution is small relative to the stride
            // term (and capture is default-OFF). At strideCompleteFrac=1.0
            // this reduces to the baseline 6*lp*(1-lp)*L*2*phaseRateHz; with
            // frac lowered it drops to 0 once lp >= frac, which is the whole
            // point of the metric.
            const float frac_clamped = constrain(_cfg.strideCompleteFrac, 0.05f, 1.0f);
            const float lp_clamped   = constrain(_halfPhase, 0.0f, 1.0f);
            float dxdt_mm_s = 0.0f;
            if (lp_clamped < frac_clamped) {
                const float lpw      = lp_clamped / frac_clamped;
                const float dsw_dlp  = 6.0f * lpw * (1.0f - lpw) / frac_clamped;
                const float dlp_dt   = 2.0f * _cfg.phaseRateHz;
                dxdt_mm_s            = _cfg.stepLengthMm * dsw_dlp * dlp_dt;
            }
            _state.touchdownSwingVelX_mm_s = dxdt_mm_s;
            _touchdownLockedH_mm  = _cfg.stanceHeightMm
                                  - _swingFrac(_halfPhase) * _cfg.stepHeightMm;
            // H-restore: the deficit (stanceHeightMm - frozen touchdown h) is
            // NOT recorded here. _advanceHalf computes and arms it from the
            // still-valid _touchdownLockedH_mm at the role swap. Writing it here
            // corrupted an in-progress restore on the CONTRALATERAL leg: that
            // restore reads the shared _hRestoreDeficitMm every tick in
            // _buildFootTarget, and whenever this swing leg latched before that
            // restore completed (only possible when hRestoreCompleteFrac >
            // touchdownGatePhase) the write clobbered the deficit mid-decay —
            // the leg-mirrored "abrupt extension" bug above frac 0.5.
            // Lateral-capture freeze: capture the live ramped+clamped value
            // BEFORE flipping _touchdownLocked, because _latCaptureOffsetMm()
            // short-circuits to _touchdownLockedY_mm once the latch is set.
            // After this point, _buildFootTarget reuses this snapshot for the
            // rest of the half-cycle so the lateral contribution stops
            // tracking live rollRate on a planted foot (would otherwise
            // re-introduce stance-style body-shove). Mirrors the X snapshot.
            _touchdownLockedY_mm  = _latCaptureOffsetMm(_halfPhase);
            _touchdownLockedPhase = _halfPhase;
            _touchdownLocked      = true;
            _touchdownMs          = millis();  // step-timing: planted-but-not-advanced start

            // Stage 0 telemetry mirror.
            _state.touchdownSeen      = true;
            _state.touchdownPhase     = _halfPhase;
            _state.touchdownSwingX_mm = _touchdownLockedX_mm;
            _state.touchdownSide      = static_cast<uint8_t>(_swingSide);
            _state.touchdownLocked    = true;

            // Measurement snapshot (telemetry only — no control effect): the
            // swing foot's IMU roll from its per-foot estimator, and the
            // neutral-corrected torso roll. Lets a swing-preload A/B sweep
            // observe the landed swing attitude objectively.
            const StateEstimator* swingEst = _footEst[sideIdx(_swingSide)];
            _state.touchdownSwingFootRollRad =
                swingEst ? swingEst->getState().roll : 0.0f;
            _state.touchdownTorsoRollRad =
                _lastImuState.roll - _cfg.torsoRollNeutralRad;

            // Rollover instrumentation: open the post-contact observation window
            // on the just-landed foot (= current _swingSide; it becomes stance
            // after _advanceHalf, but the window tracks the same physical foot
            // throughout). Reset the per-step summaries so the new step starts
            // clean. TELEMETRY ONLY — no control effect. _updateRolloverTelemetry
            // services this window each tick.
            _rolloverActive        = true;
            _rolloverStartMs       = _touchdownMs;   // == millis() at the latch
            _rolloverSide          = static_cast<uint8_t>(_swingSide);
            _rolloverFlatReached   = false;
            _rolloverPeakComVelAbs = 0.0f;
            _state.rolloverActive               = true;
            _state.rolloverSide                 = _rolloverSide;
            _state.rolloverFootPitchRad         = 0.0f;
            _state.ankleCmdVsFootPitchDivergenceDeg = 0.0f;
            _state.rolloverDurationMs           = 0;
            _state.postContactCoMVelSign        = 0;
            _state.postContactBodyPitchPeak     = 0.0f;

            // W4 — touchdown compliance. The same early-contact event that
            // latches the swing leg also fires a brief stance-height yield:
            // both legs flex by complianceDepthMm to absorb the landing
            // impulse, then recover smoothly (see _updateCompliance()). This
            // reuses the touchdown event verbatim — no separate detection.
            // The compliance timer is independent of the latch, so the yield
            // decays cleanly even after the latch is cleared at _advanceHalf.
            if (_cfg.complianceEnabled && _cfg.complianceDepthMm > 0.0f) {
                _complianceStartMs  = millis();
                _complianceActive   = true;
                _complianceOffsetMm = -_cfg.complianceDepthMm;
                _state.complianceOffsetMm = _complianceOffsetMm;
                _state.complianceActive   = true;
            }

            // Heel-strike — start the timer-based RELEASE at touchdown. The
            // helper snapshots the live engage fraction (continuous transition,
            // no upward jump) and locks _heelStrikeActiveSide to the current
            // swing leg, so the bias keeps applying to the just-planted foot
            // through the _advanceHalf role swap and out the other side.
            // _startHeelStrikeRelease is a no-op when heelStrikeEnabled=false.
            _startHeelStrikeRelease();

            Serial.printf("[GaitCtrl] TOUCHDOWN  swing=%s  phase=%.3f  "
                          "x=%.1f mm  delta_g=%.3f\n",
                          sideName(_swingSide),
                          _halfPhase,
                          _touchdownLockedX_mm,
                          imp.last_delta_g);
        }
    }
    // Keep the public mirror in sync every tick (clears if a future hook
    // ever clears the internal latch without going through _clearTouchdownLatch).
    _state.touchdownLocked = _touchdownLocked;

    // Rollover instrumentation — service the post-contact observation window
    // (opened at the latch above). TELEMETRY ONLY; reads this tick's torso
    // IMUState, submits nothing. Runs every active tick so the window can span
    // the _advanceHalf role swap.
    _updateRolloverTelemetry(imuState);

    // Active gait modes only: cycle-role aware stance (Stage 3). Both legs
    // receive the stride formula so accumulated body displacement is preserved
    // through WAIT_SHIFT and STABILIZING. Later branches (SWINGING) overwrite
    // one leg with a swing-arc target.
    _submitCycleStanceBoth(_halfPhase, bc);

    if (_subState == StepSubState::STOPPING) {
        const uint32_t elapsed = millis() - _stopStartMs;
        const float    t       = (elapsed >= GAIT_STOP_RAMP_MS)
                                ? 1.0f
                                : (float)elapsed / (float)GAIT_STOP_RAMP_MS;
        const float    smooth  = t * t * (3.0f - 2.0f * t);
        const float    liftRamp = 1.0f - smooth;
        const float stopFrac    = _stopSwingFrac * liftRamp;
        const float strideScale = liftRamp;   // identical time profile

        const bool leftIsSwing = (_stopSwingSide == Side::LEFT);
        const FootTarget tL = _buildFootTarget(false, stopFrac,  leftIsSwing,
                                               _halfPhase, bc, strideScale);
        const FootTarget tR = _buildFootTarget(true,  stopFrac, !leftIsSwing,
                                               _halfPhase, bc, strideScale);
        const LegIKResult rL = _submitFootTargetIK(false, tL);
        const LegIKResult rR = _submitFootTargetIK(true,  tR);

        _state.lastIKStatusL = static_cast<uint8_t>(rL.status);
        _state.lastIKStatusR = static_cast<uint8_t>(rR.status);
        _state.lastReachPctL = rL.sagittal_reach_pct;
        _state.lastReachPctR = rR.sagittal_reach_pct;
        _state.lastFlatFootErrL_deg = rL.flat_foot_error_deg;
        _state.lastFlatFootErrR_deg = rR.flat_foot_error_deg;
        _state.liftL_deg = leftIsSwing  ? rL.hip_pitch_deg : 0.0f;
        _state.liftR_deg = !leftIsSwing ? rR.hip_pitch_deg : 0.0f;

        // Telemetry: applied capture + lateral trim also fade — surface the
        // actual (faded) value, not the raw computation.
        _state.captureOffsetMm = _captureOffsetMm()    * strideScale;
        // Pitch-ANGLE placement fades through STOPPING exactly like capture.
        _state.pitchPlacementMm = _pitchPlacementOffsetMm(_halfPhase) * strideScale;
        _state.latCaptureMm    = _latCaptureOffsetMm(_halfPhase) * strideScale;
        _state.comLatTrimMm    = _comLateralTrimMm()   * strideScale;
        _state.cycleXDispl_mm  = _halfPhase * _cfg.stepLengthMm * strideScale;
        // Forward CoM progression — applied magnitude this tick, faded.
        _state.fwdComProgressionMm = _fwdComProgressionMm(_halfPhase) * strideScale;

        if (elapsed >= GAIT_STOP_RAMP_MS / 2 && _ws && _lastShiftDir != ShiftDirection::NONE) {
            _ws->trigger(ShiftDirection::NONE);
            _lastShiftDir = ShiftDirection::NONE;
        }

        const bool wsSettled = (!_ws) ||
                               (fabsf(_ws->getState().progress) < 0.01f
                                && !_ws->getState().ramping);
        if (elapsed >= GAIT_STOP_RAMP_MS && wsSettled) {
            // Single canonical IDLE entry. Defer to start(GaitMode::IDLE) so the
            // FULL deterministic reset runs — touchdown latch, W4 compliance,
            // h-restore, heel-strike, trim suppression, every _state telemetry
            // field, and WeightShift centering — exactly as the recovery-
            // completion path already does (see _runRecoveryStep COMPLETE).
            //
            // This replaces a hand-rolled partial reset that cleared only a
            // subset of _state and left gait-transient latch/offset state
            // (_touchdownLocked, _hRestoreActive, _hRestoreDeficitMm, ...) set
            // on entry to IDLE — the root cause of the idle-pose leakage. With
            // one reset path, STOP from ANY phase or side converges to the same
            // idle pose. The swing leg has already been ramped to stanceHeight
            // by the strideScale-faded freeze/restore above, so this is a clean
            // hand-off with no h discontinuity.
            Serial.println("[GaitCtrl] STOP complete — entering IDLE.");
            start(GaitMode::IDLE);
        }
        return;
    }

    // ── POSE_STEP: if currently paused, re-submit lift to hold pose ───────────
    if (_state.mode == GaitMode::POSE_STEP && _poseHoldPt > 0) {
        if (_subState == StepSubState::SWINGING) {
            const float liftFrac = _swingFrac(_halfPhase);
            const bool leftIsSwing = (_swingSide == Side::LEFT);

            const FootTarget tL = _buildFootTarget(/*isRight=*/false, liftFrac,
                                                    /*isSwing=*/  leftIsSwing,
                                                    _halfPhase, bc);
            const FootTarget tR = _buildFootTarget(/*isRight=*/true,  liftFrac,
                                                    /*isSwing=*/ !leftIsSwing,
                                                    _halfPhase, bc);

            const LegIKResult rL = _submitFootTargetIK(false, tL);
            const LegIKResult rR = _submitFootTargetIK(true,  tR);

            _state.lastIKStatusL = static_cast<uint8_t>(rL.status);
            _state.lastIKStatusR = static_cast<uint8_t>(rR.status);
            _state.lastReachPctL = rL.sagittal_reach_pct;
            _state.lastReachPctR = rR.sagittal_reach_pct;
            _state.lastFlatFootErrL_deg = rL.flat_foot_error_deg;
            _state.lastFlatFootErrR_deg = rR.flat_foot_error_deg;

            // Repurposed UI fields: hip pitch produced by IK on the swing leg.
            // (Renaming to liftL_hipDeg in Stage 2 alongside the UI.)
            _state.liftL_deg = leftIsSwing  ? rL.hip_pitch_deg : 0.0f;
            _state.liftR_deg = !leftIsSwing ? rR.hip_pitch_deg : 0.0f;
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
                _swingSide   = otherSide(_swingSide);
                _triggerShiftForSwing(_swingSide);
            }
        }
        return;
    }

    // ── RUNNING / SINGLE_STEP / POSE_STEP: weight-shift-gated stepping ────────

    // STABILIZING is entered by _advanceHalf() between half-cycles. Hold pose
    // (no submits, no phase advance) until _cfg.stabilizeMs has elapsed, then
    // arm WAIT_SHIFT and trigger the new half's shift. This separates the prior
    // shift's mechanical settling from the next shift's command edge so the two
    // do not superimpose into a destabilising transient.
    if (_subState == StepSubState::STABILIZING) {
        if (millis() - _stabilizeStartMs >= _cfg.stabilizeMs) {
            _subState         = StepSubState::WAIT_SHIFT;
            _state.subState   = _subState;
            _waitShiftStartMs = millis();
            _gateOkSinceMs    = 0;
            _triggerShiftForSwing(_swingSide);
            Serial.printf("[GaitCtrl] STABILIZE done → WAIT_SHIFT swing=%s\n",
                          sideName(_swingSide));
        }
        return;
    }

    if (_subState == StepSubState::WAIT_SHIFT) {
        const uint32_t now = millis();
        // Gate opened — begin the swing arc from zero for a clean sin profile.
        // Always update elapsed telemetry, regardless of gate state. The UI
        // shows this so the operator can see how long transfer has been
        // attempted -- useful during tuning even when dwell/timeout are off.
        _state.waitShiftElapsedMs = now - _waitShiftStartMs;

        // Dwell tracking. Gate must remain continuously OK for liftDwellMs
        // before the transition fires. Any tick where liftGateOK goes false
        // resets the counter -- there is no "credit" carried across gaps.
        // liftDwellMs == 0 reduces to current single-tick behavior because
        // the (now - _gateOkSinceMs >= 0) check passes immediately.
        if (_state.liftGateOK) {
            if (_gateOkSinceMs == 0) _gateOkSinceMs = now;
            _state.gateDwellMs = now - _gateOkSinceMs;
        } else {
            _gateOkSinceMs     = 0;
            _state.gateDwellMs = 0;
        }

        // Timeout check. Aborts to IDLE if WAIT_SHIFT has exceeded its budget
        // without producing a confirmed gate. waitShiftMaxMs == 0 disables
        // the timeout entirely (pre-Stage-2 indefinite-wait behavior).
        // Disabled in POSE_STEP -- the operator is in the loop and may be
        // taking arbitrary time between pose advances.
        const bool timeoutEnabled = (_cfg.waitShiftMaxMs > 0)
                                    && (_state.mode != GaitMode::POSE_STEP);
        if (timeoutEnabled && _state.waitShiftElapsedMs >= _cfg.waitShiftMaxMs) {
            Serial.printf("[GaitCtrl] WAIT_SHIFT TIMEOUT swing=%s  elapsed=%lu ms  "
                          "ws=%.3f  flat=%d loaded=%d imuRoll=%d â€” aborting to IDLE\n",
                          sideName(_swingSide),
                          (unsigned long)_state.waitShiftElapsedMs,
                          wsProgress,
                          (int)_state.gateStanceFlat,
                          (int)_state.gateStanceLoaded,
                          (int)_state.gateImuRollOK);
            _state.waitShiftTimedOut = true;   // one-tick edge for UI
            stop();                            // clean ramp to IDLE
            return;
        }

        // Gate not yet held long enough: continue waiting.
        const bool dwellSatisfied = (_gateOkSinceMs > 0)
                                    && (now - _gateOkSinceMs >= _cfg.liftDwellMs);
        if (!dwellSatisfied) return;

        // Dwell satisfied — commit to swing.
        _subState  = StepSubState::SWINGING;
        _halfPhase = 0.0f;
        _state.subState = _subState;
        _state.waitShiftTimedOut = false;
        _swingStartMs = millis();   // step-timing: lift-off commit timestamp
        Serial.printf("[GaitCtrl] Gate open  swing=%s  ws=%.3f  roll=%.4f rad  "
                      "dwell=%lu ms  elapsed=%lu ms\n",
                      sideName(_swingSide), wsProgress, imuState.roll,
                      (unsigned long)_state.gateDwellMs,
                      (unsigned long)_state.waitShiftElapsedMs);

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
    const bool leftIsSwing = (_swingSide == Side::LEFT);

    // Build BOTH foot targets every tick. Stance leg also goes through IK
    // so it holds its commanded pose against priority arbitration.
    const FootTarget tL = _buildFootTarget(/*isRight=*/false, liftFrac,
                                            /*isSwing=*/  leftIsSwing,
                                            _halfPhase, bc);
    const FootTarget tR = _buildFootTarget(/*isRight=*/true,  liftFrac,
                                            /*isSwing=*/ !leftIsSwing,
                                            _halfPhase, bc);

    const LegIKResult rL = _submitFootTargetIK(false, tL);
    const LegIKResult rR = _submitFootTargetIK(true,  tR);

    _state.lastIKStatusL = static_cast<uint8_t>(rL.status);
    _state.lastIKStatusR = static_cast<uint8_t>(rR.status);
    _state.lastReachPctL = rL.sagittal_reach_pct;
    _state.lastReachPctR = rR.sagittal_reach_pct;
    _state.lastFlatFootErrL_deg = rL.flat_foot_error_deg;
    _state.lastFlatFootErrR_deg = rR.flat_foot_error_deg;

    // Stage 3 telemetry: estimated body translation this half-cycle.
    // _halfPhase × stepLengthMm. Resets to 0 in _advanceHalf().
    _state.cycleXDispl_mm = _halfPhase * _cfg.stepLengthMm;
    // Stage 2.2 — broadcast the capture offset that was applied this tick.
    // Reads as 0 when capture disabled or vx_CoM ≈ 0.
    _state.captureOffsetMm = _captureOffsetMm();
    // Pitch-ANGLE placement — applied sagittal value this tick (mm). Reads 0
    // when pitchPlacement disabled, gain=0 (default), IMU invalid, or pitch
    // inside the deadband. strideScale = 1.0 in SWINGING, so the raw helper
    // output is the applied value (mirrors captureOffsetMm above).
    _state.pitchPlacementMm = _pitchPlacementOffsetMm(_halfPhase);
    // Lateral capture telemetry — applied world-frame signed value this tick
    // (post-clamp, post-ramp, post-freeze; pre-leg-sign — that flip happens
    // at the foot in _buildFootTarget). Reads 0 when latCapture disabled,
    // IMU invalid, or roll rate ≈ 0. strideScale = 1.0 in SWINGING.
    _state.latCaptureMm = _latCaptureOffsetMm(_halfPhase);
    // Forward CoM progression — applied magnitude this tick (mm). 0 when
    // fwdComMaxMm = 0 (default), IDLE, or estopped. strideScale = 1.0 in
    // SWINGING so the raw helper output is the applied value.
    _state.fwdComProgressionMm = _fwdComProgressionMm(_halfPhase);

    // Repurposed UI fields: hip pitch produced by IK on the swing leg.
    _state.liftL_deg = leftIsSwing  ? rL.hip_pitch_deg : 0.0f;
    _state.liftR_deg = !leftIsSwing ? rR.hip_pitch_deg : 0.0f;

    if (_cfg.phaseRateHz > 0.0f) {
        const float halfPeriod_s = 1.0f / (2.0f * _cfg.phaseRateHz);
        const float newPhase     = _halfPhase + dt_s / halfPeriod_s;

        // POSE_STEP: pause at peak (halfPhase first crosses 0.5).
        if (_state.mode == GaitMode::POSE_STEP &&
            _halfPhase < 0.5f && newPhase >= 0.5f) {
            _halfPhase      = 0.5f;
            _poseHoldPt     = 2;
            _state.poseHold = 2;
            _state.phase    = static_cast<uint8_t>(_swingSide) * 0.5f + _halfPhase * 0.5f;
            Serial.println("[GaitCtrl] POSE_STEP: hold=2 peak");
            return;
        }

        _halfPhase = newPhase;
    }

    if (_halfPhase >= 1.0f) {
        // POSE_STEP: pause at landing before advancing to next half-cycle.
        // Timing is intentionally distorted by the operator hold, so step-
        // timing metrics are skipped on this path.
        if (_state.mode == GaitMode::POSE_STEP) {
            _halfPhase      = 0.999f;  // hold near end; nextPose() will advance
            _poseHoldPt     = 3;
            _state.poseHold = 3;
            _state.phase    = static_cast<uint8_t>(_swingSide) * 0.5f + _halfPhase * 0.5f;
            Serial.println("[GaitCtrl] POSE_STEP: hold=3 landing");
            return;
        }

        // ── Step-timing capture ───────────────────────────────────────────
        // Captured here, before _advanceHalf() flips _swingSide and clears
        // the touchdown latch, so all timestamps still refer to THIS swing.
        {
            const uint32_t now = millis();

            // Unsigned subtraction is wrap-safe (both are millis() values).
            const uint32_t stepPeriod = (_lastFootfallMs != 0)
                                        ? (now - _lastFootfallMs) : 0;
            const uint32_t swingDur   = (_swingStartMs != 0)
                                        ? (now - _swingStartMs) : 0;
            // waitMs: from WAIT_SHIFT entry to lift-off commit.
            const uint32_t waitDur    = (_swingStartMs != 0 && _waitShiftStartMs != 0)
                                        ? (_swingStartMs - _waitShiftStartMs) : 0;
            // stabMs: from STABILIZE entry to WAIT_SHIFT entry; 0 on first swing.
            const uint32_t stabDur    = (_stabilizeStartMs != 0)
                                        ? (_waitShiftStartMs - _stabilizeStartMs) : 0;
            // postContactMs: time the foot was planted while phase still ran to 1.0.
            const uint32_t postCtDur  = (_touchdownLocked && _touchdownMs != 0)
                                        ? (now - _touchdownMs) : 0;

            _state.stepPeriodMs    = stepPeriod;
            _state.swingMs         = swingDur;
            _state.waitMs          = waitDur;
            _state.stabilizeMsMeas = stabDur;
            _state.postContactMs   = postCtDur;
            _state.lastStepSide    = static_cast<uint8_t>(_swingSide);

            if (stepPeriod > 0) {
                _state.cadenceHz       = 1000.0f / (2.0f * static_cast<float>(stepPeriod));
                _state.cadenceStepsMin = 60000.0f / static_cast<float>(stepPeriod);

                // Ring buffer — overwrite oldest when full.
                _stepRingPeriodMs[_stepRingHead] = stepPeriod;
                _stepRingSide[_stepRingHead]     = static_cast<uint8_t>(_swingSide);
                _stepRingSwingMs[_stepRingHead]  = swingDur;
                _stepRingHead = (_stepRingHead + 1) % STEP_TIMING_RING_SIZE;
                if (_stepRingCount < STEP_TIMING_RING_SIZE) ++_stepRingCount;
            } else {
                _state.cadenceHz       = 0.0f;
                _state.cadenceStepsMin = 0.0f;
            }

            // Rolling means. Iterate newest-first so the per-side sums stay
            // correct regardless of which slot is oldest. Uses int arithmetic
            // for the index to avoid uint8_t underflow.
            {
                float sumP = 0.0f, sumPL = 0.0f, sumPR = 0.0f;
                float sumSL = 0.0f, sumSR = 0.0f;
                uint8_t nL = 0, nR = 0;
                for (uint8_t i = 0; i < _stepRingCount; ++i) {
                    const uint8_t idx = (uint8_t)(
                        ((int)_stepRingHead - 1 - (int)i + STEP_TIMING_RING_SIZE)
                        % STEP_TIMING_RING_SIZE);
                    sumP += static_cast<float>(_stepRingPeriodMs[idx]);
                    if (_stepRingSide[idx] == static_cast<uint8_t>(Side::LEFT)) {
                        sumPL += static_cast<float>(_stepRingPeriodMs[idx]);
                        sumSL += static_cast<float>(_stepRingSwingMs[idx]);
                        ++nL;
                    } else {
                        sumPR += static_cast<float>(_stepRingPeriodMs[idx]);
                        sumSR += static_cast<float>(_stepRingSwingMs[idx]);
                        ++nR;
                    }
                }
                const float cnt = static_cast<float>(_stepRingCount);
                _state.meanStepPeriodMs = (cnt > 0.0f) ? (sumP / cnt) : 0.0f;
                _state.meanCadenceHz    = (_state.meanStepPeriodMs > 0.0f)
                                          ? (1000.0f / (2.0f * _state.meanStepPeriodMs))
                                          : 0.0f;
                _state.meanStepPeriodL  = (nL > 0) ? (sumPL / static_cast<float>(nL)) : 0.0f;
                _state.meanStepPeriodR  = (nR > 0) ? (sumPR / static_cast<float>(nR)) : 0.0f;
                _state.meanSwingMsL     = (nL > 0) ? (sumSL / static_cast<float>(nL)) : 0.0f;
                _state.meanSwingMsR     = (nR > 0) ? (sumSR / static_cast<float>(nR)) : 0.0f;
            }

            _lastFootfallMs = now;
        }
        // ─────────────────────────────────────────────────────────────────

        _halfPhase = 0.0f;
        _advanceHalf();
    }
}

// ---------------------------------------------------------------------------
//  triggerRecovery  —  arm a 2D directional catch step (Stage 2).
// ---------------------------------------------------------------------------
void GaitController::triggerRecovery(float tilt_pitch_rad, float tilt_roll_rad) {
    // Refractory lockout: ignore triggers shortly after the previous step completed.
    if (millis() < _rcvLockoutUntilMs) {
        Serial.printf("[GaitCtrl] RECOVERY blocked — lockout %u ms remaining\n",
                      (unsigned)(_rcvLockoutUntilMs - millis()));
        return;
    }
    if (oe_is_estopped())                        return;
    if (_state.mode == GaitMode::RECOVERY_STEP)  return;  // no re-trigger mid-step

    // Combined disturbance magnitude (rad).
    const float tilt_mag = sqrtf(tilt_pitch_rad * tilt_pitch_rad
                                + tilt_roll_rad  * tilt_roll_rad);

    // Direction unit vector (±1 for telemetry).
    _rcvDirX = (tilt_pitch_rad > 0.0f) ? +1 : (tilt_pitch_rad < 0.0f ? -1 : 0);
    _rcvDirY = (tilt_roll_rad  > 0.0f) ? +1 : (tilt_roll_rad  < 0.0f ? -1 : 0);

    // Swing leg = leg on the side body is falling toward.
    _rcvSwingSide = (tilt_roll_rad >= 0.0f) ? Side::RIGHT : Side::LEFT;

    // Step magnitudes proportional to per-axis tilt component, capped.
    // Sign of lateral recovery is expressed by the swing leg selection, not
    // by the y_mm magnitude. Use absolute lateral magnitude so left & right
    // outward steps are both positive in the swing leg frontal frame.
    const float gain = RECOVERY_MAGNITUDE_GAIN_MM_PER_RAD;
    _rcvStepX_mm = constrain(tilt_pitch_rad * gain,
                              -RECOVERY_STEP_FORWARD_MAX_MM,
                              +RECOVERY_STEP_FORWARD_MAX_MM);
    _rcvStepY_mm = constrain(fabsf(tilt_roll_rad) * gain,
                              -RECOVERY_STEP_LATERAL_MAX_MM,
                              +RECOVERY_STEP_LATERAL_MAX_MM);

    // Apply minimum magnitude floor to dominant axis only.
    if (fabsf(_rcvStepX_mm) > fabsf(_rcvStepY_mm)) {
        if (fabsf(_rcvStepX_mm) < RECOVERY_STEP_MIN_MAGNITUDE_MM) {
            _rcvStepX_mm = copysignf(RECOVERY_STEP_MIN_MAGNITUDE_MM, _rcvStepX_mm);
        }
    } else {
        if (fabsf(_rcvStepY_mm) < RECOVERY_STEP_MIN_MAGNITUDE_MM) {
            _rcvStepY_mm = copysignf(RECOVERY_STEP_MIN_MAGNITUDE_MM, _rcvStepY_mm);
        }
    }

    // Mirror into telemetry state.
    _state.rcvStepX_mm  = _rcvStepX_mm;
    _state.rcvStepY_mm  = _rcvStepY_mm;
    _state.rcvMagnitude = sqrtf(_rcvStepX_mm * _rcvStepX_mm
                               + _rcvStepY_mm * _rcvStepY_mm);

    // Clear any residual gait or weight-shift activity.
    if (_ws) _ws->trigger(ShiftDirection::NONE);
    _lastShiftDir    = ShiftDirection::NONE;
    _halfPhase       = 0.0f;
    _swingSide       = Side::LEFT;
    _poseHoldPt      = 0;
    _state.poseHold  = 0;

    // Reset leveler IIR state on recovery entry (Risk #9): recovery uses fast
    // smoothstep trajectories; the leveler would fight them if left armed.
    if (_swingLeveler) _swingLeveler->reset();

    // Recovery uses its own trajectory and never reads the touchdown latch.
    // Clear it on entry so a residual lock from the previous gait swing does
    // not leak telemetry into the recovery panel.
    _clearTouchdownLatch();
    // Recovery sets h_sagittal_mm directly in _runRecoveryStep() — it never
    // calls _buildFootTarget — so the compliance offset would not affect the
    // trajectory anyway. Reset it on entry purely so the telemetry panel does
    // not show a stale decay carried over from the interrupted gait swing.
    _resetCompliance();
    _resetRolloverTelemetry();   // telemetry only — clear stale rollover window
    // Recovery sets h_sagittal_mm directly and must not inherit a stale h-
    // restore armed by the interrupted gait swing — clear it on entry.
    _resetHRestore();
    // Recovery uses a separate trajectory (open-loop directional catch step),
    // not the engage/release model. Wipe heel-strike state so a release in
    // progress when the disturbance hit does not leak into the recovery ankle.
    _resetHeelStrike();

    _state.mode    = GaitMode::RECOVERY_STEP;
    _state.running = true;
    _enterRcvSub(RecoverySubState::TRIGGERED);

    Serial.printf("[GaitCtrl] RECOVERY triggered  pitch=%.3f roll=%.3f mag=%.3f  "
                  "step=(%.1f,%.1f)mm  swing=%s\n",
                  tilt_pitch_rad, tilt_roll_rad, tilt_mag,
                  _rcvStepX_mm, _rcvStepY_mm,
                  sideName(_rcvSwingSide));
}

// ---------------------------------------------------------------------------
//  tareToTorso  --  capture current imuState.pitch/roll as the neutral.
//
//  Rejected outside IDLE so the tare cannot capture transient pose during
//  a gait cycle or recovery. Rejected when IMU is invalid so the tare
//  cannot capture pre-calibration garbage. Both rejections log a clear
//  reason; the WebSocket response surfaces the success bit to the UI.
// ---------------------------------------------------------------------------
bool GaitController::tareToTorso(const IMUState& imuState) {
    if (_state.mode != GaitMode::IDLE) {
        Serial.printf("[GaitCtrl] TORSO_TARE rejected: mode=%d (must be IDLE)\n",
                      static_cast<int>(_state.mode));
        return false;
    }
    if (!imuState.valid) {
        Serial.println("[GaitCtrl] TORSO_TARE rejected: IMU not valid");
        return false;
    }
    _cfg.torsoRollNeutralRad  = imuState.roll;
    _cfg.torsoPitchNeutralRad = imuState.pitch;
    Serial.printf("[GaitCtrl] TORSO_TARE: pitch=%.4f rad  roll=%.4f rad\n",
                  _cfg.torsoPitchNeutralRad, _cfg.torsoRollNeutralRad);
    return true;
}

// NOTE: the gait-local neutral trim was removed in the 2026-05-20 migration.
// Servo-neutral drift is now compensated globally by JointModel's runtime
// session trim (see JointModel::captureSessionNeutral) — gait submits raw IK
// angles and inherits the correction at the joint↔servo conversion boundary.

// ---------------------------------------------------------------------------
void GaitController::_enterRcvSub(RecoverySubState s) {
    _rcvSub              = s;
    _state.recoverySub   = s;
    _rcvSubStartMs       = millis();
}

// ---------------------------------------------------------------------------
//  _runRecoveryStep  —  open-loop catch-step state machine (Stage 2).
//
//  Sub-state sequence:
//    TRIGGERED → LIFTING → LANDING → DAMP_HOLD → SOFT_RETURN → COMPLETE
//  Each timed stage submits FootTargets through _submitFootTargetIK() so
//  MotionManager priority arbitration is respected. Balance controller's
//  SOURCE_BALANCE submissions still override SOURCE_GAIT on shared joints.
//
//  swingTargetY_mm = _rcvStepY_mm (signed per-trigger). T2.1 on hardware must
//  verify all four directions; fix any sign errors in triggerRecovery().
// ---------------------------------------------------------------------------
void GaitController::_runRecoveryStep(float dt_s, const IMUState& imuState) {
    const uint32_t elapsed = millis() - _rcvSubStartMs;
    _state.recoveryElapsedMs = elapsed;
    _state.recoveryDirX      = _rcvDirX;
    _state.recoveryDirY      = _rcvDirY;

    // Phase 4 — collapse the per-leg telemetry triplet (status / reachPct /
    // flatFootErr) into one helper so the swing/stance ternaries don't have to
    // be repeated three times below. Captures _state by reference.
    auto writeLegTelemetry = [&](Side side, const LegIKResult& r) {
        const uint8_t status = static_cast<uint8_t>(r.status);
        if (side == Side::LEFT) {
            _state.lastIKStatusL        = status;
            _state.lastReachPctL        = r.sagittal_reach_pct;
            _state.lastFlatFootErrL_deg = r.flat_foot_error_deg;
        } else {
            _state.lastIKStatusR        = status;
            _state.lastReachPctR        = r.sagittal_reach_pct;
            _state.lastFlatFootErrR_deg = r.flat_foot_error_deg;
        }
    };
    const Side swingSide  = _rcvSwingSide;
    const Side stanceSide = otherSide(swingSide);

    // Step targets set at trigger time. y_mm sign convention: positive = outward
    // in the swing leg's frontal frame (same as Stage 1 +30 mm). For right-swing
    // this is rightward; for left-swing leftward. The IK and JointConfig direction
    // fields handle the per-leg mapping. T2.1 must verify all four directions on
    // hardware; if a sign is wrong, fix it in triggerRecovery(), not here.
    const float swingTargetX_mm = _rcvStepX_mm;               // sagittal (signed: + = forward)
    const float swingTargetY_mm = _rcvStepY_mm;               // lateral  (signed: + = outward for swing leg)
    const bool  sagittalDominant = fabsf(_rcvStepX_mm) > fabsf(_rcvStepY_mm);
    const float effectiveStepHeight_mm = sagittalDominant
        ? RECOVERY_STEP_HEIGHT_SAGITTAL_MM
        : RECOVERY_STEP_HEIGHT_LATERAL_MM;

    switch (_rcvSub) {

        case RecoverySubState::TRIGGERED:
            _enterRcvSub(RecoverySubState::PRE_LIFT);
            break;

        case RecoverySubState::PRE_LIFT: {
            const float t        = 1.0f;
            const float liftFrac = 0.0f;

            FootTarget tSwing;
            tSwing.isRightLeg    = isRight(swingSide);
            tSwing.x_mm          = GAIT_STANCE_X_OFFSET_MM;
            tSwing.y_mm          = 0.0f;
            tSwing.h_sagittal_mm = _cfg.stanceHeightMm;
            tSwing.h_frontal_mm  = 0.0f;
            tSwing.valid         = true;

            FootTarget tStance;
            tStance.isRightLeg    = isRight(stanceSide);
            tStance.x_mm          = GAIT_STANCE_X_OFFSET_MM
                                   - swingTargetX_mm * RECOVERY_STANCE_COUNTERSHIFT_FRAC * t;
            // Positive y_mm = outward in the stance leg's own frontal frame.
            // Stance hip abducts AWAY from the fall, widening the base of support.
            const float baseWidenMm = fabsf(swingTargetX_mm) * 0.20f * t;
            tStance.y_mm          = fmaxf(+fabsf(swingTargetY_mm) * RECOVERY_STANCE_COUNTERSHIFT_FRAC * t,
                                         baseWidenMm);
            _state.rcvStanceY_mm  = tStance.y_mm;
            tStance.h_sagittal_mm = _cfg.stanceHeightMm;
            tStance.h_frontal_mm  = 0.0f;
            tStance.valid         = true;

            const LegIKResult rSwing  = _submitFootTargetIK(swingSide,  tSwing);
            const LegIKResult rStance = _submitFootTargetIK(stanceSide, tStance);
            writeLegTelemetry(swingSide,  rSwing);
            writeLegTelemetry(stanceSide, rStance);

            if (elapsed >= RECOVERY_PRE_LIFT_MS) {
                _enterRcvSub(RecoverySubState::LIFTING);
            }
            break;
        }

        case RecoverySubState::LIFTING: {
            const float t        = fminf(static_cast<float>(elapsed)
                                         / static_cast<float>(RECOVERY_LIFT_MS), 1.0f);
            // Smoothstep ascent: zero foot velocity at lift-off (t=0) AND at peak (t=1).
            // sin(t·π/2) had peak velocity at t=0, which slammed the foot off the ground —
            // only invisible at low servo speeds because the smooth-stepper clipped it.
            const float liftFrac = 3.0f * t * t - 2.0f * t * t * t;

            FootTarget tSwing;
            tSwing.isRightLeg    = isRight(swingSide);
            // Smoothstep on horizontal motion: zero velocity at t=0 (foot still grounded)
            // and t=1 (foot landed). Vertical lift uses sin*π/2 which has FULL velocity at
            // t=0 — together this means the foot rises before it moves forward, eliminating
            // the scrub mechanism. Cosmetic improvement at minimum, mechanical at best.
            const float t_xy = 3.0f * t * t - 2.0f * t * t * t;
            tSwing.x_mm = swingTargetX_mm * t_xy + GAIT_STANCE_X_OFFSET_MM;
            tSwing.y_mm = swingTargetY_mm * t_xy;
            tSwing.h_sagittal_mm = _cfg.stanceHeightMm - liftFrac * effectiveStepHeight_mm;
            tSwing.h_frontal_mm  = 0.0f;
            tSwing.valid         = (tSwing.h_sagittal_mm >= 20.0f);

            FootTarget tStance;
            tStance.isRightLeg    = isRight(stanceSide);
            // Counter-shift is fully established by end of PRE_LIFT. Holding it constant
            // here prevents the swing hip from being dragged backward during the lift,
            // which manifests as ground-scrubbing (the "shuffle"). The * t ramp was a
            // silent bug: it threw away PRE_LIFT's load transfer at every sub-state edge.
            tStance.x_mm          = GAIT_STANCE_X_OFFSET_MM
                                   - swingTargetX_mm * RECOVERY_STANCE_COUNTERSHIFT_FRAC;
            // Positive y_mm = outward in the stance leg's own frontal frame.
            // Stance hip abducts AWAY from the fall, widening the base of support.
            // Was: -swingTargetY * frac (pushed stance toward the fall — wrong).
            const float baseWidenMm = fabsf(swingTargetX_mm) * 0.20f;
            tStance.y_mm          = fmaxf(+fabsf(swingTargetY_mm) * RECOVERY_STANCE_COUNTERSHIFT_FRAC,
                                         baseWidenMm);
            _state.rcvStanceY_mm  = tStance.y_mm;
            tStance.h_sagittal_mm = _cfg.stanceHeightMm;
            tStance.h_frontal_mm  = 0.0f;
            tStance.valid         = true;

            const LegIKResult rSwing  = _submitFootTargetIK(swingSide,  tSwing);
            const LegIKResult rStance = _submitFootTargetIK(stanceSide, tStance);
            writeLegTelemetry(swingSide,  rSwing);
            writeLegTelemetry(stanceSide, rStance);

            if (elapsed >= RECOVERY_LIFT_MS) {
                _enterRcvSub(RecoverySubState::LANDING);
            }
            break;
        }

        case RecoverySubState::LANDING: {
            const float t        = fminf(static_cast<float>(elapsed)
                                         / static_cast<float>(RECOVERY_LAND_MS), 1.0f);
            // Smoothstep descent: zero foot velocity at peak (t=0) AND at touchdown (t=1).
            // cos(t·π/2) had peak velocity at touchdown — the literal cause of the high-
            // speed tipping. The body was experiencing a 250 mm/s vertical foot impulse
            // at ground contact instead of a smoothed deceleration.
            const float liftFrac = 1.0f - (3.0f * t * t - 2.0f * t * t * t);

            FootTarget tSwing;
            tSwing.isRightLeg    = isRight(swingSide);
            tSwing.x_mm          = swingTargetX_mm + GAIT_STANCE_X_OFFSET_MM;
            tSwing.y_mm          = swingTargetY_mm;
            tSwing.h_sagittal_mm = _cfg.stanceHeightMm - liftFrac * effectiveStepHeight_mm;
            tSwing.h_frontal_mm  = 0.0f;
            tSwing.valid         = (tSwing.h_sagittal_mm >= 20.0f);

            FootTarget tStance;
            tStance.isRightLeg    = isRight(stanceSide);
            // Counter-shift is fully established by end of PRE_LIFT. Holding it constant
            // here prevents the swing hip from being dragged backward during the lift,
            // which manifests as ground-scrubbing (the "shuffle"). The * t ramp was a
            // silent bug: it threw away PRE_LIFT's load transfer at every sub-state edge.
            tStance.x_mm          = GAIT_STANCE_X_OFFSET_MM
                                   - swingTargetX_mm * RECOVERY_STANCE_COUNTERSHIFT_FRAC;
            const float baseWidenMm = fabsf(swingTargetX_mm) * 0.20f;
            tStance.y_mm          = fmaxf(+fabsf(swingTargetY_mm) * RECOVERY_STANCE_COUNTERSHIFT_FRAC,
                                         baseWidenMm);
            _state.rcvStanceY_mm  = tStance.y_mm;
            tStance.h_sagittal_mm = _cfg.stanceHeightMm;
            tStance.h_frontal_mm  = 0.0f;
            tStance.valid         = true;

            const LegIKResult rSwing  = _submitFootTargetIK(swingSide,  tSwing);
            const LegIKResult rStance = _submitFootTargetIK(stanceSide, tStance);
            writeLegTelemetry(swingSide,  rSwing);
            writeLegTelemetry(stanceSide, rStance);

            if (elapsed >= RECOVERY_LAND_MS) {
                _enterRcvSub(RecoverySubState::DAMP_HOLD);
            }
            break;
        }

        case RecoverySubState::DAMP_HOLD: {
            // Hold both feet at planted positions briefly so structural transients
            // damp before the foot starts returning to centered stance.
            FootTarget tSwing;
            tSwing.isRightLeg    = isRight(swingSide);
            tSwing.x_mm          = swingTargetX_mm + GAIT_STANCE_X_OFFSET_MM;
            tSwing.y_mm          = swingTargetY_mm;
            tSwing.h_sagittal_mm = _cfg.stanceHeightMm;
            tSwing.h_frontal_mm  = 0.0f;
            tSwing.valid         = true;

            FootTarget tStance;
            tStance.isRightLeg    = isRight(stanceSide);
            tStance.x_mm          = GAIT_STANCE_X_OFFSET_MM
                                   - swingTargetX_mm * RECOVERY_STANCE_COUNTERSHIFT_FRAC;
            const float baseWidenMm = fabsf(swingTargetX_mm) * 0.20f;
            tStance.y_mm          = fmaxf(+fabsf(swingTargetY_mm) * RECOVERY_STANCE_COUNTERSHIFT_FRAC,
                                         baseWidenMm);
            _state.rcvStanceY_mm  = tStance.y_mm;
            tStance.h_sagittal_mm = _cfg.stanceHeightMm;
            tStance.h_frontal_mm  = 0.0f;
            tStance.valid         = true;

            _submitFootTargetIK(swingSide,  tSwing);
            _submitFootTargetIK(stanceSide, tStance);

            if (elapsed >= RECOVERY_DAMP_HOLD_MS) {
                _enterRcvSub(RecoverySubState::SOFT_RETURN);
            }
            break;
        }

        case RecoverySubState::SOFT_RETURN: {
            // Smoothstep decay: foot offset decays 1.0 → 0.0 over RECOVERY_SOFT_RETURN_MS.
            const float t     = fminf(static_cast<float>(elapsed)
                                      / static_cast<float>(RECOVERY_SOFT_RETURN_MS), 1.0f);
            const float decay = 1.0f - (3.0f * t * t - 2.0f * t * t * t);

            FootTarget tSwing;
            tSwing.isRightLeg    = isRight(swingSide);
            tSwing.x_mm          = swingTargetX_mm * decay + GAIT_STANCE_X_OFFSET_MM;
            tSwing.y_mm          = swingTargetY_mm * decay;
            tSwing.h_sagittal_mm = _cfg.stanceHeightMm;
            tSwing.h_frontal_mm  = 0.0f;
            tSwing.valid         = true;

            FootTarget tStance;
            tStance.isRightLeg    = isRight(stanceSide);
            tStance.x_mm          = GAIT_STANCE_X_OFFSET_MM
                                   - swingTargetX_mm * RECOVERY_STANCE_COUNTERSHIFT_FRAC * decay;
            const float baseWidenMm = fabsf(swingTargetX_mm) * 0.20f * decay;
            tStance.y_mm          = fmaxf(+fabsf(swingTargetY_mm) * RECOVERY_STANCE_COUNTERSHIFT_FRAC * decay,
                                         baseWidenMm);
            _state.rcvStanceY_mm  = tStance.y_mm;
            tStance.h_sagittal_mm = _cfg.stanceHeightMm;
            tStance.h_frontal_mm  = 0.0f;
            tStance.valid         = true;

            _submitFootTargetIK(swingSide,  tSwing);
            _submitFootTargetIK(stanceSide, tStance);

            if (elapsed >= RECOVERY_SOFT_RETURN_MS) {
                _enterRcvSub(RecoverySubState::COMPLETE);
            }
            break;
        }

        case RecoverySubState::COMPLETE: {
            // Chain detection: if the body is still tilted after landing, a single
            // step was insufficient (large disturbance or marginal catch). Re-trigger
            // immediately rather than entering the refractory lockout, so the robot
            // takes a second step while it still has time to recover.
            //
            // start(IDLE) must come BEFORE triggerRecovery() because triggerRecovery()
            // guards against mode == RECOVERY_STEP. start(IDLE) clears that flag.
            const bool stillTilted =
                (fabsf(imuState.roll)  > RECOVERY_CHAIN_TILT_THRESHOLD_RAD) ||
                (fabsf(imuState.pitch) > RECOVERY_CHAIN_TILT_THRESHOLD_RAD);
            start(GaitMode::IDLE);  // clears recovery state, centers WeightShift
            if (stillTilted) {
                Serial.printf("[GaitCtrl] RECOVERY CHAIN — residual tilt "
                              "roll=%.3f pitch=%.3f — stepping again.\n",
                              imuState.roll, imuState.pitch);
                triggerRecovery(imuState.pitch, imuState.roll);
                // No lockout: triggerRecovery() above is the next step.
            } else {
                _rcvLockoutUntilMs = millis() + RECOVERY_LOCKOUT_MS;
                Serial.printf("[GaitCtrl] RECOVERY complete — lockout %u ms.\n",
                              (unsigned)RECOVERY_LOCKOUT_MS);
            }
            break;
        }
    }
}

// ---------------------------------------------------------------------------
//  _publishSwingLegMaskToWS  -  inform WeightShift which leg is airborne.
//
//  Called once at the start of update(). WeightShift consumes the published
//  hint on its NEXT tick (1 tick / 2.5 ms staleness - same as the previous
//  cross-module mask design). Always publishes - sticky semantics in WS would
//  otherwise leave stale state on early-return paths.
// ---------------------------------------------------------------------------
void GaitController::_publishSwingLegMaskToWS() const {
    if (!_ws) return;

    if (oe_is_estopped()) {
        _ws->setSwingLegMask(SwingLeg::NONE, 0.0f);
        return;
    }

    if (_state.mode == GaitMode::IDLE || _state.mode == GaitMode::WSHIFT_ONLY) {
        _ws->setSwingLegMask(SwingLeg::NONE, 0.0f);
        return;
    }

    if (_state.mode == GaitMode::RECOVERY_STEP) {
        // PRE_LIFT and SOFT_RETURN keep full bias (foot grounded). LIFTING /
        // LANDING / DAMP_HOLD are airborne -> liftFrac = 1.
        const bool airborne = (_rcvSub == RecoverySubState::LIFTING)
                           || (_rcvSub == RecoverySubState::LANDING)
                           || (_rcvSub == RecoverySubState::DAMP_HOLD);
        const SwingLeg swing = (_rcvSwingSide == Side::RIGHT) ? SwingLeg::RIGHT : SwingLeg::LEFT;
        _ws->setSwingLegMask(swing, airborne ? 1.0f : 0.0f);
        return;
    }

    if (_subState == StepSubState::SWINGING) {
        const SwingLeg swing = (_swingSide == Side::LEFT) ? SwingLeg::LEFT : SwingLeg::RIGHT;
        _ws->setSwingLegMask(swing, _swingFrac(_halfPhase));
        return;
    }

    if (_subState == StepSubState::STOPPING) {
        const uint32_t elapsed  = millis() - _stopStartMs;
        const float    t        = (elapsed >= GAIT_STOP_RAMP_MS)
                                 ? 1.0f
                                 : (float)elapsed / (float)GAIT_STOP_RAMP_MS;
        const float    smooth   = t * t * (3.0f - 2.0f * t);
        const float    liftRamp = 1.0f - smooth;
        const SwingLeg  swing   = (_stopSwingSide == Side::LEFT) ? SwingLeg::LEFT : SwingLeg::RIGHT;
        _ws->setSwingLegMask(swing, _stopSwingFrac * liftRamp);
        return;
    }

    // WAIT_SHIFT, STABILIZING: foot grounded - full bias both legs.
    _ws->setSwingLegMask(SwingLeg::NONE, 0.0f);
}
