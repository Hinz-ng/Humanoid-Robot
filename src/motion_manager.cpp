// =============================================================================
//  motion_manager.cpp
//  Purpose : Joint authority arbitration layer.
//
//  TICK LIFECYCLE:
//    submit() ×N  →  flush() ×1  →  slots cleared
//
//  submit() writes to _slots[channel] only when the incoming priority is
//  >= the existing slot's priority, so high-priority sources always win
//  regardless of submission order within a tick.
//
//  flush() calls the appropriate ServoControl method based on the source,
//  then zeroes every slot to prepare for the next tick.
// =============================================================================

#include "motion_manager.h"
#include "oe_control.h"   // oe_is_estopped() — gate prevents stale commands reaching hardware
// ---------------------------------------------------------------------------
MotionManager::MotionManager()
    : _servo(nullptr)
{
    // Zero-initialise all slots. occupied=false means no submission yet.
    // This is safe to call at global-scope construction time (no hardware access).
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        _slots[i] = {};   // aggregate zero-init: occupied=false, angleDeg=0
    }
}

// ---------------------------------------------------------------------------
void MotionManager::init(ServoControl* servo) {
    if (servo == nullptr) {
        // Fail fast: a null gateway means flush() would silently do nothing.
        // Surface this immediately rather than causing mysterious silent failures.
        Serial.println("[MotionManager] ERROR: init() received null ServoControl pointer!");
        return;
    }
    _servo = servo;
    Serial.printf("[MotionManager] Initialized. Managing %d joints. Priority levels: "
                  "UI=%d  GAIT=%d  BALANCE=%d\n",
                  NUM_JOINTS,
                  (int)SOURCE_UI, (int)SOURCE_GAIT, (int)SOURCE_BALANCE);
}

// ---------------------------------------------------------------------------
//  submit() — register a desired joint angle for this tick.
// ---------------------------------------------------------------------------
void MotionManager::submit(MotionSource src, uint8_t channel, float angleDeg) {
    // Guard: ignore out-of-range channels rather than writing to garbage memory.
    if (channel >= NUM_JOINTS) {
        Serial.printf("[MotionManager] WARNING: submit() channel=%d out of range "
                      "(max=%d). Dropped.\n", channel, (int)NUM_JOINTS - 1);
        return;
    }

    uint8_t      incomingPriority = (uint8_t)src;
    MotionCommand& slot           = _slots[channel];

    // Accept if the slot is empty OR the new command has equal/higher priority.
    // "Equal" allows the last same-priority submit to win (e.g. two UI events
    // for the same joint in one tick — take the most recent one).
    if (!slot.occupied || incomingPriority >= slot.priority) {
        slot.occupied = true;
        slot.source   = src;
        slot.priority = incomingPriority;
        slot.angleDeg = angleDeg;
    }
    // Lower-priority submission → silently discard. No log needed; this is
    // the normal, expected case when balance is active and UI also submits.
}

// ---------------------------------------------------------------------------
//  flush() — apply all pending commands to hardware, then clear slots.
// ---------------------------------------------------------------------------
void MotionManager::flush() {
    if (_servo == nullptr) return;

    // Drain the slot table while estopped — discard every pending command.
    // Rationale: commands submitted during estop are contextually stale.
    // Balance corrections calculated when the robot was falling, or slider
    // positions sent while outputs were disabled, must not be applied when
    // OE re-enables. Draining here (rather than in submit()) keeps the
    // submission path clean and makes the discard policy explicit and visible.
    if (oe_is_estopped()) {
        for (uint8_t ch = 0; ch < NUM_JOINTS; ch++) {
            _slots[ch] = {};
        }
        return;
    }

    for (uint8_t ch = 0; ch < NUM_JOINTS; ch++) {
        if (!_slots[ch].occupied) continue;
        _applyCommand(ch, _slots[ch]);
        _slots[ch] = {};
    }
}

// ---------------------------------------------------------------------------
//  _applyCommand() — dispatch one resolved command to ServoControl.
//
//  Each case maps to the write method appropriate for that source's frame:
//    BALANCE/GAIT → immediate joint-relative write (bypasses smooth stepping)
//    UI           → smooth-stepped absolute write  (smooth stepping runs normally)
// ---------------------------------------------------------------------------
void MotionManager::_applyCommand(uint8_t channel, const MotionCommand& cmd) {
    switch (cmd.source) {

        // ── BALANCE ──────────────────────────────────────────────────────────
        case SOURCE_BALANCE:
            // Joint-relative angle, immediate hardware write.
            // The balance loop runs at 400 Hz; smooth-stepping lag is unacceptable.
            // setJointAngleDirect() still enforces joint limits defined in joint_config.h.
            _servo->setJointAngleDirect(channel, cmd.angleDeg);
            break;

        // ── GAIT ────────────────────────────────────────────────────
        case SOURCE_GAIT:
            // Joint-relative angle, smooth-stepped write.
            // Unlike SOURCE_BALANCE (400 Hz closed-loop), gait commands are
            // one-shot or slowly-ramped. Routing through the smooth-stepper
            // makes all IK panel and future gait module moves respect the
            // per-joint speed set via the speed control panel.
            // WeightShift per-tick deltas are small enough that the stepper
            // tracks them without meaningful lag.
            _servo->setJointAngleSmooth(channel, cmd.angleDeg);
            break;

        // ── UI ───────────────────────────────────────────────────────────────
        case SOURCE_UI:
            // Absolute servo angle (0–270°), smooth-stepped (immediate=false).
            // UI commands are infrequent and user-driven; smooth motion is correct.
            // When BALANCE is active, its setJointAngleDirect() overrides the hardware
            // target each tick anyway — the smooth-stepper target is silently shadowed.
            _servo->setTargetAngle(channel, cmd.angleDeg, /*immediate=*/false);
            break;

        // ── UNKNOWN ──────────────────────────────────────────────────────────
        default:
            // Unknown source: log and skip. Silently corrupting a joint is worse
            // than doing nothing. This should never fire in a correct build.
            Serial.printf("[MotionManager] ERROR: unknown MotionSource=%d on ch=%d. "
                          "Skipped.\n", (int)cmd.source, channel);
            break;
    }
}

// ---------------------------------------------------------------------------
uint8_t MotionManager::pendingCount() const {
    uint8_t count = 0;
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        if (_slots[i].occupied) count++;
    }
    return count;
}