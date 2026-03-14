#ifndef MOTION_MANAGER_H
#define MOTION_MANAGER_H

#include <Arduino.h>
#include "joint_config.h"    // NUM_JOINTS, IDX_* constants
#include "servo_control.h"   // ServoControl — the only path to hardware

// =============================================================================
//  MOTION MANAGER  —  Joint Authority Layer
// =============================================================================
//
//  PROBLEM SOLVED
//  ──────────────
//  When BalanceController and a future GaitController both want to command the
//  same joint (e.g. ankle pitch) on the same tick, "last write wins" causes
//  undefined behaviour. MotionManager gives every joint a single authoritative
//  write per tick, chosen by priority.
//
//  HOW IT WORKS (one tick):
//    1. Each source calls submit() to declare its desired joint angle.
//       Higher-priority submissions silently overwrite lower-priority ones.
//    2. main.cpp calls flush() once, after all sources have submitted.
//       flush() dispatches the winning command per joint to ServoControl,
//       then clears all slots for the next tick.
//
//  PRIORITY LEVELS  (higher uint8_t value = higher authority)
//  ────────────────────────────────────────────────────────────
//  SOURCE_UI      (0) — WebSocket slider commands. Active when controller is off.
//  SOURCE_GAIT    (1) — Future walking controller. Medium authority.
//  SOURCE_BALANCE (2) — PD balance controller. Wins on shared joints.
//
//  COORDINATE FRAMES
//  ──────────────────
//  Each source submits in its natural frame; flush() dispatches accordingly:
//    SOURCE_BALANCE / SOURCE_GAIT → joint-relative degrees (from neutral)
//                                   → ServoControl::setJointAngleDirect()
//    SOURCE_UI                    → absolute servo degrees (0–270°)
//                                   → ServoControl::setTargetAngle()
//
//  ADDING A NEW SOURCE  (e.g. RecoveryController)
//  ────────────────────────────────────────────────
//  1. Add an entry to MotionSource with the appropriate priority integer.
//  2. Add a case in MotionManager::_applyCommand() for the correct write method.
//  3. Call submit(SOURCE_RECOVERY, ch, angleDeg) from your controller each tick.
//  4. No other files change.
//
//  WIRING IN main.cpp
//  ───────────────────
//  In setup():
//    motionManager.init(&servoController);
//    balanceController.setMotionManager(&motionManager);
//    webComm.setMotionManager(&motionManager);
//
//  In the 400 Hz gate, after all submit() calls:
//    motionManager.flush();
// =============================================================================

// ---------------------------------------------------------------------------
//  MotionSource — who is submitting a command.
//  The numeric value IS the priority: higher = higher authority.
// ---------------------------------------------------------------------------
enum MotionSource : uint8_t {
    SOURCE_UI      = 0,   // Web UI sliders      — lowest authority
    SOURCE_GAIT    = 1,   // Walking controller  — medium authority  (future)
    SOURCE_BALANCE = 2,   // PD balance loop     — highest authority
};

// ---------------------------------------------------------------------------
//  MotionCommand — one pending command for a single joint in a single tick.
//  Stored internally by MotionManager; you do not create these directly.
// ---------------------------------------------------------------------------
struct MotionCommand {
    bool         occupied;    // true = a command was submitted for this joint this tick
    MotionSource source;      // who submitted it — determines the write method in flush()
    uint8_t      priority;    // == (uint8_t)source; cached here to avoid repeated casts
    float        angleDeg;    // desired angle in the source's coordinate frame (see above)
};

// ---------------------------------------------------------------------------
//  MotionManager class
// ---------------------------------------------------------------------------
class MotionManager {
public:
    MotionManager();

    // -------------------------------------------------------------------------
    //  init()  —  link to the hardware gateway.
    //  Call once in setup(), after servoController.init().
    // -------------------------------------------------------------------------
    void init(ServoControl* servo);

    // -------------------------------------------------------------------------
    //  submit()  —  declare a desired joint angle for this tick.
    //
    //  src      : the source registering this command (determines priority + write mode).
    //  channel  : joint index 0–15. Use IDX_* constants from joint_config.h.
    //  angleDeg : angle in the source's coordinate frame:
    //               SOURCE_BALANCE / SOURCE_GAIT  →  joint-relative (degrees from neutral)
    //               SOURCE_UI                     →  absolute servo angle (0–270°)
    //
    //  Acceptance rules:
    //    • Slot empty                     → always accepted.
    //    • Incoming priority > existing   → accepted; overwrites.
    //    • Incoming priority == existing  → accepted; overwrites (last same-priority wins).
    //    • Incoming priority < existing   → silently dropped.
    //    • channel >= NUM_JOINTS          → silently dropped with a Serial warning.
    // -------------------------------------------------------------------------
    void submit(MotionSource src, uint8_t channel, float angleDeg);

    // -------------------------------------------------------------------------
    //  flush()  —  resolve all pending commands and write to hardware.
    //
    //  Call EXACTLY ONCE per control tick from main.cpp, after every source
    //  has called submit(). All filled slots are written, then cleared.
    //  Empty slots are silently skipped (that joint is not touched this tick).
    //
    //  This is a safe no-op if init() has not been called.
    // -------------------------------------------------------------------------
    void flush();

    // -------------------------------------------------------------------------
    //  pendingCount()  —  how many joints have a submission this tick.
    //  Useful for telemetry or debug logging.
    // -------------------------------------------------------------------------
    uint8_t pendingCount() const;

private:
    ServoControl* _servo;              // hardware gateway — owned externally
    MotionCommand _slots[NUM_JOINTS];  // one slot per joint; cleared each flush()

    // Dispatch one resolved command to ServoControl.
    // Separated from flush() so each write path is easy to find and modify.
    void _applyCommand(uint8_t channel, const MotionCommand& cmd);
};

#endif // MOTION_MANAGER_H