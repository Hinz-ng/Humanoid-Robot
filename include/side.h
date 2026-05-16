// =============================================================================
// FILE:    side.h
// MODULE:  side
// LAYER:   0 — vocabulary only (no logic, no state)
//
// PURPOSE:
//   Single source of truth for left/right distinction across the robot.
//   Replaces positional `(leftValue, rightValue)` parameters, paired
//   `_left*` / `_right*` member duplication, and `bool isRight` flags
//   with one typed enum plus a function that maps (side, joint role)
//   to a PCA9685 channel.
//
// CONTRACT:
//   - No global state. Pure value types and pure functions.
//   - `channelFor(side, joint)` is the ONLY function that translates
//     (side, anatomical role) into a hardware channel.
//   - `IDX_*` macros in joint_config.h are kept as aliases over the
//     same numeric values for backward compatibility.
//
// LAST CHANGED: 2026-05-XX | Hinz | Phase 0 introduction
// =============================================================================

#ifndef SIDE_H
#define SIDE_H

#include <stdint.h>
#include <stddef.h>

// ---------------------------------------------------------------------------
//  Side — left or right body half.
//  Underlying values 0/1 chosen so sideIdx(Side::LEFT)=0 indexes naturally
//  into `T arr[SIDE_COUNT]`.
// ---------------------------------------------------------------------------
enum class Side : uint8_t {
    LEFT  = 0,
    RIGHT = 1,
};

constexpr uint8_t SIDE_COUNT = 2;

// Container index. Use for `T arr[SIDE_COUNT]` patterns.
constexpr size_t sideIdx(Side s) { return static_cast<size_t>(s); }

// The opposite side. Total function (never returns NONE).
constexpr Side otherSide(Side s) {
    return s == Side::LEFT ? Side::RIGHT : Side::LEFT;
}

// Single-character label for telemetry and logging.
constexpr const char* sideName(Side s) {
    return s == Side::LEFT ? "L" : "R";
}

// Compatibility helpers for migration. Keep both directions during the
// phased rollout so old `bool isRight` call sites can adapt incrementally.
constexpr bool isRight(Side s)            { return s == Side::RIGHT; }
constexpr Side sideFromIsRight(bool isR)  { return isR ? Side::RIGHT : Side::LEFT; }

// ---------------------------------------------------------------------------
//  LegJoint — anatomical role of a leg joint, side-independent.
// ---------------------------------------------------------------------------
enum class LegJoint : uint8_t {
    HIP_PITCH    = 0,
    HIP_ROLL     = 1,
    HIP_YAW      = 2,
    KNEE_PITCH   = 3,
    ANKLE_PITCH  = 4,
    ANKLE_ROLL   = 5,
};

constexpr uint8_t LEG_JOINT_COUNT = 6;

// ---------------------------------------------------------------------------
//  channelFor — map (side, joint role) to a PCA9685 channel index.
//
//  Implementation lives in side.cpp so it can use IDX_* values from
//  joint_config.h without creating a header cycle. Definition is a
//  6×2 constant lookup table; static_asserts in side.cpp prove every
//  cell equals its IDX_* alias.
// ---------------------------------------------------------------------------
uint8_t channelFor(Side side, LegJoint joint);

#endif // SIDE_H