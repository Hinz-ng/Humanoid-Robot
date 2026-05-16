// =============================================================================
// FILE:    side.cpp
// LAST CHANGED: 2026-05-XX | Hinz | Phase 0 introduction
// =============================================================================

#include "side.h"
#include "joint_config.h"   // IDX_* aliases — authoritative numeric values

// Static lookup. Order matches LegJoint enum.
// LEFT row first, RIGHT row second. Side maps to row via sideIdx().
static const uint8_t kChannelTable[SIDE_COUNT][LEG_JOINT_COUNT] = {
    /* LEFT  */ { IDX_L_HIP_PITCH,  IDX_L_HIP_ROLL,  IDX_L_HIP_YAW,
                  IDX_L_KNEE_PITCH, IDX_L_ANKLE_PITCH, IDX_L_ANKLE_ROLL },
    /* RIGHT */ { IDX_R_HIP_PITCH,  IDX_R_HIP_ROLL,  IDX_R_HIP_YAW,
                  IDX_R_KNEE_PITCH, IDX_R_ANKLE_PITCH, IDX_R_ANKLE_ROLL },
};

// Compile-time sanity: catches accidental row/column transposition
// before the firmware ever ships.
static_assert(IDX_L_HIP_PITCH   == 6,  "Left hip pitch channel changed");
static_assert(IDX_R_HIP_PITCH   == 5,  "Right hip pitch channel changed");
static_assert(IDX_L_ANKLE_ROLL  == 15, "Left ankle roll channel changed");
static_assert(IDX_R_ANKLE_ROLL  == 0,  "Right ankle roll channel changed");
// Add more rows here as you discover regressions during testing —
// each one is a free safety net.

uint8_t channelFor(Side side, LegJoint joint) {
    return kChannelTable[sideIdx(side)][static_cast<uint8_t>(joint)];
}