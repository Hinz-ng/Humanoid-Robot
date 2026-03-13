#include "joint_config.h"

// =============================================================================
//  JOINT CONFIGURATION DATA  —  EDIT THIS TABLE TO CALIBRATE YOUR ROBOT
// =============================================================================
//
//  See joint_config.h for a full explanation of every field.
//
//  Quick reference:
//    neutralDeg  — absolute servo angle at standing neutral (0°–270°)
//    direction   — +1 if increasing servo angle = flexion; −1 if inverted
//    minAngle    — max degrees in the NEGATIVE joint direction (≤ 0, usually 0 for knees)
//    maxAngle    — max degrees in the POSITIVE joint direction (≥ 0)
//
//  Search "// TUNE" to find values that still need hardware calibration.
//  Replace "// TUNE" with "// OK" once a joint is verified.
// =============================================================================

const JointConfig JOINT_CONFIG[NUM_JOINTS] = {

// ┌─── ch ──── name ─────────────────── neutral   dir    min°     max°
// │                                     (abs °)  (±1)   (≤ 0)   (≥ 0)
// │
// ▼  RIGHT LEG  ───────────────────────────────────────────────────────

    {  0,  "right_ankle_roll",    121.0f,   +1,   -30.0f,  +30.0f  },  // TUNE
    {  1,  "right_ankle_pitch",   125.0f,   +1,   -60.0f,  +60.0f  },  // TUNE
    {  2,  "right_knee_pitch",    123.0f,   -1,     0.0f,  +100.0f  },  // TUNE — knee only flexes one way
    {  3,  "right_hip_roll",      125.0f,   +1,   -20.0f,  +80.0f  },  // TUNE
    {  4,  "right_hip_yaw",       130.0f,   +1,   -50.0f,  +50.0f  },  // TUNE
    {  5,  "right_hip_pitch",     145.0f,   +1,   -25.0f,  +90.0f  },  // TUNE

// ▼  UNUSED CHANNELS  ─────────────────────────────────────────────────
//    Zero limits prevent any movement on stub channels.

// ▼  LEFT LEG  ────────────────────────────────────────────────────────
    { 6,  "left_hip_pitch",      145.0f,   -1,   -25.0f,  +90.0f  },  // TUNE
    { 7,  "left_hip_yaw",        130.0f,   -1,   -50.0f,  +50.0f  },  // TUNE
   
    {  8,  "unused_8",            135.0f,   +1,     0.0f,   0.0f   },
    {  9,  "unused_9",            135.0f,   +1,     0.0f,   0.0f   },
    {  10,  "unused_6",            135.0f,   +1,     0.0f,   0.0f   },
    {  11,  "unused_7",            135.0f,   +1,     0.0f,   0.0f   },

// ▼  LEFT LEG  ────────────────────────────────────────────────────────
//    Note direction = −1 for mirrored joints.
    { 12,  "left_hip_roll",       130.0f,   -1,   -20.0f,  +80.0f  },  // TUNE
    { 13,  "left_knee_pitch",     125.0f,   +1,     0.0f,  +100.0f  },  // TUNE — knee only flexes one way
    { 14,  "left_ankle_pitch",    120.0f,   -1,   -60.0f,  +60.0f  },  // TUNE
    { 15,  "left_ankle_roll",     122.0f,   -1,   -30.0f,  +30.0f  },  // TUNE

};