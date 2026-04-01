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

//  Replace "// TUNE" with "// OK" once a joint is verified.
// =============================================================================

const JointConfig JOINT_CONFIG[NUM_JOINTS] = {

// ┌─ ch ──── name ──────────────── neutral   dir    min°     max°    range°  noLoad°/s
// │                                 (abs °)  (±1)   (≤ 0)   (≥ 0)
// │
// ▼  RIGHT LEG  ──────────────────────────────────────────────────────────────────────

    //  Ankle Roll  — 270° servo, 0.18 s/60° → 333 °/s
    {  0,  "right_ankle_roll",    130.0f,  +1,  -30.0f,  +30.0f,  270.0f,  333.0f  },  // TUNE

    //  Ankle Pitch (QY3242BLS)  — 180° servo, 0.085 s/60° → 706 °/s
    {  1,  "right_ankle_pitch",   138.0f,  +1,  -60.0f,  +60.0f,  180.0f,  706.0f  },  // TUNE

    //  Knee Pitch (same servo as hip roll)  — 270° servo, 0.12 s/60° → 500 °/s
    {  2,  "right_knee_pitch",    138.0f,  -1,  0.0f, +100.0f,  270.0f,  500.0f  },  // TUNE

    //  Hip Roll (same servo as knee pitch)  — 270° servo, 0.12 s/60° → 500 °/s
    {  3,  "right_hip_roll",      135.0f,  +1,  -20.0f,  +80.0f,  270.0f,  500.0f  },  // TUNE

    //  Hip Yaw  — 180° servo, 0.09 s/60° → 667 °/s
    {  4,  "right_hip_yaw",       135.0f,  +1,  -50.0f,  +50.0f,  180.0f,  667.0f  },  // TUNE

    //  Hip Pitch  — 270° servo, 0.11 s/60° → 545 °/s
    {  5,  "right_hip_pitch",     128.0f,  +1,  -25.0f,  +90.0f,  270.0f,  545.0f  },  // TUNE

// ▼  LEFT LEG  ───────────────────────────────────────────────────────────────────────

    {  6,  "left_hip_pitch",      135.0f,  -1,  -25.0f,  +90.0f,  270.0f,  545.0f  },  // TUNE
    {  7,  "left_hip_yaw",        130.0f,  -1,  -50.0f,  +50.0f,  180.0f,  667.0f  },  // TUNE

// ▼  TORSO  ──────────────────────────────────────────────────────────────────────────
//   Torso servos: 270°, 0.06 s/60° → 1000 °/s
//   Direction unverified. Confirm before enabling active balance on IDX_TORSO_PITCH.

    {  8,  "torso_roll",          130.0f,  +1,  -25.0f,  +25.0f,  270.0f, 1000.0f  },  // TUNE
    {  9,  "torso_pitch",         125.0f,  +1,  -20.0f,  +30.0f,  270.0f, 1000.0f  },  // TUNE
    { 10,  "unused_10",           135.0f,  +1,    0.0f,    0.0f,  270.0f, 1000.0f  },  // not assigned
    { 11,  "torso_rotation",      135.0f,  +1,  -65.0f,  +65.0f,  270.0f, 1000.0f  },  // TUNE

// ▼  LEFT LEG (continued)  ───────────────────────────────────────────────────────────

    { 12,  "left_hip_roll",       130.0f,  -1,  -20.0f,  +80.0f,  270.0f,  500.0f  },  // TUNE
    { 13,  "left_knee_pitch",     140.0f,  +1,  0.0f, +100.0f,  270.0f,  500.0f  },  // TUNE
    { 14,  "left_ankle_pitch",    130.0f,  -1,  -60.0f,  +60.0f,  180.0f,  706.0f  },  // TUNE
    { 15,  "left_ankle_roll",     137.0f,  -1,  -30.0f,  +30.0f,  270.0f,  333.0f  },  // TUNE

};
// Guard: array length must match NUM_JOINTS. If you add or remove a joint,
// update NUM_JOINTS in joint_config.h to match. This fires at compile time.
static_assert(
    sizeof(JOINT_CONFIG) / sizeof(JOINT_CONFIG[0]) == NUM_JOINTS,
    "JOINT_CONFIG array length does not match NUM_JOINTS — "
    "update joint_config.cpp and joint_config.h together."
);