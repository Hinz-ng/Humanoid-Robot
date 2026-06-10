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
//    txGain      — transmission ratio (anatomical° / commanded servo°). 1.0 = direct drive.
//
//  Replace "// TUNE" with "// OK" once a joint is verified.
//
// =============================================================================
//  TRANSMISSION GAIN — measurement log
// -----------------------------------------------------------------------------
//  ANKLE PITCH (both sides) — gain 0.60
//    Measured 2026-05-27. The ankle is driven through a 4-bar linkage; for a
//    commanded servo deviation of θ_s from neutral, the foot rotates ≈ 0.60·θ_s
//    anatomically. Without correction, IK-correct foot pitch arrives
//    under-rotated and the foot lands plantar-flexed.
//
//    CAVEAT: measured UNLOADED with a digital protractor only up to ~20° of
//    deviation. Operating range under load reaches ~27° (crouch / stance
//    transfer). If a loaded foot-IMU sweep shows residual error growing with
//    crouch depth, the linear model is no longer adequate — upgrade this seam
//    to a per-joint lookup table (do NOT pre-build it; flag it from the data).
//
//  KNEE / HIP — gain 1.0 (PROVISIONAL)
//    Knee and hip joints have not been measured. They are direct-drive to the
//    eye but the knee in particular sits behind a short pushrod. Keep at 1.0
//    until verified with a two-point protractor check (command +20° / +30°,
//    record the actual bone angle on each side, fit gain).
//    TODO(transmission-gain): verify knee and hip transmission ratios; raise
//    this comment to a measured value or document why it remained 1.0.
// =============================================================================

const JointConfig JOINT_CONFIG[NUM_JOINTS] = {

// ┌─ ch ──── name ──────────────── neutral   dir    min°     max°    range°  noLoad°/s  txGain
// │                                 (abs °)  (±1)   (≤ 0)   (≥ 0)                       (anat/srv)
// │
// ▼  RIGHT LEG  ──────────────────────────────────────────────────────────────────────

    //  Ankle Roll  — 270° servo, 0.18 s/60° → 333 °/s
    {  0,  "right_ankle_roll",    130.0f,  +1,  -25.0f,  +25.0f,  270.0f,  333.0f,  1.00f  },  // TUNE

    //  Ankle Pitch (QY3242BLS)  — 180° servo, 0.085 s/60° → 706 °/s
    //  4-bar linkage: 0.60 anat°/servo° unloaded (5–20°, 2026-05-27).
    {  1,  "right_ankle_pitch",   136.0f,  +1,  -60.0f,  +60.0f,  180.0f,  706.0f,  0.72f  },  // TUNE

    //  Knee Pitch (same servo as hip roll)  — 270° servo, 0.12 s/60° → 500 °/s
    {  2,  "right_knee_pitch",    137.0f,  -1,  0.0f, +100.0f,  270.0f,  500.0f,  1.00f  },  // TUNE  (txGain unverified)

    //  Hip Roll (same servo as knee pitch)  — 270° servo, 0.12 s/60° → 500 °/s
    {  3,  "right_hip_roll",      135.0f,  +1,  -20.0f,  +80.0f,  270.0f,  500.0f,  1.00f  },  // TUNE  (txGain unverified)

    //  Hip Yaw  — 180° servo, 0.09 s/60° → 667 °/s
    {  4,  "right_hip_yaw",       118.0f,  +1,  -50.0f,  +50.0f,  180.0f,  667.0f,  1.00f  },  // TUNE  (txGain unverified)

    //  Hip Pitch  — 270° servo, 0.11 s/60° → 545 °/s
    {  5,  "right_hip_pitch",     124.0f,  +1,  -25.0f,  +90.0f,  270.0f,  545.0f,  1.00f  },  // TUNE  (txGain unverified)

// ▼  LEFT LEG  ───────────────────────────────────────────────────────────────────────

    {  6,  "left_hip_pitch",      129.0f,  -1,  -25.0f,  +90.0f,  270.0f,  545.0f,  1.00f  },  // TUNE  (txGain unverified)
    {  7,  "left_hip_yaw",        127.0f,  -1,  -50.0f,  +50.0f,  180.0f,  667.0f,  1.00f  },  // TUNE  (txGain unverified)

// ▼  TORSO  ──────────────────────────────────────────────────────────────────────────
//   Torso servos: 270°, 0.06 s/60° → 1000 °/s
//   Direction unverified. Confirm before enabling active balance on IDX_TORSO_PITCH.

    {  8,  "torso_roll",          130.0f,  +1,  -25.0f,  +25.0f,  270.0f, 1000.0f,  1.00f  },  // TUNE
    {  9,  "torso_pitch",         140.0f,  +1,  -20.0f,  +30.0f,  270.0f, 1000.0f,  1.00f  },  // TUNE
    { 10,  "unused_10",           135.0f,  +1,    0.0f,    0.0f,  270.0f, 1000.0f,  1.00f  },  // not assigned
    { 11,  "torso_rotation",      140.0f,  +1,  -65.0f,  +65.0f,  270.0f, 1000.0f,  1.00f  },  // TUNE

// ▼  LEFT LEG (continued)  ───────────────────────────────────────────────────────────

    { 12,  "left_hip_roll",       127.0f,  -1,  -20.0f,  +80.0f,  270.0f,  500.0f,  1.00f  },  // TUNE  (txGain unverified)
    { 13,  "left_knee_pitch",     136.0f,  +1,    0.0f,  +100.0f,  270.0f,  500.0f,  1.00f  },  // TUNE  (txGain unverified)
    //  Left ankle pitch — same 4-bar linkage as right; mirrors txGain.
    { 14,  "left_ankle_pitch",    120.0f,  -1,  -60.0f,  +60.0f,  180.0f,  706.0f,  0.72f  },  // TUNE
    { 15,  "left_ankle_roll",     134.0f,  -1,  -25.0f,  +25.0f,  270.0f,  333.0f,  1.00f  },  // TUNE

};
// Guard: array length must match NUM_JOINTS. If you add or remove a joint,
// update NUM_JOINTS in joint_config.h to match. This fires at compile time.
static_assert(
    sizeof(JOINT_CONFIG) / sizeof(JOINT_CONFIG[0]) == NUM_JOINTS,
    "JOINT_CONFIG array length does not match NUM_JOINTS — "
    "update joint_config.cpp and joint_config.h together."
);