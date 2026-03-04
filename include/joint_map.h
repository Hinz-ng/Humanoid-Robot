#ifndef JOINT_MAP_H
#define JOINT_MAP_H

#include <Arduino.h>

// =============================================================================
//  JOINT MAP  —  Single Source of Truth for all servo metadata
// =============================================================================
//
//  PULSE MATH (centralized here, used everywhere):
//    USMIN = 500 µs  → 0°
//    USMAX = 2500 µs → 270°
//    pulse = round((angle / 270.0) * 2000 + 500)
//    angle = ((pulse - 500) / 2000.0) * 270.0
//    1 degree = (2500-500)/270 = 7.407 µs
//
//  MOVEMENT DELTA CONVENTION:
//    delta_per_unit : µs per 1 unit of magnitude (authoritative, NEW).
//                     Default US_PER_DEG = 7.407 → magnitude units are degrees.
//    delta_flex     : LEGACY fixed µs offset, kept for backward compat only.
//    delta_extend   : LEGACY fixed µs offset, kept for backward compat only.
//
//  CORRECTED formula (fixes the "always 20°" bug):
//    "flex"   → neutral_pulse + (ui_direction_hint *  delta_per_unit * magnitude)
//    "extend" → neutral_pulse - (ui_direction_hint *  delta_per_unit * magnitude)
//    magnitude is a SIGNED FLOAT. Positive = move in labeled direction.
//    Result clamped to [USMIN, USMAX].
//
//  ui_direction_hint (+1 or -1):
//    +1 → increasing pulse = anatomical flexion for this joint
//    -1 → decreasing pulse = anatomical flexion for this joint
//    DISPLAY-ONLY in the UI badge. Helper functions handle actual math.
//
//  TUNING: search "// TUNE:" to find per-joint values to adjust.
// =============================================================================

#define NUM_SERVOS      16
#define SERVO_FREQ      50
#define USMIN           500
#define USMAX           2500
#define ANGLE_RANGE     270.0f

// µs per degree — derived from hardware constants.
// Do not change unless USMIN/USMAX/ANGLE_RANGE change.
#define US_PER_DEG  ((float)(USMAX - USMIN) / ANGLE_RANGE)  // = 7.407f

struct JointDef {
    uint8_t     channel;             // PCA9685 channel (0–15)
    const char* name;                // Snake_case joint name
    int         neutral_pulse;       // Neutral position in µs (authoritative)
    float       neutral_deg;         // Neutral position in degrees (convenience)
    int         delta_flex;          // LEGACY: fixed µs offset — ignored by new API
    int         delta_extend;        // LEGACY: fixed µs offset — ignored by new API
    int         ui_direction_hint;   // +1 or -1 (display only)
    float       neutralAngle;        // Alias for neutral_deg (backward compat)
    // --- ADD START: per-unit scaling factor ---
    // µs of pulse change per 1 unit of magnitude.
    // US_PER_DEG default → magnitude=1 means exactly 1° of physical movement.
    // TUNE: set higher for joints where 1° feels too small a step.
    float       delta_per_unit;
    // --- ADD END ---
};

#define NUM_ACTIVE_JOINTS 12

#define IDX_R_ANKLE_ROLL   0
#define IDX_R_ANKLE_PITCH  1
#define IDX_R_KNEE_PITCH   2
#define IDX_R_HIP_ROLL     3
#define IDX_R_HIP_YAW      4
#define IDX_R_HIP_PITCH    5
// channels 6-9: unused stubs
#define IDX_L_HIP_PITCH   10
#define IDX_L_HIP_YAW     11
#define IDX_L_HIP_ROLL    12
#define IDX_L_KNEE_PITCH  13
#define IDX_L_ANKLE_PITCH 14
#define IDX_L_ANKLE_ROLL  15

extern JointDef JOINT_MAP[NUM_SERVOS];

// =============================================================================
//  PULSE ↔ DEGREE CONVERSION  (free functions, unit-testable)
// =============================================================================

inline int deg_to_pulse(float deg) {
    if (deg < 0.0f)        deg = 0.0f;
    if (deg > ANGLE_RANGE) deg = ANGLE_RANGE;
    return (int)((deg / ANGLE_RANGE) * (float)(USMAX - USMIN) + (float)USMIN + 0.5f);
}

inline float pulse_to_deg(int pulse) {
    if (pulse < USMIN) pulse = USMIN;
    if (pulse > USMAX) pulse = USMAX;
    return ((float)(pulse - USMIN) / (float)(USMAX - USMIN)) * ANGLE_RANGE;
}

inline int clamp_pulse(int pulse) {
    if (pulse < USMIN) return USMIN;
    if (pulse > USMAX) return USMAX;
    return pulse;
}

// =============================================================================
//  NAMED MOVEMENT API
// =============================================================================

// Returns a target pulse for a semantic movement.
//   joint_index   : index into JOINT_MAP[]
//   movement_name : "flex" or "extend"
//   magnitude     : SIGNED FLOAT in degrees (US_PER_DEG scale).
//                   e.g. magnitude=30.0 → 30° toward flexion direction.
//                   Negative magnitude reverses direction.
//
// BUG FIX: previous version used sign(magnitude) only — magnitude value was
// discarded, always producing ±20°. Now uses delta_per_unit * magnitude.
int pulse_for_named_movement(uint8_t joint_index, const char* movement_name, float magnitude);

// Returns a target pulse offset from neutral by [degree_offset] degrees (raw).
int pulse_for_degree_offset(uint8_t joint_index, float degree_offset);

// =============================================================================
//  NON-BLOCKING INTERPOLATOR
// =============================================================================

struct InterpState {
    int           start_pulse;
    int           target_pulse;
    unsigned long start_ms;
    unsigned long duration_ms;
    bool          active;
};

void interp_start(InterpState* s, int current_pulse, int target_pulse, unsigned long duration_ms);
int  interp_update(InterpState* s);

#endif // JOINT_MAP_H