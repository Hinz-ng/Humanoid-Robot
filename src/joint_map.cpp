#include "joint_map.h"
#include <string.h>
#include <Arduino.h>

// =============================================================================
//  AUTHORITATIVE JOINT DEFINITIONS
// =============================================================================
//  Column order:
//    channel, name, neutral_pulse, neutral_deg,
//    delta_flex(LEGACY), delta_extend(LEGACY), ui_direction_hint, neutralAngle,
//    delta_per_unit   ← NEW: µs per 1° of magnitude (US_PER_DEG = 7.407)
//
//  neutral_pulse = round((neutral_deg / 270.0) * 2000 + 500)
//
//  TUNE: delta_per_unit
//    Default US_PER_DEG (7.407) → magnitude=1 unit = 1° of servo travel.
//    Increase to make movements larger for a given magnitude value.
//    Example: 14.814 → magnitude=1 unit = 2° of travel.
//
//  ui_direction_hint:
//    +1 = increasing pulse moves toward anatomical flexion
//    -1 = decreasing pulse moves toward anatomical flexion
// =============================================================================

JointDef JOINT_MAP[NUM_SERVOS] = {
//  ch   name                  nPulse  nDeg    dFx  dEx  hint  nAngle    dpu
    { 0, "right_ankle_roll",   1470, 131.0f,  150, 150,  +1, 131.0f, US_PER_DEG },  // TUNE
    { 1, "right_ankle_pitch",  1426, 125.0f,  150, 150,  +1, 125.0f, US_PER_DEG },  // TUNE
    { 2, "right_knee_pitch",   1485, 133.0f,  150, 150,  -1, 133.0f, US_PER_DEG },  // TUNE: -1 = dec pulse = extension
    { 3, "right_hip_roll",     1463, 130.0f,  150, 150,  +1, 130.0f, US_PER_DEG },  // TUNE
    { 4, "right_hip_yaw",      1463, 130.0f,  150, 150,  +1, 130.0f, US_PER_DEG },  // TUNE
    { 5, "right_hip_pitch",    1574, 145.0f,  150, 150,  +1, 145.0f, US_PER_DEG },  // TUNE

    // Unused channels — stubs, do not drive
    { 6, "unused_6",           1500, 135.0f,    0,   0,  +1, 135.0f, 0.0f },
    { 7, "unused_7",           1500, 135.0f,    0,   0,  +1, 135.0f, 0.0f },
    { 8, "unused_8",           1500, 135.0f,    0,   0,  +1, 135.0f, 0.0f },
    { 9, "unused_9",           1500, 135.0f,    0,   0,  +1, 135.0f, 0.0f },

    {10, "left_hip_pitch",     1574, 145.0f,  150, 150,  -1, 145.0f, US_PER_DEG },  // TUNE: mirrored
    {11, "left_hip_yaw",       1463, 130.0f,  150, 150,  -1, 130.0f, US_PER_DEG },  // TUNE
    {12, "left_hip_roll",      1493, 134.0f,  150, 150,  -1, 134.0f, US_PER_DEG },  // TUNE
    {13, "left_knee_pitch",    1448, 128.0f,  150, 150,  +1, 128.0f, US_PER_DEG },  // TUNE
    {14, "left_ankle_pitch",   1426, 125.0f,  150, 150,  -1, 125.0f, US_PER_DEG },  // TUNE
    {15, "left_ankle_roll",    1441, 127.0f,  150, 150,  -1, 127.0f, US_PER_DEG },  // TUNE
};

// =============================================================================
//  NAMED MOVEMENT HELPER  —  BUG FIX: now scales by magnitude, not just sign
// =============================================================================

int pulse_for_named_movement(uint8_t joint_index, const char* movement_name, float magnitude) {
    if (joint_index >= NUM_SERVOS) {
        Serial.printf("[JointMap] ERROR: joint_index %d out of range\n", joint_index);
        return USMIN;
    }

    const JointDef& j = JOINT_MAP[joint_index];

    // --- ADD START: debug log incoming parameters ---
    Serial.printf("[JointMap] pulse_for_named_movement: joint=%d (%s) move=%s magnitude=%.2f\n",
                  joint_index, j.name, movement_name, magnitude);
    // --- ADD END ---

    // --- FIX START ---
    // BEFORE (broken): used sign(magnitude) only → always ±delta_flex µs regardless of magnitude value
    //   result = j.neutral_pulse + (j.ui_direction_hint * j.delta_flex * sign);
    //
    // AFTER (correct): uses magnitude as a scalar multiplied by delta_per_unit
    //   magnitude=20 → 20 * 7.407 = 148µs ≈ 20° ✓
    //   magnitude=45 → 45 * 7.407 = 333µs ≈ 45° ✓
    //   magnitude=-10 → moves in reverse direction ✓
    int result;
    if (strcmp(movement_name, "flex") == 0) {
        result = j.neutral_pulse + (int)((float)j.ui_direction_hint * j.delta_per_unit * magnitude + 0.5f);
    }
    else if (strcmp(movement_name, "extend") == 0) {
        result = j.neutral_pulse - (int)((float)j.ui_direction_hint * j.delta_per_unit * magnitude + 0.5f);
    }
    else {
        Serial.printf("[JointMap] WARNING: unknown movement '%s' on joint %d — returning neutral\n",
                      movement_name, joint_index);
        result = j.neutral_pulse;
    }
    // --- FIX END ---

    int clamped = clamp_pulse(result);

    // --- ADD START: debug log computed output ---
    Serial.printf("[JointMap] → raw=%d  clamped=%d  (%.1f°)\n",
                  result, clamped, pulse_to_deg(clamped));
    // --- ADD END ---

    return clamped;
}

// =============================================================================
//  DEGREE-OFFSET HELPER
// =============================================================================

int pulse_for_degree_offset(uint8_t joint_index, float degree_offset) {
    if (joint_index >= NUM_SERVOS) return USMIN;
    int result = JOINT_MAP[joint_index].neutral_pulse
                 + (int)(degree_offset * US_PER_DEG + 0.5f);
    return clamp_pulse(result);
}

// =============================================================================
//  NON-BLOCKING INTERPOLATOR
// =============================================================================

void interp_start(InterpState* s, int current_pulse, int target_pulse, unsigned long duration_ms) {
    s->start_pulse  = current_pulse;
    s->target_pulse = clamp_pulse(target_pulse);
    s->start_ms     = millis();
    s->duration_ms  = duration_ms;
    s->active       = (current_pulse != s->target_pulse) && (duration_ms > 0);
}

int interp_update(InterpState* s) {
    if (!s->active) return s->target_pulse;

    unsigned long elapsed = millis() - s->start_ms;
    if (elapsed >= s->duration_ms) {
        s->active = false;
        return s->target_pulse;
    }

    float t = (float)elapsed / (float)s->duration_ms;
    // Optional cubic ease: uncomment to use
    // t = t < 0.5f ? 4.0f*t*t*t : 1.0f - pow(-2.0f*t+2.0f,3.0f)/2.0f;

    int result = s->start_pulse + (int)((float)(s->target_pulse - s->start_pulse) * t + 0.5f);
    return clamp_pulse(result);
}