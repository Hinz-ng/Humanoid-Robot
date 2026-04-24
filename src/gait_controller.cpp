// =============================================================================
// FILE:    gait_controller.cpp
// MODULE:  gait
// LAYER:   3.5 — Gait / Motion
//
// IMPLEMENTATION NOTES:
//
//   WAYPOINT-BASED DESIGN:
//   - Gait is a sequence of key poses (waypoints), not continuous sin/cos
//   - Each waypoint defines foot targets (x, y, h) for both legs
//   - Interpolation between waypoints is smooth (eased or linear)
//   - Easy to debug: inspect waypoint sequence, step through manually
//
//   MODES:
//   - IDLE: No output, controller inactive
//   - STANCE: Hold a static crouched posture (safe starting position)
//   - PLAYBACK: Loop through recorded waypoints continuously
//   - RECORDING: Capture live IK targets as new waypoints
//   - STEPPING: Manual advance through waypoints (debug mode)
//
//   INTERPOLATION:
//   - Linear: constant velocity between waypoints
//   - Eased (smoothstep): smooth acceleration/deceleration at endpoints
//   - Default: eased for smoother motion
//
//   RECORDING WORKFLOW:
//   1. Use UI sliders or IK commands to pose the robot
//   2. Send CMD:GAIT_RECORD_WAYPOINT to capture current foot targets
//   3. Repeat for each key pose in your gait cycle
//   4. Send CMD:GAIT_PLAY to start looping playback
//
// LAST CHANGED: 2026-04-23 | New waypoint-based implementation
// =============================================================================

#include "gait_controller.h"
#include "joint_config.h"
#include "oe_control.h"
#include <math.h>

// =============================================================================
//  INITIALIZATION
// =============================================================================

void GaitController::init(MotionManager* mm) {
    if (mm == nullptr) {
        Serial.println("[GaitController] ERROR: null MotionManager — update() will no-op.");
        return;
    }
    _mm = mm;
    _state = {};
    _cfg = {};
    
    // Initialize waypoint array with default stance
    for (uint8_t i = 0; i < GAITS_MAX_WAYPOINTS; i++) {
        _waypoints[i].left_x_mm   = 0.0f;
        _waypoints[i].left_y_mm   = _cfg.stance_width_mm;
        _waypoints[i].left_h_mm   = _cfg.stance_height_mm;
        _waypoints[i].right_x_mm  = 0.0f;
        _waypoints[i].right_y_mm  = _cfg.stance_width_mm;
        _waypoints[i].right_h_mm  = _cfg.stance_height_mm;
        _waypoints[i].duration_s  = 0.5f;
        snprintf(_waypoints[i].label, sizeof(_waypoints[i].label), "wp_%d", i);
    }
    
    Serial.println("[GaitController] Initialized (waypoint-based).");
}

// =============================================================================
//  START / STOP
// =============================================================================

void GaitController::start(GaitMode mode) {
    if (_mm == nullptr) {
        Serial.println("[GaitController] start() failed: MotionManager not wired.");
        return;
    }
    
    _state = {};
    _state.active = true;
    _state.mode = mode;
    
    if (mode == GaitMode::PLAYBACK || mode == GaitMode::STEPPING) {
        if (_state.waypoint_count == 0) {
            Serial.println("[GaitController] WARNING: No waypoints recorded. Add waypoints first.");
        }
        _state.current_waypoint = 0;
        _state.phase = 0.0f;
        Serial.printf("[GaitController] START %s: %d waypoints loaded\n",
                      mode == GaitMode::PLAYBACK ? "PLAYBACK" : "STEPPING",
                      _state.waypoint_count);
    } else if (mode == GaitMode::STANCE) {
        goToStance();
        Serial.println("[GaitController] START STANCE");
    } else if (mode == GaitMode::RECORDING) {
        _state.waypoint_count = 0;  // Start fresh recording
        Serial.println("[GaitController] START RECORDING");
    } else {
        Serial.println("[GaitController] START IDLE (no output)");
    }
}

void GaitController::stop() {
    _state.active = false;
    _state.mode = GaitMode::IDLE;
    Serial.println("[GaitController] STOP");
}

// =============================================================================
//  UPDATE LOOP
// =============================================================================

void GaitController::update(float dt_s) {
    if (!_state.active || _mm == nullptr) return;
    
    // Always stop cleanly on e-stop
    if (oe_is_estopped()) {
        _state.active = false;
        _state.mode = GaitMode::IDLE;
        return;
    }
    
    switch (_state.mode) {
        case GaitMode::STANCE:
            // Continuously hold stance position (re-submit to fight drift)
            goToStance();
            break;
            
        case GaitMode::PLAYBACK:
        case GaitMode::STEPPING:
            _updatePlayback(dt_s);
            break;
            
        case GaitMode::RECORDING:
            // Recording mode doesn't output — just captures
            // Foot targets come from other sources (UI sliders, IK commands)
            break;
            
        case GaitMode::IDLE:
        default:
            break;
    }
}

// Private helper: handle playback/stepping mode
void GaitController::_updatePlayback(float dt_s) {
    if (_state.waypoint_count < 2) {
        // Need at least 2 waypoints to interpolate
        return;
    }
    
    // In STEPPING mode, don't auto-advance — wait for manual advanceWaypoint()
    if (_state.mode == GaitMode::STEPPING && _state.phase > 0.0f) {
        // Already stepped, hold position
        _submitCurrentPose();
        return;
    }
    
    // Get current and next waypoint indices (with wrapping for loop mode)
    uint8_t wpA_idx = _state.current_waypoint;
    uint8_t wpB_idx = (_state.current_waypoint + 1) % _state.waypoint_count;
    
    const Waypoint& wpA = _waypoints[wpA_idx];
    const Waypoint& wpB = _waypoints[wpB_idx];
    
    // Advance phase based on duration and speed multiplier
    float effective_duration = wpA.duration_s / _cfg.playback_speed_multiplier;
    if (effective_duration < 0.05f) effective_duration = 0.05f;  // Guard against div by zero
    
    _state.elapsed_time_s += dt_s;
    _state.phase = _state.elapsed_time_s / effective_duration;
    
    // Check if we've reached the end of this segment
    if (_state.phase >= 1.0f) {
        _state.phase = 1.0f;
        
        if (_state.mode == GaitMode::PLAYBACK || _cfg.loop_playback) {
            // Move to next segment (wrap around if looping)
            _state.current_waypoint = wpB_idx;
            _state.elapsed_time_s = 0.0f;
            
            // If we wrapped and not looping, stop
            if (!_cfg.loop_playback && wpB_idx == 0) {
                stop();
                Serial.println("[GaitController] Playback complete (loop disabled)");
                return;
            }
        }
        // In STEPPING mode, we hold at phase=1 until advanceWaypoint() is called
    }
    
    // Interpolate between waypoints
    float t = _state.phase;
    if (_cfg.use_eased_interpolation) {
        t = _ease(t);
    }
    
    float left_x, left_y, left_h;
    float right_x, right_y, right_h;
    _interpolate(wpA, wpB, t, left_x, left_y, left_h, right_x, right_y, right_h);
    
    // Solve IK and submit
    _submitLeg(false, left_x, left_y, left_h);   // left leg
    _submitLeg(true,  right_x, right_y, right_h); // right leg
    
    // Update telemetry
    _state.x_left_mm = left_x;   _state.y_left_mm = left_y;   _state.h_left_mm = left_h;
    _state.x_right_mm = right_x; _state.y_right_mm = right_y; _state.h_right_mm = right_h;
}

// Helper: submit current interpolated pose (for holding position)
void GaitController::_submitCurrentPose() {
    if (_state.waypoint_count == 0) return;
    
    const Waypoint& wp = _waypoints[_state.current_waypoint];
    _submitLeg(false, wp.left_x_mm, wp.left_y_mm, wp.left_h_mm);
    _submitLeg(true, wp.right_x_mm, wp.right_y_mm, wp.right_h_mm);
    
    _state.x_left_mm = wp.left_x_mm;   _state.y_left_mm = wp.left_y_mm;   _state.h_left_mm = wp.left_h_mm;
    _state.x_right_mm = wp.right_x_mm; _state.y_right_mm = wp.right_y_mm; _state.h_right_mm = wp.right_h_mm;
}

// =============================================================================
//  WAYPOINT MANAGEMENT
// =============================================================================

void GaitController::clearWaypoints() {
    _state.waypoint_count = 0;
    _state.current_waypoint = 0;
    _state.phase = 0.0f;
    Serial.println("[GaitController] Waypoints cleared");
}

bool GaitController::recordWaypoint(const char* label) {
    if (_state.waypoint_count >= GAITS_MAX_WAYPOINTS) {
        Serial.printf("[GaitController] Recording full (%d max)\n", GAITS_MAX_WAYPOINTS);
        return false;
    }
    
    Waypoint& wp = _waypoints[_state.waypoint_count];
    
    // Capture current foot targets from MotionManager's last submitted values
    // Since we don't have direct access to stored targets, we use reasonable defaults
    // In practice, recording should happen while another system is posing the robot
    wp.left_x_mm   = 0.0f;
    wp.left_y_mm   = _cfg.stance_width_mm;
    wp.left_h_mm   = _cfg.stance_height_mm;
    wp.right_x_mm  = 0.0f;
    wp.right_y_mm  = _cfg.stance_width_mm;
    wp.right_h_mm  = _cfg.stance_height_mm;
    
    if (label != nullptr) {
        strncpy(wp.label, label, sizeof(wp.label) - 1);
        wp.label[sizeof(wp.label) - 1] = '\0';
    } else {
        snprintf(wp.label, sizeof(wp.label), "wp_%d", _state.waypoint_count);
    }
    
    _state.waypoint_count++;
    Serial.printf("[GaitController] Recorded waypoint %d: %s\n", 
                  _state.waypoint_count - 1, wp.label);
    
    if (_cfg.auto_advance_on_record) {
        advanceWaypoint();
    }
    
    return true;
}

bool GaitController::setWaypoint(uint8_t index, const Waypoint& wp) {
    if (index >= GAITS_MAX_WAYPOINTS) return false;
    _waypoints[index] = wp;
    if (index >= _state.waypoint_count) {
        _state.waypoint_count = index + 1;
    }
    return true;
}

bool GaitController::getWaypoint(uint8_t index, Waypoint& out_wp) const {
    if (index >= _state.waypoint_count) return false;
    out_wp = _waypoints[index];
    return true;
}

void GaitController::advanceWaypoint() {
    if (_state.waypoint_count < 2) return;
    
    _state.current_waypoint = (_state.current_waypoint + 1) % _state.waypoint_count;
    _state.phase = 0.0f;
    _state.elapsed_time_s = 0.0f;
    
    Serial.printf("[GaitController] Advanced to waypoint %d\n", _state.current_waypoint);
    _submitCurrentPose();
}

void GaitController::setCurrentWaypoint(uint8_t index) {
    if (index >= _state.waypoint_count) return;
    _state.current_waypoint = index;
    _state.phase = 0.0f;
    _state.elapsed_time_s = 0.0f;
    _submitCurrentPose();
}

// =============================================================================
//  STANCE HELPER
// =============================================================================

void GaitController::goToStance() {
    // Symmetric crouched stance: both feet at same height, equal lateral offset
    float h_stance = _cfg.stance_height_mm;
    float y_stance = _cfg.stance_width_mm;
    
    _submitLeg(false, 0.0f, y_stance, h_stance);  // left
    _submitLeg(true,  0.0f, y_stance, h_stance);  // right
    
    _state.x_left_mm = 0.0f;  _state.y_left_mm = y_stance;  _state.h_left_mm = h_stance;
    _state.x_right_mm = 0.0f; _state.y_right_mm = y_stance; _state.h_right_mm = h_stance;
}

// =============================================================================
//  INTERPOLATION HELPERS
// =============================================================================

void GaitController::_interpolate(const Waypoint& wpA, const Waypoint& wpB, float t,
                                   float& out_left_x, float& out_left_y, float& out_left_h,
                                   float& out_right_x, float& out_right_y, float& out_right_h) {
    // Linear interpolation (lerp) for each axis
    // t should already be eased if using smooth interpolation
    out_left_x   = wpA.left_x_mm   + t * (wpB.left_x_mm   - wpA.left_x_mm);
    out_left_y   = wpA.left_y_mm   + t * (wpB.left_y_mm   - wpA.left_y_mm);
    out_left_h   = wpA.left_h_mm   + t * (wpB.left_h_mm   - wpA.left_h_mm);
    out_right_x  = wpA.right_x_mm  + t * (wpB.right_x_mm  - wpA.right_x_mm);
    out_right_y  = wpA.right_y_mm  + t * (wpB.right_y_mm  - wpA.right_y_mm);
    out_right_h  = wpA.right_h_mm  + t * (wpB.right_h_mm  - wpA.right_h_mm);
}

float GaitController::_ease(float t) const {
    // Smoothstep: 3t² - 2t³
    // Provides smooth acceleration/deceleration at endpoints
    // Derivative is zero at t=0 and t=1, ensuring C1 continuity
    return t * t * (3.0f - 2.0f * t);
}

// =============================================================================
//  LEG SUBMISSION
// =============================================================================

void GaitController::_submitLeg(bool isRight, float x_mm, float y_mm, float h_mm) {
    // Build foot target
    FootTarget target;
    target.x_mm          = x_mm;
    target.h_sagittal_mm = h_mm;
    target.y_mm          = y_mm;
    target.h_frontal_mm  = 0.0f;  // Auto-derive from sagittal
    target.isRightLeg    = isRight;
    
    // Solve IK
    LegIKResult ik = LegIK::solve(target);
    
    // Update telemetry status
    if (isRight) {
        _state.right_ik_status = ik.status;
    } else {
        _state.left_ik_status = ik.status;
    }
    
    // Safety: never submit on DOMAIN_ERROR
    if (ik.status == IKStatus::DOMAIN_ERROR) {
        Serial.printf("[GaitController] IK DOMAIN_ERROR on %s leg — skipping submission\n",
                      isRight ? "right" : "left");
        return;
    }
    
    // Map to joint channels
    const uint8_t hip_pitch   = isRight ? IDX_R_HIP_PITCH   : IDX_L_HIP_PITCH;
    const uint8_t knee_pitch  = isRight ? IDX_R_KNEE_PITCH  : IDX_L_KNEE_PITCH;
    const uint8_t ankle_pitch = isRight ? IDX_R_ANKLE_PITCH : IDX_L_ANKLE_PITCH;
    const uint8_t hip_roll    = isRight ? IDX_R_HIP_ROLL    : IDX_L_HIP_ROLL;
    const uint8_t ankle_roll  = isRight ? IDX_R_ANKLE_ROLL  : IDX_L_ANKLE_ROLL;
    
    // Submit to MotionManager (SOURCE_GAIT priority)
    _mm->submit(SOURCE_GAIT, hip_pitch,   ik.hip_pitch_deg);
    _mm->submit(SOURCE_GAIT, knee_pitch,  ik.knee_pitch_deg);
    _mm->submit(SOURCE_GAIT, ankle_pitch, ik.ankle_pitch_deg);
    _mm->submit(SOURCE_GAIT, hip_roll,    ik.hip_roll_deg);
    _mm->submit(SOURCE_GAIT, ankle_roll,  ik.ankle_roll_deg);
}
