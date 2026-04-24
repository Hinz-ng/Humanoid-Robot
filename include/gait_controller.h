// =============================================================================
// FILE:    gait_controller.h
// MODULE:  gait
// LAYER:   3.5 — Gait / Motion
//
// PURPOSE:
//   Waypoint-based walking gait generator with recording/playback capability.
//   
//   DESIGN PHILOSOPHY:
//   - Replace parametric sin/cos gait with explicit key poses (waypoints)
//   - Each waypoint defines foot targets (x, y, h) for both legs
//   - Smooth interpolation between waypoints over time
//   - Easy to debug: inspect waypoint sequence, step through manually
//   - Recording mode: capture current IK targets as new waypoints
//
//   ROBUST WAYPOINT STRATEGY (for stable walking):
//   1. Pre-Crouch: Both legs reduce h to ~170mm (never walk on straight legs)
//   2. Shift: Shift y to the stance leg, wait for oscillations to settle
//   3. Lift (Waypoint 1): Decrease h on swing leg to lift it, x remains 0
//   4. Swing (Waypoint 2): Increase x to move foot forward, h stays elevated
//   5. Place (Waypoint 3): Increase h to plant the foot
//   6. Shift & Recover: Shift weight back to center, reset for next step
//
//   ARCHITECTURE:
//     GaitController submits SOURCE_GAIT (priority 1).
//     BalanceController submits SOURCE_BALANCE (priority 2, wins on shared joints).
//     For open-loop walking: disable balance (CMD:PITCH_OFF, CMD:ROLL_OFF).
//     For closed-loop walking later: enable balance; it corrects on top of gait.
//
// INPUTS:  WaypointSequence, GaitConfig (tunable at runtime), float dt_s
// OUTPUTS: IK-solved joint angles via MotionManager::submit(SOURCE_GAIT, ...)
//
// DEPENDENCIES:
//   leg_ik.h         — LegIK::solve(), FootTarget, LegIKResult, IKStatus
//   motion_manager.h — MotionManager::submit(), SOURCE_GAIT
//   oe_control.h     — oe_is_estopped() gate
//   joint_config.h   — IDX_* channel constants
//
// LAST CHANGED: 2026-04-23 | New waypoint-based implementation
// =============================================================================

#ifndef GAIT_CONTROLLER_H
#define GAIT_CONTROLLER_H

#include <Arduino.h>
#include "leg_ik.h"
#include "motion_manager.h"

// =============================================================================
//  CONFIGURATION LIMITS
// =============================================================================

// Maximum number of waypoints in a recorded sequence.
// ESP32 has plenty of RAM; 32 waypoints = ~2KB, very safe.
// Adjust if you need longer sequences.
static constexpr uint8_t GAITS_MAX_WAYPOINTS = 32;

// Default crouch height for stable walking stance (mm).
// This is the "pre-crouch" position: both legs bent, ready to step.
// Must be >= 130mm (IK minimum) and <= 185mm (singularity avoidance).
// At 170mm: knee ≈ 50° — good clearance, stable base. ✓
static constexpr float GAIT_DEFAULT_CROUCH_H_MM = 170.0f;

// =============================================================================
//  Waypoint — one key pose in the gait sequence
// =============================================================================
struct Waypoint {
    // Left foot target in body frame (mm)
    float left_x_mm   = 0.0f;   // forward/backward (+forward)
    float left_y_mm   = 25.0f;  // lateral offset (+outward from centerline)
    float left_h_mm   = 160.0f; // sagittal chain height (hip pitch → ankle pitch)

    // Right foot target in body frame (mm)
    float right_x_mm  = 0.0f;
    float right_y_mm  = 25.0f;
    float right_h_mm  = 160.0f;

    // Duration to reach this waypoint from the previous one (seconds).
    // First waypoint's duration is the time to hold it before moving.
    float duration_s  = 0.5f;

    // Optional: human-readable label for debugging/telemetry.
    // Stored as fixed-size array to avoid dynamic allocation.
    char label[24] = "";
};

// =============================================================================
//  GaitMode — current operating state of the gait controller
// =============================================================================
enum class GaitMode : uint8_t {
    IDLE,           // Not running; no output
    STANCE,         // Holding a static crouched stance
    PLAYBACK,       // Playing back a recorded waypoint sequence (looping)
    RECORDING,      // Capturing waypoints from live IK commands
    STEPPING        // Manual step-through mode (advance one waypoint per command)
};

// =============================================================================
//  GaitConfig — tunable parameters for playback and stance
// =============================================================================
struct GaitConfig {
    // Stance configuration
    float stance_height_mm = GAIT_DEFAULT_CROUCH_H_MM;  // crouch height for stance
    float stance_width_mm  = 25.0f;   // each foot's lateral offset from centerline

    // Playback configuration
    float playback_speed_multiplier = 1.0f;  // 0.5 = half speed, 2.0 = double
    bool  loop_playback             = true;  // if false, stop at end of sequence

    // Interpolation: use eased (smooth) or linear interpolation
    // Eased: smoother acceleration/deceleration at waypoints
    // Linear: constant velocity, easier to predict timing
    bool use_eased_interpolation = true;

    // Recording configuration
    bool auto_advance_on_record = false;  // if true, advance phase after each recorded waypoint
};

// =============================================================================
//  GaitState — read-only telemetry
// =============================================================================
struct GaitState {
    GaitMode mode              = GaitMode::IDLE;
    bool     active            = false;   // true while any gait mode is running
    uint8_t  current_waypoint  = 0;       // index of current target waypoint
    float    phase             = 0.0f;    // [0,1] interpolation progress within current segment
    uint8_t  waypoint_count    = 0;       // number of recorded waypoints (for playback/recording)
    float    elapsed_time_s    = 0.0f;    // time since start of current segment

    // Task-space foot targets (mm) — for telemetry display
    float x_right_mm = 0.0f;
    float h_right_mm = 0.0f;
    float y_right_mm = 0.0f;
    float x_left_mm  = 0.0f;
    float h_left_mm  = 0.0f;
    float y_left_mm  = 0.0f;

    // IK solution quality
    IKStatus right_ik_status = IKStatus::OK;
    IKStatus left_ik_status  = IKStatus::OK;
};

// =============================================================================
//  GaitController
// =============================================================================
class GaitController {
public:
    GaitController() = default;

    // Call once in setup(), AFTER motionManager.init().
    void init(MotionManager* mm);

    // Start/stop gait operation. Mode determines behavior.
    void start(GaitMode mode = GaitMode::PLAYBACK);
    void stop();

    // Call every tick in the 400Hz gate:
    //   gaitController.update(dt)
    //   motionManager.flush()
    void update(float dt_s);

    // ── Waypoint Sequence Management ───────────────────────────────────────
    // Clear all recorded waypoints
    void clearWaypoints();

    // Record current foot targets as a new waypoint.
    // Returns true if recorded, false if sequence is full.
    bool recordWaypoint(const char* label = nullptr);

    // Set a specific waypoint by index (for manual editing/debugging)
    bool setWaypoint(uint8_t index, const Waypoint& wp);

    // Get a waypoint by index (returns copy)
    bool getWaypoint(uint8_t index, Waypoint& out_wp) const;

    // Advance to next waypoint manually (for STEPPING mode)
    void advanceWaypoint();

    // Set current waypoint index (for jumping to specific pose)
    void setCurrentWaypoint(uint8_t index);

    // ── Configuration Access ───────────────────────────────────────────────
    void              setConfig(const GaitConfig& cfg) { _cfg = cfg; }
    const GaitConfig& getConfig() const                { return _cfg; }
    const GaitState&  getState()  const                { return _state; }

    // ── Stance Helper ──────────────────────────────────────────────────────
    // Move to a stable crouched stance posture.
    // This goes through the normal IK pipeline — safe and repeatable.
    void goToStance();

private:
    MotionManager* _mm = nullptr;
    GaitConfig     _cfg;
    GaitState      _state;

    // Waypoint storage (circular buffer for recording, array for playback)
    Waypoint _waypoints[GAITS_MAX_WAYPOINTS];

    // Interpolation helper: compute interpolated foot target between two waypoints
    void _interpolate(const Waypoint& wpA, const Waypoint& wpB, float t,
                      float& out_left_x, float& out_left_y, float& out_left_h,
                      float& out_right_x, float& out_right_y, float& out_right_h);

    // Ease function for smooth interpolation (smoothstep)
    float _ease(float t) const;

    // Submit SOURCE_GAIT commands for one leg
    void _submitLeg(bool isRight, float x_mm, float y_mm, float h_mm);
};

#endif // GAIT_CONTROLLER_H