// =============================================================================
// FILE:    leg_ik.h
// MODULE:  kinematics
// LAYER:   3.5 — Gait / Motion
//
// PURPOSE:
//   Closed-form 2D inverse kinematics for one 5-DOF leg.
//
//   Sagittal plane (XZ): hip_pitch + knee_pitch + ankle_pitch.
//     2-link law-of-cosines. Posterior-knee configuration.
//     Flat-foot constraint: ankle_pitch compensates the full sagittal chain.
//
//   Frontal plane (YZ): hip_roll + ankle_roll.
//     Single-rigid-link atan2 model.
//     Flat-foot constraint: ankle_roll = −hip_roll.
//
//   The two planes use SEPARATE height parameters — they must not be combined.
//
// GEOMETRY:
//   Sagittal: hip PITCH → ankle PITCH, L_THIGH + L_SHANK = 191.4mm.
//   Frontal:  hip ROLL  → ankle ROLL,  L_FRONTAL_MM       = 166.0mm.
//   Relationship: h_frontal = h_sagittal − CHAIN_HEIGHT_DELTA_MM (= 25.4mm).
//   This delta is derived from confirmed chain lengths, not from per-axis offsets.
//
// SIGN CONVENTIONS (joint-relative, positive = anatomical flexion):
//   hip_pitch:   + = thigh FORWARD (hip flexion)
//   knee_pitch:  + = knee BENDS (flexion; 0 at full extension, always >= 0)
//   ankle_pitch: + = DORSIFLEXION (toes up; compensates backward shank)
//   hip_roll:    + = leg OUTWARD (abduction)
//   ankle_roll:  + = INVERSION — verify direction on first hardware run.
//                    Expected: ankle_roll = −hip_roll keeps sole flat.
//
// INTEGRATION:
//   Submit output via MotionManager::submit(SOURCE_GAIT, IDX_*, angleDeg).
//   JointModel direction fields in joint_config.cpp apply servo mirroring for
//   left-side joints automatically. Submit IDENTICAL anatomical angles for both
//   legs — do NOT negate for the left leg in calling code.
//
// CALIBRATION NOTE:
//   This module computes geometry only. Neutral servo angles are calibration
//   data stored in joint_config.cpp, not embedded here. validateNeutral()
//   takes expected angles as parameters — the caller is responsible for
//   supplying measured values.
//
// DEPENDENCIES:
//   robot_geometry.h — all chain lengths and geometry constants
//
// LAST CHANGED: 2026-04-21 | Corrected: L_FRONTAL_MM direct measurement,
//                             CHAIN_HEIGHT_DELTA_MM replaces per-axis offsets.
// =============================================================================

#ifndef LEG_IK_H
#define LEG_IK_H

#include <Arduino.h>
#include "robot_geometry.h"
#include "gait_types.h"   // canonical FootTarget + BalanceCorrection

// =============================================================================
//  IK-SPECIFIC SOLVER CONSTANTS
//  These tune solver conservatism, not robot geometry.
// =============================================================================

// Maximum fraction of full leg extension the solver may target.
// Prevents the kinematic singularity region where ∂θ/∂r → ∞ near full extension.
// At 0.99: max sagittal reach ≈ 189.5mm vs full 191.4mm.
// Reduce toward 0.95 if oscillation occurs near full extension.
static constexpr float IK_MAX_REACH_FRACTION = 0.99f;

// Soft joint limits (degrees, joint-relative).
// Must remain inside the hard limits in joint_config.cpp.
// Widen only after hardware testing confirms the full range is safe.
static constexpr float IK_HIP_PITCH_MIN_DEG   = -20.0f;
static constexpr float IK_HIP_PITCH_MAX_DEG   =  60.0f;
static constexpr float IK_KNEE_MIN_DEG        =   0.0f;  // knee never hyperextends
static constexpr float IK_KNEE_MAX_DEG        =  90.0f;
static constexpr float IK_ANKLE_PITCH_MIN_DEG = -50.0f;
static constexpr float IK_ANKLE_PITCH_MAX_DEG =  50.0f;
static constexpr float IK_HIP_ROLL_MIN_DEG    = -15.0f;
static constexpr float IK_HIP_ROLL_MAX_DEG    =  20.0f;
static constexpr float IK_ANKLE_ROLL_MIN_DEG  = -20.0f;
static constexpr float IK_ANKLE_ROLL_MAX_DEG  =  20.0f;

// =============================================================================
//  STATUS
// =============================================================================

enum class IKStatus : uint8_t {
    OK            = 0,  // Valid solution, all limits satisfied.
    UNREACHABLE   = 1,  // Target exceeded reach; input clamped, solution valid.
    LIMIT_CLAMPED = 2,  // Solution found; ≥1 joint clamped to soft limit.
                        // Check flat_foot_error_deg — nonzero if ankle was clamped.
    DOMAIN_ERROR  = 3,  // acos/asin argument out of domain, or h < safety floor.
                        // Output is zero-initialised. Do NOT submit.
};

// =============================================================================
//  INPUT — defined in gait_types.h
// =============================================================================
// FootTarget pulled from gait_types.h — see that file for sign conventions.

// =============================================================================
//  OUTPUT
// =============================================================================

struct LegIKResult {
    // ── Joint angles (degrees, joint-relative from neutral) ───────────────────
    // Submit directly: motionManager.submit(SOURCE_GAIT, IDX_*, angleDeg)
    float hip_pitch_deg   = 0.0f;
    float knee_pitch_deg  = 0.0f;
    float ankle_pitch_deg = 0.0f;  // enforced by flat-foot constraint
    float hip_roll_deg    = 0.0f;
    float ankle_roll_deg  = 0.0f;  // enforced by flat-foot constraint (= −hip_roll)

    IKStatus status = IKStatus::OK;

    // ── Diagnostics ───────────────────────────────────────────────────────────
    float sagittal_reach_mm   = 0.0f;  // actual input reach magnitude (mm)
    float sagittal_reach_pct  = 0.0f;  // % of full extension (100% = danger)
    float flat_foot_error_deg = 0.0f;  // deviation from ideal flat-foot ankle.
                                       // Zero if no joint limit was hit.
                                       // Abort gait step if this exceeds ~2°.
    bool any_joint_at_limit   = false;

    bool ok() const {
        return status == IKStatus::OK || status == IKStatus::LIMIT_CLAMPED;
    }
};

// =============================================================================
//  SOLVER
// =============================================================================

class LegIK {
public:
    // Solve IK for one leg. Pure function — no internal state.
    static LegIKResult solve(const FootTarget& target);

    // FK helper: given hip_pitch and knee_pitch (degrees, joint-relative),
    // returns the ankle PITCH axis position relative to the hip PITCH axis.
    //   x_out_mm: mm, forward positive
    //   h_out_mm: mm, downward positive
    // Public for external validation and gait planning.
    static void forwardSagittal(float hip_pitch_deg, float knee_pitch_deg,
                                float& x_out_mm, float& h_out_mm);

    // Roundtrip test: solve IK, apply FK, compare foot position to target.
    // Prints full diagnostics to Serial. Returns true if position error < toleranceMm
    // AND flat_foot_error < 0.1°.
    // Run once from setup() before using IK in any motion loop.
    static bool validateRoundtrip(const FootTarget& target,
                                  float toleranceMm = 0.5f);

    // Neutral comparison: given expected joint angles (measured from hardware),
    // compute the foot position they imply via FK, then run IK and verify we
    // recover those angles within toleranceDeg.
    // expectedHip/Knee/Ankle must come from physical measurement, not from this
    // module. This function performs geometry validation, not calibration.
    static bool validateNeutral(float expectedHipPitchDeg,
                                float expectedKneeDeg,
                                float expectedAnkleDeg,
                                float toleranceDeg = 1.5f);

private:
    struct SagittalSol {
        float    hip_pitch_deg   = 0.0f;
        float    knee_pitch_deg  = 0.0f;
        float    ankle_pitch_deg = 0.0f;
        float    reach_mm        = 0.0f;
        IKStatus status          = IKStatus::OK;
    };

    struct FrontalSol {
        float    hip_roll_deg   = 0.0f;
        float    ankle_roll_deg = 0.0f;
        IKStatus status         = IKStatus::OK;
    };

    static SagittalSol solveSagittal(float x_mm, float h_mm);
    static FrontalSol  solveFrontal (float y_mm, float h_frontal_mm);
    static float       clampJoint   (float deg, float lo, float hi,
                                     bool& clampedFlag);
};

#endif // LEG_IK_H