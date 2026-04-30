// =============================================================================
// FILE:    gait_types.h
// MODULE:  gait_types
// LAYER:   3.5 — shared type definitions (no logic)
//
// PURPOSE:
//   Canonical structs for the GaitController → LegIK → BalanceController
//   boundary. Defining these here prevents circular includes and gives
//   every consumer one source of truth.
//
//   FootTarget mirrors UVC's footCont(x, y, h, s) inputs. A FootTarget is
//   intent — what the controller wants the foot pose to be, in mm relative
//   to the hip. LegIK::solve() converts intent into joint angles.
// =============================================================================

#ifndef GAIT_TYPES_H
#define GAIT_TYPES_H

// FootTarget — one leg's commanded foot pose, hip-relative.
//
// Sign / unit conventions match leg_ik.cpp; do not change without re-reading
// the IK math comment block at the top of leg_ik.cpp.
//
//   x_mm           forward(+) / backward(-) from hip pitch axis
//   y_mm           outward(+) / inward(-) in the leg's own frontal frame
//   h_sagittal_mm  hip pitch → ankle pitch vertical drop. Always positive.
//                  Safe range: ~130–189 mm. Full extension = 191.4 mm.
//   h_frontal_mm   hip roll → ankle roll vertical drop. Set to 0 to
//                  auto-derive from h_sagittal via CHAIN_HEIGHT_DELTA_MM.
//   isRightLeg     diagnostic labelling only; not used by IK math.
//   valid          false → caller should skip submitting this leg this tick.
struct FootTarget {
    float x_mm          = 0.0f;
    float h_sagittal_mm = 160.0f;
    float y_mm          = 0.0f;
    float h_frontal_mm  = 0.0f;
    bool  isRightLeg    = true;
    bool  valid         = true;
};

// BalanceCorrection — task-space corrections from BalanceController, to be
// added to the FootTarget by the merge step BEFORE LegIK::solve().
//
//   Stage 3 only — unused in Stage 1. Defined here so the GaitController
//   interface accepting FootTarget can later accept (FootTarget + correction)
//   without renaming.
struct BalanceCorrection {
    float dx_mm      = 0.0f;
    float dy_mm      = 0.0f;
    float dPitch_deg = 0.0f;  // reserved for future trunk-trim; unused in Stage 3
    float dRoll_deg  = 0.0f;  // reserved for future trunk-trim; unused in Stage 3
    bool  valid      = false;
};

// mergeBalanceCorrection — apply a BalanceCorrection to a FootTarget in place.
//
// Sign conventions:
//   dx_mm > 0  → balance pushes body forward → foot target shifts BACKWARD
//                (foot stays planted while body comes forward over it).
//   dy_mm > 0  → balance pushes body to the RIGHT.
//                In hip-relative frame:
//                  right leg: y_mm increases (right foot moves outward / abduction)
//                  left  leg: y_mm decreases (left  foot moves inward  / adduction)
//                Net effect: body translates right relative to feet.
//   dPitch_deg, dRoll_deg — reserved, not applied in Stage 3.
//
// Returns true if the correction was applied.
// Returns false (no-op) when bc.valid == false.
inline bool mergeBalanceCorrection(FootTarget& target,
                                    const BalanceCorrection& bc) {
    if (!bc.valid) return false;
    target.x_mm -= bc.dx_mm;
    target.y_mm += target.isRightLeg ? +bc.dy_mm : -bc.dy_mm;
    return true;
}

#endif // GAIT_TYPES_H
