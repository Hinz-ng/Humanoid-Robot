#ifndef ROBOT_GEOMETRY_H
#define ROBOT_GEOMETRY_H

// Leg Segment Lengths (mm)
const float L_THIGH = 96.0f;
const float L_SHANK = 95.4f;
const float L_ANKLE_TO_SOLE = 15.4f; // Vertical offset

// Ankle Linkage Geometry (mm)
const float LINK_ROD = 50.6f;
const float LINK_SERVO_CRANK = 20.2f;
const float LINK_OUTPUT_CRANK = 19.9f;

// =============================================================================
//  AXIS GEOMETRY — axis-to-axis measurements, leg fully extended, robot upright.
//  These supplement the linkage lengths above.
// =============================================================================

// Frontal chain: hip ROLL axis → ankle ROLL axis (axis-to-axis, straight leg).
// DIRECTLY CONFIRMED. Do not derive from sagittal chain ± per-axis offsets;
// those combine to 167.5mm, which disagrees with this measurement by 1.5mm.
// When in doubt, the direct chain measurement wins.
const float L_FRONTAL_MM = 166.0f;

// Net structural height delta: sagittal chain − frontal chain at full extension.
// Used to derive h_frontal from h_sagittal at ANY knee angle:
//   h_frontal = h_sagittal − CHAIN_HEIGHT_DELTA_MM
// Recalculate only if L_THIGH, L_SHANK, or L_FRONTAL_MM is re-measured.
const float CHAIN_HEIGHT_DELTA_MM = (L_THIGH + L_SHANK) - L_FRONTAL_MM;  // 25.4mm

// -- Informational per-joint axis offsets (NOT used in IK math) ---------------
// These describe internal joint positions, useful for CAD/documentation.
// IK uses CHAIN_HEIGHT_DELTA_MM (derived from confirmed chain lengths) instead.
// Hip pitch axis is this many mm ABOVE hip roll axis (same vertical column).
const float HIP_PITCH_OVER_ROLL_MM   = 29.8f;
// Ankle pitch axis is this many mm ABOVE ankle roll axis.
const float ANKLE_PITCH_OVER_ROLL_MM = 5.9f;
// Note: 29.8 − 5.9 = 23.9mm ≠ CHAIN_HEIGHT_DELTA_MM (25.4mm).
// The 1.5mm gap is within expected measurement precision for individually
// measured small offsets vs. a single axis-to-axis chain measurement.

// Hip pitch axis protrudes this many mm FORWARD of ankle pitch axis (leg straight).
// Structural frame property — informs foot placement planning, not IK angles.
const float HIP_PITCH_FORE_AFT_OFFSET_MM = 12.8f;

// Foot sole midpoint is approximately this far forward of hip pitch axis at neutral.
const float FOOT_FORE_OF_HIP_PITCH_MM = 2.0f;

#endif