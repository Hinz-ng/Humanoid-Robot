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

// Squat Configuration
const float SQUAT_DEPTH_MAX = 50.0f;   // mm to lower
const float SQUAT_DURATION = 3000.0f; // ms for one full cycle (down/up)

#endif