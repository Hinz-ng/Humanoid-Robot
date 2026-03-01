#include "ik_math.h"
#include "robot_geometry.h"
#include <math.h>
#include <Arduino.h>

LegAngles IK_Solver::solveLeg(float x, float z) {
    LegAngles result;
    
    // Distance from Hip to Ankle
    float d_sq = x*x + z*z;
    float d = sqrt(d_sq);

    // Check reachability
    if (d > (L_THIGH + L_SHANK) || d < fabsf(L_THIGH - L_SHANK)) {
        result.reachable = false;
        return result;
    }

    // Law of Cosines for Knee
    float cosKnee = (L_THIGH*L_THIGH + L_SHANK*L_SHANK - d_sq) / (2 * L_THIGH * L_SHANK);
    float kneeRad = acos(constrain(cosKnee, -1.0f, 1.0f));
    
    // Law of Cosines for Hip
    float alpha = atan2(x, -z); // Angle to the ankle point
    float cosBeta = (L_THIGH*L_THIGH + d_sq - L_SHANK*L_SHANK) / (2 * L_THIGH * d);
    float beta = acos(constrain(cosBeta, -1.0f, 1.0f));

    // Convert to Degrees
    result.kneePitch = 180.0f - (kneeRad * 180.0f / M_PI); // Internal angle to joint angle
    result.hipPitch = (alpha + beta) * 180.0f / M_PI;
    
    // Foot-Flat: Ankle pitch must cancel Hip and Knee
    // This keeps the sole parallel to the Hip reference plane (torso)
    result.anklePitch = -(result.hipPitch - result.kneePitch); 
    
    result.reachable = true;
    return result;
}

float IK_Solver::solveAnkleLinkage(float targetAnkleAngle) {
    // For a 1:1-ish linkage, we use a linear approximation for now.
    // In a final build, you would use a lookup table or a Law of Cosines 
    // solver for the 4-bar linkage.
    return targetAnkleAngle; 
}