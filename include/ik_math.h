#ifndef IK_MATH_H
#define IK_MATH_H

struct LegAngles {
    float hipPitch;
    float kneePitch;
    float anklePitch;
    bool reachable;
};

class IK_Solver {
public:
    // x, z: desired ankle position relative to hip
    static LegAngles solveLeg(float x, float z);
    
    // Converts desired joint angle to the required servo angle via linkage
    static float solveAnkleLinkage(float targetAnkleAngle);
};

#endif