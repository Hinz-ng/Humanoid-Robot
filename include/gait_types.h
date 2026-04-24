// Module:      gait_types
// Layer:       3.5 (shared type definitions)
// Purpose:     Canonical structs for GaitController ↔ IK ↔ BalanceController boundary
// Inputs:      (none — type definitions only)
// Outputs:     FootTarget, BalanceCorrection
// Dependencies: (none)

#pragma once

struct FootTarget {
    float x_mm;    // forward/back foot position relative to hip
    float y_mm;    // lateral foot position relative to hip
    float h_mm;    // foot height (0 = ground contact)
    bool  valid;   // false → do not command this leg this tick
};

struct BalanceCorrection {
    float dx_mm;       // forward/back body correction
    float dy_mm;       // lateral body correction
    float dPitch_deg;  // pitch trim applied to foot targets
    float dRoll_deg;   // roll trim applied to foot targets
    bool  valid;
};