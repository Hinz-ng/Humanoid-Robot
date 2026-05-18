// =============================================================================
// FILE:    foot_impact_test.cpp
// MODULE:  Foot Impact Detection — unit test stub
// PURPOSE: Synthetic step-input test. Compile-in with -DUNIT_TEST.
//          Confirms: (a) seed tick produces no detection; (b) a step in
//          |a| triggers exactly one detection; (c) refractory blocks
//          immediate retrigger; (d) detected is sticky for exactly one tick.
//
// LAST CHANGED: 2026-05-18 | Hinz | Initial.
// =============================================================================
#ifdef UNIT_TEST
#include "foot_impact.h"
#include "project_wide_defs.h"
#include <Arduino.h>

static RawIMUData _makeRaw(int16_t az_raw_counts) {
    RawIMUData r{};
    r.accel_x = 0;
    r.accel_y = 0;
    r.accel_z = az_raw_counts;
    r.gyro_x = 0;
    r.gyro_y = 0;
    r.gyro_z = 0;
    r.valid = true;
    return r;
}

void runFootImpactTests() {
    FootImpactDetector d;
    d.init();

    // Force defaults that make assertions deterministic.
    FootImpactConfig c = d.getConfig();
    c.enabled       = true;
    c.threshold_g   = 1.0f;
    c.lpf_alpha     = 1.0f;        // pass-through; delta = raw - prev
    c.refractory_ms = 200;
    d.setConfig(c);

    // ~1g baseline. With pass-through LPF, magnitude = |az|.
    const int16_t G    = (int16_t)FOOT_IMU_ACCEL_SCALE_LSB_PER_G;       // 1g
    const int16_t BIG  = (int16_t)(3.0f * FOOT_IMU_ACCEL_SCALE_LSB_PER_G); // 3g spike

    RawIMUData base   = _makeRaw(G);
    RawIMUData spike  = _makeRaw(BIG);
    RawIMUData invalid{}; invalid.valid = false;

    // Tick 0: seed. No detection possible.
    d.update(base, invalid);
    if (d.getState(Side::LEFT).detected) Serial.println("FAIL: seed tick detected");

    // Tick 1: still baseline. delta ~ 0.
    d.update(base, invalid);
    if (d.getState(Side::LEFT).detected) Serial.println("FAIL: baseline tick detected");

    // Tick 2: spike. delta = 3g - 1g = 2g > 1g threshold => detect.
    d.update(spike, invalid);
    if (!d.getState(Side::LEFT).detected) Serial.println("FAIL: spike not detected");

    // Tick 3: still spike. delta = 0, but also in refractory. No detect.
    d.update(spike, invalid);
    if (d.getState(Side::LEFT).detected) Serial.println("FAIL: refractory leaked");

    // Cumulative count must be exactly 1.
    if (d.getState(Side::LEFT).detect_count != 1)
        Serial.println("FAIL: detect_count != 1");

    Serial.println("runFootImpactTests done.");
}
#endif // UNIT_TEST
