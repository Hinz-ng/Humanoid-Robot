// =============================================================================
// FILE:    foot_impact.h
// MODULE:  Foot Impact Detection (per-foot)
// LAYER:   3.5 — Stateful detector (peer of foot_contact, swing_foot_leveler)
//
// PURPOSE: Detect transient foot-ground impact events from per-foot IMU
//          accelerometer data. Publishes a per-side "detected this tick"
//          flag plus diagnostic telemetry. No control coupling — consumer
//          policy lives in whichever module subscribes (none at L0).
//
// INPUTS:  Two RawIMUData snapshots per tick — left foot and right foot,
//          AFTER the main.cpp axis remap to robot frame. Raw counts
//          (int16) on the accel axes; scale constant
//          FOOT_IMU_ACCEL_SCALE_LSB_PER_G converts to g.
//
// OUTPUTS: Per-side scalars:
//            - lastMagnitude_g       last filtered |a| this tick, in g
//            - lastDelta_g           last first-difference of filtered |a|, in g
//            - detected              sticky for one tick after an event
//            - armed                 enabled && seeded && refractory expired
//            - msSinceLastImpact     time since detected last rose, in ms
//          (No "phase at last impact" field — detector is gait-agnostic.
//           UI composes phase by latching GAIT:phase when detected rises.)
//
// SIGNAL:  Per side, per tick:
//            a_mag_g       = sqrt(ax^2 + ay^2 + az^2) / SCALE_LSB_PER_G
//            a_mag_filt_g  = alpha * a_mag_g + (1 - alpha) * prev_filt_g
//            delta_g       = a_mag_filt_g - prev_filt_g     (first difference)
//          Trigger when |delta_g| > threshold AND refractory expired.
//          Rationale (see foot_impact.cpp for derivation): stance foot sits
//          at |a|=1g approximately constant; airborne foot sits near a
//          different but also approximately constant level during normal
//          swing (centripetal at this scale is < 0.05 g). Only the impact
//          transient produces large per-tick deltas. Magnitude (not z-axis)
//          is used so the signal is rotation-invariant.
//
// DEPENDENCIES:
//   include/imu.h              (RawIMUData) — same struct used by torso & feet
//   include/project_wide_defs.h (FOOT_IMU_ACCEL_SCALE_LSB_PER_G,
//                                FOOT_IMPACT_* defaults)
//   include/side.h             (Side enum, SIDE_COUNT, sideIdx)
//
// LAST CHANGED: 2026-05-18 | Hinz | Initial L0 implementation — telemetry only.
// =============================================================================
#ifndef FOOT_IMPACT_H
#define FOOT_IMPACT_H

#include <Arduino.h>
#include "imu.h"   // RawIMUData
#include "side.h"        // Side, SIDE_COUNT

// -----------------------------------------------------------------------------
// FootImpactConfig — runtime-tunable parameters.
//
// Defaults are mirrored from project_wide_defs.h. Tune via
// CMD:FOOT_IMPACT_TUNE in WebComm. All fields validated by the parser
// before assignment.
// -----------------------------------------------------------------------------
struct FootImpactConfig {
    // Master enable. At L0 default is true so telemetry flows immediately
    // when the operator opens the panel — actuation is a separate concern
    // that does not exist at L0. Toggle off only if telemetry traffic is a
    // problem (it should not be — single small packet at rate-limited cadence).
    bool  enabled = true;                  // default loaded in setConfig()

    // Detection threshold on |delta_g|. The right value depends on
    // empirically-measured spike shape; default is conservative and
    // EXPECTED to be re-tuned after L0 telemetry capture.
    float threshold_g = 0.15f;              // delta magnitude, g

    // First-order LPF on the accel magnitude before differencing.
    //   alpha = 1.0  =>  no smoothing (pass-through)
    //   alpha = 0.5  =>  ~32 Hz cutoff at 400 Hz tick rate (default)
    //   alpha = 0.1  =>  heavy smoothing; will compress real impact spikes
    // Heavier smoothing reduces noise but also attenuates the genuine
    // transient. The L0 default trades a small amount of noise tolerance
    // for higher signal fidelity.
    float lpf_alpha = 0.5f;                // dimensionless, [0..1]

    // After a detection, suppress further detections on the same foot for
    // this long. Prevents one physical impact (which may oscillate the
    // sensor for several ticks) from registering as multiple events.
    uint32_t refractory_ms = 200;          // ms
};

// -----------------------------------------------------------------------------
// FootImpactState — per-side diagnostic snapshot. One copy per Side.
//
// All fields are populated every tick. `detected` is the only field that
// carries a one-tick semantic (rising edge); the rest are continuous.
// -----------------------------------------------------------------------------
struct FootImpactSideState {
    bool     valid              = false;    // mirrors RawIMUData::valid
    bool     armed              = false;    // enabled && seeded && refractory expired
    bool     detected           = false;    // STICKY: true for one tick on event
    float    magnitude_g        = 0.0f;     // current |a| filtered, g
    float    delta_g            = 0.0f;     // current |a|_filt first difference, g
    uint32_t detect_count       = 0;        // cumulative detections since reset
    uint32_t ms_since_last      = 0;        // ms since last detect rising edge
    float    last_magnitude_g   = 0.0f;     // |a|_filt at last detect
    float    last_delta_g       = 0.0f;     // delta_g at last detect
};

// -----------------------------------------------------------------------------
// FootImpactDetector
//
// Owns per-side filter state. One instance globally (in main.cpp). Updated
// once per tick from main.cpp's 400 Hz gate, after the foot IMU axis remap.
//
// Architectural note: The detector consumes RAW remapped accel counts,
// NOT IMUState orientation. This is intentional — the impact signal is a
// short-duration acceleration transient, and routing it through the
// complementary filter would attenuate exactly the band of interest.
// The detector is therefore independent of state_estimator's calibration
// progress, which means it goes live as soon as RawIMUData.valid is true.
// -----------------------------------------------------------------------------
class FootImpactDetector {
public:
    FootImpactDetector() = default;

    // Apply config from project_wide_defs.h. Call once in setup() after the
    // detector is constructed but before the first update() tick.
    void init();

    // Hard reset of all transient state. Filter state cleared, refractory
    // cleared, detect_count zeroed. Call from oe_clear() in main.cpp.
    // Does NOT modify _cfg.
    void reset();

    // Per-tick update. Pass the post-remap RawIMUData for each foot
    // exactly once per tick. Both arguments may be invalid (valid=false);
    // in that case the corresponding side's state is held but armed=false.
    void update(const RawIMUData& leftFoot, const RawIMUData& rightFoot);

    // Read-only accessors.
    const FootImpactSideState& getState(Side s) const { return _state[sideIdx(s)]; }

    // Live config get/set. Setter validates each field; out-of-range
    // values are clamped (see foot_impact.cpp::_clampConfig).
    const FootImpactConfig& getConfig() const          { return _cfg; }
    void                    setConfig(const FootImpactConfig& c);

private:
    FootImpactConfig     _cfg;
    FootImpactSideState  _state[SIDE_COUNT];

    // Per-side filter & timing state. Held separately from public state
    // so the public struct stays small.
    struct PrivSide {
        bool     seeded             = false;  // false until first valid sample seen
        float    a_mag_filt_g       = 0.0f;   // LPF state
        float    prev_a_mag_filt_g  = 0.0f;   // previous tick's LPF output (for delta)
        uint32_t refractory_until_ms = 0;     // millis() value at which arming resumes
        uint32_t last_detect_ms     = 0;      // 0 until first detection
    };
    PrivSide _priv[SIDE_COUNT];

    void _updateSide(Side s, const RawIMUData& raw, uint32_t now_ms);
    static FootImpactConfig _clampConfig(FootImpactConfig c);
};

#endif // FOOT_IMPACT_H
