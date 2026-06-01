// =============================================================================
// FILE:    walking_instrumentation.h
// MODULE:  diagnostics
// LAYER:   3.5 — Telemetry aggregator (peer of foot_impact, foot_contact)
//
// PURPOSE: Per-step walking diagnostics. Aggregates 17 metrics once per
//          completed step (one swing half-cycle) into a fixed 10-step ring
//          buffer so competing walking-bottleneck hypotheses (swing-leg
//          momentum, touchdown impulse, support-transfer, double-touchdown,
//          vertical CoM bob, reach saturation) can be compared from data
//          instead of by eye.
//
//          INSTRUMENTATION ONLY. This module is a read-only consumer of state
//          already computed earlier in the 400 Hz tick. It never submits joint
//          commands (no MotionManager) and never touches OE / e-stop. It feeds
//          no controller — the only "detection" it does (touchdown onset /
//          lift-off FSM) is for measurement and reads existing signals.
//
// INPUTS  (passed by const-ref each tick, see update()):
//   - IMUState torso          : measured body response (vx/vy CoM, rollRate, roll)
//   - IMUState left/right foot : foot gyro rates for the double-touchdown lift FSM
//   - FootImpactDetector       : already-filtered per-foot accel magnitude (g)
//   - GaitState                : step boundary (subState), commanded foot targets,
//                                reach %, flat-foot error, touchdown anchor
//
// OUTPUT: WalkingInstrState via getState() — current step, last-10 history, and
//         per-metric averages. Pulled by WebComm into the "WALKING:" packet.
//
// METRIC PROXY NOTES:
//   - Touchdown Impulse Proxy [g*s]: unit-mass specific impulse — the area of
//     the accel magnitude above the 1 g resting level, integrated over the
//     contact window. Lacking force/contact sensing, accel-above-gravity over
//     time is the lightest physically-meaningful proxy.
//   - Swing Energy Proxy [mm^2/s^2]: 0.5 * v_peak^2 — specific (unit-mass) peak
//     kinetic energy of the swing foot. Cheap, directly comparable across steps.
//   - CoM Bob / Vertical CoM Velocity [COMMANDED]: derived from the stance
//     leg's commanded hip height ( max(h_sagittal_L, h_sagittal_R) ), NOT an IMU
//     measurement — the firmware has no vertical-CoM estimate. Captures the
//     vertical motion commanded by W4 compliance, H-restore, and step transition.
//   - Body-response metrics #12-#15 [MEASURED] come straight from the torso
//     estimator (inverted-pendulum CoM velocity + roll rate). These are the
//     headline readouts for the swing-leg-momentum hypothesis: correlate them
//     against the swing-energy proxy / touchdown swing speed across steps.
//
// DOUBLE-TOUCHDOWN — measures the PHENOMENON, not "two accel spikes":
//   On this hardware |a| sits near 1 g BOTH in contact and airborne, so two
//   spikes alone do not prove a bounce. A genuine double-touchdown is flagged
//   only when, between the primary touchdown and a second accel onset on the
//   same foot, the foot's gyro confirms it actually lifted (rotated faster than
//   bounceGyroThreshDps, sustained >= releaseDwellMs). The raw onset count is
//   surfaced separately (lastOnsetCount) so the boolean stays auditable against
//   logged traces. Treat the boolean as provisional until validated on hardware.
//
// DEPENDENCIES: state_estimator.h (IMUState), foot_impact.h (FootImpactDetector),
//               gait_controller.h (GaitState, StepSubState), side.h
// LAST CHANGED: 2026-05-31 | Walking instrumentation panel — initial version.
// =============================================================================
#ifndef WALKING_INSTRUMENTATION_H
#define WALKING_INSTRUMENTATION_H

#include <Arduino.h>
#include "side.h"
#include "state_estimator.h"   // IMUState
#include "foot_impact.h"       // FootImpactDetector / FootImpactSideState
#include "gait_controller.h"   // GaitState, StepSubState

// -----------------------------------------------------------------------------
// Metric index. A flat float array indexed by this enum keeps the ring buffer,
// averaging, and packet-building uniform loops. Order is the wire/UI order.
// -----------------------------------------------------------------------------
enum WiMetric : uint8_t {
    WI_TD_ACCEL = 0,    // Peak Touchdown Acceleration          [g]
    WI_TD_IMPULSE,      // Touchdown Impulse Proxy              [g*s]
    WI_TD_SWING_SPEED,  // Touchdown Swing Speed (3D @ contact) [mm/s]
    WI_DOUBLE_TD,       // Double-Touchdown Detected            [0/1]
    WI_DOUBLE_TD_INT,   // Double-Touchdown Interval            [ms]
    WI_SWING_SPEED,     // Peak Swing Speed (3D)                [mm/s]
    WI_SWING_HORIZ,     // Peak Swing Horizontal Speed          [mm/s]
    WI_SWING_VERT,      // Peak Swing Vertical Speed            [mm/s]
    WI_SWING_ENERGY,    // Swing Energy Proxy (0.5*v_peak^2)    [mm^2/s^2]
    WI_COM_BOB,         // Peak-to-Peak CoM Bob (commanded)     [mm]
    WI_COM_VZ_TD,       // Vertical CoM Vel @ Touchdown (cmd)   [mm/s]
    WI_COM_LAT_SPEED,   // Peak Lateral CoM Speed (measured)    [mm/s]
    WI_COM_SAG_SPEED,   // Peak Sagittal CoM Speed (measured)   [mm/s]
    WI_TORSO_ROLL_RATE, // Peak Torso Roll Rate (measured)      [deg/s]
    WI_TORSO_ROLL_TD,   // Torso Roll @ Touchdown (measured)    [deg]
    WI_REACH_PCT,       // Peak Reach Percentage                [%]
    WI_FLATFOOT_ACT,    // Flat-Foot Guard Activations          [count]
    WI_METRIC_COUNT
};

static constexpr uint8_t WI_HISTORY_LEN = 10;  // last N completed steps

// One completed step's metrics (flat array, uniform handling).
struct WalkingStepMetrics {
    float v[WI_METRIC_COUNT] = {0.0f};
};

// -----------------------------------------------------------------------------
// WalkingInstrConfig — runtime-tunable measurement parameters. Defaults are
// conservative starting points; expect to re-tune after the first trace capture.
// -----------------------------------------------------------------------------
struct WalkingInstrConfig {
    bool     enabled             = true;   // false → freeze metrics, emit en=0 only

    // Touchdown-onset detector (instrumentation-local, on filtered magnitude_g).
    float    contactThreshG      = 0.25f;  // onset when |a| rises this far above baseline
    float    contactBaselineG    = 1.0f;   // resting |a| ≈ 1 g
    uint32_t onsetRefractoryMs   = 50;     // min spacing between counted onsets (per foot)
    uint32_t impulseWindowMs     = 100;    // integrate impulse for this long after primary

    // Double-touchdown lift-off confirmation (rejects single-impact ringing).
    float    bounceGyroThreshDps = 60.0f;  // foot must rotate this fast to count as "lifted"
    uint32_t releaseDwellMs      = 20;     // min sustained lifted time before a re-contact counts

    // Reach / flat-foot.
    float    flatFootErrThreshDeg = 0.10f; // |flat_foot_error| above this = guard active
};

// -----------------------------------------------------------------------------
// WalkingInstrState — read-only snapshot pulled by WebComm. Recomputed once per
// completed step (not per tick). hist[0] is the most recent step (Step -1).
// -----------------------------------------------------------------------------
struct WalkingInstrState {
    uint32_t completedSteps = 0;                 // monotonic count since reset
    uint8_t  histCount      = 0;                 // valid entries in hist[], 0..WI_HISTORY_LEN
    bool     enabled        = true;              // mirrors config for the UI toggle
    uint8_t  lastOnsetCount = 0;                 // raw onsets on the landing foot, last step (audit)

    WalkingStepMetrics current;                  // == hist[0] (Step -1)
    float              avg[WI_METRIC_COUNT] = {0.0f};
    WalkingStepMetrics hist[WI_HISTORY_LEN];     // hist[0]=Step-1 ... hist[N-1]=Step-N
};

// -----------------------------------------------------------------------------
// WalkingInstrumentation
//
// One instance globally (in main.cpp). update() is called once per 400 Hz tick
// AFTER gaitController.update(), with all inputs already computed this tick.
// -----------------------------------------------------------------------------
class WalkingInstrumentation {
public:
    WalkingInstrumentation() = default;

    void init();    // clear everything; call once in setup().
    void reset();   // clear ring + accumulators (e-stop / boot / UI Reset). Keeps _cfg.

    // Per-tick measurement. dt_s is the actual 400 Hz gate dt (seconds).
    void update(float dt_s,
                const IMUState&           torso,
                const IMUState&           leftFoot,
                const IMUState&           rightFoot,
                const FootImpactDetector& impact,
                const GaitState&          gait);

    const WalkingInstrState&  getState()  const { return _state; }
    const WalkingInstrConfig& getConfig() const { return _cfg; }
    void setConfig(const WalkingInstrConfig& c) { _cfg = c; _state.enabled = c.enabled; }
    void setEnabled(bool e)                     { _cfg.enabled = e; _state.enabled = e; }

private:
    // Per-foot touchdown-onset / lift FSM state (indexed by sideIdx).
    struct FootDetect {
        bool     above        = false;  // hysteresis: magnitude currently above onset band
        uint32_t lastOnsetMs  = 0;      // refractory anchor
    };

    // Live per-step accumulators. Reset by _resetStepAccum().
    struct StepAccum {
        // swing-foot speeds (peaks over the step, max of both feet)
        float peakSwing3D = 0.0f;
        float peakSwingH  = 0.0f;
        float peakSwingV  = 0.0f;
        // commanded CoM bob
        bool  bodyHseen   = false;
        float bodyHmin    = 0.0f;
        float bodyHmax    = 0.0f;
        // measured body response (peaks of |·|)
        float peakVyCoM   = 0.0f;
        float peakVxCoM   = 0.0f;
        float peakRollDps = 0.0f;
        // reach / flat-foot
        float    peakReachPct = 0.0f;
        uint16_t flatActCount = 0;
        // touchdown snapshots (captured at primary touchdown)
        bool     primarySet      = false;
        uint8_t  landingFoot     = 0;      // sideIdx of the swing/landing foot
        uint32_t firstOnsetMs    = 0;
        float    peakAccelG      = 0.0f;
        float    impulseGs       = 0.0f;
        bool     impulseOpen     = false;
        uint32_t impulseStartMs  = 0;
        float    tdSwingSpeed    = 0.0f;
        float    comVzTd         = 0.0f;
        float    torsoRollTdDeg  = 0.0f;
        // double-touchdown
        uint8_t  onsetCount      = 0;      // onsets on landing foot from primary onward
        bool     doubleTd        = false;
        float    doubleTdIntMs   = 0.0f;
        bool     liftTimerActive = false;  // landing foot currently above gyro thresh
        uint32_t liftTimerStart  = 0;
        bool     liftConfirmed   = false;  // sustained lift seen since primary
    };

    void _resetStepAccum();
    void _finalizeStep();
    // Returns true on a per-foot onset rising edge (with refractory + hysteresis).
    bool _detectOnset(uint8_t side, float magG, uint32_t now);

    WalkingInstrConfig _cfg;
    WalkingInstrState  _state;
    StepAccum          _acc;
    FootDetect         _foot[SIDE_COUNT];

    // Cross-tick trackers (continuous, NOT reset per step — keep deltas small).
    bool         _running       = false;            // gait was running last tick
    StepSubState _prevSubState  = StepSubState::WAIT_SHIFT;
    bool         _prevTdSeen    = false;            // gait.touchdownSeen last tick (edge detect)
    bool         _prevTargValid = false;
    float        _prevFx[SIDE_COUNT] = {0.0f, 0.0f};
    float        _prevFy[SIDE_COUNT] = {0.0f, 0.0f};
    float        _prevFh[SIDE_COUNT] = {0.0f, 0.0f};
    bool         _prevBodyHValid = false;
    float        _prevBodyH      = 0.0f;
    float        _bodyHvelPrev   = 0.0f;            // 1-tick-lagged d(bodyH)/dt for CoM-vz@TD
    bool         _prevFlatActive[SIDE_COUNT] = {false, false};
};

#endif // WALKING_INSTRUMENTATION_H
