// =============================================================================
// FILE:    walking_instrumentation.h
// MODULE:  diagnostics
// LAYER:   3.5 — Telemetry aggregator (peer of foot_impact, foot_contact)
//
// PURPOSE: Per-step walking diagnostics. Aggregates 22 scalar metrics plus an
//          onset timeline once per completed step (one swing half-cycle) into a
//          fixed 10-step ring buffer so competing walking-bottleneck hypotheses
//          (swing-leg momentum, touchdown impulse + its vertical/horizontal
//          decomposition and timing, support-transfer, double-touchdown,
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
//   - RawIMUData left/right foot: REMAPPED (robot/foot-frame) raw foot accel — the
//                                same lf_raw/rf_raw the impact detector consumes.
//                                Used ONLY to decompose the touchdown impulse into
//                                its sole-normal (vertical) and in-sole (horizontal)
//                                axes. The remap is a constant chip→foot mounting
//                                transform, so remapped Z is the foot's local
//                                vertical (sole-normal) and X/Y span the sole plane.
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
// TOUCHDOWN DECOMPOSITION (added 2026-06-01) — answers "where/how does the
// disturbance enter": all anchored to the SAME primary-touchdown latch already
// used for the impulse proxy, so they are directly comparable.
//   - Peak Roll-Rate Time [ms]: signed offset (peak-roll sample time − primary
//     touchdown time). Negative = the body's biggest kick precedes contact;
//     positive = it follows. Localizes the dominant disturbance in the step.
//   - Touchdown-Window Peak Roll Rate [deg/s]: max |rollRate| inside a fixed
//     [start,end] ms window after primary touchdown (touchdown-specific, not
//     whole-step). Window bounds are config (tdRollWin{Start,End}Ms).
//   - Vertical / Horizontal Impact Peak [g]: peak of the foot-local accel axes
//     over the impulse window on the LANDING foot — remapped Z (sole-normal) vs
//     hypot(remapped X,Y) (in-sole). Same one-pole LPF (alpha) the impact
//     detector uses, applied per-axis: a decomposition of the same conditioned
//     signal, not new filtering. Both include the ~1 g rest term (so at flat
//     rest vert≈1, horiz≈0, and vert²+horiz²≈|a|²), matching WI_TD_ACCEL.
//   - Touchdown Impulse Duration [ms]: total time within the impulse window the
//     landing-foot magnitude stays at/above the onset threshold
//     (contactBaselineG+contactThreshG). Sharp spike → short; soft impact → long.
//   - Onset Timeline [ms list]: offsets (from primary, primary itself = 0) of
//     every onset the FSM fires on the landing foot this step. Pure audit of the
//     double-touchdown detector — exposes the exact sequence, adds no detection.
//
// DEPENDENCIES: state_estimator.h (IMUState), imu.h (RawIMUData),
//               foot_impact.h (FootImpactDetector),
//               gait_controller.h (GaitState, StepSubState), side.h
// LAST CHANGED: 2026-06-01 | Touchdown decomposition metrics (#18-#22 + onset
//               timeline) — localize/decompose the touchdown disturbance.
//               2026-05-31 | Walking instrumentation panel — initial version.
// =============================================================================
#ifndef WALKING_INSTRUMENTATION_H
#define WALKING_INSTRUMENTATION_H

#include <Arduino.h>
#include "side.h"
#include "imu.h"                // RawIMUData (remapped foot accel, vert/horiz split)
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
    // ---- touchdown decomposition (added 2026-06-01) ----
    WI_TD_VERT_PEAK,    // Touchdown Vertical Impact Peak (sole-Z)   [g]
    WI_TD_HORIZ_PEAK,   // Touchdown Horizontal Impact Peak (in-sole)[g]
    WI_TD_IMPULSE_DUR,  // Touchdown Impulse Duration (|a|≥onset)    [ms]
    WI_TD_WIN_ROLL,     // Touchdown-Window Peak Roll Rate           [deg/s]
    WI_PEAK_ROLL_TIME,  // Peak Roll-Rate Time (offset from primary) [ms]
    WI_METRIC_COUNT
};

static constexpr uint8_t WI_HISTORY_LEN = 10;  // last N completed steps
static constexpr uint8_t WI_ONSET_MAX   = 8;   // max onsets recorded per step (timeline)

// One completed step's metrics (flat array + onset timeline, uniform handling).
struct WalkingStepMetrics {
    float    v[WI_METRIC_COUNT]   = {0.0f};
    uint8_t  onsetN               = 0;          // onsets recorded on landing foot this step
    uint16_t onsetMs[WI_ONSET_MAX] = {0};       // offsets from primary TD, ms (onsetMs[0]=0)
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

    // Touchdown-response roll window — max torso roll rate is measured only
    // inside [start,end] ms after primary touchdown (touchdown-specific kick,
    // isolated from the whole-step peak). Default per spec; reflash to change.
    uint32_t tdRollWinStartMs    = 50;     // window opens this long after primary
    uint32_t tdRollWinEndMs      = 100;    // window closes this long after primary

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
                const RawIMUData&         leftFootRaw,
                const RawIMUData&         rightFootRaw,
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
        // Per-axis accel LPF (g), seeded on first valid sample. Same one-pole
        // alpha as the impact detector — decomposes the same conditioned signal.
        bool     accSeeded    = false;
        float    axF          = 0.0f;   // robot/foot-frame X (in-sole, forward)
        float    ayF          = 0.0f;   // robot/foot-frame Y (in-sole, left)
        float    azF          = 0.0f;   // robot/foot-frame Z (sole-normal, vertical)
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
        uint32_t peakRollTimeAbsMs = 0;    // millis() of the step's peak roll-rate sample
        float    tdWinRollDps      = 0.0f; // max |rollRate| inside the touchdown-response window
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
        // touchdown decomposition (landing foot, within the impulse window)
        float    tdVertPeakG     = 0.0f;   // peak sole-normal accel  |a_z|        [g]
        float    tdHorizPeakG    = 0.0f;   // peak in-sole accel  hypot(a_x,a_y)   [g]
        float    impulseDurMs    = 0.0f;   // time landing-foot |a| stayed ≥ onset [ms]
        // double-touchdown + onset timeline
        uint8_t  onsetCount      = 0;      // onsets on landing foot from primary onward (uncapped)
        uint8_t  onsetN          = 0;      // onsets recorded in the timeline (capped at WI_ONSET_MAX)
        uint16_t onsetMs[WI_ONSET_MAX] = {0};  // offsets from primary, ms (onsetMs[0]=0)
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
