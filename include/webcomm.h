#ifndef WEBCOMM_H
#define WEBCOMM_H

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "servo_control.h"
#include "imu.h" 
#include "state_estimator.h"
#include "balance_controller.h"
#include "foot_contact.h"

// Forward declarations — only pointers are stored; full types in respective headers.
class MotionManager;
class WeightShift;
class GaitController;    // forward declare — only pointer stored
class TorsoStabilizer;   // forward declare — joint-space torso balance
class LSM6DSVDriver;     // forward declare — foot IMU diagnostic
class SwingFootLeveler;  // forward declare — swing-foot leveler (L0+)
class FootImpactDetector;
class WalkingInstrumentation;  // forward declare — per-step walking diagnostics

class WebComm {
public:    // ctor can be invoked at global scope; it simply stores the servo pointer.
    // legacy users passed a gait object as second argument, which is now ignored.
    // Pass both controllers at construction. stateEst used for on-connect CALIB push.
    WebComm(ServoControl* servo = nullptr);
    void setStateEstimator(StateEstimator* stateEst);  // call before init()
    void setGaitController(GaitController* gc);
    void init();
    void cleanupClients();
    void broadcastState();
    void broadcastJointInfo();
    void broadcastIMU(RawIMUData data); // --- ADD: sends raw IMU values to UI panel
    void broadcastEstimate(IMUState state); // --- ADD: sends pitch/roll/rates to UI
    // Phase F1 — foot IMU telemetry. Both methods rate-limit to 20 Hz internally.
    void broadcastFootIMU(const RawIMUData& left, const RawIMUData& right);
    void broadcastFootEstimate(const IMUState& left, const IMUState& right);
    // Accepts a pre-computed snapshot so the UI displays the same booleans
    // the gait controller saw this tick. Pass nullptr to no-op (used when
    // FOOT_IMU is compiled out).
    // lf/rf are the per-foot IMUStates (robot frame) — used ONLY to compute the
    // neutral-corrected telemetry preview (pdev + neutral-aware flat/loaded).
    // The gate booleans still come from `snap` (zero-referenced), so what the UI
    // shows for the gate matches the actual gate input. Telemetry only.
    void broadcastFootContact(const FootContactSnapshot* snap,
                              const IMUState& lf, const IMUState& rf);
    void broadcastFootImpact();
    // Per-step walking diagnostics ("WALKING:" packet). Sends on step change plus
    // a ≤1 Hz heartbeat. No-op if no WalkingInstrumentation is wired.
    void broadcastWalkingInstr();
    void broadcastCalibStatus(CalibState state, float progress); // --- ADD
    void broadcastBalanceState(const BalanceState& state);
    // Independent torso stabiliser telemetry (TORSO: packet). Rate-limited to
    // 20 Hz internally. No-op if no TorsoStabilizer is wired.
    void broadcastTorsoState();
    void broadcastSpeeds();
    // Per-joint transmission state — gain, anatomical-vs-corrected-servo
    // deviation, and clamp flag. Lets the foot-IMU sweep compare commanded
    // bone angle vs the servo-side command field-by-field. Mirrors
    // broadcastSpeeds() (call on connect, after tune commands, etc).
    void broadcastTransmissionState();
    void broadcastGaitState();
    // Sends current WeightShiftConfig to a specific client (called on connect).
    // Firmware is source of truth; client updates sliders without sending back.
    void broadcastWeightShiftConfig(AsyncWebSocketClient* client);
    void setBalanceController(BalanceController* bal);
    // Wire the joint-space torso stabiliser — an independent subsystem with its
    // own TorsoConfig, tuned via TORSO_TUNE and broadcast in the TORSO: packet.
    void setTorsoStabilizer(TorsoStabilizer* ts);
    // Wire the joint authority layer. Call once in setup() after motionManager.init().
    // When wired, slider commands are routed through MotionManager so they silently
    // lose to any active controller (BALANCE or GAIT) on shared joints.
    void setMotionManager(MotionManager* mm);
    // Wire weight shift module. Call once in setup() after weightShift.init().
    void setWeightShift(WeightShift* ws);
    // Wire foot IMUs for diagnostic probing via WebSocket.
    void setFootIMUs(LSM6DSVDriver* left, LSM6DSVDriver* right);
    void setFootContactConfig(FootContactConfig* cfg);
    // Wire swing-foot leveler for CMD:SWING_LEVELER_TUNE and GAIT: telemetry.
    void setSwingFootLeveler(SwingFootLeveler* l);
    void setFootImpactDetector(FootImpactDetector* d);
    // Wire per-step walking instrumentation for the "WALKING:" packet and the
    // CMD:WALKING_INSTR:reset/enable commands.
    void setWalkingInstrumentation(WalkingInstrumentation* w);
    // Phase 3: dual-parameter form preserved for callers that set both
    // neutrals atomically (setup() in main.cpp). Prefer the Side-keyed overload
    // for new code — positional parameter order is the silent-swap hazard this
    // refactor closes.
    void setFootNeutralPitch(float leftRad, float rightRad);
    void setFootNeutralPitch(Side side, float rad);
    // Boot-tare neutral capture (pitch + roll + validity), pushed once per foot
    // from main.cpp when its estimator reaches CALIB_DONE. `valid=false` (unclean
    // calibration) leaves the stored neutral at 0 and flags it in telemetry —
    // fail loud, never silently use garbage. Telemetry / preview only; does not
    // touch the control-path foot-contact snapshot.
    void setFootNeutral(Side side, float pitchRad, float rollRad, bool valid);
private:
    AsyncWebServer server;
    AsyncWebSocket ws;
    ServoControl* _servoCtrl;
    StateEstimator* _stateEst;   // used to push CALIB status on new client connect
    uint32_t _lastIMUBroadcast_us = 0;  // --- ADD: rate-limit timestamp for IMU broadcast
    uint32_t _lastEstimateBroadcast_us  = 0;
    uint32_t _lastFootIMUBroadcast_us       = 0;
    uint32_t _lastFootEstimateBroadcast_us  = 0;
    uint32_t _lastFootContactBroadcast_us   = 0;
    uint32_t _lastFootImpactBroadcast_us    = 0;
    uint32_t _lastCalibBroadcast_us     = 0;   // rate-limits calib status to 10 Hz
    BalanceController* _balCtrl         = nullptr;
    TorsoStabilizer*   _torsoStab       = nullptr;  // joint-space torso balance
    uint32_t           _lastBalanceBroadcast_us = 0;
    uint32_t           _lastTorsoBroadcast_us   = 0;
    MotionManager*     _motionManager   = nullptr;  // joint authority layer
    WeightShift*       _weightShift     = nullptr;  // gait: CoM lateral shift
    GaitController*    _gaitCtrl            = nullptr;
    uint32_t           _lastGaitBroadcast_us = 0;
    LSM6DSVDriver*     _leftFootIMU         = nullptr;
    LSM6DSVDriver*     _rightFootIMU        = nullptr;
    FootContactConfig* _footContactCfg      = nullptr;
    float              _leftFootNeutralPitchRad  = 0.0f;
    float              _rightFootNeutralPitchRad = 0.0f;
    float              _leftFootNeutralRollRad   = 0.0f;
    float              _rightFootNeutralRollRad  = 0.0f;
    bool               _leftFootNeutralValid     = false;  // false until a clean tare
    bool               _rightFootNeutralValid    = false;
    SwingFootLeveler*  _swingLeveler        = nullptr;
    FootImpactDetector* _footImpact         = nullptr;
    WalkingInstrumentation* _walkInstr      = nullptr;  // per-step walking diagnostics
    uint32_t           _lastWalkInstrBroadcast_us = 0;
    uint32_t           _lastWalkInstrStep   = 0;        // last broadcast step counter

    void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
    void handleWebSocketMessage(void *arg, uint8_t *data, size_t len);

    // Per-client filtered broadcast. Skips clients with backlog > queue
    // threshold instead of short-circuiting for ALL clients. Replaces
    // ws.textAll() in rate-limited telemetry broadcasts where a single
    // zombie client (laptop sleep, mobile background) used to freeze
    // telemetry to every connected client.
    void _sendFiltered(const char* msg);
};

#endif
