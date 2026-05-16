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
class GaitController;   // forward declare — only pointer stored
class LSM6DSVDriver;    // forward declare — foot IMU diagnostic

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
    void broadcastFootContact(const FootContactSnapshot* snap);
    void broadcastCalibStatus(CalibState state, float progress); // --- ADD
    void broadcastBalanceState(const BalanceState& state);
    void broadcastSpeeds();
    void broadcastGaitState();
    // Sends current WeightShiftConfig to a specific client (called on connect).
    // Firmware is source of truth; client updates sliders without sending back.
    void broadcastWeightShiftConfig(AsyncWebSocketClient* client);
    void setBalanceController(BalanceController* bal);
    // Wire the joint authority layer. Call once in setup() after motionManager.init().
    // When wired, slider commands are routed through MotionManager so they silently
    // lose to any active controller (BALANCE or GAIT) on shared joints.
    void setMotionManager(MotionManager* mm);
    // Wire weight shift module. Call once in setup() after weightShift.init().
    void setWeightShift(WeightShift* ws);
    // Wire foot IMUs for diagnostic probing via WebSocket.
    void setFootIMUs(LSM6DSVDriver* left, LSM6DSVDriver* right);
    void setFootContactConfig(FootContactConfig* cfg);
    // Phase 3: dual-parameter form preserved for callers that set both
    // neutrals atomically (setup() in main.cpp). Prefer the Side-keyed overload
    // for new code — positional parameter order is the silent-swap hazard this
    // refactor closes.
    void setFootNeutralPitch(float leftRad, float rightRad);
    void setFootNeutralPitch(Side side, float rad);
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
    uint32_t _lastCalibBroadcast_us     = 0;   // rate-limits calib status to 10 Hz
    BalanceController* _balCtrl         = nullptr;
    uint32_t           _lastBalanceBroadcast_us = 0;
    MotionManager*     _motionManager   = nullptr;  // joint authority layer
    WeightShift*       _weightShift     = nullptr;  // gait: CoM lateral shift
    GaitController*    _gaitCtrl            = nullptr;
    uint32_t           _lastGaitBroadcast_us = 0;
    LSM6DSVDriver*     _leftFootIMU         = nullptr;
    LSM6DSVDriver*     _rightFootIMU        = nullptr;
    FootContactConfig* _footContactCfg      = nullptr;
    float              _leftFootNeutralPitchRad  = 0.0f;
    float              _rightFootNeutralPitchRad = 0.0f;
    
    void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
    void handleWebSocketMessage(void *arg, uint8_t *data, size_t len);
};

#endif
