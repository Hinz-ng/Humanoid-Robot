#ifndef WEBCOMM_H
#define WEBCOMM_H

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "servo_control.h"
#include "imu.h" 
#include "state_estimator.h"
#include "balance_controller.h"

// Forward declaration — only a pointer is stored; full type in motion_manager.h.
class MotionManager;

class WebComm {
public:    // ctor can be invoked at global scope; it simply stores the servo pointer.
    // legacy users passed a gait object as second argument, which is now ignored.
    // Pass both controllers at construction. stateEst used for on-connect CALIB push.
    WebComm(ServoControl* servo = nullptr);
    void setStateEstimator(StateEstimator* stateEst);  // call before init()
    void init();
    void cleanupClients();
    void broadcastState();
    void broadcastJointInfo();
    void broadcastIMU(RawIMUData data); // --- ADD: sends raw IMU values to UI panel
    void broadcastEstimate(IMUState state); // --- ADD: sends pitch/roll/rates to UI
    void broadcastCalibStatus(CalibState state, float progress); // --- ADD
    void broadcastBalanceState(const BalanceState& state);
    void setBalanceController(BalanceController* bal);
    // Wire the joint authority layer. Call once in setup() after motionManager.init().
    // When wired, slider commands are routed through MotionManager so they silently
    // lose to any active controller (BALANCE or GAIT) on shared joints.
    void setMotionManager(MotionManager* mm);
private:
    AsyncWebServer server;
    AsyncWebSocket ws;
    ServoControl* _servoCtrl;
    StateEstimator* _stateEst;   // used to push CALIB status on new client connect
    uint32_t _lastIMUBroadcast_us = 0;  // --- ADD: rate-limit timestamp for IMU broadcast
    uint32_t _lastEstimateBroadcast_us  = 0;
    uint32_t _lastCalibBroadcast_us     = 0;   // rate-limits calib status to 10 Hz
    BalanceController* _balCtrl         = nullptr;
    uint32_t           _lastBalanceBroadcast_us = 0;
    MotionManager*     _motionManager   = nullptr;  // joint authority layer
    
    void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
    void handleWebSocketMessage(void *arg, uint8_t *data, size_t len);
};

#endif