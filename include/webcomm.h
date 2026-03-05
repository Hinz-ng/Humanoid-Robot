#ifndef WEBCOMM_H
#define WEBCOMM_H

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "servo_control.h"
#include "gait_squat.h"
#include "imu.h" 
#include "state_estimator.h"    // --- ADD: for IMUState parameter

class WebComm {
public:
    WebComm(ServoControl* servoCtrl, SquatGait* squatGait);
    void init();
    void cleanupClients();
    void broadcastState();
    void broadcastJointInfo();
    void broadcastIMU(RawIMUData data); // --- ADD: sends raw IMU values to UI panel
    void broadcastEstimate(IMUState state); // --- ADD: sends pitch/roll/rates to UI

private:
    AsyncWebServer server;
    AsyncWebSocket ws;
    ServoControl* _servoCtrl;
    SquatGait*    _squatGait;
    uint32_t _lastIMUBroadcast_us = 0;  // --- ADD: rate-limit timestamp for IMU broadcast
    uint32_t _lastEstimateBroadcast_us = 0; // --- ADD: rate-limit for estimate broadcast

    void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
    void handleWebSocketMessage(void *arg, uint8_t *data, size_t len);
};

#endif