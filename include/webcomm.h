#ifndef WEBCOMM_H
#define WEBCOMM_H

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "servo_control.h"
#include "gait_squat.h"

class WebComm {
public:
    // Update constructor to take SquatGait
    WebComm(ServoControl* servoCtrl, SquatGait* squatGait);
    void init();
    void cleanupClients();
    void broadcastState();

private:
    AsyncWebServer server;
    AsyncWebSocket ws;
    ServoControl* _servoCtrl;
    SquatGait* _squatGait; // Store the pointer

    void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
    void handleWebSocketMessage(void *arg, uint8_t *data, size_t len);
};

#endif