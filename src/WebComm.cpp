#include "WebComm.h"
#include <LittleFS.h>

// Update Constructor to accept SquatGait pointer
WebComm::WebComm(ServoControl* servoCtrl, SquatGait* squatGait) 
    : server(80), ws("/ws"), _servoCtrl(servoCtrl), _squatGait(squatGait) {}

void listFiles() { 
    Serial.println("[LittleFS] Listing files:");
    File root = LittleFS.open("/");
    File file = root.openNextFile();
    while(file){
        Serial.printf(" - %s (%d bytes)\n", file.name(), file.size());
        file = root.openNextFile();
    }
}

void WebComm::init() {
    WiFi.softAP(AP_SSID, AP_PASS);
    delay(500); 
    Serial.printf("[WiFi] Started. IP: %s\n", WiFi.softAPIP().toString().c_str());

    if (!LittleFS.begin(true)) { Serial.println("[LittleFS] Mount Failed!"); } 
    else { Serial.println("[LittleFS] Mounted."); listFiles(); }

    ws.onEvent([this](AsyncWebSocket *server, AsyncWebSocketClient *client, 
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
        this->onEvent(server, client, type, arg, data, len);
    });
    server.addHandler(&ws);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        if (LittleFS.exists("/index.html")) request->send(LittleFS, "/index.html", "text/html");
        else request->send(200, "text/plain", "index.html missing");
    });
    server.begin();
}

void WebComm::cleanupClients() { ws.cleanupClients(); }

void WebComm::broadcastState() {
    if (!_servoCtrl) return;
    String stateMsg = "STATE:";
    for (int i = 0; i < NUM_SERVOS; i++) {
        stateMsg += String(i) + "=" + String(_servoCtrl->getTargetAngle(i));
        if (i < NUM_SERVOS - 1) stateMsg += ",";
    }
    ws.textAll(stateMsg);
}

void WebComm::onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        if (_servoCtrl) {
            client->text(_servoCtrl->listPoses());
            broadcastState(); 
        }
    }
    if (type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            handleWebSocketMessage(arg, data, len);
        }
    }
}

void WebComm::handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
    data[len] = 0;
    String message = (char *)data;
    if (!_servoCtrl) return;

    if (message.startsWith("CMD:")) {
        String cmd = message.substring(4);
        
        // --- COMMANDS ---
        if (cmd == "RESETP") {
            Serial.println("CMD: Reset Neutral");
            for (int i = 0; i < NUM_SERVOS; i++) {
                _servoCtrl->setTargetAngle(i, JOINT_MAP[i].neutralAngle);
                _servoCtrl->setGaitOffset(i, 0.0f);
            }
            broadcastState();
        } 
        else if (cmd == "SQUAT_DOWN") {
            if (_squatGait) _squatGait->squatDown();
        }
        else if (cmd == "STAND_UP") {
            if (_squatGait) _squatGait->standUp();
        }
        else if (cmd.startsWith("LOAD:")) {
            String poseName = cmd.substring(5);
            _servoCtrl->loadPose(poseName);
            broadcastState();
        }
    } 
    else if (message.startsWith("SAVE:")) {
        String poseName = message.substring(5);
        _servoCtrl->saveCurrentPose(poseName);
        ws.textAll(_servoCtrl->listPoses()); 
    }
    else {
        // Slider Control
        int separatorIndex = message.indexOf(':');
        if (separatorIndex != -1) {
            uint8_t channel = message.substring(0, separatorIndex).toInt();
            float angle = message.substring(separatorIndex + 1).toFloat();
            _servoCtrl->setTargetAngle(channel, angle);
        }
    }
}