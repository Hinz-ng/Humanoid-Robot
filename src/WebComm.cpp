#include "WebComm.h"
#include <LittleFS.h>
#include "oe_control.h"
#include "joint_config.h"
#include "servo_driver.h"

// Constructor ---------------------------------------------------------------
WebComm::WebComm(ServoControl* servo)
    : server(80), ws("/ws"), _servoCtrl(servo) {
    // older versions accepted a second pointer to a SquatGait object;
    // the argument has been dropped. nothing else to do here.
}


// ---------------------------------------------------------------------------

void listFiles() {
    Serial.println("[LittleFS] Listing files:");
    File root = LittleFS.open("/");
    File file = root.openNextFile();
    while (file) {
        Serial.printf(" - %s (%d bytes)\n", file.name(), file.size());
        file = root.openNextFile();
    }
}

// ---------------------------------------------------------------------------

void WebComm::init() {
    WiFi.softAP(AP_SSID, AP_PASS);
    delay(500);
    Serial.printf("[WiFi] Started. IP: %s\n", WiFi.softAPIP().toString().c_str());

    if (!LittleFS.begin(true)) { Serial.println("[LittleFS] Mount Failed!"); }
    else { Serial.println("[LittleFS] Mounted."); listFiles(); }

    ws.onEvent([this](AsyncWebSocket* server, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len) {
        this->onEvent(server, client, type, arg, data, len);
    });
    server.addHandler(&ws);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
        if (LittleFS.exists("/index.html")) request->send(LittleFS, "/index.html", "text/html");
        else request->send(200, "text/plain", "index.html missing");
    });
    server.begin();
}

void WebComm::cleanupClients() { ws.cleanupClients(); }

// =============================================================================
//  STATE BROADCAST
//  Protocol: "STATE:<ch>=<pulse_us>:<deg>,..."
//  Also appends E-stop status so UI can reflect it on reconnect.
// =============================================================================

void WebComm::broadcastState() {
    if (!_servoCtrl) return;
    String msg = "STATE:";
    for (int i = 0; i < NUM_JOINTS; i++) {
        int   p = _servoCtrl->getTargetPulse(i);
        float d = _servoCtrl->getTargetAngle(i);
        msg += String(i) + "=" + String(p) + ":" + String(d, 1);
        if (i < NUM_JOINTS - 1) msg += ",";
    }
    // Append E-stop flag so UI always knows current safety state
    msg += ";ESTOP=" + String(oe_is_estopped() ? "1" : "0");
    ws.textAll(msg);
}

// =============================================================================
//  JOINT INFO BROADCAST
//  Protocol: "JOINTS:<ch>|<name>|<neutral_pulse>|<neutral_deg>|<hint>,..."
// =============================================================================

void WebComm::broadcastJointInfo() {
    // Now reads from JOINT_CONFIG (the canonical config table in joint_config.h).
    // neutral_pulse is computed on the fly from neutralDeg — no separate field needed.
    String msg = "JOINTS:";
    for (int i = 0; i < NUM_JOINTS; i++) {
        const JointConfig& j = JOINT_CONFIG[i];
        int neutral_pulse = ServoDriver::degToPulse(j.neutralDeg);
        msg += String(j.channel)           + "|"
             + String(j.name)              + "|"
             + String(neutral_pulse)       + "|"
             + String(j.neutralDeg, 1)     + "|"
             + String(j.direction);        // direction replaces ui_direction_hint
        if (i < NUM_JOINTS - 1) msg += ",";
    }
    ws.textAll(msg);
}

// =============================================================================
//  IMU BROADCAST                                                   --- ADD ---
//  Protocol: "IMU:ax=X,ay=Y,az=Z,gx=X,gy=Y,gz=Z"
//  Rate-limited to 20 Hz internally. Skips silently if no clients
//  are connected or the data struct is marked invalid.
// =============================================================================

void WebComm::broadcastIMU(RawIMUData data) {
    if (!data.valid || ws.count() == 0) return;

    const uint32_t IMU_BROADCAST_INTERVAL_US = 50000; // 20 Hz
    uint32_t now = micros();
    if ((now - _lastIMUBroadcast_us) < IMU_BROADCAST_INTERVAL_US) return;
    _lastIMUBroadcast_us = now;

    char buf[64];
    snprintf(buf, sizeof(buf),
        "IMU:ax=%d,ay=%d,az=%d,gx=%d,gy=%d,gz=%d",
        data.accel_x, data.accel_y, data.accel_z,
        data.gyro_x,  data.gyro_y,  data.gyro_z
    );
    ws.textAll(buf);
}
// =============================================================================
//  ESTIMATE BROADCAST                                              --- ADD ---
//  Protocol: "ESTIMATE:pitch=X,roll=X,pitchRate=X,rollRate=X"
//  Values: pitch/roll in radians (4 decimal places), rates in rad/s.
//  Rate-limited to 20 Hz — same cadence as raw IMU broadcast.
// =============================================================================
void WebComm::broadcastEstimate(IMUState state) {
    if (!state.valid || ws.count() == 0) return;

    const uint32_t INTERVAL_US = 50000; // 20 Hz
    uint32_t now = micros();
    if ((now - _lastEstimateBroadcast_us) < INTERVAL_US) return;
    _lastEstimateBroadcast_us = now;

    char buf[96];
    snprintf(buf, sizeof(buf),
        "ESTIMATE:pitch=%.4f,roll=%.4f,pitchRate=%.4f,rollRate=%.4f",
        state.pitch, state.roll, state.pitchRate, state.rollRate
    );
    ws.textAll(buf);
}

// =============================================================================
//  CALIBRATION STATUS BROADCAST                                   --- ADD ---
void WebComm::broadcastCalibStatus(CalibState state, float progress) {
    if (ws.count() == 0) return;

    // Rate-limit to 10 Hz — prevents WS send-queue overflow at 400 Hz loop rate.
    // Exception: always send the final "done" packet immediately.
    const uint32_t INTERVAL_US = 100000; // 10 Hz
    uint32_t now = micros();
    if (state != CALIB_DONE && (now - _lastCalibBroadcast_us) < INTERVAL_US) return;
    _lastCalibBroadcast_us = now;

    // Remaining time estimate: 2000 samples / 400 Hz = 5.0s total.
    // Progress is 0.0→1.0 so remaining = total * (1 - progress).
    const float CALIB_TOTAL_S = FilterConfig::CALIB_SAMPLES / 400.0f;
    float remaining_s = (state == CALIB_DONE) ? 0.0f : CALIB_TOTAL_S * (1.0f - progress);

    char buf[80];
    snprintf(buf, sizeof(buf),
        "CALIB:state=%s,progress=%.2f,remaining=%.1f",
        (state == CALIB_DONE) ? "done" : "collecting",
        progress,
        remaining_s
    );
    ws.textAll(buf);
}
// =============================================================================
//  EVENT HANDLER
// =============================================================================

void WebComm::onEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.printf("[WebComm] Client #%u connected\n", client->id());
        if (_servoCtrl) {
            client->text(_servoCtrl->listPoses());
            broadcastJointInfo();
            broadcastState();
        }
    }
    if (type == WS_EVT_DISCONNECT) {
        Serial.printf("[WebComm] Client #%u disconnected\n", client->id());
    }
    if (type == WS_EVT_DATA) {
        AwsFrameInfo* info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            handleWebSocketMessage(arg, data, len);
        }
    }
}

// =============================================================================
//  MESSAGE HANDLER
// =============================================================================

void WebComm::handleWebSocketMessage(void* arg, uint8_t* data, size_t len) {
    data[len] = 0;
    String message = (char*)data;
    if (!_servoCtrl) return;

    Serial.printf("[WebComm] Received: %s\n", message.c_str());

    if (message.startsWith("CMD:")) {
        String cmd = message.substring(4);

        if (cmd == "RESETP") {
            Serial.println("[WebComm] CMD: Reset Neutral");
            for (int i = 0; i < NUM_JOINTS; i++) {
                // Use JOINT_CONFIG (new SSOT) instead of legacy JOINT_MAP.
                // setGaitOffset(ch, 0) moves to neutral with zero offset.
                _servoCtrl->setGaitOffset(i, 0.0f);
            }
            broadcastState();
        }
        else if (cmd.startsWith("LOAD:")) {
            _servoCtrl->loadPose(cmd.substring(5));
            broadcastState();
        }
        // Named movement
        // Protocol: CMD:MOVE_NAMED:<channel>:<movement>:<magnitude>
        // Example:  CMD:MOVE_NAMED:2:flex:45.5
        else if (cmd.startsWith("MOVE_NAMED:")) {
            String params = cmd.substring(11);
            int sep1 = params.indexOf(':');
            int sep2 = params.indexOf(':', sep1 + 1);
            if (sep1 != -1 && sep2 != -1) {
                uint8_t ch  = (uint8_t)params.substring(0, sep1).toInt();
                String  mv  = params.substring(sep1 + 1, sep2);
                float   mag = params.substring(sep2 + 1).toFloat();

                Serial.printf("[WebComm] MOVE_NAMED: ch=%d  move=%s  magnitude=%.2f\n",
                              ch, mv.c_str(), mag);

                _servoCtrl->applyNamedMovement(ch, mv.c_str(), mag, false);
                broadcastState();
            } else {
                Serial.printf("[WebComm] MOVE_NAMED parse error: params='%s'\n", params.c_str());
            }
        }
        // Emergency Stop
        else if (cmd == "ESTOP") {
            oe_estop();
            broadcastState();
            ws.textAll("ESTOP:ACTIVE");
            Serial.println("[WebComm] CMD: ESTOP received — outputs disabled");
        }
        else if (cmd == "CLEAR_ESTOP") {
            oe_clear();
            broadcastState();
            ws.textAll("ESTOP:CLEAR");
            Serial.println("[WebComm] CMD: CLEAR_ESTOP received — outputs enabled");
        }
    }
    else if (message.startsWith("SAVE:")) {
        _servoCtrl->saveCurrentPose(message.substring(5));
        ws.textAll(_servoCtrl->listPoses());
    }
    else {
        // Slider control: "<channel>:<angle_degrees>" (existing protocol unchanged)
        int sep = message.indexOf(':');
        if (sep != -1) {
            uint8_t ch    = (uint8_t)message.substring(0, sep).toInt();
            float   angle = message.substring(sep + 1).toFloat();
            _servoCtrl->setTargetAngle(ch, angle);
            broadcastState();
        }
    }
}