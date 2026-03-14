#include "WebComm.h"
#include <LittleFS.h>
#include "oe_control.h"
#include "joint_config.h"
#include "servo_driver.h"
#include "motion_manager.h"    // MotionSource, MotionManager::submit()

// Constructor ---------------------------------------------------------------
// AFTER:
// AFTER:
WebComm::WebComm(ServoControl* servo)
    : server(80), ws("/ws"), _servoCtrl(servo), _stateEst(nullptr) {
}
// ADD this function after the constructor closing brace:
void WebComm::setStateEstimator(StateEstimator* stateEst) {
    _stateEst = stateEst;
}

void WebComm::setBalanceController(BalanceController* bal) {
    _balCtrl = bal;
}
void WebComm::setMotionManager(MotionManager* mm) {
    _motionManager = mm;
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
// AFTER:
void WebComm::broadcastCalibStatus(CalibState state, float progress) {
    if (ws.count() == 0) return;

    // Rate-limit to 10 Hz. Always let CALIB_DONE through immediately (final packet).
    const uint32_t INTERVAL_US = 100000; // 10 Hz
    uint32_t now = micros();
    if (state != CALIB_DONE && (now - _lastCalibBroadcast_us) < INTERVAL_US) return;
    _lastCalibBroadcast_us = now;

    // remaining_s is only meaningful during COLLECTING (known sample count / rate).
    // During WAITING, remaining is undefined — send -1 so the UI can show "waiting" text.
    const float CALIB_TOTAL_S = (float)FilterConfig::CALIB_SAMPLES / 400.0f;
    float remaining_s;
    const char* stateStr;
    if (state == CALIB_DONE) {
        stateStr    = "done";
        remaining_s = 0.0f;
    } else if (state == CALIB_COLLECTING) {
        stateStr    = "collecting";
        remaining_s = CALIB_TOTAL_S * (1.0f - progress);
    } else {
        // CALIB_WAITING — robot is moving, unknown duration
        stateStr    = "waiting";
        remaining_s = CALIB_TOTAL_S;   // show full duration as "will take ~Xs once still"
    }

    char buf[80];
    snprintf(buf, sizeof(buf),
        "CALIB:state=%s,progress=%.2f,remaining=%.1f",
        stateStr, progress, remaining_s
    );
    ws.textAll(buf);
}

void WebComm::broadcastBalanceState(const BalanceState& state) {
    if (ws.count() == 0 || _balCtrl == nullptr) return;

    const uint32_t INTERVAL_US = 50000;  // 20 Hz
    uint32_t now = micros();
    if ((now - _lastBalanceBroadcast_us) < INTERVAL_US) return;
    _lastBalanceBroadcast_us = now;

    const BalanceConfig& cfg = _balCtrl->getConfig();
    char buf[128];
    // AFTER:
    snprintf(buf, sizeof(buf),
        "BALANCE:enabled=%d,err=%.4f,u=%.3f,ankle=%.2f,hip=%.2f,Kp=%.3f,Kd=%.3f,sp=%.4f",
        (int)state.active,
        state.pitch_error,
        state.u_clamped,
        state.ankle_cmd_deg,
        state.hip_cmd_deg,
        cfg.Kp, cfg.Kd,
        cfg.pitch_setpoint_rad
    );
    ws.textAll(buf);
}

// =============================================================================
//  EVENT HANDLER
// =============================================================================

void WebComm::onEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len) {
    // AFTER:
    if (type == WS_EVT_CONNECT) {
        Serial.printf("[WebComm] Client #%u connected\n", client->id());
        if (_servoCtrl) {
            client->text(_servoCtrl->listPoses());
            broadcastJointInfo();
            broadcastState();
        }
        // Push current calibration state immediately to the new client.
        // Without this, any client that connects after calibration finishes
        // would never receive a CALIB packet and the bar stays at 0%.
        if (_stateEst) {
            char buf[80];
            CalibState cs  = _stateEst->getCalibState();
            float      prog = _stateEst->getCalibProgress();
            const float CALIB_TOTAL_S = (float)FilterConfig::CALIB_SAMPLES / 400.0f;
            float remaining_s = (cs == CALIB_DONE) ? 0.0f : CALIB_TOTAL_S * (1.0f - prog);
            const char* stateStr = (cs == CALIB_DONE)       ? "done"
                                 : (cs == CALIB_COLLECTING) ? "collecting"
                                                             : "waiting";
            snprintf(buf, sizeof(buf),
                "CALIB:state=%s,progress=%.2f,remaining=%.1f",
                stateStr, prog, remaining_s
            );
            client->text(buf);
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
    // Build String from explicit length to avoid buffer overrun or missing null terminator.
    String message((const char*)data, len);
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
        // AFTER:
        else if (cmd == "BALANCE_ON") {
            if (_balCtrl) {
                BalanceConfig cfg = _balCtrl->getConfig();
                cfg.enabled = true;
                _balCtrl->setConfig(cfg);
            }
            Serial.println("[WebComm] CMD: BALANCE_ON");
        }
        else if (cmd == "BALANCE_OFF") {
            if (_balCtrl) {
                BalanceConfig cfg = _balCtrl->getConfig();
                cfg.enabled = false;
                _balCtrl->setConfig(cfg);
            }
            Serial.println("[WebComm] CMD: BALANCE_OFF");
        }
        // AFTER (complete replacement — all braces balanced):
        else if (cmd.startsWith("BALANCE_TUNE:")) {
            if (_balCtrl) {
                BalanceConfig cfg = _balCtrl->getConfig();
                String params = cmd.substring(13);
                int start = 0;
                while (start < (int)params.length()) {
                    int comma = params.indexOf(',', start);
                    String pair = (comma == -1) ? params.substring(start)
                                                : params.substring(start, comma);
                    int eq = pair.indexOf('=');
                    if (eq != -1) {
                        String key = pair.substring(0, eq);
                        float  val = pair.substring(eq + 1).toFloat();
                        if      (key == "Kp")       cfg.Kp                    = val;
                        else if (key == "Kd")       cfg.Kd                    = val;
                        else if (key == "setpoint") cfg.pitch_setpoint_rad    = val;
                        else if (key == "ankle")    cfg.ankle_ratio           = val;
                        else if (key == "hip")      cfg.hip_ratio             = val;
                        else if (key == "max")      cfg.max_correction_deg    = val;
                    }
                    start = (comma == -1) ? params.length() : comma + 1;
                }
                _balCtrl->setConfig(cfg);
                Serial.printf("[WebComm] BALANCE_TUNE: Kp=%.3f Kd=%.3f sp=%.4f\n",
                               cfg.Kp, cfg.Kd, cfg.pitch_setpoint_rad);
            }
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
    } else {
        // Slider control: "<channel>:<angle_degrees>" (existing protocol unchanged).
        // Route through MotionManager when wired so UI authority is explicitly
        // declared at SOURCE_UI priority. Any active controller (BALANCE, GAIT)
        // will silently override shared joints — no code change needed in either
        // controller to suppress UI when active.
        int sep = message.indexOf(':');
        if (sep != -1) {
            uint8_t ch    = (uint8_t)message.substring(0, sep).toInt();
            float   angle = message.substring(sep + 1).toFloat();

            if (_motionManager) {
                // Preferred path: submit as SOURCE_UI. flush() dispatches via
                // setTargetAngle() which preserves smooth-stepping behaviour.
                _motionManager->submit(SOURCE_UI, ch, angle);
            } else {
                // Fallback: direct write (pre-MotionManager behaviour preserved).
                _servoCtrl->setTargetAngle(ch, angle);
            }
            broadcastState();
        }
    }
}
