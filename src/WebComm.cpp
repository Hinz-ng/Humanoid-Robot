#include "webComm.h"
#include <LittleFS.h>
#include "oe_control.h"
#include "joint_config.h"
#include "servo_driver.h"
#include "motion_manager.h"    // MotionSource, MotionManager::submit()
#include "weight_shift.h"      // WeightShift, ShiftDirection, WeightShiftConfig
#include "leg_ik.h"            // LegIK, FootTarget, LegIKResult — IK test commands
#include "gait_controller.h"

// Constructor ---------------------------------------------------------------
WebComm::WebComm(ServoControl* servo)
    : server(80), ws("/ws"), _servoCtrl(servo), _stateEst(nullptr) {
}

void WebComm::setStateEstimator(StateEstimator* stateEst) {
    _stateEst = stateEst;
}

void WebComm::setBalanceController(BalanceController* bal) {
    _balCtrl = bal;
}

void WebComm::setMotionManager(MotionManager* mm) {
    _motionManager = mm;
}

void WebComm::setWeightShift(WeightShift* ws) {
    _weightShift = ws;
}

void WebComm::setGaitController(GaitController* gc) {
    _gaitCtrl = gc;
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
    String msg = "JOINTS:";
    for (int i = 0; i < NUM_JOINTS; i++) {
        const JointConfig& j = JOINT_CONFIG[i];
        int neutral_pulse = ServoDriver::degToPulse(j.neutralDeg);
        msg += String(j.channel)           + "|"
             + String(j.name)              + "|"
             + String(neutral_pulse)       + "|"
             + String(j.neutralDeg, 1)     + "|"
             + String(j.direction);
        if (i < NUM_JOINTS - 1) msg += ",";
    }
    ws.textAll(msg);
}

// =============================================================================
//  IMU BROADCAST
//  Protocol: "IMU:ax=X,ay=Y,az=Z,gx=X,gy=Y,gz=Z"
//  Rate-limited to 20 Hz internally.
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
//  ESTIMATE BROADCAST
//  Protocol: "ESTIMATE:pitch=X,roll=X,pitchRate=X,rollRate=X"
//  Rate-limited to 20 Hz.
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
//  CALIBRATION STATUS BROADCAST
// =============================================================================

void WebComm::broadcastCalibStatus(CalibState state, float progress) {
    if (ws.count() == 0) return;

    // Rate-limit to 10 Hz. Always let CALIB_DONE through immediately (final packet).
    const uint32_t INTERVAL_US = 100000; // 10 Hz
    uint32_t now = micros();
    if (state != CALIB_DONE && (now - _lastCalibBroadcast_us) < INTERVAL_US) return;
    _lastCalibBroadcast_us = now;

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
        stateStr    = "waiting";
        remaining_s = CALIB_TOTAL_S;
    }

    char buf[80];
    snprintf(buf, sizeof(buf),
        "CALIB:state=%s,progress=%.2f,remaining=%.1f",
        stateStr, progress, remaining_s
    );
    ws.textAll(buf);
}

// =============================================================================
//  BALANCE STATE BROADCAST
// =============================================================================

void WebComm::broadcastBalanceState(const BalanceState& state) {
    if (ws.count() == 0 || _balCtrl == nullptr) return;

    const uint32_t INTERVAL_US = 50000;  // 20 Hz
    uint32_t now = micros();
    if ((now - _lastBalanceBroadcast_us) < INTERVAL_US) return;
    _lastBalanceBroadcast_us = now;

    const BalanceConfig& cfg = _balCtrl->getConfig();
    // buf sized for full pitch + roll + task-space telemetry payload (~430 chars).
    char buf[512];
    snprintf(buf, sizeof(buf),
        // Controller enable flags + overall active state
        "BALANCE:p_en=%d,r_en=%d,"
        "eff_kp=%.2f,eff_kpr=%.2f,"           // effective Kp (with boost)
        // Pitch live outputs
        "p_err=%.4f,p_u=%.3f,p_ank=%.2f,p_hip=%.2f,p_trs=%.2f,"
        // Roll live outputs
        "r_err=%.4f,r_u=%.3f,r_ank=%.2f,r_hip=%.2f,r_trs=%.2f,"
        // Pitch config
        "Kp=%.3f,Kd=%.3f,sp=%.4f,a_r=%.2f,h_r=%.2f,t_r=%.2f,sgn=%.1f,mx=%.1f,"
        // Roll config
        "Kp_r=%.3f,Kd_r=%.3f,sp_r=%.4f,a_rr=%.2f,h_rr=%.2f,t_rr=%.2f,sgn_r=%.1f,mx_r=%.1f,"
        // Safety
        "fell=%d,"
        // Weight shift telemetry — progress and ramping flag for UI progress bar.
        "ws_prog=%.2f,ws_ramp=%d,ws_rp=%.2f,ws_lp=%.2f,"
        // Stage 3: task-space mode flag and live foot-target corrections.
        "ts=%d,dx=%.2f,dy=%.2f",
        (int)cfg.pitch_enabled, (int)cfg.roll_enabled,
        state.effective_kp,
        state.effective_kp_roll,
        state.pitch_error,   state.u_clamped,
        state.ankle_cmd_deg, state.hip_cmd_deg, state.torso_cmd_deg,
        state.roll_error,    state.u_roll_clamped,
        state.ankle_roll_cmd_deg, state.hip_roll_cmd_deg, state.torso_roll_cmd_deg,
        cfg.Kp, cfg.Kd, cfg.pitch_setpoint_rad,
        cfg.ankle_ratio, cfg.hip_ratio, cfg.torso_ratio,
        cfg.correction_sign, cfg.max_correction_deg,
        cfg.Kp_roll, cfg.Kd_roll, cfg.roll_setpoint_rad,
        cfg.ankle_roll_ratio, cfg.hip_roll_ratio, cfg.torso_roll_ratio,
        cfg.roll_correction_sign, cfg.max_roll_correction_deg,
        (int)state.fell,
        _weightShift ? _weightShift->getState().progress        : 0.0f,
        _weightShift ? (int)_weightShift->getState().ramping     : 0,
        _weightShift ? _weightShift->getState().right_progress   : 0.0f,
        _weightShift ? _weightShift->getState().left_progress    : 0.0f,
        (int)cfg.task_space_output,
        _balCtrl->getLastCorrection().dx_mm,
        _balCtrl->getLastCorrection().dy_mm
    );
    ws.textAll(buf);
}
// =============================================================================
//  GAIT STATE BROADCAST
//  Protocol: "GAIT:phase=X.XXX,mode=N,sub=N,liftL=X.X,liftR=X.X,ws=X.XXX,gate=0|1"
//  Fields:
//    phase  — display phase [0, 1): currentHalf × 0.5 + halfPhase × 0.5
//    mode   — 0=IDLE, 1=RUNNING, 2=SINGLE_STEP, 3=WSHIFT_ONLY
//    sub    — 0=WAIT_SHIFT, 1=SWINGING
//    liftL  — left hip extension command this tick (deg)
//    liftR  — right hip extension command this tick (deg)
//    ws     — WeightShift progress (signed, [-1, +1])
//    gate   — 1 if weight shift threshold met and foot lift is allowed
//  Rate-limited to 20 Hz (same as balance state broadcast).
// =============================================================================

// =============================================================================
//  WEIGHT SHIFT CONFIG BROADCAST (one-time, on connect)
//  Protocol: "WSHIFT_STATE:ankle=X,lateral_mm=X,forward_mm=X,torso=X,
//             ramp=X,phase_delay=X,setpoint_deg=X"
//  UI reads this to sync sliders to firmware state on reconnect.
//  Firmware is source of truth — UI must NOT send back a tune on receipt.
// =============================================================================

void WebComm::broadcastWeightShiftConfig(AsyncWebSocketClient* client) {
    if (!_weightShift || !client) return;
    const WeightShiftConfig& cfg = _weightShift->getConfig();
    const float spDeg = cfg.setpoint_shift_rad * (180.0f / (float)M_PI);
    char buf[192];
    snprintf(buf, sizeof(buf),
        "WSHIFT_STATE:ankle=%.1f,lateral_mm=%.1f,forward_mm=%.1f,"
        "torso=%.1f,ramp=%.0f,phase_delay=%.0f,setpoint_deg=%.2f",
        cfg.ankle_shift_deg, cfg.lateral_shift_mm, cfg.forward_lean_mm,
        cfg.torso_shift_deg, cfg.ramp_ms, cfg.shift_phase_delay_ms, spDeg
    );
    client->text(buf);
}

void WebComm::broadcastGaitState() {
    if (!_gaitCtrl || ws.count() == 0) return;

    const uint32_t INTERVAL_US = 50000;  // 20 Hz
    uint32_t now = micros();
    if ((now - _lastGaitBroadcast_us) < INTERVAL_US) return;
    _lastGaitBroadcast_us = now;

    const GaitState& s = _gaitCtrl->getState();
    char buf[192];
    snprintf(buf, sizeof(buf),
        "GAIT:phase=%.3f,mode=%d,sub=%d,liftL=%.1f,liftR=%.1f,ws=%.3f,gate=%d,hold=%d,"
        "stepH=%.1f,stanceH=%.1f",
        s.phase,
        static_cast<int>(s.mode),
        static_cast<int>(s.subState),
        s.liftL_deg,
        s.liftR_deg,
        s.wsProgress,
        static_cast<int>(s.liftGateOK),
        static_cast<int>(s.poseHold),
        _gaitCtrl->getConfig().stepHeightMm,
        _gaitCtrl->getConfig().stanceHeightMm
    );
    ws.textAll(buf);
}

// =============================================================================
//  SPEED BROADCAST
//  Protocol: "SPEED:<ch0_speed>,<ch1_speed>,...,<ch15_speed>"
//  Sent once on connect and after every SPEED or SPEED_ALL command.
//  Clients use this to sync their speed sliders on reconnect.
// =============================================================================

void WebComm::broadcastSpeeds() {
    if (!_servoCtrl || ws.count() == 0) return;
    String msg = "SPEED:";
    for (int i = 0; i < NUM_JOINTS; i++) {
        msg += String(_servoCtrl->getJointSpeed(i), 1);
        if (i < NUM_JOINTS - 1) msg += ",";
    }
    ws.textAll(msg);
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
            broadcastSpeeds();   // sync speed sliders on reconnect
        }
        // Push current calibration state immediately to the new client.
        // Without this, any client that connects after calibration finishes
        // would never receive a CALIB packet and the bar stays at 0%.
        broadcastWeightShiftConfig(client);

        if (_stateEst) {
            char buf[80];
            CalibState cs   = _stateEst->getCalibState();
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
    String message((const char*)data, len);
    if (!_servoCtrl) return;

    Serial.printf("[WebComm] Received: %s\n", message.c_str());

    if (message.startsWith("CMD:")) {
        String cmd = message.substring(4);

        // ---------------------------------------------------------------------
        //  RESETP — return all verified joints to neutral standing pose.
        //  Channels in SKIP_BOOT_NEUTRAL_CHANNELS are excluded: their mounting
        //  orientation has not been physically confirmed; commanding an unverified
        //  neutral risks mechanical damage.
        //  Routes through MotionManager so SOURCE_UI authority applies correctly:
        //  if the balance controller is active it wins on shared joints.
        // ---------------------------------------------------------------------
        if (cmd == "RESETP") {
            for (int i = 0; i < NUM_JOINTS; i++) {
                bool skip = false;
                for (uint8_t k = 0; k < SKIP_BOOT_NEUTRAL_COUNT; k++) {
                    if (static_cast<uint8_t>(i) == SKIP_BOOT_NEUTRAL_CHANNELS[k]) {
                        skip = true;
                        break;
                    }
                }
                if (!skip) {
                    if (_motionManager) {
                        // Route through authority layer so RESETP competes correctly with
                        // any active controller. SOURCE_UI loses to SOURCE_BALANCE, so if
                        // balance is active, the correction wins — which is correct.
                        _motionManager->submit(SOURCE_UI,
                                               static_cast<uint8_t>(i),
                                               JOINT_CONFIG[i].neutralDeg);
                    } else {
                        _servoCtrl->setGaitOffset(i, 0.0f);
                    }
                }
            }
            broadcastState();
        }
        // ---------------------------------------------------------------------
        else if (cmd.startsWith("LOAD:")) {
            _servoCtrl->loadPose(cmd.substring(5));
            broadcastState();
        }
        // ---------------------------------------------------------------------
        //  Named movement
        //  Protocol: CMD:MOVE_NAMED:<channel>:<movement>:<magnitude>
        //  Example:  CMD:MOVE_NAMED:2:flex:45.5
        // ---------------------------------------------------------------------
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
        // ---------------------------------------------------------------------
        //  Emergency Stop
        // ---------------------------------------------------------------------
        else if (cmd == "ESTOP") {
            oe_estop();
            broadcastState();
            ws.textAll("ESTOP:ACTIVE");
            Serial.println("[WebComm] CMD: ESTOP received — outputs disabled");
        }
        // ---------------------------------------------------------------------
        //  Balance controller enable / disable
        // BALANCE_ON / BALANCE_OFF — legacy shorthand: enable/disable BOTH controllers.
        else if (cmd == "BALANCE_ON") {
            if (_balCtrl) {
                BalanceConfig cfg = _balCtrl->getConfig();
                cfg.pitch_enabled = true;
                cfg.roll_enabled  = true;
                _balCtrl->setConfig(cfg);
            }
            Serial.println("[WebComm] CMD: BALANCE_ON (both pitch + roll)");
        }
        else if (cmd == "BALANCE_OFF") {
            if (_balCtrl) {
                BalanceConfig cfg = _balCtrl->getConfig();
                cfg.pitch_enabled = false;
                cfg.roll_enabled  = false;
                _balCtrl->setConfig(cfg);
            }
            Serial.println("[WebComm] CMD: BALANCE_OFF (both pitch + roll)");
        }
        // Individual pitch / roll enable commands — use these during testing.
        else if (cmd == "PITCH_ON") {
            if (_balCtrl) {
                BalanceConfig cfg = _balCtrl->getConfig();
                cfg.pitch_enabled = true;
                _balCtrl->setConfig(cfg);
            }
            Serial.println("[WebComm] CMD: PITCH_ON");
        }
        else if (cmd == "PITCH_OFF") {
            if (_balCtrl) {
                BalanceConfig cfg = _balCtrl->getConfig();
                cfg.pitch_enabled = false;
                _balCtrl->setConfig(cfg);
            }
            Serial.println("[WebComm] CMD: PITCH_OFF");
        }
        else if (cmd == "ROLL_ON") {
            if (_balCtrl) {
                BalanceConfig cfg = _balCtrl->getConfig();
                cfg.roll_enabled = true;
                _balCtrl->setConfig(cfg);
            }
            Serial.println("[WebComm] CMD: ROLL_ON");
        }
        else if (cmd == "ROLL_OFF") {
            if (_balCtrl) {
                BalanceConfig cfg = _balCtrl->getConfig();
                cfg.roll_enabled = false;
                _balCtrl->setConfig(cfg);
            }
            Serial.println("[WebComm] CMD: ROLL_OFF");
        }
        // ---------------------------------------------------------------------
        //  Weight shift commands
        //  Protocol: CMD:WEIGHT_SHIFT:left | right | center
        // ---------------------------------------------------------------------
        else if (cmd.startsWith("WEIGHT_SHIFT:")) {
            if (_weightShift) {
                String dir = cmd.substring(13);
                dir.trim();
                if (dir == "left") {
                    _weightShift->trigger(ShiftDirection::LEFT);
                    Serial.println("[WebComm] CMD: WEIGHT_SHIFT left");
                } else if (dir == "right") {
                    _weightShift->trigger(ShiftDirection::RIGHT);
                    Serial.println("[WebComm] CMD: WEIGHT_SHIFT right");
                } else if (dir == "center") {
                    _weightShift->trigger(ShiftDirection::NONE);
                    Serial.println("[WebComm] CMD: WEIGHT_SHIFT center");
                } else {
                    Serial.printf("[WebComm] WEIGHT_SHIFT: unknown direction '%s'\n",
                                  dir.c_str());
                }
            }
        }
        // ---------------------------------------------------------------------
        //  Weight shift tuning
        //  Protocol: CMD:WEIGHT_SHIFT_TUNE:setpoint=0.05,hip=0.0,torso=0.0,ramp=500
        // ---------------------------------------------------------------------
        else if (cmd.startsWith("WEIGHT_SHIFT_TUNE:")) {
            if (_weightShift) {
                WeightShiftConfig cfg = _weightShift->getConfig();
                String params = cmd.substring(18);
                int start = 0;
                while (start < (int)params.length()) {
                    int comma = params.indexOf(',', start);
                    String pair = (comma == -1) ? params.substring(start)
                                                : params.substring(start, comma);
                    int eq = pair.indexOf('=');
                    if (eq != -1) {
                        String key = pair.substring(0, eq);
                        float  val = pair.substring(eq + 1).toFloat();
                        // NEW:
                        if      (key == "setpoint")    cfg.setpoint_shift_rad      = val;
                        else if (key == "ankle")        cfg.ankle_shift_deg         = val;
                        else if (key == "torso")        cfg.torso_shift_deg         = val;
                        else if (key == "ramp")         cfg.ramp_ms                 = val;
                        // Stage 2: task-space shift authoring (mm).
                        else if (key == "lateral_mm")   cfg.lateral_shift_mm = constrain(val, -50.0f, 50.0f);
                        else if (key == "forward_mm")   cfg.forward_lean_mm  = constrain(val, 0.0f, 50.0f);
                        else if (key == "phase_delay") {
            // Swing-before-stance phase delay in ms.
            // 0 = simultaneous ramp. 500 = default (swing 0.5s before stance).
            cfg.shift_phase_delay_ms = constrain(val, 0.0f, 2000.0f);
        }
                    }
                    start = (comma == -1) ? params.length() : comma + 1;
                }
                _weightShift->setConfig(cfg);
                Serial.printf("[WebComm] WEIGHT_SHIFT_TUNE: setpoint=%.3f rad  "
                              "lat=%.1fmm  fwd=%.1fmm  torso=%.1f°  ramp=%.0f ms\n",
                              cfg.setpoint_shift_rad, cfg.lateral_shift_mm,
                              cfg.forward_lean_mm, cfg.torso_shift_deg, cfg.ramp_ms);
            }
        }
        
        // ---------------------------------------------------------------------
        //  Balance tuning — parses key=value pairs, normalises ratios
        // ---------------------------------------------------------------------
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
                        if      (key == "Kp") {
                            // Hard ceiling from project_wide_defs.h — prevents
                            // phase-margin violation from accidental high-gain input.
                            cfg.Kp = constrain(val, 0.0f, BALANCE_KP_MAX);
                            if (val > BALANCE_KP_MAX) {
                                Serial.printf("[WebComm] BALANCE_TUNE: Kp=%.1f clamped "
                                              "to BALANCE_KP_MAX=%.1f\n", val, BALANCE_KP_MAX);
                            }
                        }
                        else if (key == "Kd") {
                            cfg.Kd = constrain(val, 0.0f, BALANCE_KD_MAX);
                            if (val > BALANCE_KD_MAX) {
                                Serial.printf("[WebComm] BALANCE_TUNE: Kd=%.2f clamped "
                                              "to BALANCE_KD_MAX=%.2f\n", val, BALANCE_KD_MAX);
                            }
                        }
                        else if (key == "setpoint") cfg.pitch_setpoint_rad        = val;
                        else if (key == "deadband") cfg.pitch_deadband_rad         = val;
                        else if (key == "d_lpf")    cfg.derivative_lpf_alpha       = val;
                        else if (key == "tau")      cfg.servo_lag_compensation_s   = val;
                        else if (key == "rate")     cfg.max_output_rate_deg_per_tick = val;
                        else if (key == "ankle")    cfg.ankle_ratio        = val;
                        else if (key == "hip")      cfg.hip_ratio          = val;
                        else if (key == "torso")    cfg.torso_ratio        = val;
                        else if (key == "max")      cfg.max_correction_deg = val;
                        else if (key == "sign")     cfg.correction_sign        = val;
                        // ── Roll fields ───────────────────────────────────────
                        else if (key == "Kp_r") {
                            cfg.Kp_roll = constrain(val, 0.0f, BALANCE_KP_MAX);
                            if (val > BALANCE_KP_MAX) {
                                Serial.printf("[WebComm] BALANCE_TUNE: Kp_r=%.1f clamped "
                                              "to BALANCE_KP_MAX=%.1f\n", val, BALANCE_KP_MAX);
                            }
                        }
                        else if (key == "Kd_r") {
                            cfg.Kd_roll = constrain(val, 0.0f, BALANCE_KD_MAX);
                            if (val > BALANCE_KD_MAX) {
                                Serial.printf("[WebComm] BALANCE_TUNE: Kd_r=%.2f clamped "
                                              "to BALANCE_KD_MAX=%.2f\n", val, BALANCE_KD_MAX);
                            }
                        }
                        else if (key == "sp_r")     cfg.roll_setpoint_rad      = val;
                        else if (key == "a_roll")   cfg.ankle_roll_ratio       = val;
                        else if (key == "h_roll")   cfg.hip_roll_ratio         = val;
                        else if (key == "t_roll")   cfg.torso_roll_ratio       = val;
                        else if (key == "max_r")    cfg.max_roll_correction_deg = val;
                        else if (key == "sign_r")   cfg.roll_correction_sign   = val;
                        else if (key == "iir")      cfg.output_iir_alpha = constrain(val, 0.0f, 0.99f);
                        else if (key == "boost_thresh") {
            // Pitch nonlinear gain boost threshold (rad).
            // Errors below this use base Kp; above use Kp × boost_factor.
            // Default 0.10 rad (≈5.7°). Must be above typical oscillation amplitude.
            cfg.gain_boost_threshold_rad = constrain(val, 0.01f, 0.50f);
        }
        else if (key == "boost_factor") {
            // Pitch gain multiplier for large errors. 1.0 = disabled (flat Kp).
            // Default 2.0 (doubles Kp at 2× threshold). Cap at BALANCE_KP_MAX/Kp
            // so effective Kp never exceeds the safety ceiling.
            cfg.gain_boost_factor = constrain(val, 1.0f, 5.0f);
        }
        else if (key == "boost_thresh_r") {
            cfg.gain_boost_threshold_roll_rad = constrain(val, 0.01f, 0.50f);
        }
        else if (key == "boost_factor_r") {
            cfg.gain_boost_factor_roll = constrain(val, 1.0f, 5.0f);
        }
                        else if (key == "roll_db")  cfg.roll_deadband_rad = constrain(val, 0.0f, 0.20f);
                        else if (key == "roll_dlpf") cfg.roll_derivative_lpf_alpha = constrain(val, 0.0f, 0.99f);
                        else if (key == "fall_det")  cfg.fall_detection_enabled     = (val > 0.5f);
                        // Stage 3: task-space output keys.
                        else if (key == "task_space") {
                            bool newVal = (val > 0.5f);
                            if (newVal != cfg.task_space_output) {
                                // Zero both paths' IIR state on toggle to prevent spikes.
                                _balCtrl->resetOutputState();
                            }
                            if (newVal && !cfg.task_space_output) {
                                // Entering task-space: clear stale roll setpoint that
                                // WeightShift::update() may have injected in legacy mode.
                                // resetOutputState() above zeros IIR; this zeros the config
                                // field the PD law reads on the first new tick.
                                cfg.roll_setpoint_rad = 0.0f;
                            }
                            cfg.task_space_output = newVal;
                        }
                        else if (key == "dx_per_deg") cfg.pitch_to_dx_mm_per_deg = constrain(val, 0.5f, 6.0f);
                        else if (key == "dy_per_deg") cfg.roll_to_dy_mm_per_deg  = constrain(val, 0.5f, 6.0f);
                    }
                    start = (comma == -1) ? params.length() : comma + 1;
                }
                // Normalise pitch ratios so ankle + hip + torso always sum to 1.0.
                // This is enforced here (single source of truth) so any sender
                // — UI, serial, or future script — is automatically corrected.
                float ratioSum = cfg.ankle_ratio + cfg.hip_ratio + cfg.torso_ratio;
                if (ratioSum > 1e-3f) {
                    cfg.ankle_ratio /= ratioSum;
                    cfg.hip_ratio   /= ratioSum;
                    cfg.torso_ratio /= ratioSum;
                } else {
                    // All three at zero — clamp ankle to 1 as a safe fallback.
                    // Prevents a silent "no joints actuated" state.
                    Serial.println("[WebComm] BALANCE_TUNE: all ratios zero — clamping ankle=1.0");
                    cfg.ankle_ratio = 1.0f;
                }
                // Normalise roll ratios — same invariant as pitch.
                float rollRatioSum = cfg.ankle_roll_ratio + cfg.hip_roll_ratio + cfg.torso_roll_ratio;
                if (rollRatioSum > 1e-3f) {
                    cfg.ankle_roll_ratio  /= rollRatioSum;
                    cfg.hip_roll_ratio    /= rollRatioSum;
                    cfg.torso_roll_ratio  /= rollRatioSum;
                } else {
                    Serial.println("[WebComm] BALANCE_TUNE: all roll ratios zero — clamping ankle_roll=1.0");
                    cfg.ankle_roll_ratio = 1.0f;
                }
                _balCtrl->setConfig(cfg);
                Serial.printf("[WebComm] BALANCE_TUNE: Kp=%.3f Kd=%.3f sp=%.4f  ankle=%.2f hip=%.2f torso=%.2f\n",
                               cfg.Kp, cfg.Kd, cfg.pitch_setpoint_rad,
                               cfg.ankle_ratio, cfg.hip_ratio, cfg.torso_ratio);
            }
        }
       
        // ---------------------------------------------------------------------
        //  Per-joint speed control
        //  Protocol: CMD:SPEED:<channel>:<deg_per_sec>
        //  Example:  CMD:SPEED:1:45.0
        // ---------------------------------------------------------------------
        else if (cmd.startsWith("SPEED:")) {
            String params = cmd.substring(6);
            int sep = params.indexOf(':');
            if (sep != -1 && _servoCtrl) {
                uint8_t ch    = (uint8_t)params.substring(0, sep).toInt();
                float   speed = params.substring(sep + 1).toFloat();
                _servoCtrl->setJointSpeed(ch, speed);
                Serial.printf("[WebComm] SPEED: ch=%d  speed=%.1f deg/s\n", ch, speed);
                broadcastSpeeds();
            }
        }
        // ---------------------------------------------------------------------
        //  Global speed — applies the same speed to all joints at once.
        //  Protocol: CMD:SPEED_ALL:<deg_per_sec>
        //  Example:  CMD:SPEED_ALL:90.0
        // ---------------------------------------------------------------------
        else if (cmd.startsWith("SPEED_ALL:")) {
            if (_servoCtrl) {
                float speed = cmd.substring(10).toFloat();
                _servoCtrl->setAllJointsSpeed(speed);
                Serial.printf("[WebComm] SPEED_ALL: all joints → %.1f deg/s\n", speed);
                broadcastSpeeds();
            }
        }
       
        // ---------------------------------------------------------------------
        //  Clear E-stop
        // ---------------------------------------------------------------------
        else if (cmd == "CLEAR_ESTOP") {
            oe_clear();
            broadcastState();
            ws.textAll("ESTOP:CLEAR");
            Serial.println("[WebComm] CMD: CLEAR_ESTOP received — outputs enabled");
        }
        // ---------------------------------------------------------------------
        //  Gait controller commands (Phase 1: stepping in place)
        //
        //  CMD:GAIT_START   — begin continuous stepping
        //  CMD:GAIT_STOP    — stop gait, return to IDLE (centers WeightShift)
        //  CMD:GAIT_STEP    — one full cycle (both legs), then IDLE
        //  CMD:GAIT_WSHIFT  — weight-shift-only mode (no foot lift)
        //
        //  CMD:GAIT_TUNE:phaseRate=X,liftGate=X,wsThresh=X
        //    All fields optional; unrecognised keys are silently skipped.
        //    phaseRate: cycles/sec (0.1–1.0). Start at 0.3.
        //    liftGate: |wsProgress| threshold before lift allowed (0.0–1.0).
        //    wsThresh: IMU roll threshold (rad) — logged only in Phase 1.
        // ---------------------------------------------------------------------
        else if (cmd == "GAIT_START") {
            if (_gaitCtrl) {
                _gaitCtrl->start(GaitMode::RUNNING);
                Serial.println("[WebComm] CMD: GAIT_START");
            }
        }
        else if (cmd == "GAIT_STOP") {
            if (_gaitCtrl) {
                _gaitCtrl->stop();
                Serial.println("[WebComm] CMD: GAIT_STOP");
            }
        }
        else if (cmd == "GAIT_STEP") {
            if (_gaitCtrl) {
                _gaitCtrl->singleStep();
                Serial.println("[WebComm] CMD: GAIT_STEP");
            }
        }
        else if (cmd == "GAIT_WSHIFT") {
            if (_gaitCtrl) {
                _gaitCtrl->start(GaitMode::WSHIFT_ONLY);
                Serial.println("[WebComm] CMD: GAIT_WSHIFT");
            }
        }
        else if (cmd == "GAIT_POSE_STEP") {
            if (_gaitCtrl) {
                _gaitCtrl->start(GaitMode::POSE_STEP);
                Serial.println("[WebComm] CMD: GAIT_POSE_STEP");
            }
        }
        else if (cmd == "GAIT_NEXT") {
            if (_gaitCtrl) {
                _gaitCtrl->nextPose();
                Serial.println("[WebComm] CMD: GAIT_NEXT");
            }
        }
        else if (cmd.startsWith("GAIT_TUNE:")) {
            if (_gaitCtrl) {
                GaitConfig cfg = _gaitCtrl->getConfig();
                const float gaitMaxReachMm = IK_MAX_REACH_FRACTION * (L_THIGH + L_SHANK);
                String params = cmd.substring(10);
                int start = 0;
                while (start < (int)params.length()) {
                    int comma = params.indexOf(',', start);
                    String pair = (comma == -1) ? params.substring(start)
                                                : params.substring(start, comma);
                    int eq = pair.indexOf('=');
                    if (eq != -1) {
                        String key = pair.substring(0, eq);
                        float  val = pair.substring(eq + 1).toFloat();
                        if      (key == "phaseRate")   cfg.phaseRateHz       = constrain(val, 0.05f, 2.0f);
                        else if (key == "liftGate")    cfg.liftGateThreshold = constrain(val, 0.0f, 0.99f);
                        else if (key == "wsThresh")    cfg.wsThresholdRad    = constrain(val, 0.01f, 0.50f);
                        else if (key == "stepH_mm") {
                            const float maxStep = fmaxf(5.0f, cfg.stanceHeightMm - 130.0f);
                            cfg.stepHeightMm = constrain(val, 5.0f, maxStep);
                        }
                        else if (key == "stanceH_mm") {
                            const float minStance = 130.0f + cfg.stepHeightMm;
                            cfg.stanceHeightMm = constrain(val, minStance, gaitMaxReachMm);
                        }
                    }
                    start = (comma == -1) ? params.length() : comma + 1;
                }
                _gaitCtrl->setConfig(cfg);
                Serial.printf("[WebComm] GAIT_TUNE: rate=%.2f Hz  gate=%.2f  wsThresh=%.3f rad  "
                              "stepH=%.1f mm  stanceH=%.1f mm\n",
                              cfg.phaseRateHz, cfg.liftGateThreshold, cfg.wsThresholdRad,
                              cfg.stepHeightMm, cfg.stanceHeightMm);
            }
        }

        // ─────────────────────────────────────────────────────────────────────
        //  IK TEST COMMANDS
        //
        //  These commands expose LegIK functions to the UI so tests can be
        //  triggered without serial access.
        //
        //  Validate commands (IK_VALIDATE, IK_NEUTRAL):
        //    Print full diagnostics to Serial; broadcast a summary to the UI.
        //    Do NOT move servos.
        //
        //  Motion commands (IK_CROUCH, IK_FRONTAL_R/L, IK_SOLVE with submit=1):
        //    Submit angles to MotionManager via SOURCE_GAIT.
        //    SOURCE_BALANCE (balance controller) will override if active.
        //    DISABLE BALANCE CONTROLLER BEFORE RUNNING MOTION TESTS.
        //
        //  Response format (broadcast to all clients):
        //    IK_RESULT:status=N,hip=X,knee=X,ankle=X,h_roll=X,a_roll=X,
        //              reach=X,ff=X,pos_err=X,sub=0|1
        //    IK_ERROR:reason
        // ─────────────────────────────────────────────────────────────────────
        else if (cmd.startsWith("IK_")) {

            // Inline broadcast: packs result into one WS message.
            // pos_err: sagittal FK roundtrip error in mm (0 if not computed).
            // submitted: true if angles were dispatched to MotionManager.
            auto broadcastIK = [&](const LegIKResult& r, float pos_err, bool submitted) {
                char buf[256];
                snprintf(buf, sizeof(buf),
                    "IK_RESULT:status=%d,hip=%.3f,knee=%.3f,ankle=%.3f,"
                    "h_roll=%.3f,a_roll=%.3f,reach=%.1f,ff=%.4f,pos_err=%.5f,sub=%d",
                    static_cast<int>(r.status),
                    r.hip_pitch_deg, r.knee_pitch_deg, r.ankle_pitch_deg,
                    r.hip_roll_deg,  r.ankle_roll_deg,
                    r.sagittal_reach_pct, r.flat_foot_error_deg,
                    pos_err, static_cast<int>(submitted));
                ws.textAll(buf);
            };

            // FK roundtrip position error helper.
            // Returns the Euclidean distance (mm) between the IK target and
            // the foot position recovered from the computed angles via FK.
            auto fkError = [](const LegIKResult& r, float x_mm, float h_mm) -> float {
                float fk_x, fk_h;
                LegIK::forwardSagittal(r.hip_pitch_deg, r.knee_pitch_deg, fk_x, fk_h);
                const float dx = fk_x - x_mm;
                const float dh = fk_h - h_mm;
                return sqrtf(dx*dx + dh*dh);
            };

            // Reusable key=value parser — same pattern as BALANCE_TUNE.
            struct KVParams {
                float x = 0.0f, h = 160.0f, y = 0.0f;
                float hip = 0.0f, knee = 0.0f, ankle = 0.0f;
                String leg = "both";
                bool submit = false;
            };
            auto parseKV = [](const String& params) -> KVParams {
                KVParams p;
                int start = 0;
                while (start < (int)params.length()) {
                    int comma = params.indexOf(',', start);
                    String pair = (comma == -1) ? params.substring(start)
                                                : params.substring(start, comma);
                    int eq = pair.indexOf('=');
                    if (eq != -1) {
                        String key = pair.substring(0, eq);
                        String val = pair.substring(eq + 1);
                        if      (key == "x")      p.x      = val.toFloat();
                        else if (key == "h")      p.h      = val.toFloat();
                        else if (key == "y")      p.y      = val.toFloat();
                        else if (key == "hip")    p.hip    = val.toFloat();
                        else if (key == "knee")   p.knee   = val.toFloat();
                        else if (key == "ankle")  p.ankle  = val.toFloat();
                        else if (key == "leg")    p.leg    = val;
                        else if (key == "submit") p.submit = (val.toInt() == 1);
                    }
                    start = (comma == -1) ? params.length() : comma + 1;
                }
                return p;
            };

            // ── IK_VALIDATE:x=...,h=...,y=... ────────────────────────────────
            // Runs validateRoundtrip (full diagnostics to Serial).
            // Broadcasts angle summary and FK position error to UI.
            // Does NOT move any servos.
            if (cmd.startsWith("IK_VALIDATE:")) {
                KVParams p = parseKV(cmd.substring(12));
                FootTarget t; t.x_mm = p.x; t.h_sagittal_mm = p.h; t.y_mm = p.y;
                LegIK::validateRoundtrip(t, 0.5f);  // detailed output → Serial
                LegIKResult r = LegIK::solve(t);
                broadcastIK(r, fkError(r, p.x, p.h), false);
            }
            // ── IK_NEUTRAL:hip=...,knee=...,ankle=... ────────────────────────
            // Runs validateNeutral against measured calibration angles.
            // Broadcasts the IK result at the implied foot position.
            // Does NOT move any servos.
            else if (cmd.startsWith("IK_NEUTRAL:")) {
                KVParams p = parseKV(cmd.substring(11));
                LegIK::validateNeutral(p.hip, p.knee, p.ankle, 1.5f);  // → Serial
                // Compute foot position implied by these angles via FK, then
                // solve IK so the UI sees what the geometry predicts.
                float x_impl, h_impl;
                LegIK::forwardSagittal(p.hip, p.knee, x_impl, h_impl);
                FootTarget t; t.x_mm = x_impl; t.h_sagittal_mm = h_impl;
                LegIKResult r = LegIK::solve(t);
                broadcastIK(r, fkError(r, x_impl, h_impl), false);
            }
            // ── IK_CROUCH ─────────────────────────────────────────────────────
            // Sagittal sign test: x=0 mm, h=160 mm, y=0 mm, both legs.
            // With balance OFF, robot should lower into a symmetric crouch.
            // If it arches backward, flip correction_sign in balance_controller.
            else if (cmd == "IK_CROUCH") {
                if (!_motionManager) { ws.textAll("IK_ERROR:MotionManager not wired"); return; }
                FootTarget t; t.x_mm = 0.0f; t.h_sagittal_mm = 160.0f; t.y_mm = 0.0f;
                LegIKResult r = LegIK::solve(t);
                if (r.ok()) {
                    _motionManager->submit(SOURCE_GAIT, IDX_R_HIP_PITCH,   r.hip_pitch_deg);
                    _motionManager->submit(SOURCE_GAIT, IDX_R_KNEE_PITCH,  r.knee_pitch_deg);
                    _motionManager->submit(SOURCE_GAIT, IDX_R_ANKLE_PITCH, r.ankle_pitch_deg);
                    _motionManager->submit(SOURCE_GAIT, IDX_L_HIP_PITCH,   r.hip_pitch_deg);
                    _motionManager->submit(SOURCE_GAIT, IDX_L_KNEE_PITCH,  r.knee_pitch_deg);
                    _motionManager->submit(SOURCE_GAIT, IDX_L_ANKLE_PITCH, r.ankle_pitch_deg);
                    broadcastIK(r, fkError(r, 0.0f, 160.0f), true);
                    Serial.printf("[WebComm] IK_CROUCH: hip=%.2f knee=%.2f ankle=%.2f\n",
                                  r.hip_pitch_deg, r.knee_pitch_deg, r.ankle_pitch_deg);
                } else {
                    ws.textAll("IK_ERROR:DOMAIN_ERROR in sagittal solve");
                }
            }
            // ── IK_FRONTAL_R / IK_FRONTAL_L ──────────────────────────────────
            // Frontal sign test: y=20 mm outward on one leg. Balance must be OFF.
            // Expected: sole tilts outward (inversion) on the commanded leg.
            // If sole tilts inward: flip ankle_roll_deg sign in solveFrontal().
            // If asymmetric L vs R: check direction=−1 on left ankle roll in
            // joint_config.cpp.
            else if (cmd == "IK_FRONTAL_R" || cmd == "IK_FRONTAL_L") {
                if (!_motionManager) { ws.textAll("IK_ERROR:MotionManager not wired"); return; }
                const bool isRight = (cmd == "IK_FRONTAL_R");
                FootTarget t; t.x_mm = 0.0f; t.h_sagittal_mm = 160.0f; t.y_mm = 20.0f;
                LegIKResult r = LegIK::solve(t);
                if (r.ok()) {
                    const uint8_t hipCh   = isRight ? IDX_R_HIP_ROLL   : IDX_L_HIP_ROLL;
                    const uint8_t ankleCh = isRight ? IDX_R_ANKLE_ROLL : IDX_L_ANKLE_ROLL;
                    _motionManager->submit(SOURCE_GAIT, hipCh,   r.hip_roll_deg);
                    _motionManager->submit(SOURCE_GAIT, ankleCh, r.ankle_roll_deg);
                    broadcastIK(r, 0.0f, true);
                    Serial.printf("[WebComm] IK_FRONTAL_%c: h_roll=%.2f a_roll=%.2f\n",
                                  isRight ? 'R' : 'L', r.hip_roll_deg, r.ankle_roll_deg);
                } else {
                    ws.textAll("IK_ERROR:DOMAIN_ERROR in frontal solve");
                }
            }
            // ── IK_SOLVE:x=...,h=...,y=...,leg=right|left|both,submit=0|1 ────
            // Custom foot target from the UI panel.
            // submit=0: compute and display only (no servo movement).
            // submit=1: also dispatches to MotionManager — MOVES SERVOS.
            else if (cmd.startsWith("IK_SOLVE:")) {
                KVParams p = parseKV(cmd.substring(9));
                FootTarget t; t.x_mm = p.x; t.h_sagittal_mm = p.h; t.y_mm = p.y;
                LegIKResult r = LegIK::solve(t);
                bool submitted = false;
                if (p.submit && r.ok() && _motionManager) {
                    const bool doRight = (p.leg == "right" || p.leg == "both");
                    const bool doLeft  = (p.leg == "left"  || p.leg == "both");
                    if (doRight) {
                        _motionManager->submit(SOURCE_GAIT, IDX_R_HIP_PITCH,   r.hip_pitch_deg);
                        _motionManager->submit(SOURCE_GAIT, IDX_R_KNEE_PITCH,  r.knee_pitch_deg);
                        _motionManager->submit(SOURCE_GAIT, IDX_R_ANKLE_PITCH, r.ankle_pitch_deg);
                        _motionManager->submit(SOURCE_GAIT, IDX_R_HIP_ROLL,    r.hip_roll_deg);
                        _motionManager->submit(SOURCE_GAIT, IDX_R_ANKLE_ROLL,  r.ankle_roll_deg);
                    }
                    if (doLeft) {
                        _motionManager->submit(SOURCE_GAIT, IDX_L_HIP_PITCH,   r.hip_pitch_deg);
                        _motionManager->submit(SOURCE_GAIT, IDX_L_KNEE_PITCH,  r.knee_pitch_deg);
                        _motionManager->submit(SOURCE_GAIT, IDX_L_ANKLE_PITCH, r.ankle_pitch_deg);
                        _motionManager->submit(SOURCE_GAIT, IDX_L_HIP_ROLL,    r.hip_roll_deg);
                        _motionManager->submit(SOURCE_GAIT, IDX_L_ANKLE_ROLL,  r.ankle_roll_deg);
                    }
                    submitted = true;
                }
                broadcastIK(r, fkError(r, p.x, p.h), submitted);
                Serial.printf("[WebComm] IK_SOLVE: x=%.1f h=%.1f y=%.1f leg=%s "
                              "submit=%d status=%d\n",
                              p.x, p.h, p.y, p.leg.c_str(), (int)p.submit,
                              static_cast<int>(r.status));
            }
        }
    }
    
    // -------------------------------------------------------------------------
    //  Pose save
    // -------------------------------------------------------------------------
    else if (message.startsWith("SAVE:")) {
        _servoCtrl->saveCurrentPose(message.substring(5));
        ws.textAll(_servoCtrl->listPoses());
    }
    // -------------------------------------------------------------------------
    //  Slider control: "<channel>:<angle_degrees>"
    //  Routes through MotionManager so SOURCE_UI authority is declared correctly.
    //  Any active controller (BALANCE, GAIT) will silently override shared joints.
    // -------------------------------------------------------------------------
    else {
        int sep = message.indexOf(':');
        if (sep != -1) {
            uint8_t ch    = (uint8_t)message.substring(0, sep).toInt();
            float   angle = message.substring(sep + 1).toFloat();

            if (_motionManager) {
                _motionManager->submit(SOURCE_UI, ch, angle);
            } else {
                _servoCtrl->setTargetAngle(ch, angle);
            }
            broadcastState();
        }
    }
}
