#include "webComm.h"
#include <LittleFS.h>
#include "oe_control.h"
#include "joint_config.h"
#include "servo_driver.h"
#include "motion_manager.h"    // MotionSource, MotionManager::submit()
#include "weight_shift.h"      // WeightShift, ShiftDirection, WeightShiftConfig
#include "leg_ik.h"            // LegIK, FootTarget, LegIKResult — IK test commands
#include "gait_controller.h"
#include "torso_stabilizer.h"  // TorsoStabilizer, TorsoConfig — joint-space torso balance
#include "swing_foot_leveler.h" // SwingFootLeveler — SWING_LEVELER_TUNE + GAIT: telemetry
#include "lsm6dsv_driver.h"    // LSM6DSVDriver for foot IMU diagnostic
#include "foot_contact.h"
#include "foot_impact.h"
#include "walking_instrumentation.h"  // per-step walking diagnostics (telemetry only)

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

void WebComm::setTorsoStabilizer(TorsoStabilizer* ts) {
    _torsoStab = ts;
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

void WebComm::setFootIMUs(LSM6DSVDriver* left, LSM6DSVDriver* right) {
    _leftFootIMU  = left;
    _rightFootIMU = right;
}

void WebComm::setFootContactConfig(FootContactConfig* cfg) {
    _footContactCfg = cfg;
}

void WebComm::setSwingFootLeveler(SwingFootLeveler* l) {
    _swingLeveler = l;
}

void WebComm::setFootImpactDetector(FootImpactDetector* d) {
    _footImpact = d;
}

void WebComm::setWalkingInstrumentation(WalkingInstrumentation* w) {
    _walkInstr = w;
}

void WebComm::setFootNeutralPitch(float leftRad, float rightRad) {
    _leftFootNeutralPitchRad  = leftRad;
    _rightFootNeutralPitchRad = rightRad;
}

// Phase 3 — Side-keyed overload. See header for migration guidance.
void WebComm::setFootNeutralPitch(Side side, float rad) {
    if (side == Side::LEFT) _leftFootNeutralPitchRad  = rad;
    else                     _rightFootNeutralPitchRad = rad;
}

// Boot-tare neutral capture (pitch + roll + validity). On an unclean tare
// (valid=false) the stored neutral is forced to 0 so a stale or garbage
// attitude can never leak into the telemetry preview.
void WebComm::setFootNeutral(Side side, float pitchRad, float rollRad, bool valid) {
    const float p = valid ? pitchRad : 0.0f;
    const float r = valid ? rollRad  : 0.0f;
    if (side == Side::LEFT) {
        _leftFootNeutralPitchRad = p;
        _leftFootNeutralRollRad  = r;
        _leftFootNeutralValid    = valid;
    } else {
        _rightFootNeutralPitchRad = p;
        _rightFootNeutralRollRad  = r;
        _rightFootNeutralValid    = valid;
    }
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

// ---------------------------------------------------------------------------
//  _sendFiltered — per-client broadcast helper.
//
//  Iterates over connected clients and skips THIS client (continue) if its
//  send queue depth exceeds the safety threshold. Replaces the prior pattern
//  of scanning queue depth and returning globally on any backed-up client,
//  which caused intermittent UI telemetry freezes: a single ungracefully
//  disconnected client (laptop sleep, mobile background) would leave a
//  zombie session whose queueLen never drained, blocking server->client
//  broadcasts to every other client until cleanupClients() reaped it.
//
//  Applied to all char-buffer broadcasts (rate-limited telemetry + one-shot
//  per-client responses like IK_RESULT, FOOT_PROBE, TORSO_TARE). String
//  literals like "ESTOP:ACTIVE" and "ESTOP:CLEAR" still use ws.textAll —
//  those are short, infrequent, and benign even on a backed-up client.
// ---------------------------------------------------------------------------
void WebComm::_sendFiltered(const char* msg) {
    if (msg == nullptr) return;
    for (auto& c : ws.getClients()) {
        if (c.status() != WS_CONNECTED) continue;
        if (c.queueLen() > 16) continue;   // skip THIS client, not all
        c.text(msg);
    }
}

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
    _sendFiltered(buf);
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

    char buf[128];
    snprintf(buf, sizeof(buf),
        "ESTIMATE:pitch=%.4f,roll=%.4f,pitchRate=%.4f,rollRate=%.4f,"
        "vxCoM=%.1f,vyCoM=%.1f",
        state.pitch, state.roll, state.pitchRate, state.rollRate,
        state.vx_CoM_mm_s, state.vy_CoM_mm_s
    );
    _sendFiltered(buf);
}

// =============================================================================
//  FOOT IMU BROADCAST  (Phase F1)
//  Protocol: "FOOT_IMU:l_ax=...,l_ay=...,...,l_gz=...;r_ax=...,...,r_gz=..."
//  Rate-limited to 20 Hz.
// =============================================================================
void WebComm::broadcastFootIMU(const RawIMUData& left, const RawIMUData& right) {
    if (ws.count() == 0) return;
    const uint32_t INTERVAL_US = 100000;  // 10 Hz
    uint32_t now = micros();
    if ((now - _lastFootIMUBroadcast_us) < INTERVAL_US) return;
    _lastFootIMUBroadcast_us = now;

    // Single packet for both feet so the UI sees them together with one parse.
    // Invalid samples broadcast 0 — UI should display the validity flag instead
    // of trying to interpret 0 as "near vertical".
    char buf[160];
    snprintf(buf, sizeof(buf),
        "FOOT_IMU:l_ax=%d,l_ay=%d,l_az=%d,l_gx=%d,l_gy=%d,l_gz=%d,l_v=%d;"
        "r_ax=%d,r_ay=%d,r_az=%d,r_gx=%d,r_gy=%d,r_gz=%d,r_v=%d",
        left.accel_x, left.accel_y, left.accel_z,
        left.gyro_x,  left.gyro_y,  left.gyro_z,  (int)left.valid,
        right.accel_x, right.accel_y, right.accel_z,
        right.gyro_x,  right.gyro_y,  right.gyro_z, (int)right.valid
    );
    _sendFiltered(buf);
}

// =============================================================================
//  FOOT ESTIMATE BROADCAST  (Phase F1)
//  Protocol: "FOOT_ESTIMATE:l_pitch=X,l_roll=X,l_pr=X,l_rr=X,l_v=N;
//             r_pitch=X,r_roll=X,r_pr=X,r_rr=X,r_v=N"
//  Rate-limited to 20 Hz. Pitch/roll in radians; rates in rad/s.
// =============================================================================
void WebComm::broadcastFootEstimate(const IMUState& left, const IMUState& right) {
    if (ws.count() == 0) return;
    const uint32_t INTERVAL_US = 100000;  // 10 Hz
    uint32_t now = micros();
    if ((now - _lastFootEstimateBroadcast_us) < INTERVAL_US) return;
    _lastFootEstimateBroadcast_us = now;

    char buf[200];
    snprintf(buf, sizeof(buf),
        "FOOT_ESTIMATE:l_pitch=%.4f,l_roll=%.4f,l_pr=%.4f,l_rr=%.4f,l_v=%d;"
        "r_pitch=%.4f,r_roll=%.4f,r_pr=%.4f,r_rr=%.4f,r_v=%d",
        left.pitch,  left.roll,  left.pitchRate,  left.rollRate,  (int)left.valid,
        right.pitch, right.roll, right.pitchRate, right.rollRate, (int)right.valid
    );
    _sendFiltered(buf);
}

// =============================================================================
//  FOOT CONTACT BROADCAST  (Phase F2)
//  Protocol: "FOOT_CONTACT:l_flat=0|1,l_loaded=0|1,l_pdev=X,
//             r_flat=0|1,r_loaded=0|1,r_pdev=X,
//             flatP=X,flatR=X,loadDev=X,loadSign=+1|-1"
//  Rate-limited to 10 Hz. Pitch deviation fields are signed radians after the
//  loaded-sign transform, so the UI can show margin-to-threshold directly.
// =============================================================================
void WebComm::broadcastFootContact(const FootContactSnapshot* snap,
                                   const IMUState& lf, const IMUState& rf) {
    if (snap == nullptr || _footContactCfg == nullptr || ws.count() == 0) return;
    const uint32_t INTERVAL_US = 100000;  // 10 Hz
    uint32_t now = micros();
    if ((now - _lastFootContactBroadcast_us) < INTERVAL_US) return;
    _lastFootContactBroadcast_us = now;

    const FootContactConfig& cfg = *_footContactCfg;

    // Neutral-corrected pitch deviation (the F4 pdev the old stub anticipated):
    //   pdev = (foot.pitch - neutralPitch) * loaded_pitch_sign
    // i.e. the exact quantity isFootLoaded() thresholds. Signed radians.
    const float l_pdev = lf.valid
        ? (lf.pitch - _leftFootNeutralPitchRad)  * cfg.loaded_pitch_sign : 0.0f;
    const float r_pdev = rf.valid
        ? (rf.pitch - _rightFootNeutralPitchRad) * cfg.loaded_pitch_sign : 0.0f;

    // Neutral-corrected predicate PREVIEW (telemetry only — distinct from the
    // gate booleans above). Routes the captured neutral through the SAME
    // foot_contact predicate functions the gate uses, so the operator can see
    // what the gate WOULD read once the control path is flipped onto the tare.
    const bool l_flatN   = isFootFlat(lf, _leftFootNeutralPitchRad,
                                      _leftFootNeutralRollRad, cfg);
    const bool r_flatN   = isFootFlat(rf, _rightFootNeutralPitchRad,
                                      _rightFootNeutralRollRad, cfg);
    const bool l_loadedN = isFootLoaded(lf, _leftFootNeutralPitchRad,  cfg);
    const bool r_loadedN = isFootLoaded(rf, _rightFootNeutralPitchRad, cfg);

    char buf[512];
    snprintf(buf, sizeof(buf),
        "FOOT_CONTACT:l_flat=%d,l_loaded=%d,l_pdev=%.4f,"
        "r_flat=%d,r_loaded=%d,r_pdev=%.4f,"
        "flatP=%.4f,flatR=%.4f,loadDev=%.4f,loadSign=%.1f,"
        // Boot-tare neutrals (rad) + validity, and the neutral-corrected
        // predicate preview. nV=0 means an unclean tare (fail-loud fallback=0).
        "l_neutP=%.4f,l_neutR=%.4f,l_neutV=%d,l_flatN=%d,l_loadedN=%d,"
        "r_neutP=%.4f,r_neutR=%.4f,r_neutV=%d,r_flatN=%d,r_loadedN=%d",
        (int)snap->leftFlat,  (int)snap->leftLoaded,  l_pdev,
        (int)snap->rightFlat, (int)snap->rightLoaded, r_pdev,
        cfg.flat_pitch_threshold_rad,
        cfg.flat_roll_threshold_rad,
        cfg.loaded_pitch_deviation_rad,
        cfg.loaded_pitch_sign,
        _leftFootNeutralPitchRad,  _leftFootNeutralRollRad,
        (int)_leftFootNeutralValid,  (int)l_flatN, (int)l_loadedN,
        _rightFootNeutralPitchRad, _rightFootNeutralRollRad,
        (int)_rightFootNeutralValid, (int)r_flatN, (int)r_loadedN
    );
    _sendFiltered(buf);
}

// =============================================================================
//  FOOT IMPACT BROADCAST  (L0)
//  Protocol: "FOOT_IMPACT:en=0|1,thr=X,lpf=X,ref=N,v_l=0|1,..."
//  Rate-limited to 10 Hz.
// =============================================================================
void WebComm::broadcastFootImpact() {
    if (_footImpact == nullptr || ws.count() == 0) return;
    const uint32_t INTERVAL_US = 100000;  // 10 Hz
    uint32_t now = micros();
    if ((now - _lastFootImpactBroadcast_us) < INTERVAL_US) return;
    _lastFootImpactBroadcast_us = now;

    const FootImpactConfig& c = _footImpact->getConfig();
    const FootImpactSideState& L = _footImpact->getState(Side::LEFT);
    const FootImpactSideState& R = _footImpact->getState(Side::RIGHT);

    char buf[320];
    snprintf(buf, sizeof(buf),
        "FOOT_IMPACT:"
        "en=%d,thr=%.3f,lpf=%.3f,ref=%lu,"
        "v_l=%d,ar_l=%d,d_l=%d,m_l=%.3f,dl_l=%.3f,c_l=%lu,ms_l=%lu,lm_l=%.3f,ld_l=%.3f,"
        "v_r=%d,ar_r=%d,d_r=%d,m_r=%.3f,dl_r=%.3f,c_r=%lu,ms_r=%lu,lm_r=%.3f,ld_r=%.3f",
        c.enabled ? 1 : 0, c.threshold_g, c.lpf_alpha, (unsigned long)c.refractory_ms,
        L.valid ? 1 : 0, L.armed ? 1 : 0, L.detected ? 1 : 0,
        L.magnitude_g, L.delta_g, (unsigned long)L.detect_count,
        (unsigned long)L.ms_since_last, L.last_magnitude_g, L.last_delta_g,
        R.valid ? 1 : 0, R.armed ? 1 : 0, R.detected ? 1 : 0,
        R.magnitude_g, R.delta_g, (unsigned long)R.detect_count,
        (unsigned long)R.ms_since_last, R.last_magnitude_g, R.last_delta_g
    );
    _sendFiltered(buf);
}

// =============================================================================
//  WALKING INSTRUMENTATION BROADCAST
//
//  Per-step walking diagnostics. Emits one "WALKING:" packet whenever a step
//  completes, plus a ≤1 Hz heartbeat so a freshly-opened panel populates.
//  Packet layout (text, comma-separated key=value, history is ';'-joined):
//    WALKING:en=..,step=..,n=..,onsets=..,<k>=cur,<k>A=avg,<k>H=h1;h2;..,..
//  History is newest-first (h1 = Step -1). No-op if not wired / no clients.
// =============================================================================
void WebComm::broadcastWalkingInstr() {
    if (_walkInstr == nullptr || ws.count() == 0) return;

    const WalkingInstrState& s = _walkInstr->getState();

    // Send on step change; otherwise heartbeat at ≤1 Hz (metrics only change
    // once per completed step, so a fixed high rate would be pure waste).
    const uint32_t now = micros();
    const bool stepChanged = (s.completedSteps != _lastWalkInstrStep);
    if (!stepChanged && (now - _lastWalkInstrBroadcast_us) < 1000000UL) return;
    _lastWalkInstrBroadcast_us = now;
    _lastWalkInstrStep         = s.completedSteps;

    // Short wire keys, in WiMetric enum order.
    static const char* const WI_KEYS[WI_METRIC_COUNT] = {
        "tdAcc", "tdImp", "tdSwSpd", "dblTd", "dblInt",
        "swSpd", "swHor", "swVer", "swEng",
        "comBob", "comVz", "comLat", "comSag", "trRate", "trTd",
        "reach", "ffGuard",
        "tdVert", "tdHoriz", "tdDur", "tdWinRoll", "rollT"
    };

    char buf[4096];
    const int cap = (int)sizeof(buf);
    int n = snprintf(buf, cap, "WALKING:en=%d,step=%lu,n=%u,onsets=%u",
                     s.enabled ? 1 : 0,
                     (unsigned long)s.completedSteps,
                     (unsigned)s.histCount,
                     (unsigned)s.lastOnsetCount);
    if (n < 0) return;
    if (n > cap) n = cap;

    for (uint8_t k = 0; k < WI_METRIC_COUNT; ++k) {
        if (n >= cap) break;
        int w = snprintf(buf + n, cap - n, ",%s=%.3f,%sA=%.3f,%sH=",
                         WI_KEYS[k], s.current.v[k], WI_KEYS[k], s.avg[k], WI_KEYS[k]);
        if (w < 0) break;
        n += w; if (n >= cap) { n = cap; break; }
        for (uint8_t i = 0; i < s.histCount; ++i) {
            if (n >= cap) break;
            int wh = snprintf(buf + n, cap - n, "%s%.3f",
                              (i == 0) ? "" : ";", s.hist[i].v[k]);
            if (wh < 0) break;
            n += wh; if (n >= cap) { n = cap; break; }
        }
    }

    // Onset timeline (audit): per-step onset offsets in ms (primary = 0),
    // newest-first. Steps joined by '|', offsets within a step by ';'.
    //   onTL=0;41;88|0;52|0
    if (n < cap) {
        int w = snprintf(buf + n, cap - n, ",onTL=");
        if (w > 0) { n += w; if (n > cap) n = cap; }
    }
    for (uint8_t i = 0; i < s.histCount && n < cap; ++i) {
        if (i > 0) {
            int w = snprintf(buf + n, cap - n, "|");
            if (w < 0) break; n += w; if (n >= cap) { n = cap; break; }
        }
        const WalkingStepMetrics& st = s.hist[i];
        for (uint8_t j = 0; j < st.onsetN && n < cap; ++j) {
            int w = snprintf(buf + n, cap - n, "%s%u",
                             (j == 0) ? "" : ";", (unsigned)st.onsetMs[j]);
            if (w < 0) break; n += w; if (n >= cap) { n = cap; break; }
        }
    }
    _sendFiltered(buf);
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
    _sendFiltered(buf);
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
    // buf sized for full pitch + roll + task-space + recovery telemetry
    // + recovery thresholds + boost + dx/dy + iir config fields (~600 chars).
    char buf[800];
    snprintf(buf, sizeof(buf),
        // Controller enable flags + overall active state
        "BALANCE:p_en=%d,r_en=%d,"
        "eff_kp=%.2f,eff_kpr=%.2f,"           // effective Kp (with boost)
        // Pitch live outputs
        "p_err=%.4f,p_u=%.3f,p_ank=%.2f,p_hip=%.2f,"
        // Roll live outputs
        "r_err=%.4f,r_u=%.3f,r_ank=%.2f,r_hip=%.2f,"
        // Pitch config
        "Kp=%.3f,Kd=%.3f,sp=%.4f,sgn=%.1f,mx=%.1f,"
        // Roll config
        "Kp_r=%.3f,Kd_r=%.3f,sp_r=%.4f,sgn_r=%.1f,mx_r=%.1f,"
        // Safety
        "fell=%d,"
        // Weight shift telemetry — progress and ramping flag for UI progress bar.
        "ws_prog=%.2f,ws_ramp=%d,ws_rp=%.2f,ws_lp=%.2f,"
        // Live task-space foot-target corrections.
        "dx=%.2f,dy=%.2f,"
        // Stage 0: catch-step recovery detection telemetry.
        "rcv=%d,sat=%u,rdb=%u,"
        "bias_l=%.2f,bias_r=%.2f,"
        // Recovery detection thresholds (broadcast so UI sliders sync
        // without a separate one-shot packet — see ownership rules in
        // index.html). Units: pitch/roll in rad, sat/conf in ticks.
        "rcv_pitch=%.4f,rcv_roll=%.4f,rcv_sat=%u,rcv_conf=%u,"
        // Gain-boost (pitch + roll), task-space gain, output IIR. Required
        // so the dedicated sliders on these knobs sync to firmware state
        // and never push their HTML defaults back.
        "boost_th=%.4f,boost_fc=%.2f,boost_thR=%.4f,boost_fcR=%.2f,"
        "dx_pd=%.2f,dy_pd=%.2f,iir=%.3f",
        (int)cfg.pitch_enabled, (int)cfg.roll_enabled,
        state.effective_kp,
        state.effective_kp_roll,
        state.pitch_error,   state.u_clamped,
        state.ankle_cmd_deg, state.hip_cmd_deg,
        state.roll_error,    state.u_roll_clamped,
        state.ankle_roll_cmd_deg, state.hip_roll_cmd_deg,
        cfg.Kp, cfg.Kd, cfg.pitch_setpoint_rad,
        cfg.correction_sign, cfg.max_correction_deg,
        cfg.Kp_roll, cfg.Kd_roll, cfg.roll_setpoint_rad,
        cfg.roll_correction_sign, cfg.max_roll_correction_deg,
        (int)state.fell,
        _weightShift ? _weightShift->getState().progress        : 0.0f,
        _weightShift ? (int)_weightShift->getState().ramping     : 0,
        _weightShift ? _weightShift->getState().right_progress   : 0.0f,
        _weightShift ? _weightShift->getState().left_progress    : 0.0f,
        _balCtrl->getLastCorrection().dx_mm,
        _balCtrl->getLastCorrection().dy_mm,
        (int)state.recovery_flag,
        (unsigned)state.saturation_count,
        (unsigned)state.recovery_debounce,
        cfg.ankle_roll_bias_l_deg,
        cfg.ankle_roll_bias_r_deg,
        cfg.recovery_tilt_pitch_rad,
        cfg.recovery_tilt_roll_rad,
        (unsigned)cfg.recovery_saturation_ticks,
        (unsigned)cfg.recovery_confirm_ticks,
        cfg.gain_boost_threshold_rad,
        cfg.gain_boost_factor,
        cfg.gain_boost_threshold_roll_rad,
        cfg.gain_boost_factor_roll,
        cfg.pitch_to_dx_mm_per_deg,
        cfg.roll_to_dy_mm_per_deg,
        cfg.output_iir_alpha
    );
    _sendFiltered(buf);
}

// =============================================================================
//  TORSO STABILISER STATE BROADCAST
//  Protocol: "TORSO:p_en=..,r_en=..,p_err=..,p_u=..,p_cmd=..,r_err=..,r_u=..,
//             r_cmd=..,<config echoes for slider sync>"
//  Independent of BALANCE:. Same 20 Hz cadence. Config echoes let the Torso
//  panel sliders sync to firmware state instead of pushing HTML defaults.
// =============================================================================
void WebComm::broadcastTorsoState() {
    if (ws.count() == 0 || _torsoStab == nullptr) return;
    const uint32_t INTERVAL_US = 50000;  // 20 Hz
    uint32_t now = micros();
    if ((now - _lastTorsoBroadcast_us) < INTERVAL_US) return;
    _lastTorsoBroadcast_us = now;

    const TorsoConfig& cfg = _torsoStab->getConfig();
    const TorsoState   st  = _torsoStab->getState();
    char buf[420];
    snprintf(buf, sizeof(buf),
        "TORSO:p_en=%d,r_en=%d,"
        // Pitch live outputs
        "p_err=%.4f,p_u=%.3f,p_cmd=%.2f,"
        // Roll live outputs
        "r_err=%.4f,r_u=%.3f,r_cmd=%.2f,"
        // Pitch config
        "Kp=%.3f,Kd=%.3f,sp=%.4f,db=%.4f,dlpf=%.3f,sgn=%.1f,mx=%.1f,"
        // Roll config
        "Kp_r=%.3f,Kd_r=%.3f,sp_r=%.4f,db_r=%.4f,dlpf_r=%.3f,sgn_r=%.1f,mx_r=%.1f,"
        // Output shaping
        "iir=%.3f,rate=%.3f,kv=%.3f,omx=%.1f",
        (int)cfg.pitch_enabled, (int)cfg.roll_enabled,
        st.pitch_error, st.pitch_u_clamped, st.pitch_cmd_deg,
        st.roll_error,  st.roll_u_clamped,  st.roll_cmd_deg,
        cfg.Kp, cfg.Kd, cfg.pitch_setpoint_rad, cfg.pitch_deadband_rad,
        cfg.derivative_lpf_alpha, cfg.correction_sign, cfg.max_correction_deg,
        cfg.Kp_roll, cfg.Kd_roll, cfg.roll_setpoint_rad, cfg.roll_deadband_rad,
        cfg.roll_derivative_lpf_alpha, cfg.roll_correction_sign, cfg.max_roll_correction_deg,
        cfg.output_iir_alpha, cfg.max_output_rate_deg_per_tick,
        cfg.output_damping_kv, cfg.max_output_deg
    );
    _sendFiltered(buf);
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
    char buf[224];
    snprintf(buf, sizeof(buf),
        "WSHIFT_STATE:ankle=%.1f,lateral_mm=%.1f,forward_mm=%.1f,"
        "torso=%.1f,ramp=%.0f,phase_delay=%.0f,setpoint_deg=%.2f,"
        "swingPreload=%.2f",
        cfg.ankle_shift_deg, cfg.lateral_shift_mm, cfg.forward_lean_mm,
        cfg.torso_shift_deg, cfg.ramp_ms, cfg.shift_phase_delay_ms, spDeg,
        cfg.swingPreloadCompleteFrac
    );
    client->text(buf);
}

void WebComm::broadcastGaitState() {
    if (!_gaitCtrl || ws.count() == 0) return;
    const uint32_t INTERVAL_US = 50000;  // 20 Hz
    uint32_t now = micros();
    if ((now - _lastGaitBroadcast_us) < INTERVAL_US) return;
    _lastGaitBroadcast_us = now;

    // -----------------------------------------------------------------------
    // Telemetry freeze after gait stop. When gait leaves any active mode and
    // returns to IDLE, GaitController::start(IDLE) resets its GaitState
    // diagnostics to zero — wiping the diagnostics panel the instant the
    // operator stops. To keep the last runtime values visible until gait runs
    // again, snapshot the live state on every active tick and replay that
    // snapshot while IDLE.
    //
    //   - Config echoes further down are NOT frozen: they read getConfig()
    //     directly, so sliders still sync when the operator tunes at rest.
    //   - mode / running are taken LIVE so the mode badge still reads IDLE
    //     while the (frozen) diagnostic values are held.
    //   - The swing-foot leveler snapshot is frozen alongside it so the same
    //     panel freezes consistently across every mode that stops updates.
    //
    // This is a single cached snapshot (the "last published values"), not a
    // parallel telemetry system. Static is safe: WebComm is a singleton.
    static GaitState             s_gaitFreeze;
    static SwingFootLevelerState s_lvlFreeze;
    static bool                  s_gaitFreezeValid = false;

    const GaitState&            live    = _gaitCtrl->getState();
    // Pull leveler state (all zeros when leveler not wired or disabled).
    const SwingFootLevelerState liveLvl = _swingLeveler
                                          ? _swingLeveler->getState()
                                          : SwingFootLevelerState{};

    const bool gaitActive = (live.mode != GaitMode::IDLE);
    if (gaitActive) {
        s_gaitFreeze      = live;     // refresh the snapshot every active tick
        s_lvlFreeze       = liveLvl;
        s_gaitFreezeValid = true;
    }

    GaitState heldState;
    const GaitState*            statePtr = &live;
    const SwingFootLevelerState* lvlPtr  = &liveLvl;
    if (!gaitActive && s_gaitFreezeValid) {
        heldState         = s_gaitFreeze;
        heldState.mode    = live.mode;      // identity stays live → badge reads IDLE
        heldState.running = live.running;
        statePtr          = &heldState;
        lvlPtr            = &s_lvlFreeze;
    }
    const GaitState&            s   = *statePtr;
    const SwingFootLevelerState  lvl = *lvlPtr;

    char buf[2688];
    snprintf(buf, sizeof(buf),
        "GAIT:phase=%.3f,mode=%d,sub=%d,liftL=%.1f,liftR=%.1f,ws=%.3f,gate=%d,hold=%d,"
        "stepH=%.1f,stanceH=%.1f,stepLen=%.1f,stanceInset=%.1f,"
        // Gait config fields the UI sliders need to sync (firmware is source
        // of truth). Single-key sends in the UI rely on these never being
        // stale, so they live in the continuous broadcast.
        "phaseRate=%.3f,liftGate=%.3f,wsThresh=%.4f,capMax=%.1f,"
        // Swing-foot leveler kP fields. enabled is below as lvl_en.
        "lvl_kpP=%.1f,lvl_kpR=%.1f,"
        "gateMode=%d,gImu=%d,gFlat=%d,gLoad=%d,gStanceL=%d,"
        "comLatMm=%.1f,gateComLat=%d,trNeutP=%.5f,trNeutR=%.5f,comLatGate=%.1f,"
        "rsub=%d,rdx=%d,rdy=%d,rms=%lu,"
        "rcvX=%.1f,rcvY=%.1f,rcvMag=%.1f,rcvSY=%.1f,xDispl=%.1f,"
        "ikL=%d,ikR=%d,reachL=%.1f,reachR=%.1f,ffL=%.3f,ffR=%.3f,capOffset=%.1f,capEn=%d,capGain=%.2f,"
        "wsElapsed=%lu,gateDwell=%lu,wsTo=%d,dwellMs=%lu,waitMaxMs=%lu,"
        "lvl_act=%d,lvl_en=%d,lvl_v=%d,lvl_pbias=%.3f,lvl_rbias=%.3f,"
        "lvl_pfoot=%.4f,lvl_rfoot=%.4f,lvl_env=%.3f,"
        // Touchdown-awareness (Stage 0/1).
        //   tdEn   - master enable from GaitConfig
        //   tdGate - phase gate floor [0..1)
        //   tdSeen - sticky: detection seen during current swing
        //   tdLkd  - latch active this tick
        //   tdPh   - _halfPhase at first gated detection
        //   tdSide - 0=LEFT 1=RIGHT (matches Side enum)
        //   tdX    - swing-leg stride x snapshot (mm) at detection
        //   tdFootRoll - swing foot IMU roll (rad) at detection
        //   tdTorsoRoll - neutral-corrected torso roll (rad) at detection
        "tdEn=%d,tdGate=%.2f,tdSeen=%d,tdLkd=%d,tdPh=%.3f,tdSide=%d,tdX=%.1f,"
        "tdFootRoll=%.4f,tdTorsoRoll=%.4f,"
        // Touchdown trim suppression (W2/W5 phase-gate) — config echoes plus
        // the live suppression-active flag.
        //   tdTrimSup    - master enable (config)
        //   tdTrimHold   - post-latch hold window, ms (config)
        //   tdTrimFrz    - mode: 1=freeze last clean trim, 0=force zero (config)
        //   tdTrimSupAct - suppression active this tick (telemetry)
        "tdTrimSup=%d,tdTrimHold=%lu,tdTrimFrz=%d,tdTrimSupAct=%d,"
        // W1 swing-envelope apex fraction (config echo for slider sync).
        // W2 lateral CoM trim — config echoes (enable / gain / cap) plus the
        // live applied trim (comLatTrimMm) for the source-of-truth UI sync.
        "swingApex=%.3f,comLatTrim=%d,comLatGain=%.2f,comLatMax=%.1f,comLatTrimMm=%.1f,"
        // W5 lateral-trim velocity damping — config echoes (Kd, roll-rate LPF
        // alpha) plus live telemetry (filtered roll rate rad/s, velocity
        // contribution mm pre-clamp).
        "comLatKd=%.3f,comLatRateLpf=%.2f,comLatRollRate=%.4f,comLatTrimVel=%.1f,"
        // W4 touchdown compliance — config echoes (enable / depth / decay)
        // plus live telemetry (cmpOff = signed offset applied this tick,
        // cmpAct = decay in progress).
        "cmpEn=%d,cmpDepth=%.1f,cmpDecay=%lu,cmpOff=%.2f,cmpAct=%d,"
        // Session neutral trim — ntActive (any channel trimmed) + ntMax
        // (largest |trim|, deg). Lets the UI hold a persistent status line.
        "ntActive=%d,ntMax=%.2f,"
        // Per-leg final FootTarget components (post-WS, post-W2 trim,
        // post-W4 compliance, post BalanceCorrection merge). Diagnostic
        // snapshot of what LegIK received this tick.
        "fxL=%.1f,fxR=%.1f,fyL=%.1f,fyR=%.1f,fhL=%.1f,fhR=%.1f,"
        // Lateral (frontal-plane) capture-point swing-foot placement —
        // additive A/B candidate parallel to comLatTrim + roll-PD. Config
        // echoes (enable / gain / cap / sign) + live applied value (world-
        // signed mm, post-clamp, post-strideScale, post-freeze; 0 on
        // stance / disabled / IDLE / estopped / IMU invalid).
        "latCapEn=%d,latCapGain=%.2f,latCapMax=%.1f,latCapSign=%.0f,latCap=%.1f,"
        // Horizontal stride completion fraction (config echo) + per-swing
        // horizontal swing-foot velocity at the moment the touchdown latch
        // fires (mm/s). At strideComplete=1.0 (default) the velocity reports
        // the unwarped baseline; with strideComplete lowered, drops toward 0.
        "strideComplete=%.2f,tdSwingVx=%.1f,"
        // Forward CoM progression — swing-phase-timed body offset. Config
        // echoes (max magnitude mm, lead fraction) + live applied magnitude
        // (mm, post-shape, post-strideScale; 0 on default-OFF / IDLE /
        // estopped). Read against the A/B failure-mode split when sweeping.
        "fwdCom=%.1f,fwdComLead=%.2f,fwdComMm=%.2f,"
        // Heel-strike rollover — config echoes (enable / peak deg / gate phase /
        // release ms) + live applied bias and release-active flag. Sign is
        // intrinsic to the helper (always dorsiflex/negative), not transported.
        "heelStrike=%d,heelDeg=%.1f,heelGate=%.2f,heelRelMs=%lu,heelBias=%.2f,heelRelAct=%d,heelSide=%d,"
        // Swing-foot ankle-roll preparation (consolidation refactor) — config
        // echoes (enable / gain / front-load frac) + live applied swing-prep
        // bias, the migrated CoM-shift bias on the grounded leg, and the swing
        // side index (0=LEFT,1=RIGHT).
        "swingPrep=%d,swingPrepGain=%.2f,swingPrepFrac=%.2f,swingPrepBias=%.2f,comShiftRoll=%.2f,swingPrepSide=%d,"
        "stabilizeMs=%lu,skipNone=%d,"
        // Per-step timing (updated once per footfall, zero until first complete step).
        "stepMs=%lu,waitMs=%lu,swingMs=%lu,stabMsMeas=%lu,postCtMs=%lu,"
        "lastSide=%d,cadHz=%.2f,cadSpm=%.1f,"
        "meanStepMs=%.0f,meanCadHz=%.2f,meanStepL=%.0f,meanStepR=%.0f,"
        "meanSwingL=%.0f,meanSwingR=%.0f,"
        // H-restore — deferred swing-leg stance-height restoration. Config
        // echoes (enable / completion fraction) + live telemetry (hRestoreOff
        // = signed offset applied to the restoring leg this tick (<=0),
        // hRestoreAct = restore in progress).
        "hRestore=%d,hRestoreFrac=%.2f,hRestoreOff=%.2f,hRestoreAct=%d,"
        // Pitch-ANGLE swing-foot forward placement (sagittal, ANGLE-based) —
        // config echoes (enable / gain mm-per-rad / cap mm / deadband rad / sign)
        // plus the live applied value (pPlMm, post-clamp, post-ramp, post-
        // strideScale; 0 on disabled / gain=0 / IDLE / estopped / deadband).
        // ffGuard = cumulative flat-foot-guard activations this gait session.
        "pPlEn=%d,pPlGain=%.1f,pPlMax=%.1f,pPlDb=%.3f,pPlSign=%.0f,pPlMm=%.2f,ffGuard=%lu,"
        // Rollover instrumentation (TELEMETRY ONLY) — per just-landed foot,
        // anchored at the touchdown latch.
        //   rollSide    - 0=LEFT,1=RIGHT (the foot that landed)
        //   rollAct     - inside the post-contact window this tick
        //   rollFP      - live neutral-corrected foot pitch (rad)
        //   rollDiv     - commanded ankle pitch − measured foot pitch (deg, proxy)
        //   rollDur     - latch→first-flat (ms); 0 if flat not reached in-window
        //   rollCoMSign - sign of post-contact sagittal CoM velocity (-1/0/+1)
        //   rollPitchPk - peak (signed) torso pitch in-window (rad)
        "rollSide=%d,rollAct=%d,rollFP=%.4f,rollDiv=%.2f,rollDur=%lu,"
        "rollCoMSign=%d,rollPitchPk=%.4f",
        s.phase,
        static_cast<int>(s.mode),
        static_cast<int>(s.subState),
        s.liftL_deg,
        s.liftR_deg,
        s.wsProgress,
        static_cast<int>(s.liftGateOK),
        static_cast<int>(s.poseHold),
        _gaitCtrl->getConfig().stepHeightMm,
        _gaitCtrl->getConfig().stanceHeightMm,
        _gaitCtrl->getConfig().stepLengthMm,
        _gaitCtrl->getConfig().stanceInsetMm,
        _gaitCtrl->getConfig().phaseRateHz,
        _gaitCtrl->getConfig().liftGateThreshold,
        _gaitCtrl->getConfig().wsThresholdRad,
        _gaitCtrl->getConfig().captureOffsetMaxMm,
        _swingLeveler ? _swingLeveler->getConfig().kp_pitch_deg_per_rad : 0.0f,
        _swingLeveler ? _swingLeveler->getConfig().kp_roll_deg_per_rad  : 0.0f,
        static_cast<int>(_gaitCtrl->getConfig().liftGateMode),
        static_cast<int>(s.gateImuRollOK),
        static_cast<int>(s.gateStanceFlat),
        static_cast<int>(s.gateStanceLoaded),
        static_cast<int>(s.gateStanceIsLeft),
        s.comLateralMm,
        (int)s.gateComLateralOK,
        _gaitCtrl->getConfig().torsoPitchNeutralRad,
        _gaitCtrl->getConfig().torsoRollNeutralRad,
        _gaitCtrl->getConfig().comLateralGateMm,
        static_cast<int>(s.recoverySub),
        static_cast<int>(s.recoveryDirX),
        static_cast<int>(s.recoveryDirY),
        static_cast<unsigned long>(s.recoveryElapsedMs),
        s.rcvStepX_mm, s.rcvStepY_mm, s.rcvMagnitude, s.rcvStanceY_mm,
        s.cycleXDispl_mm,
        static_cast<int>(s.lastIKStatusL),
        static_cast<int>(s.lastIKStatusR),
        s.lastReachPctL, s.lastReachPctR,
        s.lastFlatFootErrL_deg, s.lastFlatFootErrR_deg,
        s.captureOffsetMm,
        static_cast<int>(_gaitCtrl->getConfig().captureEnabled),
        _gaitCtrl->getConfig().captureGain,
        (unsigned long)s.waitShiftElapsedMs,
        (unsigned long)s.gateDwellMs,
        (int)s.waitShiftTimedOut,
        (unsigned long)_gaitCtrl->getConfig().liftDwellMs,
        (unsigned long)_gaitCtrl->getConfig().waitShiftMaxMs,
        // Swing-foot leveler fields (L0+):
        //   lvl_act   - controller armed (enabled + IMU valid)
        //   lvl_en    - enabled config flag (allows UI to sync badge on reconnect)
        //   lvl_v     - foot IMU valid (replaces the imu_valid UI heuristic)
        //   lvl_pbias - would-be pitch bias (PRE-envelope, PRE-IIR);
        //               non-zero whenever foot tilted, even at rest
        //   lvl_rbias - would-be roll bias
        //   lvl_pfoot - filtered swing-foot pitch (rad)
        //   lvl_rfoot - filtered swing-foot roll (rad)
        //   lvl_env   - envelope scale [0..1]; non-zero only mid-swing
        static_cast<int>(lvl.armed),
        _swingLeveler ? static_cast<int>(_swingLeveler->getConfig().enabled) : 0,
        static_cast<int>(lvl.foot_imu_valid),
        lvl.ankle_pitch_bias_deg,
        lvl.ankle_roll_bias_deg,
        lvl.foot_pitch_rad,
        lvl.foot_roll_rad,
        lvl.envelope_scale,
        // Touchdown-awareness — config + per-swing telemetry.
        static_cast<int>(_gaitCtrl->getConfig().touchdownEnabled),
        _gaitCtrl->getConfig().touchdownGatePhase,
        static_cast<int>(s.touchdownSeen),
        static_cast<int>(s.touchdownLocked),
        s.touchdownPhase,
        static_cast<int>(s.touchdownSide),
        s.touchdownSwingX_mm,
        s.touchdownSwingFootRollRad,
        s.touchdownTorsoRollRad,
        // Touchdown trim suppression — config + live suppression flag.
        static_cast<int>(_gaitCtrl->getConfig().touchdownTrimSuppressEnabled),
        (unsigned long)_gaitCtrl->getConfig().touchdownTrimHoldMs,
        static_cast<int>(_gaitCtrl->getConfig().touchdownTrimFreezeHold),
        static_cast<int>(s.trimSuppressActive),
        // W1 / W2 config echoes + W2 live trim.
        _gaitCtrl->getConfig().swingApexFrac,
        static_cast<int>(_gaitCtrl->getConfig().comLatTrimEnabled),
        _gaitCtrl->getConfig().comLatTrimGain,
        _gaitCtrl->getConfig().comLatTrimMaxMm,
        s.comLatTrimMm,
        // W5 — velocity-damping config echoes + live telemetry.
        _gaitCtrl->getConfig().comLatTrimKd,
        _gaitCtrl->getConfig().comLatTrimRateLpfAlpha,
        s.comLatRollRateFilt,
        s.comLatTrimVelMm,
        // W4 touchdown compliance — config + live telemetry.
        static_cast<int>(_gaitCtrl->getConfig().complianceEnabled),
        _gaitCtrl->getConfig().complianceDepthMm,
        (unsigned long)_gaitCtrl->getConfig().complianceDecayMs,
        s.complianceOffsetMm,
        static_cast<int>(s.complianceActive),
        // Runtime session neutral trim — owned by JointModel, surfaced via
        // ServoControl. Rides the GAIT packet purely as transport (the UI
        // panel lives in the gait tab); it is not a gait-owned value.
        static_cast<int>(_servoCtrl && _servoCtrl->hasSessionTrim()),
        _servoCtrl ? _servoCtrl->maxSessionTrimDeg() : 0.0f,
        // Per-leg final FootTarget components (mm).
        s.lastFootTargetXL_mm, s.lastFootTargetXR_mm,
        s.lastFootTargetYL_mm, s.lastFootTargetYR_mm,
        s.lastFootTargetHL_mm, s.lastFootTargetHR_mm,
        // Lateral capture-point — config echoes + live applied value.
        static_cast<int>(_gaitCtrl->getConfig().latCaptureEnabled),
        _gaitCtrl->getConfig().latCaptureGain,
        _gaitCtrl->getConfig().latCaptureMaxMm,
        _gaitCtrl->getConfig().latCaptureSign,
        s.latCaptureMm,
        // Horizontal stride completion fraction + per-swing touchdown velocity.
        _gaitCtrl->getConfig().strideCompleteFrac,
        s.touchdownSwingVelX_mm_s,
        // Forward CoM progression — config echoes + live applied magnitude.
        _gaitCtrl->getConfig().fwdComMaxMm,
        _gaitCtrl->getConfig().fwdComLeadFrac,
        s.fwdComProgressionMm,
        // Heel-strike rollover — config echoes + live applied bias +
        // release-active flag + active-side index (0=LEFT, 1=RIGHT).
        static_cast<int>(_gaitCtrl->getConfig().heelStrikeEnabled),
        _gaitCtrl->getConfig().heelStrikeDorsiflexDeg,
        _gaitCtrl->getConfig().heelStrikeGatePhase,
        (unsigned long)_gaitCtrl->getConfig().heelStrikeReleaseMs,
        s.heelStrikeBiasDeg,
        static_cast<int>(s.heelStrikeReleaseActive),
        static_cast<int>(s.heelStrikeActiveSide),
        // Swing-foot ankle-roll preparation — config echoes + live telemetry.
        static_cast<int>(_gaitCtrl->getConfig().swingPrepEnabled),
        _gaitCtrl->getConfig().swingPrepGain,
        _gaitCtrl->getConfig().swingPrepCompleteFrac,
        s.swingPrepRollBiasDeg,
        s.comShiftRollBiasDeg,
        static_cast<int>(s.swingPrepSide),
        (unsigned long)_gaitCtrl->getConfig().stabilizeMs,
        (int)_gaitCtrl->getConfig().skipStabilizeNone,
        // Per-step timing fields.
        (unsigned long)s.stepPeriodMs,
        (unsigned long)s.waitMs,
        (unsigned long)s.swingMs,
        (unsigned long)s.stabilizeMsMeas,
        (unsigned long)s.postContactMs,
        (int)s.lastStepSide,
        s.cadenceHz,
        s.cadenceStepsMin,
        s.meanStepPeriodMs,
        s.meanCadenceHz,
        s.meanStepPeriodL,
        s.meanStepPeriodR,
        s.meanSwingMsL,
        s.meanSwingMsR,
        // H-restore — config echoes + live telemetry.
        static_cast<int>(_gaitCtrl->getConfig().hRestoreEnabled),
        _gaitCtrl->getConfig().hRestoreCompleteFrac,
        s.hRestoreOffsetMm,
        static_cast<int>(s.hRestoreActive),
        // Pitch-ANGLE swing-foot forward placement — config echoes + live value
        // + flat-foot-guard activation count.
        static_cast<int>(_gaitCtrl->getConfig().pitchPlacementEnabled),
        _gaitCtrl->getConfig().pitchPlacementGainMmPerRad,
        _gaitCtrl->getConfig().pitchPlacementMaxMm,
        _gaitCtrl->getConfig().pitchPlacementDeadbandRad,
        _gaitCtrl->getConfig().pitchPlacementSign,
        s.pitchPlacementMm,
        (unsigned long)s.flatFootGuardCount,
        // Rollover instrumentation.
        (int)s.rolloverSide,
        (int)s.rolloverActive,
        s.rolloverFootPitchRad,
        s.ankleCmdVsFootPitchDivergenceDeg,
        (unsigned long)s.rolloverDurationMs,
        (int)s.postContactCoMVelSign,
        s.postContactBodyPitchPeak
    );
    _sendFiltered(buf);
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
//  TRANSMISSION STATE BROADCAST
//
//  Emits one line per joint whose transmission gain ≠ 1.0 (i.e. the joints
//  where the correction is doing real work — by default just ankle pitch L/R).
//  Each field shows what the foot-IMU acceptance sweep needs to see:
//    gain     : current runtime gain (post-TRANSMISSION_TUNE)
//    anat     : commanded anatomical deviation, ° from neutral
//    servo    : corrected servo-side deviation, ° from neutral
//    clamp    : 1 if the corrected servo angle saturated the 0–270° envelope
//
//  Format: TRANSMISSION:<ch>=g:G,a:A,s:S,c:C; <ch>=...;
//
//  Sent on tune commands (parser above) and on client connect so the UI can
//  build the panel rows lazily — joints not in the message are direct-drive
//  and need no display.
// =============================================================================
void WebComm::broadcastTransmissionState() {
    if (!_servoCtrl || ws.count() == 0) return;
    String msg = "TRANSMISSION:";
    bool any = false;
    for (uint8_t ch = 0; ch < NUM_JOINTS; ch++) {
        const float gain = _servoCtrl->getTransmissionGain(ch);
        // Only joints actually being corrected appear in the broadcast.
        // Skip with float tolerance so 1.0f exactly is treated as "no-op".
        if (fabsf(gain - 1.0f) < 1e-4f) continue;

        const float anat  = _servoCtrl->getJointAngle(ch);
        const float servo = _servoCtrl->correctedServoDeviationDeg(ch, anat);
        const int   clamp = _servoCtrl->getTransmissionClampHit(ch) ? 1 : 0;

        if (any) msg += ";";
        msg += String(ch);
        msg += "=g:" + String(gain, 3);
        msg +=  ",a:" + String(anat,  2);
        msg +=  ",s:" + String(servo, 2);
        msg +=  ",c:" + String(clamp);
        any = true;
    }
    if (!any) msg += "none";
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
            broadcastTransmissionState();  // sync transmission-gain panel on reconnect
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

        // Push current BALANCE state immediately so the UI's tuning sliders sync
        // to firmware on first paint instead of flashing the HTML defaults until
        // the next 50 ms broadcast tick. Resetting the rate-limit timestamp
        // disarms the throttle for this call only — the next tick re-arms it.
        if (_balCtrl) {
            _lastBalanceBroadcast_us = 0;
            broadcastBalanceState(_balCtrl->getLastState());
        }
        if (_footImpact) {
            _lastFootImpactBroadcast_us = 0;
            broadcastFootImpact();
        }
        // Force-arm the next tick of the foot-contact and gait broadcasts so
        // their config fields (flat thresholds, gait params, leveler kP) reach
        // a freshly connected client without waiting for the natural rate-
        // limit interval. Firmware is the source of truth for these knobs;
        // the UI sliders rely on these broadcasts to sync away from the
        // (deliberately fallback-only) HTML defaults.
        _lastFootContactBroadcast_us = 0;
        _lastGaitBroadcast_us        = 0;
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
    // System-level reset: stop all controllers that own leg joints
    // before submitting neutral. Without this, GaitController's
    // per-tick stance submission (priority 1) clobbers SOURCE_UI (0).

    if (_gaitCtrl) _gaitCtrl->stop();          // → IDLE; centers WeightShift
    if (_weightShift) _weightShift->forceCenterImmediate();

    if (_balCtrl) {
        BalanceConfig cfg = _balCtrl->getConfig();
        cfg.pitch_enabled = false;
        cfg.roll_enabled  = false;
        _balCtrl->setConfig(cfg);
        _balCtrl->resetOutputState();
    }

    for (int i = 0; i < NUM_JOINTS; i++) {
        bool skip = false;
        for (uint8_t k = 0; k < SKIP_BOOT_NEUTRAL_COUNT; k++) {
            if (static_cast<uint8_t>(i) == SKIP_BOOT_NEUTRAL_CHANNELS[k]) {
                skip = true; break;
            }
        }
        if (!skip) {
            if (_motionManager) {
                // SOURCE_UI submits absolute servo degrees. Use the EFFECTIVE
                // neutral (factory + runtime session trim) so RESETP returns to
                // the same corrected neutral every other path references — not
                // the raw factory pose. With no trim captured these are equal.
                _motionManager->submit(SOURCE_UI,
                                       static_cast<uint8_t>(i),
                                       _servoCtrl->getEffectiveNeutralDeg(i));
            } else {
                _servoCtrl->setGaitOffset(i, 0.0f);
            }
        }
    }
    // Smooth the move: the SOURCE_UI pose submitted above is applied on the next
    // MotionManager::flush(), which slews the flagged joints at the dedicated
    // pose-transition speed (and disarms the latch after that batch).
    if (_servoCtrl) _servoCtrl->armTransition();
    broadcastState();
    Serial.println("[WebComm] RESETP — gait stopped, balance off, neutral submitted.");
 }
        // ---------------------------------------------------------------------
        //  STANCE — enter the idle/standing-ready stance pose without starting
        //  gait. Stops gait only (balance is left as-is, per design). When gait
        //  is already IDLE, submits the stance pose as a one-shot and smooths it
        //  via the pose-transition speed. When gait is RUNNING, stop() ramps down
        //  and already converges to this same idle stance, so no one-shot needed.
        // ---------------------------------------------------------------------
        else if (cmd == "STANCE") {
            if (_gaitCtrl) {
                const bool wasRunning = _gaitCtrl->getState().running;
                _gaitCtrl->stop();   // IDLE: no-op (returns early); RUNNING: ramp → idle stance
                if (!wasRunning) {
                    _gaitCtrl->submitStancePose();             // one-shot idle stance
                    if (_servoCtrl) _servoCtrl->armTransition();  // smooth the move
                }
            }
            broadcastState();
            Serial.println("[WebComm] STANCE — idle stance submitted (balance untouched).");
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
        //  Torso stabiliser enable / disable — independent of leg balance.
        // ---------------------------------------------------------------------
        else if (cmd == "TORSO_PITCH_ON") {
            if (_torsoStab) {
                TorsoConfig cfg = _torsoStab->getConfig();
                cfg.pitch_enabled = true;
                _torsoStab->setConfig(cfg);
            }
            Serial.println("[WebComm] CMD: TORSO_PITCH_ON");
        }
        else if (cmd == "TORSO_PITCH_OFF") {
            if (_torsoStab) {
                TorsoConfig cfg = _torsoStab->getConfig();
                cfg.pitch_enabled = false;
                _torsoStab->setConfig(cfg);
            }
            Serial.println("[WebComm] CMD: TORSO_PITCH_OFF");
        }
        else if (cmd == "TORSO_ROLL_ON") {
            if (_torsoStab) {
                TorsoConfig cfg = _torsoStab->getConfig();
                cfg.roll_enabled = true;
                _torsoStab->setConfig(cfg);
            }
            Serial.println("[WebComm] CMD: TORSO_ROLL_ON");
        }
        else if (cmd == "TORSO_ROLL_OFF") {
            if (_torsoStab) {
                TorsoConfig cfg = _torsoStab->getConfig();
                cfg.roll_enabled = false;
                _torsoStab->setConfig(cfg);
            }
            Serial.println("[WebComm] CMD: TORSO_ROLL_OFF");
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
                        else if (key == "swingPreload") {
            // Swing-bias preload completion fraction. 1.0 = no front-load
            // (bit-identical to the unmodified (1-liftFrac) mask). Lower toward
            // ~0.5 to reach full swing bias earlier in the descent. Clamped to
            // (0.05, 1.0].
            cfg.swingPreloadCompleteFrac = constrain(val, 0.05f, 1.0f);
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
        //  Foot-contact tuning (Phase F2)
        //  Protocol: CMD:FOOT_CONTACT_TUNE:flatP=0.035,flatR=0.035,
        //                                  loadDev=0.025,loadSign=+1.0
        // ---------------------------------------------------------------------
        else if (cmd.startsWith("FOOT_CONTACT_TUNE:")) {
            if (_footContactCfg) {
                FootContactConfig& cfg = *_footContactCfg;
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
                        if (key == "flatP") {
                            cfg.flat_pitch_threshold_rad = constrain(val, 0.005f, 0.20f);
                        }
                        else if (key == "flatR") {
                            cfg.flat_roll_threshold_rad = constrain(val, 0.005f, 0.20f);
                        }
                        else if (key == "loadDev") {
                            cfg.loaded_pitch_deviation_rad = constrain(val, 0.005f, 0.10f);
                        }
                        else if (key == "loadSign") {
                            if (fabsf(fabsf(val) - 1.0f) < 0.001f) {
                                cfg.loaded_pitch_sign = (val >= 0.0f) ? 1.0f : -1.0f;
                            } else {
                                Serial.printf("[WebComm] FOOT_CONTACT_TUNE: rejecting "
                                              "loadSign=%.3f (must be ±1.0)\n", val);
                            }
                        }
                    }
                    start = (comma == -1) ? params.length() : comma + 1;
                }
                _lastFootContactBroadcast_us = 0;  // allow immediate round-trip on next tick
                Serial.printf("[WebComm] FOOT_CONTACT_TUNE: flatP=%.4f flatR=%.4f "
                              "loadDev=%.4f loadSign=%.1f\n",
                              cfg.flat_pitch_threshold_rad,
                              cfg.flat_roll_threshold_rad,
                              cfg.loaded_pitch_deviation_rad,
                              cfg.loaded_pitch_sign);
            }
        }
        // ---------------------------------------------------------------------
        //  Foot-impact tuning (L0)
        //  Protocol: CMD:FOOT_IMPACT_TUNE:enabled=1,thr=1.5,lpf=0.5,ref=200
        // ---------------------------------------------------------------------
        else if (cmd.startsWith("FOOT_IMPACT_TUNE:")) {
            if (_footImpact) {
                FootImpactConfig cfg = _footImpact->getConfig();
                String params = cmd.substring(17);
                int start = 0;
                while (start < (int)params.length()) {
                    int comma = params.indexOf(',', start);
                    String pair = (comma == -1) ? params.substring(start)
                                                : params.substring(start, comma);
                    int eq = pair.indexOf('=');
                    if (eq != -1) {
                        String key = pair.substring(0, eq);
                        String val = pair.substring(eq + 1);
                        if      (key == "enabled") cfg.enabled       = (val.toInt() != 0);
                        else if (key == "thr")     cfg.threshold_g   = val.toFloat();
                        else if (key == "lpf")     cfg.lpf_alpha     = val.toFloat();
                        else if (key == "ref")     cfg.refractory_ms = (uint32_t)val.toInt();
                    }
                    start = (comma == -1) ? params.length() : comma + 1;
                }
                _footImpact->setConfig(cfg);
                _lastFootImpactBroadcast_us = 0;
                broadcastFootImpact();
                Serial.printf("[WebComm] FOOT_IMPACT_TUNE: en=%d thr=%.3f lpf=%.3f ref=%lu\n",
                              (int)_footImpact->getConfig().enabled,
                              _footImpact->getConfig().threshold_g,
                              _footImpact->getConfig().lpf_alpha,
                              (unsigned long)_footImpact->getConfig().refractory_ms);
            }
        }
        // ─────────────────────────────────────────────────────────────────
        //  WALKING INSTRUMENTATION (per-step diagnostics, telemetry only)
        //  Protocol: CMD:WALKING_INSTR:reset=1  |  CMD:WALKING_INSTR:enable=0|1
        // ─────────────────────────────────────────────────────────────────
        else if (cmd.startsWith("WALKING_INSTR:")) {
            if (_walkInstr) {
                String params = cmd.substring(14);  // after "WALKING_INSTR:"
                int start = 0;
                while (start < (int)params.length()) {
                    int comma = params.indexOf(',', start);
                    if (comma == -1) comma = params.length();
                    String pair = params.substring(start, comma);
                    int eq = pair.indexOf('=');
                    if (eq > 0) {
                        String key = pair.substring(0, eq);
                        float val = pair.substring(eq + 1).toFloat();
                        if      (key == "reset" && val != 0.0f) _walkInstr->reset();
                        else if (key == "enable")               _walkInstr->setEnabled(val != 0.0f);
                    }
                    start = comma + 1;
                }
                _lastWalkInstrStep = 0xFFFFFFFFUL;  // force an immediate refresh
                broadcastWalkingInstr();
                Serial.printf("[WebComm] WALKING_INSTR: enabled=%d steps=%lu\n",
                              (int)_walkInstr->getState().enabled,
                              (unsigned long)_walkInstr->getState().completedSteps);
            }
            return;
        }
        
        // ---------------------------------------------------------------------
        //  Swing-foot leveler tuning (L0+)
        //  Protocol: CMD:SWING_LEVELER_TUNE:enabled=0,kp_pitch=25,kp_roll=25,
        //                                   max_p=8,max_r=6,dead=0.0175,
        //                                   in_lpf=0.6,out_iir=0.5,
        //                                   eng=0.20,peak=0.55,dis=0.80
        //  All fields optional; unrecognised keys are silently skipped.
        //  Operator must keep eng < peak < dis; firmware does not cross-check.
        // ---------------------------------------------------------------------
        else if (cmd.startsWith("SWING_LEVELER_TUNE:")) {
            if (_swingLeveler) {
                SwingFootLevelerConfig cfg = _swingLeveler->getConfig();
                String params = cmd.substring(19);
                int start = 0;
                while (start < (int)params.length()) {
                    int comma = params.indexOf(',', start);
                    String pair = (comma == -1) ? params.substring(start)
                                                : params.substring(start, comma);
                    int eq = pair.indexOf('=');
                    if (eq != -1) {
                        String key = pair.substring(0, eq);
                        float  val = pair.substring(eq + 1).toFloat();
                        if      (key == "enabled")  cfg.enabled             = (val > 0.5f);
                        else if (key == "kp_pitch") cfg.kp_pitch_deg_per_rad = constrain(val, 0.0f, 120.0f);
                        else if (key == "kp_roll")  cfg.kp_roll_deg_per_rad  = constrain(val, 0.0f, 120.0f);
                        else if (key == "max_p")    cfg.max_pitch_bias_deg   = constrain(val, 0.0f, 15.0f);
                        else if (key == "max_r")    cfg.max_roll_bias_deg    = constrain(val, 0.0f, 15.0f);
                        else if (key == "dead")     cfg.deadband_rad         = constrain(val, 0.0f, 0.20f);
                        else if (key == "in_lpf")   cfg.input_lpf_alpha      = constrain(val, 0.0f, 0.99f);
                        else if (key == "out_iir")  cfg.output_iir_alpha     = constrain(val, 0.0f, 0.99f);
                        else if (key == "eng")      cfg.engage_lift_frac     = constrain(val, 0.0f, 1.0f);
                        else if (key == "peak")     cfg.peak_lift_frac       = constrain(val, 0.0f, 1.0f);
                        else if (key == "dis")      cfg.disengage_lift_frac  = constrain(val, 0.0f, 1.0f);
                    }
                    start = (comma == -1) ? params.length() : comma + 1;
                }
                // Cross-validate envelope ordering: eng < peak < dis.
                // Out-of-order would make _envelope() return 0 for all
                // liftFrac, silently disabling actuation. Reject and log.
                if (cfg.engage_lift_frac >= cfg.peak_lift_frac ||
                    cfg.peak_lift_frac   >= cfg.disengage_lift_frac) {
                    Serial.printf("[WebComm] SWING_LEVELER_TUNE: invalid envelope "
                                  "order eng=%.2f peak=%.2f dis=%.2f - "
                                  "reverting to defaults.\n",
                                  cfg.engage_lift_frac, cfg.peak_lift_frac,
                                  cfg.disengage_lift_frac);
                    cfg.engage_lift_frac    = SWING_LEVELER_ENGAGE_LIFT_FRAC;
                    cfg.peak_lift_frac      = SWING_LEVELER_PEAK_LIFT_FRAC;
                    cfg.disengage_lift_frac = SWING_LEVELER_DISENGAGE_LIFT_FRAC;
                }
                _swingLeveler->setConfig(cfg);
                Serial.printf("[WebComm] SWING_LEVELER_TUNE: en=%d kpP=%.1f kpR=%.1f "
                              "maxP=%.1f maxR=%.1f dead=%.4f in_lpf=%.2f out_iir=%.2f "
                              "eng=%.2f peak=%.2f dis=%.2f\n",
                              (int)cfg.enabled,
                              cfg.kp_pitch_deg_per_rad, cfg.kp_roll_deg_per_rad,
                              cfg.max_pitch_bias_deg,   cfg.max_roll_bias_deg,
                              cfg.deadband_rad,
                              cfg.input_lpf_alpha,      cfg.output_iir_alpha,
                              cfg.engage_lift_frac,     cfg.peak_lift_frac,
                              cfg.disengage_lift_frac);
            }
        }
        // ---------------------------------------------------------------------
        //  Balance tuning — parses key=value pairs, normalises ratios
        // ---------------------------------------------------------------------
        // ---------------------------------------------------------------------
        //  Foot IMU diagnostic probe on demand
        // ---------------------------------------------------------------------
        else if (cmd == "FOOT_IMU_PROBE") {
            if (!_leftFootIMU || !_rightFootIMU) {
                ws.textAll("FOOT_PROBE:error=drivers not wired");
                return;
            }
            auto L = _leftFootIMU->probe();
            auto R = _rightFootIMU->probe();
            char buf[160];
            snprintf(buf, sizeof(buf),
                "FOOT_PROBE:L_who=0x%02X,L_c1=0x%02X,L_c2=0x%02X,L_c3=0x%02X,L_init=%d,"
                "R_who=0x%02X,R_c1=0x%02X,R_c2=0x%02X,R_c3=0x%02X,R_init=%d",
                L.whoami, L.ctrl1, L.ctrl2, L.ctrl3, (int)L.initialized,
                R.whoami, R.ctrl1, R.ctrl2, R.ctrl3, (int)R.initialized);
            _sendFiltered(buf);
            Serial.printf("[WebComm] %s\n", buf);
        }
        else if (cmd == "TORSO_TARE") {
            if (_gaitCtrl && _stateEst) {
                const bool ok = _gaitCtrl->tareToTorso(_stateEst->getState());
                const GaitConfig& cfg = _gaitCtrl->getConfig();
                char buf[120];
                snprintf(buf, sizeof(buf),
                    "TORSO_TARE:ok=%d,pitch=%.4f,roll=%.4f",
                    (int)ok, cfg.torsoPitchNeutralRad, cfg.torsoRollNeutralRad);
                _sendFiltered(buf);
                Serial.printf("[WebComm] %s\n", buf);
            } else {
                ws.textAll("TORSO_TARE:ok=0,err=not_wired");
            }
        }
        // ─────────────────────────────────────────────────────────────────────
        //  CAPTURE_NEUTRAL — snapshot the current commanded pose as the runtime
        //  session neutral (servo-drift compensation). The trim is owned and
        //  applied globally by JointModel, so every command source inherits it.
        //  CLEAR_NEUTRAL reverts to factory neutral. Both reply with a
        //  NEUTRAL_TRIM: message the UI uses for accept/reject feedback.
        //
        //  Gated here because WebComm is the layer that knows controller state:
        //  capture is meaningful only when the joints hold the operator's manual
        //  pose — i.e. gait IDLE and balance disarmed — not a controller's output.
        // ─────────────────────────────────────────────────────────────────────
        else if (cmd == "CAPTURE_NEUTRAL") {
            if (_servoCtrl) {
                const bool gaitIdle = !_gaitCtrl
                    || _gaitCtrl->getState().mode == GaitMode::IDLE;
                bool balanceArmed = false;
                if (_balCtrl) {
                    const BalanceConfig bc = _balCtrl->getConfig();
                    balanceArmed = bc.pitch_enabled || bc.roll_enabled;
                }
                bool ok = false;
                if (!gaitIdle) {
                    Serial.println("[WebComm] CAPTURE_NEUTRAL rejected: gait not IDLE.");
                } else if (balanceArmed) {
                    Serial.println("[WebComm] CAPTURE_NEUTRAL rejected: balance armed "
                                   "— disarm it so the joints hold the manual pose.");
                } else {
                    ok = _servoCtrl->captureSessionNeutral();
                }
                char buf[96];
                snprintf(buf, sizeof(buf), "NEUTRAL_TRIM:ok=%d,active=%d,max=%.2f",
                         (int)ok, (int)_servoCtrl->hasSessionTrim(),
                         _servoCtrl->maxSessionTrimDeg());
                _sendFiltered(buf);
                Serial.printf("[WebComm] %s\n", buf);
            } else {
                ws.textAll("NEUTRAL_TRIM:ok=0,active=0,max=0.00");
            }
        }
        else if (cmd == "CLEAR_NEUTRAL") {
            if (_servoCtrl) {
                _servoCtrl->clearSessionNeutral();
                _sendFiltered("NEUTRAL_TRIM:ok=1,active=0,max=0.00");
                Serial.println("[WebComm] CLEAR_NEUTRAL");
            }
        }
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
                        else if (key == "dx_per_deg") cfg.pitch_to_dx_mm_per_deg = constrain(val, 0.5f, 6.0f);
                        else if (key == "dy_per_deg") cfg.roll_to_dy_mm_per_deg  = constrain(val, 0.5f, 6.0f);
                    }
                    start = (comma == -1) ? params.length() : comma + 1;
                }
                // Legacy joint-space leg balance was retired in the consolidation
                // refactor: ankle/hip ratios no longer exist. Torso pitch/roll
                // balance is a fully independent subsystem — see TORSO_TUNE.
                _balCtrl->setConfig(cfg);
                Serial.printf("[WebComm] BALANCE_TUNE: Kp=%.3f Kd=%.3f sp=%.4f\n",
                               cfg.Kp, cfg.Kd, cfg.pitch_setpoint_rad);
            }
        }
        // ---------------------------------------------------------------------
        //  Torso stabiliser tuning — independent joint-space PD.
        //  Protocol: CMD:TORSO_TUNE:key=value[,key=value...]
        // ---------------------------------------------------------------------
        else if (cmd.startsWith("TORSO_TUNE:")) {
            if (_torsoStab) {
                TorsoConfig cfg = _torsoStab->getConfig();
                String params = cmd.substring(11);
                int start = 0;
                while (start < (int)params.length()) {
                    int comma = params.indexOf(',', start);
                    String pair = (comma == -1) ? params.substring(start)
                                                : params.substring(start, comma);
                    int eq = pair.indexOf('=');
                    if (eq != -1) {
                        String key = pair.substring(0, eq);
                        float  val = pair.substring(eq + 1).toFloat();
                        // ── Pitch ──────────────────────────────────────────
                        if      (key == "Kp")       cfg.Kp = constrain(val, 0.0f, BALANCE_KP_MAX);
                        else if (key == "Kd")       cfg.Kd = constrain(val, 0.0f, BALANCE_KD_MAX);
                        else if (key == "setpoint") cfg.pitch_setpoint_rad   = val;
                        else if (key == "deadband") cfg.pitch_deadband_rad   = constrain(val, 0.0f, 0.20f);
                        else if (key == "d_lpf")    cfg.derivative_lpf_alpha = constrain(val, 0.0f, 0.99f);
                        else if (key == "sign")     cfg.correction_sign      = val;
                        else if (key == "max")      cfg.max_correction_deg   = val;
                        // ── Roll ───────────────────────────────────────────
                        else if (key == "Kp_r")     cfg.Kp_roll = constrain(val, 0.0f, BALANCE_KP_MAX);
                        else if (key == "Kd_r")     cfg.Kd_roll = constrain(val, 0.0f, BALANCE_KD_MAX);
                        else if (key == "sp_r")     cfg.roll_setpoint_rad         = val;
                        else if (key == "db_r")     cfg.roll_deadband_rad         = constrain(val, 0.0f, 0.20f);
                        else if (key == "dlpf_r")   cfg.roll_derivative_lpf_alpha = constrain(val, 0.0f, 0.99f);
                        else if (key == "sign_r")   cfg.roll_correction_sign      = val;
                        else if (key == "max_r")    cfg.max_roll_correction_deg   = val;
                        // ── Output shaping ─────────────────────────────────
                        else if (key == "iir")      cfg.output_iir_alpha             = constrain(val, 0.0f, 0.99f);
                        else if (key == "rate")     cfg.max_output_rate_deg_per_tick = val;
                        else if (key == "damping")  cfg.output_damping_kv            = val;
                        else if (key == "out_max")  cfg.max_output_deg               = val;
                    }
                    start = (comma == -1) ? params.length() : comma + 1;
                }
                _torsoStab->setConfig(cfg);
                Serial.printf("[WebComm] TORSO_TUNE: Kp=%.3f Kd=%.3f Kp_r=%.3f Kd_r=%.3f\n",
                               cfg.Kp, cfg.Kd, cfg.Kp_roll, cfg.Kd_roll);
            }
        }
       
        // ---------------------------------------------------------------------
        //  Recovery tuning — Stage 0 detection parameters
        //  Protocol: CMD:RECOVERY_TUNE:enabled=1,rcv_pitch=0.20,...
        // ---------------------------------------------------------------------
        else if (cmd.startsWith("RECOVERY_TUNE:")) {
            if (_balCtrl) {
                BalanceConfig cfg = _balCtrl->getConfig();
                String params = cmd.substring(14);
                int start = 0;
                while (start < (int)params.length()) {
                    int comma = params.indexOf(',', start);
                    String pair = (comma == -1) ? params.substring(start)
                                                : params.substring(start, comma);
                    int eq = pair.indexOf('=');
                    if (eq != -1) {
                        String key = pair.substring(0, eq);
                        float  val = pair.substring(eq + 1).toFloat();
                        if      (key == "enabled")   cfg.recovery_enabled              = (val > 0.5f);
                        else if (key == "supp_gait") cfg.recovery_suppress_during_gait = (val > 0.5f);
                        else if (key == "rcv_pitch") cfg.recovery_tilt_pitch_rad       = constrain(val, 0.05f, 0.60f);
                        else if (key == "rcv_roll")  cfg.recovery_tilt_roll_rad        = constrain(val, 0.05f, 0.60f);
                        else if (key == "rcv_sat")   cfg.recovery_saturation_ticks     = (uint16_t)constrain(val, 1.0f, 200.0f);
                        else if (key == "rcv_conf")  cfg.recovery_confirm_ticks        = (uint16_t)constrain(val, 1.0f, 100.0f);
                    }
                    start = (comma == -1) ? params.length() : comma + 1;
                }
                _balCtrl->setConfig(cfg);
                Serial.printf("[WebComm] RECOVERY_TUNE: en=%d supp=%d pitch=%.3f roll=%.3f sat=%u conf=%u\n",
                              (int)cfg.recovery_enabled, (int)cfg.recovery_suppress_during_gait,
                              cfg.recovery_tilt_pitch_rad, cfg.recovery_tilt_roll_rad,
                              cfg.recovery_saturation_ticks, cfg.recovery_confirm_ticks);
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
        //  Pose-transition speed — dedicated slew (deg/s) for the Neutral/Stance
        //  buttons, independent of the per-joint gait/IK cap above.
        //  Protocol: CMD:TRANS_SPEED:<deg_per_sec>
        // ---------------------------------------------------------------------
        else if (cmd.startsWith("TRANS_SPEED:")) {
            if (_servoCtrl) {
                float speed = cmd.substring(12).toFloat();
                _servoCtrl->setTransitionSpeed(speed);
                Serial.printf("[WebComm] TRANS_SPEED: pose transitions → %.1f deg/s\n",
                              _servoCtrl->getTransitionSpeed());
                broadcastSpeeds();
            }
        }
        // ---------------------------------------------------------------------
        //  Per-joint transmission gain — runtime override of the anatomical-
        //  to-servo mechanical ratio. Defaults live in joint_config.cpp; this
        //  command lets the operator sweep without reflashing.
        //
        //  Protocol: CMD:TRANSMISSION_TUNE:<ch>=<gain>[,<ch>=<gain>...]
        //  Example:  CMD:TRANSMISSION_TUNE:1=0.55,14=0.55
        //
        //  JointModel clamps the gain to [0.1, 2.0]; the corrected servo
        //  command is bounded by the 0–270° hardware envelope, and any
        //  saturation flips the per-joint TRANSMISSION_CLAMP flag visible in
        //  the TRANSMISSION broadcast below.
        // ---------------------------------------------------------------------
        else if (cmd.startsWith("TRANSMISSION_TUNE:")) {
            if (_servoCtrl) {
                // Mirrors the GAIT_TUNE key=value parser. Key is the channel
                // index; value is the gain.
                String params = cmd.substring(18);
                int start = 0;
                while (start < (int)params.length()) {
                    int comma = params.indexOf(',', start);
                    String pair = (comma == -1) ? params.substring(start)
                                                : params.substring(start, comma);
                    int eq = pair.indexOf('=');
                    if (eq != -1) {
                        int   ch   = pair.substring(0, eq).toInt();
                        float gain = pair.substring(eq + 1).toFloat();
                        if (ch >= 0 && ch < NUM_JOINTS) {
                            _servoCtrl->setTransmissionGain((uint8_t)ch, gain);
                        } else {
                            Serial.printf("[WebComm] TRANSMISSION_TUNE: ch=%d "
                                          "out of range\n", ch);
                        }
                    }
                    start = (comma == -1) ? params.length() : comma + 1;
                }
                // Echo full per-joint state so the UI can re-sync from one
                // packet — same pattern as broadcastSpeeds().
                broadcastTransmissionState();
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
        // Stage 1: manual recovery test commands (diagnostic only).
        // Simulate a rightward or leftward push at ~11.5° roll without hardware.
        // Use with balance ARMED and robot standing still. Disable before gait testing.
        else if (cmd == "GAIT_TEST_RECOVERY_R") {
            if (_gaitCtrl) {
                _gaitCtrl->triggerRecovery(0.0f, +0.20f);
                Serial.println("[WebComm] CMD: GAIT_TEST_RECOVERY_R — simulating right push");
            }
        }
        else if (cmd == "GAIT_TEST_RECOVERY_L") {
            if (_gaitCtrl) {
                _gaitCtrl->triggerRecovery(0.0f, -0.20f);
                Serial.println("[WebComm] CMD: GAIT_TEST_RECOVERY_L — simulating left push");
            }
        }
        else if (cmd == "GAIT_TEST_RECOVERY_F") {
            if (_gaitCtrl) {
                _gaitCtrl->triggerRecovery(+0.20f, 0.0f);
                Serial.println("[WebComm] CMD: GAIT_TEST_RECOVERY_F — simulating forward push");
            }
        }
        else if (cmd == "GAIT_TEST_RECOVERY_B") {
            if (_gaitCtrl) {
                _gaitCtrl->triggerRecovery(-0.20f, 0.0f);
                Serial.println("[WebComm] CMD: GAIT_TEST_RECOVERY_B — simulating backward push");
            }
        }
        // ---------------------------------------------------------------------
        //  Touchdown-awareness tuning (Stage 0/1)
        //  Protocol: CMD:TOUCHDOWN_TUNE:enabled=1,gatePhase=0.70
        //
        //  Lives in its own command (not folded into GAIT_TUNE) so the policy
        //  knob stays visible and discoverable. Fields are validated and
        //  clamped here; the gait controller takes them via setConfig().
        // ---------------------------------------------------------------------
        else if (cmd.startsWith("TOUCHDOWN_TUNE:")) {
            if (_gaitCtrl) {
                GaitConfig cfg = _gaitCtrl->getConfig();
                String params = cmd.substring(15);
                int start = 0;
                while (start < (int)params.length()) {
                    int comma = params.indexOf(',', start);
                    String pair = (comma == -1) ? params.substring(start)
                                                : params.substring(start, comma);
                    int eq = pair.indexOf('=');
                    if (eq != -1) {
                        String key = pair.substring(0, eq);
                        String val = pair.substring(eq + 1);
                        if      (key == "enabled")   cfg.touchdownEnabled   = (val.toInt() != 0);
                        // Floor at 0.30: anything lower defeats the whole
                        // point of the late-swing gate (the impact detector
                        // already has noise rejection but mid-swing impacts
                        // are nearly always linkage transients).
                        // Ceiling at 0.99: a sub-tick window before phase
                        // wraps is still meaningful at 400 Hz.
                        else if (key == "gatePhase") cfg.touchdownGatePhase = constrain(val.toFloat(), 0.30f, 0.99f);
                        // Trim suppression (W2/W5 phase-gate). Folded into this
                        // command since it is touchdown-window policy.
                        else if (key == "trimSuppress") cfg.touchdownTrimSuppressEnabled = (val.toInt() != 0);
                        // Hold window 0-400 ms. 0 = suppress only while the
                        // latch is held; above 400 ms it would bleed well into
                        // the next half-cycle's clean trim region.
                        else if (key == "trimHoldMs")   cfg.touchdownTrimHoldMs =
                                     (uint32_t)constrain(val.toFloat(), 0.0f, 400.0f);
                        else if (key == "trimFreeze")   cfg.touchdownTrimFreezeHold = (val.toInt() != 0);
                    }
                    start = (comma == -1) ? params.length() : comma + 1;
                }
                _gaitCtrl->setConfig(cfg);
                Serial.printf("[WebComm] TOUCHDOWN_TUNE: en=%d gatePhase=%.2f "
                              "trimSup=%d trimHold=%lu trimFrz=%d\n",
                              (int)cfg.touchdownEnabled,
                              cfg.touchdownGatePhase,
                              (int)cfg.touchdownTrimSuppressEnabled,
                              (unsigned long)cfg.touchdownTrimHoldMs,
                              (int)cfg.touchdownTrimFreezeHold);
            }
        }
        // ---------------------------------------------------------------------
        //  W4 touchdown-compliance tuning.
        //  Protocol: CMD:COMPLIANCE_TUNE:enabled=1,depthMm=4.0,decayMs=180
        //
        //  Its own command (a sibling of TOUCHDOWN_TUNE) so the compliance
        //  knobs stay discoverable. Fields are validated and clamped here.
        // ---------------------------------------------------------------------
        else if (cmd.startsWith("COMPLIANCE_TUNE:")) {
            if (_gaitCtrl) {
                GaitConfig cfg = _gaitCtrl->getConfig();
                String params = cmd.substring(16);
                int start = 0;
                while (start < (int)params.length()) {
                    int comma = params.indexOf(',', start);
                    String pair = (comma == -1) ? params.substring(start)
                                                : params.substring(start, comma);
                    int eq = pair.indexOf('=');
                    if (eq != -1) {
                        String key = pair.substring(0, eq);
                        String val = pair.substring(eq + 1);
                        if      (key == "enabled") cfg.complianceEnabled = (val.toInt() != 0);
                        // Depth capped at 15 mm: beyond a small knee-flex this
                        // stops being a compliance smear and becomes a squat
                        // that fights the balance controller. Floor 0 = off.
                        else if (key == "depthMm") cfg.complianceDepthMm = constrain(val.toFloat(), 0.0f, 15.0f);
                        // Decay window 20-800 ms. Below 20 ms the recovery is
                        // itself a transient; above 800 ms it bleeds into the
                        // next half-cycle's load transfer.
                        else if (key == "decayMs") cfg.complianceDecayMs =
                                     (uint32_t)constrain(val.toFloat(), 20.0f, 800.0f);
                    }
                    start = (comma == -1) ? params.length() : comma + 1;
                }
                _gaitCtrl->setConfig(cfg);
                Serial.printf("[WebComm] COMPLIANCE_TUNE: en=%d depth=%.1f mm decay=%lu ms\n",
                              (int)cfg.complianceEnabled,
                              cfg.complianceDepthMm,
                              (unsigned long)cfg.complianceDecayMs);
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
                        else if (key == "liftGateMode") {
                            const int m = constrain((int)val, 0, 5);
                            cfg.liftGateMode = static_cast<LiftGateMode>(m);
                        }
                        else if (key == "wsThresh")    cfg.wsThresholdRad    = constrain(val, 0.01f, 0.50f);
                        else if (key == "stepH_mm") {
                            const float maxStep = fmaxf(5.0f, cfg.stanceHeightMm - 130.0f);
                            cfg.stepHeightMm = constrain(val, 5.0f, maxStep);
                        }
                        else if (key == "stanceH_mm") {
                            const float minStance = 130.0f + cfg.stepHeightMm;
                            cfg.stanceHeightMm = constrain(val, minStance, gaitMaxReachMm);
                        }
                        else if (key == "stanceInset_mm") {
                            // Symmetric inward stance-width inset (mm).
                            // Positive = both feet move toward midline.
                            // 30 mm ceiling: well below frontal IK reach
                            // even when stacked with WS lateral shift,
                            // comLatTrim, and balance dy. Negative values
                            // (abduction past nominal) blocked to keep the
                            // semantic unambiguous — widen stance via
                            // hardware geometry, not a negative inset.
                            //
                            // The inset is a TASK-SPACE FootTarget y_mm baseline,
                            // applied directly in GaitController::_buildFootTarget
                            // (t.y_mm = -stanceInsetMm on both legs). Storing the
                            // config value is the whole job here — the gait loop
                            // reads it every tick, so the boot default and every
                            // stance entry honour it with no separate "apply" step.
                            cfg.stanceInsetMm = constrain(val, 0.0f, 30.0f);
                        }
                        else if (key == "stepLength_mm") {
                            // Hard cap at 20% of stance height to protect IK
                            // hip pitch limits at full extension. At stanceH=185
                            // this is 37 mm. Floor at 0 (disables stride entirely).
                            const float maxStride = cfg.stanceHeightMm * 0.20f;
                            cfg.stepLengthMm = constrain(val, 0.0f, maxStride);
                            if (val > maxStride) {
                                Serial.printf("[WebComm] GAIT_TUNE: stepLength_mm=%.1f "
                                              "clamped to %.1f (20%% of stanceH=%.1f)\n",
                                              val, maxStride, cfg.stanceHeightMm);
                            }
                        }
                        else if (key == "captureEnabled") cfg.captureEnabled = (val > 0.5f);
                        else if (key == "captureGain")    cfg.captureGain = constrain(val, 0.0f, 1.5f);
                        else if (key == "captureMax") {
                            // Hard ceiling: 30 mm. Anything more risks passing IK reach budget on
                            // most stance heights. Re-evaluate the cap before raising.
                            cfg.captureOffsetMaxMm = constrain(val, 0.0f, 30.0f);
                        }
                        // Pitch-ANGLE swing-foot forward placement (distinct from
                        // the rate-based capture point above; ANGLE-driven).
                        else if (key == "pitchPlaceEnabled") cfg.pitchPlacementEnabled = (val > 0.5f);
                        else if (key == "pitchPlaceGain") {
                            // mm of forward foot placement per rad of (deadbanded)
                            // forward pitch. 0 = inert (ship default). 300 mm/rad
                            // ceiling — well above the ~100 mm/rad first-test value;
                            // the ±max clamp bounds the actual effect regardless.
                            cfg.pitchPlacementGainMmPerRad = constrain(val, 0.0f, 300.0f);
                        }
                        else if (key == "pitchPlaceMax") {
                            // Hard cap on the placement offset (mm). 40 mm ceiling
                            // mirrors the comLatMax budget — placement stacks with
                            // stride + capture, so keep it inside sagittal IK reach.
                            cfg.pitchPlacementMaxMm = constrain(val, 0.0f, 40.0f);
                        }
                        else if (key == "pitchPlaceDeadband") {
                            // Pitch magnitude (rad) below which no placement applies.
                            // 0.20 rad (~11°) ceiling; default 0.033 rad (~1.9°).
                            cfg.pitchPlacementDeadbandRad = constrain(val, 0.0f, 0.20f);
                        }
                        else if (key == "pitchPlaceSign") {
                            // ±1 runtime sign flip. Intended +1 (forward pitch ->
                            // forward foot); snap anything else to the nearest of
                            // {-1, +1} — the physical convention is binary.
                            cfg.pitchPlacementSign = (val >= 0.0f) ? +1.0f : -1.0f;
                        }
                        else if (key == "dwellMs") {
                            // 0 disables dwell. 1000 ms cap is generous;
                            // beyond that the dwell dominates step timing.
                            cfg.liftDwellMs = (uint32_t)constrain(val, 0.0f, 1000.0f);
                        }
                        else if (key == "waitMaxMs") {
                            // 0 disables the timeout. 10 s cap because longer
                            // waits indicate a hardware or tuning fault that
                            // should be diagnosed, not tolerated.
                            cfg.waitShiftMaxMs = (uint32_t)constrain(val, 0.0f, 10000.0f);
                        }
                        else if (key == "comLatGateMm") {
                            // 0 disables the gate (always satisfies in
                            // COM_LATERAL mode); 50 mm cap because beyond
                            // that the threshold approaches lateral_shift_mm
                            // and the gate effectively never opens.
                            cfg.comLateralGateMm = constrain(val, 0.0f, 50.0f);
                        }
                        else if (key == "swingApex") {
                            // W1 — swing-envelope apex fraction. [0.05, 0.95]
                            // matches the clamp inside _swingFrac() so the UI
                            // slider and firmware never disagree.
                            cfg.swingApexFrac = constrain(val, 0.05f, 0.95f);
                        }
                        else if (key == "strideComplete") {
                            // Horizontal stride completion fraction. (0.05, 1.0]
                            // matches the clamp inside _warpedStrideFrac(). 1.0
                            // = no warp (legacy bit-identical behaviour). Lower
                            // values front-load the swing-leg stride so the
                            // foot reaches +L/2 with zero horizontal velocity
                            // before touchdown. For the effect to act before
                            // the touchdown latch can fire, keep this
                            // <= touchdownGatePhase (default 0.7). Recommended
                            // experimental range 0.5–0.7.
                            cfg.strideCompleteFrac = constrain(val, 0.05f, 1.0f);
                        }
                        else if (key == "comLatTrim") cfg.comLatTrimEnabled = (val > 0.5f);
                        else if (key == "comLatGain") {
                            // W2 — lateral trim gain. 0 nulls the trim (same
                            // as the toggle off). 1.5 cap: past full LIP
                            // correction the trim over-drives the feet.
                            cfg.comLatTrimGain = constrain(val, 0.0f, 1.5f);
                        }
                        else if (key == "comLatMax") {
                            // W2 — per-tick trim cap. 40 mm hard ceiling so
                            // the trim alone cannot approach the lateral IK
                            // reach budget on any stance height.
                            cfg.comLatTrimMaxMm = constrain(val, 0.0f, 40.0f);
                        }
                        else if (key == "comLatKd") {
                            // W5 — velocity-damping gain (seconds). 0 = no
                            // damping (pure W2). 0.5 s cap: well past any
                            // useful value — beyond ~0.15 s the Kd term tends
                            // to amplify gyro noise faster than it damps.
                            cfg.comLatTrimKd = constrain(val, 0.0f, 0.5f);
                        }
                        else if (key == "comLatRateLpf") {
                            // W5 — roll-rate LPF retention factor. Clamp to
                            // [0, 0.99]: 0 = raw rate (noisy), 0.99 = nearly
                            // frozen. Mirrors the clamp in _updateRollRateFilter.
                            cfg.comLatTrimRateLpfAlpha = constrain(val, 0.0f, 0.99f);
                        }
                        else if (key == "latCapture") cfg.latCaptureEnabled = (val > 0.5f);
                        else if (key == "latCaptureGain") {
                            // Mirrors sagittal captureGain cap. 0 nulls the
                            // term (same as the enable off). 1.5 cap matches
                            // _captureOffsetMm — past full LIP correction
                            // the term over-drives swing-foot placement.
                            cfg.latCaptureGain = constrain(val, 0.0f, 1.5f);
                        }
                        else if (key == "latCaptureMax") {
                            // 30 mm hard ceiling: above this the offset alone
                            // starts approaching frontal IK reach when stacked
                            // with WS lateral shift + comLatTrim. Re-evaluate
                            // before raising.
                            cfg.latCaptureMaxMm = constrain(val, 0.0f, 30.0f);
                        }
                        else if (key == "latCaptureSign") {
                            // ±1 sign flip (runtime). Anything else is rejected
                            // by snapping to the nearest of {-1, +1}; the
                            // physical convention is binary, no third option.
                            cfg.latCaptureSign = (val >= 0.0f) ? +1.0f : -1.0f;
                        }
                        else if (key == "torsoNeutralRoll") {
                            // +/-0.20 rad (~11 deg) hard cap. Anything larger
                            // indicates mounting/hardware fault that the
                            // operator should fix, not tare around.
                            cfg.torsoRollNeutralRad = constrain(val, -0.20f, 0.20f);
                        }
                        else if (key == "torsoNeutralPitch") {
                            cfg.torsoPitchNeutralRad = constrain(val, -0.20f, 0.20f);
                        }
                        else if (key == "fwdCom") {
                            // Forward CoM progression — max forward body offset
                            // applied to BOTH legs' x_mm at the peak of the
                            // localPhase shape. 0 = inert (default).
                            // Hard ceiling 15 mm: above this the term alone
                            // approaches the order of stepLengthMm and starts
                            // composing with stance retreat to over-translate
                            // the body forward (mode A risk). Tune up from 0
                            // in ~1 mm steps and stop when mode A starts rising.
                            cfg.fwdComMaxMm = constrain(val, 0.0f, 15.0f);
                        }
                        else if (key == "fwdComLead") {
                            // Front-load fraction. 1.0 = pure unwarped
                            // smoothstep (default). (0.05, 1.0] matches the
                            // clamp inside _fwdComProgressionMm(). Lower
                            // toward 0.5 to push the progression earlier in
                            // the swing.
                            cfg.fwdComLeadFrac = constrain(val, 0.05f, 1.0f);
                        }
                        else if (key == "heelStrike") cfg.heelStrikeEnabled = (val > 0.5f);
                        else if (key == "heelDeg") {
                            // Peak dorsiflexion bias (deg). 0 nulls the bias.
                            // Hard ceiling = GAIT_HEELSTRIKE_MAX_BIAS_DEG.
                            cfg.heelStrikeDorsiflexDeg = constrain(val, 0.0f, GAIT_HEELSTRIKE_MAX_BIAS_DEG);
                        }
                        else if (key == "heelGate") {
                            // Engage gate phase [0.05, 0.90). Keep below
                            // touchdownGatePhase (default 0.70) so the ramp
                            // is near-full at contact.
                            cfg.heelStrikeGatePhase = constrain(val, 0.05f, 0.90f);
                        }
                        else if (key == "heelRelMs") {
                            // Release window (ms). 0 = bias snaps to zero on
                            // touchdown latch. 500 ms cap — longer risks
                            // spilling into the next half-cycle.
                            cfg.heelStrikeReleaseMs = (uint32_t)constrain(val, 0.0f, 500.0f);
                        }
                        // NOTE: "heelSign" used to live here. Sign is now baked
                        // into _heelStrikeBiasDeg() as intrinsically dorsiflex
                        // (negative). Removed to prevent confusion — confirmed
                        // on hardware, not configurable.
                        else if (key == "swingPrep") cfg.swingPrepEnabled = (val > 0.5f);
                        else if (key == "swingPrepGain") {
                            // Swing-foot ankle-roll preparation gain: fraction of
                            // ankle_shift_deg used as the future-stance target.
                            // 0 = no-op. Clamp [0, 1.5].
                            cfg.swingPrepGain = constrain(val, 0.0f, 1.5f);
                        }
                        else if (key == "swingPrepFrac") {
                            // Front-load completion fraction (0.05, 1.0]. 1.0 =
                            // prep completes at touchdown; lower finishes earlier.
                            cfg.swingPrepCompleteFrac = constrain(val, 0.05f, 1.0f);
                        }
                        else if (key == "stabilizeMs") {
                            // 0..1000 ms inter-step hold. 0 = STABILIZING
                            // collapses to ~1 tick (maximally continuous).
                            // Floor risk: prior-half IIR/mechanical transient
                            // superposition; empirical knee ~80-150 ms.
                            cfg.stabilizeMs = (uint32_t)constrain(val, 0.0f, 1000.0f);
                        }
                        else if (key == "skipNone") {
                            // Step-4 probe: skip trigger(NONE) at half-cycle
                            // boundary; arm next direction immediately.
                            cfg.skipStabilizeNone = (val > 0.5f);
                        }
                        else if (key == "hRestore") cfg.hRestoreEnabled = (val > 0.5f);
                        else if (key == "hRestoreFrac") {
                            // Deferred swing-leg h-restore completion fraction.
                            // (0.05, 1.0] matches the clamp inside
                            // _updateHRestore()/_buildFootTarget. The restore
                            // completes (just-planted leg back at full stance
                            // height) by this new-half phase; default 0.5 puts
                            // it ~under the hip. Lower = quicker re-extension.
                            cfg.hRestoreCompleteFrac = constrain(val, 0.05f, 1.0f);
                        }
                    }
                    start = (comma == -1) ? params.length() : comma + 1;
                }
                _gaitCtrl->setConfig(cfg);
                Serial.printf("[WebComm] GAIT_TUNE: rate=%.2f Hz  gate=%.2f  wsThresh=%.3f rad  "
                              "stepH=%.1f mm  stanceH=%.1f mm  stepLen=%.1f mm  capEn=%d capGain=%.2f capMax=%.1f  "
                              "dwell=%lu waitMax=%lu comLat=%.1f trP=%.4f trR=%.4f  "
                              "swingApex=%.3f comLatTrim=%d comLatGain=%.2f comLatMax=%.1f "
                              "comLatKd=%.3f comLatRateLpf=%.2f  hRestore=%d hRestoreFrac=%.2f\n",
                              cfg.phaseRateHz, cfg.liftGateThreshold, cfg.wsThresholdRad,
                              cfg.stepHeightMm, cfg.stanceHeightMm, cfg.stepLengthMm,
                              (int)cfg.captureEnabled, cfg.captureGain, cfg.captureOffsetMaxMm,
                              (unsigned long)cfg.liftDwellMs, (unsigned long)cfg.waitShiftMaxMs,
                              cfg.comLateralGateMm, cfg.torsoPitchNeutralRad, cfg.torsoRollNeutralRad,
                              cfg.swingApexFrac, (int)cfg.comLatTrimEnabled,
                              cfg.comLatTrimGain, cfg.comLatTrimMaxMm,
                              cfg.comLatTrimKd, cfg.comLatTrimRateLpfAlpha,
                              (int)cfg.hRestoreEnabled, cfg.hRestoreCompleteFrac);
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
                _sendFiltered(buf);
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
