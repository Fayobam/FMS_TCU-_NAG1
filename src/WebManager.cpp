// ============================================================================
// FILE: WebManager.cpp
// VERSION: 7.0
// UPDATES: Added "limp_reset" command + safety/load fields to telemetry JSON.
// ============================================================================
#include "WebManager.h"
#include "EngineProfile.h"

WebManager::WebManager() : server(80), ws("/ws") {
    _last_broadcast_time = 0;
    _adaptives = nullptr;
}

void WebManager::setAdaptiveMemory(AdaptiveMemory* adaptives) { _adaptives = adaptives; }

void WebManager::begin() {
    if (!SPIFFS.begin(true)) return;
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_AP);
    WiFi.softAP("FMS_TCU", "shiftfast");

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        if (SPIFFS.exists("/index.html")) request->send(SPIFFS, "/index.html", "text/html");
        else request->send(200, "text/plain", "FMS TCU Server is LIVE, but index.html is MISSING!");
    });

    ws.onEvent([this](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len){
        if(type == WS_EVT_DATA){
            AwsFrameInfo *info = (AwsFrameInfo*)arg;
            if(info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT){
                this->handleWebSocketMessage(arg, data, len);   // length-aware; no OOB terminator write
            }
        }
    });

    server.addHandler(&ws);
    server.begin();
}

void WebManager::handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, data, len);  // bounded by len, no terminator needed
    if (error) return;

    String cmd = doc["cmd"];

    // --- Safety: allow the dashboard to request a limp-mode reset ---
    if (cmd == "limp_reset") {
        telemetry.limp_reset_request = true;   // honoured only when stopped & in P/N
        Serial.println("Limp reset requested via web.");
        return;
    }

    // --- Engine profile (torque table + limits + sensor cal + baseline fill) ---
    if (cmd == "get_profile") {
        EngineProfileData* p = engineProfile.raw();
        JsonDocument resp;
        resp["type"] = "profile_data";
        JsonArray tq = resp["torque"].to<JsonArray>();
        for (int i = 0; i < EP_RPM_BINS; i++)
            for (int j = 0; j < EP_MAP_BINS; j++) tq.add(p->torque[i][j]);
        JsonArray rb = resp["rpm"].to<JsonArray>();
        for (int i = 0; i < EP_RPM_BINS; i++) rb.add(p->rpm_bp[i]);
        JsonArray mb = resp["map"].to<JsonArray>();
        for (int j = 0; j < EP_MAP_BINS; j++) mb.add(p->map_bp[j]);
        resp["tmax"] = p->t_max_ref; resp["overrev"] = p->overrev_rpm; resp["lug"] = p->lug_rpm;
        resp["ppr"]  = p->eng_ppr;
        resp["tpsC"] = p->tps_closed_v; resp["tpsW"] = p->tps_wot_v;
        resp["map0"] = p->map_kpa_at_0v; resp["mapV"] = p->map_kpa_per_volt;
        JsonArray fp = resp["fillp"].to<JsonArray>();
        JsonArray ft = resp["fillt"].to<JsonArray>();
        for (int i = 0; i < 4; i++) { fp.add(p->fill_p[i]); ft.add(p->fill_t[i]); }
        char buffer[1280];
        size_t n = serializeJson(resp, buffer, sizeof(buffer));
        ws.textAll(buffer, n);
        return;
    }
    else if (cmd == "set_profile") {
        EngineProfileData* p = engineProfile.raw();
        JsonArray tq = doc["torque"].as<JsonArray>();
        if ((int)tq.size() >= EP_RPM_BINS * EP_MAP_BINS) {
            for (int i = 0; i < EP_RPM_BINS; i++)
                for (int j = 0; j < EP_MAP_BINS; j++)
                    p->torque[i][j] = (int16_t)constrain(tq[i*EP_MAP_BINS+j].as<int>(), 0, 1200);
        }
        if (doc["overrev"].is<int>()) p->overrev_rpm = (uint16_t)constrain(doc["overrev"].as<int>(), 3000, 9000);
        if (doc["lug"].is<int>())     p->lug_rpm     = (uint16_t)constrain(doc["lug"].as<int>(), 500, 2500);
        if (doc["ppr"].is<int>())     p->eng_ppr     = (uint16_t)constrain(doc["ppr"].as<int>(), 1, 200);
        if (doc["tmax"].is<int>())    p->t_max_ref   = (uint16_t)constrain(doc["tmax"].as<int>(), 100, 1200);
        if (doc["tpsC"].is<float>())  p->tps_closed_v = doc["tpsC"].as<float>();
        if (doc["tpsW"].is<float>())  p->tps_wot_v    = doc["tpsW"].as<float>();
        if (doc["map0"].is<float>())  p->map_kpa_at_0v  = doc["map0"].as<float>();
        if (doc["mapV"].is<float>())  p->map_kpa_per_volt = doc["mapV"].as<float>();
        JsonArray fp = doc["fillp"].as<JsonArray>();
        JsonArray ft = doc["fillt"].as<JsonArray>();
        if ((int)fp.size() >= 4 && (int)ft.size() >= 4)
            for (int i = 0; i < 4; i++) {
                p->fill_p[i] = (uint8_t)constrain(fp[i].as<int>(), 0, 100);
                p->fill_t[i] = (uint16_t)constrain(ft[i].as<int>(), 0, 400);
            }
        engineProfile.save();
        Serial.println("Engine profile updated via Web!");
        return;
    }

    if (_adaptives == nullptr) return;

    // Adaptation v2 cells: flat array of {fill_t_cycles, fill_p_trim, apply_trim} ×
    // (ADAPT_CLASSES*ADAPT_SHIFTS*ADAPT_TBINS). The dashboard tuner reads/writes them
    // as a flat int8 stream of length cellCount()*3.
    if (cmd == "get_cells") {
        AdaptCell* cells = _adaptives->cellsPtr();
        int n = _adaptives->cellCount();
        JsonDocument responseDoc;
        responseDoc["type"] = "cell_data";
        responseDoc["classes"] = ADAPT_CLASSES;
        responseDoc["shifts"]  = ADAPT_SHIFTS;
        responseDoc["tbins"]   = ADAPT_TBINS;
        JsonArray arr = responseDoc["data"].to<JsonArray>();
        for (int i = 0; i < n; i++) {
            arr.add(cells[i].fill_t_cycles);
            arr.add(cells[i].fill_p_trim);
            arr.add(cells[i].apply_trim);
        }
        char buffer[1536];
        size_t resLen = serializeJson(responseDoc, buffer, sizeof(buffer));
        ws.textAll(buffer, resLen);
    }
    else if (cmd == "set_cells") {
        AdaptCell* cells = _adaptives->cellsPtr();
        int n = _adaptives->cellCount();
        JsonArray arr = doc["data"].as<JsonArray>();
        if ((int)arr.size() >= n * 3) {
            for (int i = 0; i < n; i++) {
                cells[i].fill_t_cycles = (int8_t)constrain(arr[i*3+0].as<int>(), FILL_T_CYC_MIN,  FILL_T_CYC_MAX);
                cells[i].fill_p_trim   = (int8_t)constrain(arr[i*3+1].as<int>(), FILL_P_TRIM_MIN, FILL_P_TRIM_MAX);
                cells[i].apply_trim    = (int8_t)constrain(arr[i*3+2].as<int>(), APPLY_TRIM_MIN,  APPLY_TRIM_MAX);
            }
            _adaptives->markAllDirtyAndFlush();
            Serial.println("Adaptation cells updated via Web Tuner!");
        }
    }
}

// Seqlock read of a Core-1-written status string into a stable local buffer, so a
// concurrent Core-1 write can't hand us a torn string during JSON serialization.
static void readStatusString(const volatile uint8_t &seq, const char *src, char *dst, size_t n) {
    for (int tries = 0; tries < 4; tries++) {
        uint8_t s0 = seq;
        strncpy(dst, src, n - 1); dst[n - 1] = '\0';
        uint8_t s1 = seq;
        if (s0 == s1 && (s0 & 1) == 0) return;   // stable and not mid-write
    }
}

void WebManager::broadcastTelemetry() {
    // 100 Hz (10 ms gate) — matches the dashboard chart sample design and the
    // documented stream rate. Was 100 ms (10 Hz), 10× slower than every doc claim.
    if (millis() - _last_broadcast_time >= 10) {
        buildAndSendTelemetryJSON();
        _last_broadcast_time = millis();
        // Persist adaptation on Core 0 (NVS can block 1-10ms). Flushes on a 60s timer
        // or when a P/N-entry/web edit forced it — not per shift (NVS wear).
        if (_adaptives) _adaptives->processFlush();
    }
    ws.cleanupClients();
}

void WebManager::buildAndSendTelemetryJSON() {
    if (ws.count() == 0) return;
    JsonDocument doc;
    doc["type"]      = "telemetry";
    doc["prnd"]      = String(telemetry.prnd_state);
    doc["gear"]      = telemetry.current_gear;
    doc["engRpm"]    = telemetry.engine_rpm;
    doc["turbRpm"]   = telemetry.turbine_rpm;
    doc["outRpm"]    = telemetry.output_rpm;
    doc["tps"]       = telemetry.tps_pct;
    doc["map"]       = telemetry.map_kpa;
    doc["mpc"]       = telemetry.line_pressure_pct;
    doc["spc"]       = telemetry.shift_pressure_pct;
    doc["shiftTime"] = telemetry.last_shift_time_ms;
    doc["ratio"]     = telemetry.live_ratio;
    doc["flare"]     = telemetry.flare_detected;
    doc["bind"]      = telemetry.bind_detected;

    doc["tccPwm"]    = telemetry.tcc_lockup_pct;
    doc["tccTarget"] = telemetry.tcc_target_slip_rpm;
    doc["tccActual"] = telemetry.tcc_actual_slip_rpm;

    doc["limp"]      = telemetry.is_limp_mode;
    char limpReason[64], safetyEvent[64];
    readStatusString(telemetry.limp_reason_seq, telemetry.limp_mode_reason, limpReason, sizeof(limpReason));
    readStatusString(telemetry.safety_event_seq, telemetry.last_safety_event, safetyEvent, sizeof(safetyEvent));
    doc["limpReason"]= limpReason;
    doc["safety"]    = safetyEvent;
    doc["atfTemp"]   = telemetry.atf_temp_c;
    doc["htMode"]    = telemetry.high_torque_mode;
    doc["phase"]     = telemetry.shift_phase;
    doc["revAbuse"]  = telemetry.reverse_abuse_active;

    // Class engine readouts (the validation plan logs these per shift).
    doc["tEstNm"]    = telemetry.t_est_nm;
    doc["loadPct"]   = telemetry.load_pct;
    doc["shiftClass"]= telemetry.shift_class;   // 0=POWER_UP 1=COAST_UP 2=POWER_DOWN 3=COAST_DOWN
    doc["pdType"]    = telemetry.pd_type;       // 0=NONE 1=SPRAG 2=TIMED

    char buffer[1024];
    size_t len = serializeJson(doc, buffer, sizeof(buffer));
    if (len >= sizeof(buffer) - 1) Serial.println("WARNING: Telemetry JSON truncated — increase buffer!");
    ws.textAll(buffer, len);
}
