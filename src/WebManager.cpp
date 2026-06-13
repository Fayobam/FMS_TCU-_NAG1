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
        resp["engPpr"] = p->eng_ppr; resp["outPpr"] = p->out_ppr;
        resp["clEn"]   = p->cl_spc_enable; resp["clKp"] = p->cl_spc_kp;
        resp["transVariant"] = p->trans_variant;   // 0=small NAG, 1=big NAG
        resp["tcStall"] = p->tc_stall_mult_x100; resp["tcCoupSr"] = p->tc_coupling_sr_x100;
        resp["clPwr"] = p->cl_pressure_enable; resp["pFull"] = p->p_full_scale_mbar;
        JsonArray ck = resp["clutchK"].to<JsonArray>();
        JsonArray rs = resp["relSpring"].to<JsonArray>();
        for (int i = 0; i < 4; i++) { ck.add(p->clutch_k_x100[i]); rs.add(p->release_spring_mbar[i]); }
        resp["tpsC"] = p->tps_closed_v; resp["tpsW"] = p->tps_wot_v;
        resp["map0"] = p->map_kpa_at_0v; resp["mapV"] = p->map_kpa_per_volt;
        JsonArray fp = resp["fillp"].to<JsonArray>();
        JsonArray ft = resp["fillt"].to<JsonArray>();
        for (int i = 0; i < 4; i++) { fp.add(p->fill_p[i]); ft.add(p->fill_t[i]); }
        char buffer[1600];
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
        // Sensor PPR: SpeedReader hot-applies these on its next update (no reboot).
        if (doc["engPpr"].is<int>())  p->eng_ppr     = (uint16_t)constrain(doc["engPpr"].as<int>(), 1, 240);
        if (doc["outPpr"].is<int>())  p->out_ppr     = (uint16_t)constrain(doc["outPpr"].as<int>(), 1, 240);
        if (doc["clEn"].is<int>())    p->cl_spc_enable = (uint8_t)(doc["clEn"].as<int>() ? 1 : 0);
        if (doc["clKp"].is<int>())    p->cl_spc_kp   = (uint16_t)constrain(doc["clKp"].as<int>(), 0, 1000);
        // Transmission variant: ratios + tooth-blend K switch live (g_trans) — no reboot.
        if (doc["transVariant"].is<int>()) p->trans_variant = (uint8_t)constrain(doc["transVariant"].as<int>(), 0, (int)TRANS_VARIANT_COUNT - 1);
        if (doc["tcStall"].is<int>())   p->tc_stall_mult_x100  = (uint16_t)constrain(doc["tcStall"].as<int>(), 100, 350);
        if (doc["tcCoupSr"].is<int>())  p->tc_coupling_sr_x100 = (uint16_t)constrain(doc["tcCoupSr"].as<int>(), 50, 100);
        if (doc["clPwr"].is<int>())     p->cl_pressure_enable  = (uint8_t)(doc["clPwr"].as<int>() ? 1 : 0);
        if (doc["pFull"].is<int>())     p->p_full_scale_mbar   = (uint16_t)constrain(doc["pFull"].as<int>(), 4000, 25000);
        { JsonArray ck = doc["clutchK"].as<JsonArray>(); JsonArray rs = doc["relSpring"].as<JsonArray>();
          if ((int)ck.size() >= 4) for (int i=0;i<4;i++) p->clutch_k_x100[i]     = (uint16_t)constrain(ck[i].as<int>(), 200, 8000);
          if ((int)rs.size() >= 4) for (int i=0;i<4;i++) p->release_spring_mbar[i] = (uint16_t)constrain(rs[i].as<int>(), 0, 5000); }
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
        engineProfile.applyTransVariant();   // push ratio/K change into g_trans live
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
    // ~33 Hz (30 ms gate). The live gauges and the 6 s rolling chart are perfectly
    // smooth at this rate, and it cuts WebSocket queue pressure ~3× vs the old 100 Hz
    // flood — high-rate telemetry was what overflowed a slow client's TX queue. The
    // high-fidelity per-shift data goes out separately as the columnar shift_trace.
    if (millis() - _last_broadcast_time >= 30) {
        buildAndSendTelemetryJSON();
        _last_broadcast_time = millis();
        // Persist adaptation on Core 0 (NVS can block 1-10ms). Flushes on a 60s timer
        // or when a P/N-entry/web edit forced it — not per shift (NVS wear).
        if (_adaptives) _adaptives->processFlush();
    }
    sendShiftTrace();     // one-shot when a shift trace is ready (not rate-gated)
    ws.cleanupClients();
}

// Serialize the high-rate per-shift datalog (captured by Core 1) and push it to the
// dashboard as one columnar `shift_trace` message. Core 1 won't touch the ring while
// `ready` is set, so reading it here is race-free; we clear `ready` when done.
void WebManager::sendShiftTrace() {
    if (!shiftTrace.ready) return;
    if (ws.count() == 0) { shiftTrace.ready = false; return; }   // no client → drop it
    uint16_t n = shiftTrace.count;

    JsonDocument doc;
    doc["type"] = "shift_trace";
    doc["cls"] = shiftTrace.shift_class; doc["pd"] = shiftTrace.pd_type;
    doc["from"] = shiftTrace.from_gear;  doc["to"] = shiftTrace.to_gear;  doc["n"] = n;
    JsonArray t = doc["t"].to<JsonArray>(), ph = doc["ph"].to<JsonArray>();
    JsonArray spc = doc["spc"].to<JsonArray>(), mpc = doc["mpc"].to<JsonArray>();
    JsonArray ratio = doc["ratio"].to<JsonArray>(), eng = doc["eng"].to<JsonArray>();
    JsonArray turb = doc["turb"].to<JsonArray>(), out = doc["out"].to<JsonArray>();
    JsonArray cle = doc["clErr"].to<JsonArray>(), fl = doc["fl"].to<JsonArray>();
    JsonArray onc = doc["onClutch"].to<JsonArray>(), offc = doc["offClutch"].to<JsonArray>();
    for (uint16_t i = 0; i < n; i++) {
        TraceSample &s = shiftTrace.s[i];
        t.add(s.t_ms); ph.add(s.phase); spc.add(s.spc); mpc.add(s.mpc);
        ratio.add(s.ratio_x1000); eng.add(s.eng); turb.add(s.turb); out.add(s.out);
        cle.add(s.cl_err_x1000); fl.add(s.flags);
        onc.add(s.on_clutch); offc.add(s.off_clutch);
    }
    String buf;
    serializeJson(doc, buf);
    ws.textAll(buf);
    shiftTrace.ready = false;   // consumed; Core 1 may capture the next shift
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
    doc["onClutch"]  = (int)telemetry.on_clutch_rpm;   // clutch-speed model (0 unless shifting)
    doc["offClutch"] = (int)telemetry.off_clutch_rpm;
    doc["tInput"]    = (int)telemetry.t_input_nm;      // input/turbine torque (engine × converter factor)

    char buffer[1024];
    size_t len = serializeJson(doc, buffer, sizeof(buffer));
    if (len >= sizeof(buffer) - 1) Serial.println("WARNING: Telemetry JSON truncated — increase buffer!");

    // Back-pressure aware send (NOT textAll). At a steady stream rate a client whose
    // TCP has briefly stalled (WiFi power-save, throttled background tab) would otherwise
    // overflow its 32-deep async queue; the lib then drops/forces it and the client's
    // staleness watchdog reconnects → the "flapping offline/online" seen on a slow PC
    // while a faster phone stayed live. Skipping a frame for a backed-up client lets it
    // catch up without dropping the socket; healthy clients are unaffected.
    for (auto& client : ws.getClients()) {
        if (client.status() == WS_CONNECTED && client.canSend())
            client.text(buffer, len);
    }
}
