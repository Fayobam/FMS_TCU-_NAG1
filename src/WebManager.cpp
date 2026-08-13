// ============================================================================
// FILE: WebManager.cpp
// VERSION: 7.0
// UPDATES: Added "limp_reset" command + safety/load fields to telemetry JSON.
// ============================================================================
#include "WebManager.h"
#include "EngineProfile.h"
#include "TuneOverlay.h"
#include "DtcManager.h"

// Telemetry broadcast period. Core 0 only — fully decoupled from the 1 kHz shift
// physics on Core 1 — so this is bounded by WiFi/client, not by gear shifting. The
// back-pressure-aware send keeps a slow client from overflowing (it skips a frame),
// so ~60 Hz is safe and feeds high-refresh phone screens nicely.
static const uint32_t TELEMETRY_INTERVAL_MS = 16;

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

    // three.js (r148 UMD) for the 3D tab — served from SPIFFS (offline softAP, no CDN).
    // Lazy-loaded by the page only when the 3D tab is first opened. Cache hard: the
    // file only changes on a filesystem re-flash.
    server.on("/three.min.js", HTTP_GET, [](AsyncWebServerRequest *request){
        AsyncWebServerResponse *r = request->beginResponse(SPIFFS, "/three.min.js", "application/javascript");
        r->addHeader("Cache-Control", "max-age=31536000, immutable");
        request->send(r);
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

    // --- Diagnostic trouble codes ---
    if (cmd == "get_dtcs") {
        JsonDocument resp;
        resp["type"] = "dtc_data";
        JsonArray arr = resp["dtcs"].to<JsonArray>();
        for (uint8_t i = 0; i < DTC_COUNT; i++) {
            JsonObject o = arr.add<JsonObject>();
            o["code"]   = i;
            o["name"]   = dtcName(i);
            o["count"]  = dtcManager.count(i);
            o["active"] = dtcManager.active(i);
            o["lastMs"] = dtcManager.lastMs(i);
        }
        char buffer[1024];
        size_t n = serializeJson(resp, buffer, sizeof(buffer));
        ws.textAll(buffer, n);
        return;
    }
    if (cmd == "clear_dtcs") {
        dtcManager.clearAll();
        Serial.println("DTCs cleared via web.");
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
        resp["kmhRpm"] = p->kmh_per_outrpm;
        resp["transVariant"] = p->trans_variant;   // 0=small NAG, 1=big NAG
        resp["tcStall"] = p->tc_stall_mult_x100; resp["tcCoupSr"] = p->tc_coupling_sr_x100;
        resp["clPwr"] = p->cl_pressure_enable; resp["pFull"] = p->p_full_scale_mbar;
        resp["clSpeed"] = p->cl_speed_transitions;
        resp["coefStat"] = p->coef_stationary; resp["coefRel"] = p->coef_releasing;
        resp["coefCold"] = p->coef_apply_cold; resp["coefHot"] = p->coef_apply_hot;
        JsonArray af = resp["applyFric"].to<JsonArray>();
        JsonArray rf = resp["relFric"].to<JsonArray>();
        JsonArray as = resp["applySpring"].to<JsonArray>();
        JsonArray rs = resp["relSpring"].to<JsonArray>();
        for (int i = 0; i < 4; i++) {
            af.add(p->apply_friction[i]); rf.add(p->release_friction[i]);
            as.add(p->apply_spring_mbar[i]); rs.add(p->release_spring_mbar[i]);
        }
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
        if (doc["kmhRpm"].is<float>()) p->kmh_per_outrpm = constrain(doc["kmhRpm"].as<float>(), 0.001f, 1.0f);
        // Transmission variant: ratios + tooth-blend K switch live (g_trans) — no reboot.
        // On an actual CHANGE, load that gearbox's physics clutch-model defaults FIRST, so a
        // dropdown-only change (no clutch arrays sent) lands on the variant presets; a full
        // Flash that also changed the variant then overrides with the dashboard's values below.
        if (doc["transVariant"].is<int>()) {
            uint8_t newv = (uint8_t)constrain(doc["transVariant"].as<int>(), 0, (int)TRANS_VARIANT_COUNT - 1);
            if (newv != p->trans_variant) { p->trans_variant = newv; engineProfile.seedClutchModelForVariant(newv); }
        }
        if (doc["tcStall"].is<int>())   p->tc_stall_mult_x100  = (uint16_t)constrain(doc["tcStall"].as<int>(), 100, 350);
        if (doc["tcCoupSr"].is<int>())  p->tc_coupling_sr_x100 = (uint16_t)constrain(doc["tcCoupSr"].as<int>(), 50, 100);
        if (doc["clPwr"].is<int>())     p->cl_pressure_enable  = (uint8_t)(doc["clPwr"].as<int>() ? 1 : 0);
        if (doc["clSpeed"].is<int>())   p->cl_speed_transitions = (uint8_t)(doc["clSpeed"].as<int>() ? 1 : 0);
        if (doc["pFull"].is<int>())     p->p_full_scale_mbar   = (uint16_t)constrain(doc["pFull"].as<int>(), 4000, 25000);
        if (doc["coefStat"].is<int>())  p->coef_stationary = (uint8_t)constrain(doc["coefStat"].as<int>(), 50, 255);
        if (doc["coefRel"].is<int>())   p->coef_releasing  = (uint8_t)constrain(doc["coefRel"].as<int>(), 50, 255);
        if (doc["coefCold"].is<int>())  p->coef_apply_cold = (uint8_t)constrain(doc["coefCold"].as<int>(), 50, 255);
        if (doc["coefHot"].is<int>())   p->coef_apply_hot  = (uint8_t)constrain(doc["coefHot"].as<int>(), 50, 255);
        { JsonArray af = doc["applyFric"].as<JsonArray>(); JsonArray rf = doc["relFric"].as<JsonArray>();
          JsonArray as = doc["applySpring"].as<JsonArray>(); JsonArray rs = doc["relSpring"].as<JsonArray>();
          if ((int)af.size() >= 4) for (int i=0;i<4;i++) p->apply_friction[i]   = (uint16_t)constrain(af[i].as<int>(), 500, 12000);
          if ((int)rf.size() >= 4) for (int i=0;i<4;i++) p->release_friction[i] = (uint16_t)constrain(rf[i].as<int>(), 500, 12000);
          if ((int)as.size() >= 4) for (int i=0;i<4;i++) p->apply_spring_mbar[i]   = (uint16_t)constrain(as[i].as<int>(), 0, 5000);
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

    // ------------------------------------------------------------------------
    // PARAMETER REGISTRY (generic, schema-driven — see TuneOverlay.h).
    // param.list  -> schema + current values for every registered parameter
    // param.set   -> {id,row,col,val}, range-checked in TuneOverlay::paramSet
    // param.persist / param.reset
    // Adding a parameter needs NO change here or in the dashboard JS.
    // ------------------------------------------------------------------------
    if (cmd == "param.list") {
        // Sent one parameter per frame: the line-pressure table alone is 80 cells,
        // and a single document with every schema + values would blow the buffer.
        uint8_t n = TuneOverlay::paramCount();
        for (uint8_t i = 0; i < n; i++) {
            const ParamDesc& p = TuneOverlay::paramDesc(i);
            JsonDocument resp;
            resp["type"] = "param";
            resp["idx"]  = i;
            resp["n"]    = n;
            resp["id"]   = p.id;
            resp["name"] = p.name;
            resp["group"]= p.group;
            resp["kind"] = (uint8_t)p.kind;
            resp["min"]  = p.vmin;
            resp["max"]  = p.vmax;
            resp["rows"] = p.rows;
            resp["cols"] = p.cols;
            resp["unit"] = p.unit;
            resp["help"] = p.help;
            JsonArray v = resp["v"].to<JsonArray>();
            for (uint8_t r = 0; r < p.rows; r++)
                for (uint8_t c = 0; c < p.cols; c++) v.add(tuneOverlay.paramGet(i, r, c));
            char buf[1600];
            size_t len = serializeJson(resp, buf, sizeof(buf));
            if (len < sizeof(buf) - 1) ws.textAll(buf, len);
            else Serial.printf("param.list: '%s' truncated — shorten its help text\n", p.id);
        }
        return;
    }
    else if (cmd == "param.set") {
        uint8_t idx = (uint8_t)doc["idx"].as<int>();
        uint8_t row = (uint8_t)doc["row"].as<int>();
        uint8_t col = (uint8_t)doc["col"].as<int>();
        int16_t val = (int16_t)doc["val"].as<int>();
        if (!tuneOverlay.paramSet(idx, row, col, val))
            Serial.printf("param.set rejected: idx=%u r=%u c=%u v=%d\n", idx, row, col, val);
        return;   // live immediately; call param.persist to make it survive a reboot
    }
    else if (cmd == "param.persist") {
        tuneOverlay.save();
        Serial.println("Tune overlay persisted to NVS.");
        return;
    }
    else if (cmd == "param.reset") {
        tuneOverlay.resetToDefaults();
        Serial.println("Tune overlay reset to compile-time defaults.");
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
    // ~60 Hz. Back-pressure (the canSend() check in buildAndSendTelemetryJSON) is what
    // makes the higher rate safe — the old 100 Hz flood overflowed slow clients because
    // there was NO back-pressure; now a backed-up client just skips a frame. The 6 s
    // chart redraws on requestAnimationFrame, so it's already smooth at the screen's
    // full refresh; this just feeds it denser points + snappier gauges.
    if (millis() - _last_broadcast_time >= TELEMETRY_INTERVAL_MS) {
        _last_broadcast_time = millis();
        buildAndSendTelemetryJSON();
        // Persist adaptation on Core 0 (NVS can block 1-10ms). Flushes on a 60s timer
        // or when a P/N-entry/web edit forced it — not per shift (NVS wear).
        if (_adaptives) _adaptives->processFlush();
        ws.cleanupClients();
    }
    sendShiftTrace();     // one-shot when a shift trace is ready (not rate-gated)
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
    doc["tgt"]       = telemetry.target_gear;   // for the 3D gearbox transition view
    doc["mode"]      = telemetry.drive_mode;
    doc["modeName"]  = DRIVE_MODES[telemetry.drive_mode <= 4 ? telemetry.drive_mode : 0].name;
    doc["engRpm"]    = telemetry.engine_rpm;
    doc["turbRpm"]   = telemetry.turbine_rpm;
    doc["outRpm"]    = telemetry.output_rpm;
    doc["tps"]       = telemetry.tps_pct;
    doc["map"]       = telemetry.map_kpa;
    doc["mpc"]       = telemetry.line_pressure_pct;
    doc["spc"]       = telemetry.shift_pressure_pct;
    doc["shiftTime"] = telemetry.last_shift_time_ms;
    doc["ratio"]     = telemetry.live_ratio;
    doc["kmh"]       = telemetry.road_kmh;
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
    doc["tqCut"]     = telemetry.torque_cut_active;     // rusEFI shift-retard window asserted

    // Health / fault flags for the dashboard warning panel.
    doc["dtcN"]      = telemetry.dtc_active_count;
    doc["spdHwOk"]   = telemetry.speed_hw_ok;
    doc["inTrust"]   = telemetry.input_speed_trusted;
    doc["tpsOk"]     = telemetry.tps_valid;
    doc["mapOk"]     = telemetry.map_valid;

    char buffer[1536];
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
