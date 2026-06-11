// ============================================================================
// FILE: WebManager.cpp
// VERSION: 7.0
// UPDATES: Added "limp_reset" command + safety/load fields to telemetry JSON.
// ============================================================================
#include "WebManager.h"

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

    if (_adaptives == nullptr) return;

    uint8_t shift_idx = doc["shift"];
    String type = doc["type"];        // "p" or "f"
    bool is_up = (doc["dir"] == "up");

    if (cmd == "get_table") {
        int8_t* table = _adaptives->getTablePtr(is_up, shift_idx, (type == "p"));
        JsonDocument responseDoc;
        responseDoc["type"] = "table_data";
        JsonArray arr = responseDoc["data"].to<JsonArray>();
        for (int i = 0; i < 256; i++) arr.add(table[i]);
        char buffer[2048];
        size_t resLen = serializeJson(responseDoc, buffer);
        ws.textAll(buffer, resLen);
    }
    else if (cmd == "set_table") {
        bool is_pressure = (type == "p");
        int lo = is_pressure ? PRESSURE_MOD_MIN : TIMING_MOD_MIN;
        int hi = is_pressure ? PRESSURE_MOD_MAX : TIMING_MOD_MAX;
        int8_t* table = _adaptives->getTablePtr(is_up, shift_idx, is_pressure);
        JsonArray arr = doc["data"].as<JsonArray>();
        // Clamp on ingest: an out-of-range cell from the web would otherwise wrap
        // int8 (e.g. 200 -> -56) and defeat the per-type modifier limits.
        for (int i = 0; i < 256; i++) table[i] = (int8_t)constrain(arr[i].as<int>(), lo, hi);
        _adaptives->saveTable(is_up, shift_idx, is_pressure);
        Serial.println("Flash Memory Updated via Web Tuner!");
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
        // Flush one pending NVS write per cycle (deferred from Core 1 evaluateShift).
        // Core 0 only — preferences.putBytes() can block 1-10ms, unsafe on physics loop.
        if (_adaptives) _adaptives->processDirtyTables();
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

    char buffer[1024];
    size_t len = serializeJson(doc, buffer, sizeof(buffer));
    if (len >= sizeof(buffer) - 1) Serial.println("WARNING: Telemetry JSON truncated — increase buffer!");
    ws.textAll(buffer, len);
}
