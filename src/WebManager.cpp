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
                data[len] = 0; this->handleWebSocketMessage(arg, data, len);
            }
        }
    });

    server.addHandler(&ws);
    server.begin();
}

void WebManager::handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, (char*)data);
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
        int8_t* table = _adaptives->getTablePtr(is_up, shift_idx, (type == "p"));
        JsonArray arr = doc["data"].as<JsonArray>();
        for (int i = 0; i < 256; i++) table[i] = (int8_t)arr[i].as<int>();
        _adaptives->saveTable(is_up, shift_idx, (type == "p"));
        Serial.println("Flash Memory Updated via Web Tuner!");
    }
}

void WebManager::broadcastTelemetry() {
    if (millis() - _last_broadcast_time >= 100) { buildAndSendTelemetryJSON(); _last_broadcast_time = millis(); }
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

    // --- NEW: safety + limp status ---
    doc["limp"]      = telemetry.is_limp_mode;
    doc["limpReason"]= telemetry.limp_mode_reason;
    doc["safety"]    = telemetry.last_safety_event;
    doc["atfTemp"]   = telemetry.atf_temp_c;

    char buffer[640];
    size_t len = serializeJson(doc, buffer);
    ws.textAll(buffer, len);
}
