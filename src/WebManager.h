// ============================================================================
// FILE: WebManager.h
// VERSION: 7.0 — async live dash (ESP32Async), work stays on Core 0
// ============================================================================
#pragma once
#include <Arduino.h>
#include <atomic>
#include <WiFi.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "TCU_Data.h"
#include "AdaptiveMemory.h"
#include "NetworkManager.h"

class WebManager {
    bool _atfSelectorRequested = false;
  private:
    AsyncWebServer server;
    AsyncWebSocket ws;
    DNSServer dns;
    tcu::NetworkManager network;
    bool _fs_ok = false;
    bool _inflight = false, _persistTune = false;
    uint32_t _replyClient = 0, _requestId = 0;
    std::atomic<uint32_t> _telemetryQueueSkips{0}, _telemetryOversize{0}, _serviceGapMaxMs{0};
    uint32_t _lastServiceMs = 0;
    uint32_t _lastCommandMs = 0;
    AdaptiveMemory* _adaptives;
    bool _dns_ok;
    uint32_t _last_telem_ms;
    uint32_t _last_cleanup_ms;
    uint8_t  _pend;            // Core-0 reply bits (profile/params/cells/dtcs)

    static const uint8_t PEND_PROFILE = 1;
    static const uint8_t PEND_PARAMS  = 2;
    static const uint8_t PEND_CELLS   = 4;
    static const uint8_t PEND_DTCS    = 8;

    void reply(uint32_t client, uint32_t requestId, bool ok, const char* message);
    static void sendDashboard(AsyncWebServerRequest *request);
    void wsSend(const char* buf, size_t len, bool bestEffort = false, bool telemetryPacket = false);
    void fillTelemetry(JsonDocument& doc);
    void pumpTelemetry();
    void pumpTrace();
    void pumpPending();
    void drainCmd();
    void handleCommand(JsonDocument& doc);
    void sendProfile();
    void sendParams();
    void sendCells();
    void sendDtcs();

  public:
    WebManager();
    void setAdaptiveMemory(AdaptiveMemory* adaptives);
    void begin();
    void update();   // Core 0 only: DNS, queued cmds, WS push, NVS
};
