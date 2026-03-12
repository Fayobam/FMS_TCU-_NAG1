// ============================================================================
// FILE: WebManager.h
// VERSION: 4.0
// ============================================================================
#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "TCU_Data.h"
#include "AdaptiveMemory.h"

class WebManager {
  private:
    AsyncWebServer server;
    AsyncWebSocket ws;
    AdaptiveMemory* _adaptives;
    unsigned long _last_broadcast_time;

    void buildAndSendTelemetryJSON();
    void handleWebSocketMessage(void *arg, uint8_t *data, size_t len);

  public:
    WebManager();
    void setAdaptiveMemory(AdaptiveMemory* adaptives);
    void begin();
    void broadcastTelemetry(); 
};