#pragma once
#include <ArduinoJson.h>
#include "ControlBridge.h"

// Networking task only. The document must own any text copied from the snapshot,
// so it remains serializable after the caller's temporary snapshot is destroyed.
void fillTelemetryJson(JsonDocument& doc, const ControlSnapshot& snapshot,
                       uint32_t clients, uint32_t freeHeap, bool filesystemOk);
