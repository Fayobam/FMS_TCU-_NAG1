#pragma once
#include <ArduinoJson.h>
#include "ControlBridge.h"
// Returns nullptr only when every provided field has been validated.
const char* validateCommand(JsonDocument& doc, ControlCommand& out);
