#pragma once
#include <stdint.h>
// Shared snapshot and broadcast cadence. Graphics run in the browser.
constexpr uint32_t TCU_TELEMETRY_INTERVAL_MS = 100;
static_assert(TCU_TELEMETRY_INTERVAL_MS >= 50, "Keep telemetry at or below 20 Hz");

// Headroom for diagnostics, long fault strings and numeric formatting.
constexpr uint32_t TCU_TELEMETRY_BUFFER_BYTES = 3072;
