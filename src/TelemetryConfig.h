#pragma once
#include <stdint.h>
// Shared snapshot and broadcast cadence. Graphics run in the browser.
// 20 Hz. The cost is small and measured: ControlSnapshot is 520 bytes, so the
// control task spends ~11 us per second copying it, and the JSON out is ~26 KB/s.
// Raising it further is blocked below, and would not help anyway: a shift lasts
// 400-600 ms, so even 30 Hz cannot resolve its phases. That is what the 2 ms
// shiftTrace recorder is for. This rate is for reading values, not for analysis.
constexpr uint32_t TCU_TELEMETRY_INTERVAL_MS = 50;
static_assert(TCU_TELEMETRY_INTERVAL_MS >= 50, "Keep telemetry at or below 20 Hz");

// Headroom for diagnostics, long fault strings and numeric formatting.
constexpr uint32_t TCU_TELEMETRY_BUFFER_BYTES = 3072;
