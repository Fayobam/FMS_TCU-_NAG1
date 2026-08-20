// ============================================================================
// FILE: DtcManager.cpp
// VERSION: 1.0
// ============================================================================
#include "DtcManager.h"
#include "TCU_Data.h"

DtcManager dtcManager;

static const char* DTC_NAMES[DTC_COUNT] = {
    "SPEED N2/N3 MISMATCH",
    "SPEED HW INIT FAIL",
    "TPS SIGNAL RAILED",
    "MAP SIGNAL RAILED",
    "LIMP: FATAL SLIP",
    "REVERSE AT SPEED",
    "OVERREV UPSHIFT",
    "LOOP OVERRUN",
    "SHIFT UNVERIFIED",
    "TEST MODE USED",
};
const char* dtcName(uint8_t code) { return (code < DTC_COUNT) ? DTC_NAMES[code] : "?"; }

void DtcManager::begin() {
    for (int i = 0; i < DTC_COUNT; i++) { _active[i] = false; _last_ms[i] = 0; }
    prefs.begin("tcu_dtc", false);
    if (prefs.getBytes("count", _count, sizeof(_count)) != sizeof(_count))
        for (int i = 0; i < DTC_COUNT; i++) _count[i] = 0;   // blank/!match flash
    _last_flush_ms = millis();
}

// Continuous fault: count one occurrence on the rising edge only, so a fault that
// persists for seconds is logged once (not once per 1 kHz tick).
void DtcManager::setActive(DtcCode c, bool on) {
    if (c >= DTC_COUNT) return;
    if (on && !_active[c]) {
        if (_count[c] < 0xFFFF) _count[c]++;
        _last_ms[c] = millis();
        _dirty = true;
    }
    _active[c] = on;
}

// Discrete one-shot event: counted + timestamped, NOT held active (so it doesn't
// inflate activeCount() forever after a single occurrence).
void DtcManager::trip(DtcCode c) {
    if (c >= DTC_COUNT) return;
    if (_count[c] < 0xFFFF) _count[c]++;
    _last_ms[c] = millis();
    _dirty = true;
}

void DtcManager::poll() {
    // On a bench the sensor lines are open by definition. Logging those four every
    // session would bury the real faults, so suppress them while test mode is on
    // (DTC_TEST_MODE already records that the unit was bench-driven).
    bool sensors = !telemetry.test_mode;
    setActive(DTC_SPEED_N2N3_MISMATCH, sensors && !telemetry.input_speed_trusted);
    setActive(DTC_SPEED_HW_FAIL,       sensors && !telemetry.speed_hw_ok);
    setActive(DTC_TPS_RAIL,            sensors && !telemetry.tps_valid);
    setActive(DTC_MAP_RAIL,            sensors && !telemetry.map_valid);
    setActive(DTC_LIMP_SLIP,            telemetry.is_limp_mode);
    setActive(DTC_REVERSE_AT_SPEED,     telemetry.reverse_abuse_active);
    telemetry.dtc_active_count = activeCount();
}

uint8_t DtcManager::activeCount() const {
    uint8_t n = 0;
    for (int i = 0; i < DTC_COUNT; i++) if (_active[i]) n++;
    return n;
}

void DtcManager::clearAll() {
    for (int i = 0; i < DTC_COUNT; i++) { _count[i] = 0; _active[i] = false; _last_ms[i] = 0; }
    telemetry.dtc_active_count = 0;
    _dirty = true;
}

// Core 0 only (NVS can block). Persist no more than every 10 s to bound flash wear.
void DtcManager::processFlush() {
    if (!_dirty || (millis() - _last_flush_ms < 10000)) return;
    prefs.putBytes("count", _count, sizeof(_count));
    _dirty = false;
    _last_flush_ms = millis();
}
