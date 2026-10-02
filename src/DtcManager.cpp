// ============================================================================
// FILE: DtcManager.cpp
// VERSION: 1.0
// ============================================================================
#include "DtcManager.h"
#include "TCU_Data.h"
#include <string.h>   // memcpy for the snapshot/flush copies

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
    "ATF SENSOR / CIRCUIT",
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
    portENTER_CRITICAL(&_mux);
    if (on && !_active[c]) {
        if (_count[c] < 0xFFFF) _count[c]++;
        _last_ms[c] = millis();
        _dirty = true;
    }
    _active[c] = on;
    portEXIT_CRITICAL(&_mux);
}

// Discrete one-shot event: counted + timestamped, NOT held active (so it doesn't
// inflate activeCount() forever after a single occurrence).
void DtcManager::trip(DtcCode c) {
    if (c >= DTC_COUNT) return;
    portENTER_CRITICAL(&_mux);
    if (_count[c] < 0xFFFF) _count[c]++;
    _last_ms[c] = millis();
    _dirty = true;
    portEXIT_CRITICAL(&_mux);
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
    // The ATF thermistor is in series with the P/N contact, so in a forward range the
    // circuit MUST be closed and a valid reading must arrive every few ms. Moving in
    // gear with no recent measurement is a sensor or wiring fault. It cannot false-
    // trigger on a legitimately open contact in P/N, because the car is not moving in
    // gear there. Worth a code because the failure is otherwise SILENT with a wired
    // selector: the lever supplies the range, and the missing temperature only
    // degrades fill pressure and backstop scaling. (In ATF-only mode the same fault
    // is loud — it reads as permanent P/N, so the mode never authorizes.)
    setActive(DTC_ATF_CIRCUIT, sensors && telemetry.drive_engaged
        && telemetry.output_rpm > 200.0f
        && (millis() - telemetry.atf_last_valid_ms) > ATF_MEASUREMENT_TIMEOUT_MS);
    telemetry.dtc_active_count = activeCount();
}

uint8_t DtcManager::activeCount() {
    uint8_t n = 0;
    portENTER_CRITICAL(&_mux);
    for (int i = 0; i < DTC_COUNT; i++) if (_active[i]) n++;
    portEXIT_CRITICAL(&_mux);
    return n;
}

void DtcManager::clearAll() {
    portENTER_CRITICAL(&_mux);
    for (int i = 0; i < DTC_COUNT; i++) { _count[i] = 0; _active[i] = false; _last_ms[i] = 0; }
    _dirty = true;
    portEXIT_CRITICAL(&_mux);
    telemetry.dtc_active_count = 0;
    for (int i = 0; i < DTC_COUNT; i++) { _count[i] = 0; _active[i] = false; _last_ms[i] = 0; }
    telemetry.dtc_active_count = 0;
    _dirty = true;
}

// Core 0 only (NVS can block). Persist no more than every 10 s to bound flash wear.
void DtcManager::processFlush() {
    if (!_dirty || (millis() - _last_flush_ms < 10000)) return;
    // Copy first: prefs.putBytes() blocks on flash and must never run under a
    // spinlock that the 1 kHz control task can contend for.
    uint16_t counts[DTC_COUNT];
    portENTER_CRITICAL(&_mux);
    memcpy(counts, _count, sizeof(counts));
    _dirty = false;
    portEXIT_CRITICAL(&_mux);
    prefs.putBytes("count", counts, sizeof(counts));
    _last_flush_ms = millis();
}


DtcSnapshot DtcManager::snapshot() {
    DtcSnapshot s;
    portENTER_CRITICAL(&_mux);
    memcpy(s.count, _count, sizeof(s.count));
    memcpy(s.active, _active, sizeof(s.active));
    memcpy(s.last_ms, _last_ms, sizeof(s.last_ms));
    portEXIT_CRITICAL(&_mux);
    return s;
}