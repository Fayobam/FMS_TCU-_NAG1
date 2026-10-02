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
    _prev_sample_mask = 0;
    _condition_mask.store(0, std::memory_order_relaxed);
    _pending_trips.store(0, std::memory_order_relaxed);
    _clear_requested.store(false, std::memory_order_relaxed);
    _active_count = 0;
    prefs.begin("tcu_dtc", false);
    if (prefs.getBytes("count", _count, sizeof(_count)) != sizeof(_count))
        for (int i = 0; i < DTC_COUNT; i++) _count[i] = 0;   // blank/!match flash
    _last_flush_ms = millis();
}

// Continuous fault: count one occurrence on the rising edge only, so a fault that
// persists for seconds is logged once (not once per 1 kHz tick).
// ---------------------------------------------------------------------------
// CONTROL TASK. Called every tick. No lock, no array access, no millis() arithmetic
// beyond one subtraction: diagnostics must cost the 1 kHz loop effectively nothing.
// ---------------------------------------------------------------------------
void DtcManager::sample() {
    // On a bench the sensor lines are open by definition. Logging those four every
    // session would bury the real faults, so suppress them while test mode is on
    // (DTC_TEST_MODE already records that the unit was bench-driven).
    const bool sensors = !telemetry.test_mode;
    uint32_t m = 0;
    if (sensors && !telemetry.input_speed_trusted) m |= 1u << DTC_SPEED_N2N3_MISMATCH;
    if (sensors && !telemetry.speed_hw_ok)         m |= 1u << DTC_SPEED_HW_FAIL;
    if (sensors && !telemetry.tps_valid)           m |= 1u << DTC_TPS_RAIL;
    if (sensors && !telemetry.map_valid)           m |= 1u << DTC_MAP_RAIL;
    if (telemetry.is_limp_mode)                    m |= 1u << DTC_LIMP_SLIP;
    if (telemetry.reverse_abuse_active)            m |= 1u << DTC_REVERSE_AT_SPEED;
    // The ATF thermistor is in series with the P/N contact, so an open circuit IS
    // what P/N looks like and is not a fault. A forward range implies a closed
    // contact, so motion in gear with no reading is a sensor or wiring fault.
    if (sensors && telemetry.drive_engaged && telemetry.output_rpm > 200.0f
        && (millis() - telemetry.atf_last_valid_ms) > ATF_MEASUREMENT_TIMEOUT_MS)
                                                   m |= 1u << DTC_ATF_CIRCUIT;

    // Count the rising edges HERE, against a mask only this task touches. The service
    // task runs ~200x slower, so a fault that comes and goes inside one of its
    // iterations would otherwise never be counted at all.
    const uint32_t rising = m & ~_prev_sample_mask;
    _prev_sample_mask = m;
    if (rising) _pending_trips.fetch_or(rising, std::memory_order_relaxed);
    _condition_mask.store(m, std::memory_order_relaxed);
}

// One-shot discrete event: counted, never held active, so it cannot inflate
// activeCount() forever after a single occurrence. Safe from either task.
void DtcManager::trip(DtcCode c) {
    if (c >= DTC_COUNT) return;
    _pending_trips.fetch_or(1u << c, std::memory_order_relaxed);
}

void DtcManager::requestClear() { _clear_requested.store(true, std::memory_order_relaxed); }

// ---------------------------------------------------------------------------
// SERVICE TASK. Sole owner of the arrays below this line.
// ---------------------------------------------------------------------------
void DtcManager::service() {
    if (_clear_requested.exchange(false, std::memory_order_relaxed)) {
        for (int i = 0; i < DTC_COUNT; i++) { _count[i] = 0; _active[i] = false; _last_ms[i] = 0; }
        _pending_trips.store(0, std::memory_order_relaxed);   // do not resurrect cleared codes
        _dirty = true;
    }
    const uint32_t mask  = _condition_mask.load(std::memory_order_relaxed);
    const uint32_t trips = _pending_trips.exchange(0, std::memory_order_relaxed);
    const uint32_t now   = millis();
    uint8_t n = 0;
    for (uint8_t i = 0; i < DTC_COUNT; i++) {
        const uint32_t bit = 1u << i;
        if (trips & bit) {
            if (_count[i] < 0xFFFF) _count[i]++;
            _last_ms[i] = now;
            _dirty = true;
        }
        _active[i] = (mask & bit) != 0;
        if (_active[i]) n++;
    }
    _active_count = n;
}

void DtcManager::processFlush() {
    // Flash erase disables the instruction cache and stalls BOTH cores, whichever one
    // issued the write. A 1 kHz control loop cannot absorb that, so counts accumulate
    // in RAM until the car is demonstrably stopped in P/N. Accepted cost: a reboot
    // before the next stop loses the counts since the last flush.
    if (!nvsWriteSafe()) return;
    if (!_dirty || (millis() - _last_flush_ms < 10000)) return;
    prefs.putBytes("count", _count, sizeof(_count));
    _dirty = false;
    _last_flush_ms = millis();
}

DtcSnapshot DtcManager::snapshot() {
    DtcSnapshot s;
    memcpy(s.count, _count, sizeof(s.count));
    memcpy(s.active, _active, sizeof(s.active));
    memcpy(s.last_ms, _last_ms, sizeof(s.last_ms));
    return s;
}