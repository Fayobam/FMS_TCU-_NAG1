// ============================================================================
// FILE: DtcManager.h
// VERSION: 1.0
// Lightweight diagnostic trouble code store. Per-code saturating occurrence
// counter (persisted to NVS across reboots) + a session "active" flag + last-seen
// timestamp. Core 1 reports faults (poll() edges the telemetry flags; trip() logs
// one-shot events); Core 0 persists + serves them to the dashboard.
// ============================================================================
#pragma once
#include <Arduino.h>
#include <Preferences.h>

enum DtcCode : uint8_t {
    DTC_SPEED_N2N3_MISMATCH = 0, // N2/N3 disagree in gears 2/3/4 → bad speed sensor (BL-1)
    DTC_SPEED_HW_FAIL,           // MCPWM capture init failed → speed sensing disabled
    DTC_TPS_RAIL,                // TPS reading railed (open/short) → substituted (BL-16)
    DTC_MAP_RAIL,                // MAP reading railed → substituted (BL-16)
    DTC_LIMP_SLIP,               // transmission-protection limp (fatal slip)
    DTC_REVERSE_AT_SPEED,        // R selected while rolling forward
    DTC_OVERREV,                 // predictive overrev auto-upshift fired (one-shot event)
    DTC_LOOP_OVERRUN,            // 1 kHz physics loop missed its deadline (one-shot event)
    DTC_SHIFT_UNVERIFIED,        // shift hit its backstop without the ratio reaching target
    DTC_TEST_MODE,               // bench test mode was entered (provenance: this unit was bench-driven)
    DTC_COUNT                    // NOTE: appending here re-seeds the persisted count array once
};

const char* dtcName(uint8_t code);

// One coherent copy of the whole store. Core 1 writes these arrays while Core 0
// serializes them, so reading them field-by-field could mix pre- and post-clearAll
// state in a single reply. Take a snapshot, then serialize outside the lock.
struct DtcSnapshot {
    uint16_t count[DTC_COUNT];
    bool     active[DTC_COUNT];
    uint32_t last_ms[DTC_COUNT];
};

class DtcManager {
  private:
    Preferences prefs;
    uint16_t _count[DTC_COUNT];     // saturating occurrence count, persisted
    bool     _active[DTC_COUNT];    // currently asserted (session only)
    uint32_t _last_ms[DTC_COUNT];   // last assertion time, millis (session only)
    volatile bool _dirty = false;
    unsigned long _last_flush_ms = 0;
    // Guards the three arrays above. Core 1 trips/edges them, Core 0 persists and
    // serves them. Held only for bounded POD copies — never across an NVS write.
    portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;

  public:
    void begin();
    void setActive(DtcCode c, bool on);  // edge: count++ on false→true (continuous faults)
    void trip(DtcCode c);                // one-shot discrete event (counted, not held active)
    void poll();                         // edge the telemetry-derived faults — call on Core 1
    void clearAll();                     // zero counts + persist (web "clear codes")
    void processFlush();                 // Core 0: persist counts (throttled for NVS wear)

    DtcSnapshot snapshot();              // coherent copy for Core 0 serialization
    uint8_t  activeCount();
};

extern DtcManager dtcManager;
