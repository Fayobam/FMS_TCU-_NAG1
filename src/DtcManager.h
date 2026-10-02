// ============================================================================
// FILE: DtcManager.h
// VERSION: 1.0
// Lightweight diagnostic trouble code store.
//
// OWNERSHIP RULE: the control task never touches the code arrays, and never takes a
// lock. It calls sample() once per tick, which is a handful of comparisons plus at
// most two atomic word writes, and trip() for one-shot events. The service task owns
// _count/_active/_last_ms outright, so there is no mutex anywhere in this class —
// diagnostics must not be able to stall a 1 kHz control loop, not even briefly.
//
// Transients are not lost to the slower service task: the control task detects rising
// edges itself against its own private previous mask and OR-s them into _pending_trips,
// so a fault lasting one millisecond is still counted. _condition_mask carries only
// the live "is it asserted right now" view for display.
// ============================================================================
#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include <atomic>

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
    DTC_ATF_CIRCUIT,             // moving in gear with no ATF measurement → sensor/wiring fault
    DTC_COUNT                    // NOTE: appending here re-seeds the persisted count array once
};

const char* dtcName(uint8_t code);

// One coherent copy of the whole store, so a reply cannot mix pre- and post-clear
// state. Cheap: the service task owns the arrays, so this is a plain copy.
struct DtcSnapshot {
    uint16_t count[DTC_COUNT];
    bool     active[DTC_COUNT];
    uint32_t last_ms[DTC_COUNT];
};

class DtcManager {
  private:
    Preferences prefs;
    // --- Service task only. No lock, because nothing else writes them. ---
    uint16_t _count[DTC_COUNT];     // saturating occurrence count, persisted
    bool     _active[DTC_COUNT];    // currently asserted (session only)
    uint32_t _last_ms[DTC_COUNT];   // last assertion time, millis (session only)
    bool     _dirty = false;
    unsigned long _last_flush_ms = 0;
    uint8_t  _active_count = 0;

    // --- Control task only. Private edge detector; never read by the other core. ---
    uint32_t _prev_sample_mask = 0;

    // --- The entire cross-core surface: three lock-free words. ---
    std::atomic<uint32_t> _condition_mask{0};  // live asserted set, for display
    std::atomic<uint32_t> _pending_trips{0};   // counted events, drained by the service task
    std::atomic<bool>     _clear_requested{false};
    static_assert(DTC_COUNT <= 32, "the masks above are 32-bit");

  public:
    void begin();
    void sample();      // CONTROL task, every tick: build the mask. No lock, no arrays.
    void trip(DtcCode c);  // either task: one-shot discrete event, atomic.
    void requestClear();   // either task: ask the service task to zero the store.
    void service();     // SERVICE task: drain edges, count, timestamp. Owns the arrays.
    void processFlush();// SERVICE task: persist counts (gated on nvsWriteSafe()).
    DtcSnapshot snapshot();
    uint8_t  activeCount() const { return _active_count; }
};

extern DtcManager dtcManager;
