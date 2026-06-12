// ============================================================================
// FILE: SpeedReader.h
// VERSION: 3.0
// UPDATES: Pulse-PERIOD measurement via MCPWM hardware capture (80MHz timestamps)
//          replaces PCNT pulse counting. Counting quantized to a fixed
//          60000/(PPR×window) rpm (±50 rpm at 24 PPR / 50 ms) at ALL speeds —
//          unusable for the 0.10 flare threshold and the 50 rpm TCC slip target.
//          Period measurement is sub-rpm across the whole range.
//
//          Per channel: ring buffer of edge-to-edge intervals, glitch rejection
//          (implausibly short intervals dropped in the ISR), full-revolution
//          rolling average (cancels tooth-spacing error exactly) capped by a
//          time window (bounds latency at low speed), open-interval clamp
//          (tracks hard deceleration between edges), zero-speed timeout.
//
//          Engine + output PPR come from EngineProfile (NVS, web-tunable) and
//          hot-apply — no recompile to change a trigger wheel. N2/N3 stay
//          compile-time: they are OEM 722.6 internals (60 teeth).
// ============================================================================
#pragma once
#include <Arduino.h>
#include "driver/mcpwm_cap.h"
#include "TCU_Data.h"

// --- SENSOR CALIBRATION CONSTANTS (gearbox-internal, fixed) ---
const float TEETH_N2 = 60.0f;          // OEM 722.6 Internal N2 Drum
const float TEETH_N3 = 60.0f;          // OEM 722.6 Internal N3 Drum
// Output + engine PPR are configurable via EngineProfile (web "Engine Profile" tab).

#define SR_RING_N        64            // intervals per channel (power of two)
#define SR_RING_MASK     (SR_RING_N - 1)
const uint32_t SR_AVG_WINDOW_US  = 100000; // max averaging span (bounds low-speed latency)
const float    SR_ZERO_FLOOR_RPM = 25.0f;  // below this we call it stopped (sets edge timeout)
const uint32_t SR_ZERO_TIMEOUT_MAX_US = 400000; // hard cap on the no-edge timeout

// Max plausible shaft speeds — intervals implying more are rejected as glitches.
const float SR_MAX_RPM_N2  = 8500.0f;
const float SR_MAX_RPM_N3  = 8500.0f;
const float SR_MAX_RPM_OUT = 9000.0f;  // output in 5th @ 6500 engine ≈ 7800
const float SR_MAX_RPM_ENG = 8000.0f;

struct SpeedChannel {
    // --- written by the capture ISR ---
    volatile uint32_t ring[SR_RING_N]; // closed intervals, capture-timer ticks
    volatile uint16_t head = 0;        // next write slot
    volatile uint16_t count = 0;       // valid entries (saturates at SR_RING_N)
    volatile uint32_t last_cap = 0;    // last accepted edge, capture ticks
    volatile bool     has_last = false;
    volatile int64_t  last_edge_us = 0;  // esp_timer time of last accepted edge
    volatile uint32_t edge_count = 0;    // monotonic accepted-edge counter (new-edge signal)
    uint32_t min_period_ticks = 1;     // glitch floor (from max plausible rpm)

    // --- main-loop config/state ---
    float    ppr = 1.0f;
    uint16_t full_rev_intervals = 1;   // min(ppr, SR_RING_N) — perfect tooth cancellation
    int64_t  zero_timeout_us = 250000;

    mcpwm_cap_channel_handle_t cap_chan = NULL;
};

class SpeedReader {
  private:
    uint8_t _pin_n2, _pin_n3, _pin_out, _pin_eng;

    mcpwm_cap_timer_handle_t _cap_timer_g0 = NULL;  // group 0: N2, N3, OUT
    mcpwm_cap_timer_handle_t _cap_timer_g1 = NULL;  // group 1: ENG
    uint32_t _cap_res_hz = 80000000;                // actual capture resolution (queried)

    SpeedChannel _n2, _n3, _out, _eng;

    uint16_t _last_eng_ppr = 0, _last_out_ppr = 0;  // hot-apply web PPR changes
    uint32_t _last_ratio_edges = 0;                 // last N2+N3+OUT edge sum acted on (B-4 gate)

    void initChannel(mcpwm_cap_timer_handle_t timer, SpeedChannel &ch, uint8_t pin);
    void configChannel(SpeedChannel &ch, float ppr, float max_rpm);
    float readChannelRPM(SpeedChannel &ch);
    float calculateTurbineRPM(float n2_rpm, float n3_rpm);

  public:
    SpeedReader(uint8_t pin_n2, uint8_t pin_n3, uint8_t pin_out, uint8_t pin_eng);
    void begin();
    void update();   // call every loop (1kHz); recomputes every tick for fine decel tracking
};
