// ============================================================================
// FILE: AdaptiveMemory.h
// VERSION: 6.0
// UPDATES:
//   - Uses shared computeLoad/loadToBin from TCU_Data.h (one load model).
//   - Stored modifiers clamped to a safe range to prevent int8_t wrap.
//   - evaluateShift now takes explicit shift_idx + is_upshift (no off-by-one
//     guessing from gear arithmetic).
// ============================================================================
#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "TCU_Data.h"

#define NUM_UPSHIFTS 4
#define NUM_DOWNSHIFTS 4
#define NUM_LOAD_BINS 16
#define NUM_RPM_BINS 16

// Pressure modifiers: applied to SPC% — tight range, small corrections only
#define PRESSURE_MOD_MIN -30
#define PRESSURE_MOD_MAX  60
// Timing modifiers: applied to ms durations — wider range needed for large clutch volumes
#define TIMING_MOD_MIN   -30
#define TIMING_MOD_MAX   120  // allows up to 60+120=180ms prefill for K2

class AdaptiveMemory {
  private:
    Preferences preferences;

    int8_t pressure_modifiers[NUM_UPSHIFTS][NUM_LOAD_BINS][NUM_RPM_BINS];
    int8_t prefill_modifiers[NUM_UPSHIFTS][NUM_LOAD_BINS][NUM_RPM_BINS];
    int8_t ds_pressure_modifiers[NUM_DOWNSHIFTS][NUM_LOAD_BINS][NUM_RPM_BINS];
    int8_t ds_timing_modifiers[NUM_DOWNSHIFTS][NUM_LOAD_BINS][NUM_RPM_BINS];

    uint8_t getLoadBin(float tps_pct, float map_kpa);
    uint8_t getRpmBin(float engine_rpm);
    void loadUltimateNag52Defaults();
    static int8_t clampPressure(int v);
    static int8_t clampTiming(int v);

    // Bit encoding: (is_upshift?0:8) + (is_pressure?0:4) + shift_idx (0-3)
    // Bits 0-3: upshift pressure, 4-7: upshift prefill, 8-11: ds pressure, 12-15: ds timing
    volatile uint16_t _dirty_mask = 0;

  public:
    AdaptiveMemory();
    void begin();

    int8_t getModifier(bool is_upshift, uint8_t shift_idx, bool is_pressure, float tps, float map_kpa, float rpm);

    // Explicit shift_idx + direction. flare/bind passed directly.
    // Sets dirty bits — does NOT write NVS. Call processDirtyTables() from Core 0.
    void evaluateShift(bool is_upshift, uint8_t shift_idx, float tps_pct, float map_kpa,
                       float engine_rpm, unsigned long shift_time_ms, bool flare, bool bind);

    // Process ONE pending NVS write per call. Call at 100Hz from Core 0 (WebManager).
    // Max flush latency for all 16 tables = 160ms — acceptable for adaptive learning.
    void processDirtyTables();

    int8_t* getTablePtr(bool is_upshift, uint8_t shift_index, bool is_pressure);
    void saveTable(bool is_upshift, uint8_t shift_index, bool is_pressure);
};
