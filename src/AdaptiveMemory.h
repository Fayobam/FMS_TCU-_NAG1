// ============================================================================
// FILE: AdaptiveMemory.h
// VERSION: 7.0  (Adaptation v2 — class-indexed, torque-binned; ATSG spec §6)
// Replaces the 16×16 up/down tables with OEM-style adaptation categories:
//   AdaptCell[ShiftClass(4)][shift_idx(4)][torque_bin(4)]
// Each cell trims the phase profile: fill time (in 20ms cycles), fill pressure %,
// and torque-phase/catch apply %. Bins are captured at beginShift(); writes are
// gated on ATF temp and flushed on a 60s timer or on entry to P/N (not per shift).
// ============================================================================
#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "TCU_Data.h"

#define ADAPT_CLASSES 4   // ShiftClass: POWER_UP, COAST_UP, POWER_DOWN, COAST_DOWN
#define ADAPT_SHIFTS  4   // shift_idx 0-3 (1-2/2-3/3-4/4-5 up; 2-1/3-2/4-3/5-4 down)
#define ADAPT_TBINS   4   // torque bin 0-25 / 25-50 / 50-75 / 75-100 % of T_MAX

// Per-field clamps (spec §6)
#define FILL_T_CYC_MIN  (-5)
#define FILL_T_CYC_MAX  ( 5)   // ×20ms → ±100ms
#define FILL_P_TRIM_MIN (-15)
#define FILL_P_TRIM_MAX ( 15)
#define APPLY_TRIM_MIN  (-15)
#define APPLY_TRIM_MAX  ( 15)

struct AdaptCell {
    int8_t fill_t_cycles;   // ±5  (×20ms)
    int8_t fill_p_trim;     // ±15 (%)
    int8_t apply_trim;      // ±15 (%)  (torque-phase apply on up; catch on down)
};

class AdaptiveMemory {
  private:
    Preferences preferences;
    AdaptCell _cells[ADAPT_CLASSES][ADAPT_SHIFTS][ADAPT_TBINS];

    volatile uint8_t _dirty = 0;                 // one bit per ShiftClass
    volatile bool    _flush_now = false;         // forced flush (e.g. on P/N entry)
    portMUX_TYPE     _dirtyMux = portMUX_INITIALIZER_UNLOCKED;
    unsigned long    _last_flush_ms = 0;

    void loadDefaults();
    void saveClass(uint8_t sclass);

    static int8_t clampFillT(int v) { return (int8_t)constrain(v, FILL_T_CYC_MIN, FILL_T_CYC_MAX); }
    static int8_t clampFillP(int v) { return (int8_t)constrain(v, FILL_P_TRIM_MIN, FILL_P_TRIM_MAX); }
    static int8_t clampApply(int v) { return (int8_t)constrain(v, APPLY_TRIM_MIN, APPLY_TRIM_MAX); }

  public:
    AdaptiveMemory();
    void begin();

    // Read the trims for a cell (called at beginShift). Safe for out-of-range args.
    AdaptCell getCell(uint8_t sclass, uint8_t shift_idx, uint8_t tbin);

    // Apply one learning update for a completed shift. Deadband: clean shifts write
    // nothing. ATF gating is the caller's responsibility (it owns telemetry).
    void learn(uint8_t sclass, uint8_t shift_idx, uint8_t tbin, bool flare, bool harsh, bool bind);

    // Persistence (Core 0 only): flush dirty classes on the 60s timer or when forced.
    void processFlush();
    void requestFlush() { _flush_now = true; }   // Core 1 may call this (just sets a flag)

    // Web tuner access to the raw cell array (ADAPT_CLASSES*ADAPT_SHIFTS*ADAPT_TBINS cells).
    AdaptCell* cellsPtr() { return &_cells[0][0][0]; }
    int cellCount() { return ADAPT_CLASSES * ADAPT_SHIFTS * ADAPT_TBINS; }
    void markAllDirtyAndFlush();
};
