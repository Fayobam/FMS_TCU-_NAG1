// ============================================================================
// FILE: AdaptiveMemory.h
// VERSION: 5.2
// UPDATES: Added missing declaration for loadUltimateNag52Defaults().
// ============================================================================
#pragma once
#include <Arduino.h>
#include <Preferences.h>

#define NUM_UPSHIFTS 4    // 1->2, 2->3, 3->4, 4->5
#define NUM_DOWNSHIFTS 4  // 2->1, 3->2, 4->3, 5->4
#define NUM_LOAD_BINS 16  // 0% to 150% Load (10% increments)
#define NUM_RPM_BINS 16   // 400 to 6800 RPM (400 RPM increments)

class AdaptiveMemory {
  private:
    Preferences preferences;

    // --- UPSHIFT MATRICES ---
    int8_t pressure_modifiers[NUM_UPSHIFTS][NUM_LOAD_BINS][NUM_RPM_BINS];
    int8_t prefill_modifiers[NUM_UPSHIFTS][NUM_LOAD_BINS][NUM_RPM_BINS];

    // --- DOWNSHIFT MATRICES ---
    int8_t ds_pressure_modifiers[NUM_DOWNSHIFTS][NUM_LOAD_BINS][NUM_RPM_BINS];
    int8_t ds_timing_modifiers[NUM_DOWNSHIFTS][NUM_LOAD_BINS][NUM_RPM_BINS];

    uint8_t getLoadBin(float tps_pct, float map_kpa);
    uint8_t getRpmBin(float engine_rpm);
    
    // --> THIS WAS MISSING! <--
    void loadUltimateNag52Defaults(); 

  public:
    AdaptiveMemory();
    void begin();
    
    // Core Evaluators
    int8_t getModifier(bool is_upshift, uint8_t shift_idx, bool is_pressure, float tps, float map_kpa, float rpm);
    void evaluateShift(uint8_t from_gear, uint8_t to_gear, float tps_pct, float map_kpa, float engine_rpm, unsigned long shift_time_ms, bool flare, bool bind);

    // Web Tuner API Methods
    int8_t* getTablePtr(bool is_upshift, uint8_t shift_index, bool is_pressure);
    void saveTable(bool is_upshift, uint8_t shift_index, bool is_pressure);
};