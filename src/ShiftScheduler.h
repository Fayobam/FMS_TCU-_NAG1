// ============================================================================
// FILE: ShiftScheduler.h
// VERSION: 6.0
// UPDATES:
//   - Added auto-safety layer (overrev upshift + lug downshift).
//   - Added flare detection during upshift inertia phase.
//   - Centralised shift initiation in beginShift() to remove duplicated,
//     off-by-one trigger code.
//   - Limp mode now has a deliberate reset path and load-aware threshold.
// ============================================================================
#pragma once
#include <Arduino.h>
#include "TCU_Data.h"
#include "SolenoidDriver.h"
#include "AdaptiveMemory.h"

enum ShiftPhase {
    PHASE_CRUISING,
    PHASE_PREFILL,
    PHASE_OVERLAP,
    PHASE_INERTIA,
    PHASE_DS_RELEASE,
    PHASE_DS_SYNC,
    PHASE_DS_CATCH,
    PHASE_COMPLETION
};

class ShiftScheduler {
  private:
    SolenoidDriver* _solenoids;
    AdaptiveMemory* _adaptives;

    ShiftPhase _current_phase;
    TickType_t _phase_start_tick;
    uint8_t _active_routing_pin;
    bool _is_upshift;
    uint8_t _active_shift_idx;      // adaptive table index for THIS shift

    int8_t _current_pressure_mod;
    int8_t _current_timing_mod;
    unsigned long _shift_stopwatch_start;
    float _turbine_rpm_at_shift_start;
    float _output_rpm_at_shift_start;
    float _ratio_at_overlap_start;  // for flare detection

    const uint8_t HOLDING_PRESSURE_MAP[5][16] = {
        { 12, 14, 16, 20, 26, 35, 48, 62, 75, 85, 92, 98, 100, 100, 100, 100 },
        { 15, 17, 20, 24, 30, 42, 55, 68, 80, 90, 95, 100, 100, 100, 100, 100 },
        { 18, 20, 24, 28, 38, 50, 62, 75, 88, 95, 100, 100, 100, 100, 100, 100 },
        { 22, 25, 28, 35, 45, 58, 72, 85, 95, 100, 100, 100, 100, 100, 100, 100 },
        { 30, 35, 40, 48, 60, 75, 88, 98, 100, 100, 100, 100, 100, 100, 100, 100 }
    };

    void calculateLinePressure();
    void calculateLiveRatio();
    void updateTCC();
    void checkSafetyShifts();                 // NEW: overrev / lug protection
    void checkLimpMode(float target_ratio);   // factored out, load-aware
    bool beginShift(uint8_t target_gear, bool is_upshift, const char* source);
    float getTargetRatio(uint8_t gear);
    uint8_t getRoutingSolenoidForShift(uint8_t from_gear, uint8_t to_gear);

  public:
    ShiftScheduler(SolenoidDriver* solenoids, AdaptiveMemory* adaptives);
    void begin();
    void update();
};
