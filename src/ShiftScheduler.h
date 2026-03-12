// ============================================================================
// FILE: ShiftScheduler.h
// VERSION: 5.1
// UPDATES: Integrated Ultimate-NAG52 OEM EGS52 Exponential Holding Curves.
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
    
    int8_t _current_pressure_mod;
    int8_t _current_timing_mod;
    unsigned long _shift_stopwatch_start;
    float _turbine_rpm_at_shift_start;
    float _output_rpm_at_shift_start;

    // --- EGS52 EXPONENTIAL HOLDING PRESSURE MAP (W5A330) ---
    // Derived from Ultimate-NAG52 OEM Torque/Pressure translation logic.
    // 0% <----------------------------- LOAD -------------------------------> 150%
    const uint8_t HOLDING_PRESSURE_MAP[5][16] = {
        { 12, 14, 16, 20, 26, 35, 48, 62, 75, 85, 92, 98, 100, 100, 100, 100 }, // 1st 
        { 15, 17, 20, 24, 30, 42, 55, 68, 80, 90, 95, 100, 100, 100, 100, 100 }, // 2nd 
        { 18, 20, 24, 28, 38, 50, 62, 75, 88, 95, 100, 100, 100, 100, 100, 100 }, // 3rd
        { 22, 25, 28, 35, 45, 58, 72, 85, 95, 100, 100, 100, 100, 100, 100, 100 }, // 4th
        { 30, 35, 40, 48, 60, 75, 88, 98, 100, 100, 100, 100, 100, 100, 100, 100 }  // 5th
    };
    
    void calculateLinePressure();
    void calculateLiveRatio();
    void updateTCC(); // NEW: TCC Dynamic Slip Controller
    float getTargetRatio(uint8_t gear);
    uint8_t getRoutingSolenoidForShift(uint8_t from_gear, uint8_t to_gear);

  public:
    ShiftScheduler(SolenoidDriver* solenoids, AdaptiveMemory* adaptives);
    void begin();
    void update();
};