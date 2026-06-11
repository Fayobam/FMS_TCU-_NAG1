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
    float _output_rpm_at_catch_start;   // DS_CATCH bind metric: isolate clutch decel from braking
    unsigned long _catch_start_ms;
    float _ds_baseline_decel_rate;      // output rpm/ms decel measured BEFORE catch (vehicle/braking)
    bool  _prev_pn_raw;             // edge-detect for garage shift trigger
    unsigned long _engage_grace_until_ms; // suppress slip-limp during D-engagement sync
    char  _prev_prnd;              // edge-detect for reverse selection
    bool  _legit_reverse;         // R was selected while stopped → genuine reverse, allow any speed
    uint8_t _shift_load_bin;      // load/RPM bins captured at shift INITIATION (so adaptive
    uint8_t _shift_rpm_bin;       // learning writes the cell that actually caused the event)

    // TPS rate-of-change torque anticipation
    float         _prev_tps;
    bool          _high_torque_mode;
    unsigned long _ht_release_start_ms;  // 0 = not in cooldown

    // Performance bias: floor raised, boost-zone bins (4-8) pushed hard.
    // Adaptive pulls back 2% per bind event — start firm, let it learn down.
    const uint8_t HOLDING_PRESSURE_MAP[5][16] = {
        { 20, 24, 30, 40, 52, 65, 78, 88, 95, 100, 100, 100, 100, 100, 100, 100 }, // G1
        { 22, 26, 32, 42, 55, 68, 80, 90, 98, 100, 100, 100, 100, 100, 100, 100 }, // G2
        { 24, 28, 35, 45, 58, 72, 84, 94, 100, 100, 100, 100, 100, 100, 100, 100 }, // G3
        { 28, 32, 40, 50, 62, 76, 88, 97, 100, 100, 100, 100, 100, 100, 100, 100 }, // G4
        { 38, 45, 55, 65, 78, 90, 98, 100, 100, 100, 100, 100, 100, 100, 100, 100 } // G5
    };

    void calculateLinePressure();
    void calculateLiveRatio();
    void updateTCC();
    void checkSafetyShifts();
    void checkLimpMode(float target_ratio);
    void checkTpsROC();                       // TPS rate-of-change torque anticipation
    bool checkReverseInhibit();               // RP_LOCK + R-while-moving failsafe (returns true if it owns outputs)
    bool isForwardRange();                    // prnd is one of D/4/3/2/1
    void updateStandbyAndGarage();            // SPC/MPC standby duties + Y4 garage window (not shifting)
    bool beginShift(uint8_t target_gear, bool is_upshift, const char* source);
    float getTargetRatio(uint8_t gear);
    uint8_t getRoutingSolenoidForShift(uint8_t from_gear, uint8_t to_gear);

  public:
    ShiftScheduler(SolenoidDriver* solenoids, AdaptiveMemory* adaptives);
    void begin();
    void update();
};
