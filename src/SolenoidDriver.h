// ============================================================================
// FILE: SolenoidDriver.h
// VERSION: 2.0
// UPDATES: setLine/Shift/TCC now write back telemetry so the scheduler ramp
//          logic reads the REAL commanded value (fixes broken ramp).
// ============================================================================
#pragma once
#include <Arduino.h>

enum SolenoidState {
    STATE_OFF,
    STATE_KICKING,
    STATE_HOLDING
};

// ATSG standby duties (p.53). DRIVING: SPC de-energized (OFF), MPC on line schedule.
// PARK_NEUTRAL: SPC ~33% duty, MPC ~40% duty (pressure-% API → 67 / 60).
enum StandbyProfile : uint8_t { STANDBY_DRIVING, STANDBY_PARK_NEUTRAL };

struct RoutingSolenoid {
    uint8_t pin;
    SolenoidState state;
    TickType_t kick_start_tick;
};

class SolenoidDriver {
  private:
    uint8_t _mpc;
    uint8_t _spc;
    uint8_t _tcc;
    uint8_t _rp_lock;       // Reverse/Park interlock solenoid (plain digital output)

    RoutingSolenoid _y3; // 1-2 / 4-5 Shift Valve
    RoutingSolenoid _y4; // 3-4 Shift Valve
    RoutingSolenoid _y5; // 2-3 Shift Valve

    bool _y4_garage_owned = false;       // Y4 currently held by the garage pulse (not a 3-4 shift)
    bool _crank_active = false;          // Y3 boot/crank conditioning pulse in progress
    TickType_t _crank_start_tick = 0;

    void processRoutingSolenoid(RoutingSolenoid &sol);

  public:
    SolenoidDriver(uint8_t mpc, uint8_t spc, uint8_t tcc, uint8_t y3, uint8_t y4, uint8_t y5, uint8_t rp_lock);

    void begin();
    void update();

    void setLinePressure(uint8_t pct);
    void setShiftPressure(uint8_t pct);
    void setTCC(uint8_t pct);

    void fireShiftSolenoid(uint8_t pin);
    void stopShiftSolenoid(uint8_t pin);
    void stopAllShiftSolenoids();

    void setShiftLock(bool engaged);   // RP_LOCK: block lever travel into R/P while moving

    void setStandbyProfile(StandbyProfile p);  // SPC/MPC resting duties when not shifting
    void setGarageY4(bool pulsing);            // B2 counter-pressure pulse in Park / lever window
    void crankPulseY3();                       // ~400ms valve-body conditioning pulse at boot
};
