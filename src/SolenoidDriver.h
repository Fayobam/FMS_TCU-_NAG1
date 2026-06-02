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

    RoutingSolenoid _y3; // 1-2 / 4-5 Shift Valve
    RoutingSolenoid _y4; // 3-4 Shift Valve
    RoutingSolenoid _y5; // 2-3 Shift Valve

    void processRoutingSolenoid(RoutingSolenoid &sol);

  public:
    SolenoidDriver(uint8_t mpc, uint8_t spc, uint8_t tcc, uint8_t y3, uint8_t y4, uint8_t y5);

    void begin();
    void update();

    void setLinePressure(uint8_t pct);
    void setShiftPressure(uint8_t pct);
    void setTCC(uint8_t pct);

    void fireShiftSolenoid(uint8_t pin);
    void stopShiftSolenoid(uint8_t pin);
    void stopAllShiftSolenoids();

    void startGarageShiftJiggle();
    void stopGarageShiftJiggle();
};
