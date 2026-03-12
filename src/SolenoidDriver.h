#pragma once
#include <Arduino.h>

// A custom tracker to know exactly what phase a routing solenoid is in
enum SolenoidState {
    STATE_OFF,
    STATE_KICKING,
    STATE_HOLDING
};

// A struct to bundle the pin, state, and timer for each routing solenoid
struct RoutingSolenoid {
    uint8_t pin;
    SolenoidState state;
    TickType_t kick_start_tick; // FreeRTOS timer for the 60ms delay
};

class SolenoidDriver {
  private:
    uint8_t _mpc;
    uint8_t _spc;
    uint8_t _tcc;
    
    RoutingSolenoid _y3; // 1-2 / 4-5 Shift Valve
    RoutingSolenoid _y4; // 3-4 Shift Valve
    RoutingSolenoid _y5; // 2-3 Shift Valve

    // Internal helper function
    void processRoutingSolenoid(RoutingSolenoid &sol);

  public:
    // Constructor
    SolenoidDriver(uint8_t mpc, uint8_t spc, uint8_t tcc, uint8_t y3, uint8_t y4, uint8_t y5);

    // Initialization
    void begin();
    
    // The "Heartbeat" function - MUST be called continuously in the main loop
    void update(); 

    // Analog Pressure Control (0% to 100%)
    void setLinePressure(uint8_t pct);
    void setShiftPressure(uint8_t pct);
    void setTCC(uint8_t pct);

    // Digital Routing Control
    void fireShiftSolenoid(uint8_t pin);
    void stopShiftSolenoid(uint8_t pin);
    void stopAllShiftSolenoids();
    
    // Special Garage Shift features
    void startGarageShiftJiggle();
    void stopGarageShiftJiggle();
};