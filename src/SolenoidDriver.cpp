#include "SolenoidDriver.h"

// 1. CONSTRUCTOR: Link the pins provided in TCU_Data.h to this class
SolenoidDriver::SolenoidDriver(uint8_t mpc, uint8_t spc, uint8_t tcc, uint8_t y3, uint8_t y4, uint8_t y5) {
    _mpc = mpc; _spc = spc; _tcc = tcc;
    
    _y3.pin = y3; _y3.state = STATE_OFF;
    _y4.pin = y4; _y4.state = STATE_OFF;
    _y5.pin = y5; _y5.state = STATE_OFF;
}

// 2. INITIALIZATION: Setup hardware timers
void SolenoidDriver::begin() {
    // Pressure Solenoids: 1000Hz, 8-bit resolution (0-255)
    ledcAttach(_mpc, 1000, 8);
    ledcAttach(_spc, 1000, 8);
    
    // Torque Converter Clutch: 100Hz, 8-bit resolution
    ledcAttach(_tcc, 100, 8);
    
    // Routing Solenoids: User requested 1000Hz, 8-bit resolution
    ledcAttach(_y3.pin, 1000, 8);
    ledcAttach(_y4.pin, 1000, 8);
    ledcAttach(_y5.pin, 1000, 8);
    
    // Safety Init: 0% target = 100% PWM Duty (Inverted MAX pressure)
    setLinePressure(0);
    setShiftPressure(0);
    setTCC(0);
    
    stopAllShiftSolenoids();
}

// 3. THE HEARTBEAT: Non-Blocking State Machine
// This runs in < 1 microsecond. It checks if any solenoid is "Kicking"
// and drops it to "Holding" if 60ms have passed.
void SolenoidDriver::update() {
    processRoutingSolenoid(_y3);
    processRoutingSolenoid(_y4);
    processRoutingSolenoid(_y5);
}

void SolenoidDriver::processRoutingSolenoid(RoutingSolenoid &sol) {
    if (sol.state == STATE_KICKING) {
        // Read the ESP32's internal RTOS clock
        TickType_t current_tick = xTaskGetTickCount();
        
        // Check if 60 milliseconds have passed
        if ((current_tick - sol.kick_start_tick) >= (60 / portTICK_PERIOD_MS)) {
            // Drop to holding current (~33% of 255 = 85) to save the 4-ohm coil
            ledcWrite(sol.pin, 85);
            sol.state = STATE_HOLDING; 
        }
    }
}

// 4. ROUTING COMMANDS (KICK & HOLD)
void SolenoidDriver::fireShiftSolenoid(uint8_t requested_pin) {
    RoutingSolenoid *target = nullptr;
    
    // Identify which solenoid was requested
    if (requested_pin == _y3.pin) target = &_y3;
    else if (requested_pin == _y4.pin) target = &_y4;
    else if (requested_pin == _y5.pin) target = &_y5;
    
    if (target != nullptr && target->state == STATE_OFF) {
        // Step 1: KICK (Snap the valve open with ~83% of 255 = 212)
        ledcWrite(target->pin, 212);
        
        // Step 2: Record the exact time so update() knows when to drop to HOLD
        target->kick_start_tick = xTaskGetTickCount();
        target->state = STATE_KICKING;
    }
}

void SolenoidDriver::stopShiftSolenoid(uint8_t requested_pin) {
    if (requested_pin == _y3.pin) { ledcWrite(_y3.pin, 0); _y3.state = STATE_OFF; }
    else if (requested_pin == _y4.pin) { ledcWrite(_y4.pin, 0); _y4.state = STATE_OFF; }
    else if (requested_pin == _y5.pin) { ledcWrite(_y5.pin, 0); _y5.state = STATE_OFF; }
}

void SolenoidDriver::stopAllShiftSolenoids() {
    stopShiftSolenoid(_y3.pin);
    stopShiftSolenoid(_y4.pin);
    stopShiftSolenoid(_y5.pin);
}

// 5. GARAGE SHIFT JIGGLE (Park/Neutral Prep)
void SolenoidDriver::startGarageShiftJiggle() {
    if (_y4.state == STATE_OFF) {
        // Apply ~37% continuous duty cycle (95 out of 255)
        ledcWrite(_y4.pin, 95); 
        _y4.state = STATE_HOLDING; // Bypass kick, jump straight to hold
    }
}

void SolenoidDriver::stopGarageShiftJiggle() {
    stopShiftSolenoid(_y4.pin);
}

// 6. ANALOG PRESSURE COMMANDS (INVERTED LOGIC)
// Software asks for 100% Pressure -> We send 0 PWM (Fully closed valve)
// Software asks for 0% Pressure   -> We send 255 PWM (Fully open valve to exhaust)
void SolenoidDriver::setLinePressure(uint8_t pct) {
    pct = constrain(pct, 0, 100);
    uint8_t duty = map(pct, 0, 100, 255, 0); 
    ledcWrite(_mpc, duty);
}

void SolenoidDriver::setShiftPressure(uint8_t pct) {
    pct = constrain(pct, 0, 100);
    uint8_t duty = map(pct, 0, 100, 255, 0); 
    ledcWrite(_spc, duty);
}

// TCC usually operates on normal logic (0% = 0 PWM)
void SolenoidDriver::setTCC(uint8_t pct) {
    pct = constrain(pct, 0, 100);
    uint8_t duty = map(pct, 0, 100, 0, 255); 
    ledcWrite(_tcc, duty);
}