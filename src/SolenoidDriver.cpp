// ============================================================================
// FILE: SolenoidDriver.cpp
// VERSION: 2.0
// ============================================================================
#include "SolenoidDriver.h"
#include "TCU_Data.h"   // needed for telemetry feedback writes

SolenoidDriver::SolenoidDriver(uint8_t mpc, uint8_t spc, uint8_t tcc, uint8_t y3, uint8_t y4, uint8_t y5, uint8_t rp_lock) {
    _mpc = mpc; _spc = spc; _tcc = tcc; _rp_lock = rp_lock;
    _y3.pin = y3; _y3.state = STATE_OFF;
    _y4.pin = y4; _y4.state = STATE_OFF;
    _y5.pin = y5; _y5.state = STATE_OFF;
}

void SolenoidDriver::begin() {
    ledcAttach(_mpc, 1000, 8);
    ledcAttach(_spc, 1000, 8);
    ledcAttach(_tcc, 100, 8);
    ledcAttach(_y3.pin, 1000, 8);
    ledcAttach(_y4.pin, 1000, 8);
    ledcAttach(_y5.pin, 1000, 8);

    // RP_LOCK is a simple on/off solenoid, not PWM — plain digital output.
    pinMode(_rp_lock, OUTPUT);
    setShiftLock(false);            // boot released = lever free (fail-safe)

    setLinePressure(0);
    setShiftPressure(0);
    setTCC(0);
    stopAllShiftSolenoids();
}

void SolenoidDriver::update() {
    processRoutingSolenoid(_y3);
    processRoutingSolenoid(_y4);
    processRoutingSolenoid(_y5);
}

void SolenoidDriver::processRoutingSolenoid(RoutingSolenoid &sol) {
    if (sol.state == STATE_KICKING) {
        TickType_t current_tick = xTaskGetTickCount();
        if ((current_tick - sol.kick_start_tick) >= (60 / portTICK_PERIOD_MS)) {
            ledcWrite(sol.pin, 85);     // ~33% hold current to save the 4-ohm coil
            sol.state = STATE_HOLDING;
        }
    }
}

void SolenoidDriver::fireShiftSolenoid(uint8_t requested_pin) {
    RoutingSolenoid *target = nullptr;
    if (requested_pin == _y3.pin) target = &_y3;
    else if (requested_pin == _y4.pin) target = &_y4;
    else if (requested_pin == _y5.pin) target = &_y5;

    if (target != nullptr && target->state == STATE_OFF) {
        ledcWrite(target->pin, 212);    // ~83% kick to snap valve open
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

void SolenoidDriver::startGarageShiftJiggle() {
    if (_y4.state == STATE_OFF) {
        ledcWrite(_y4.pin, 95);         // ~37% continuous buffer fill in P/N
        _y4.state = STATE_HOLDING;
    }
}

void SolenoidDriver::stopGarageShiftJiggle() {
    stopShiftSolenoid(_y4.pin);
}

// ------ INVERTED-LOGIC PRESSURE COMMANDS, now with telemetry feedback ------
void SolenoidDriver::setLinePressure(uint8_t pct) {
    pct = constrain(pct, 0, 100);
    ledcWrite(_mpc, map(pct, 0, 100, 255, 0));
    telemetry.line_pressure_pct = pct;          // <-- single source of truth
}

void SolenoidDriver::setShiftPressure(uint8_t pct) {
    pct = constrain(pct, 0, 100);
    ledcWrite(_spc, map(pct, 0, 100, 255, 0));
    telemetry.shift_pressure_pct = pct;         // <-- fixes the broken ramp
}

void SolenoidDriver::setTCC(uint8_t pct) {
    pct = constrain(pct, 0, 100);
    ledcWrite(_tcc, map(pct, 0, 100, 0, 255));  // normal logic
    telemetry.tcc_lockup_pct = pct;
}

// Reverse/Park interlock. engaged=true blocks the lever from leaving the forward
// range (i.e. entering N/R/P). Feature- and polarity-gated in TCU_Data.h because
// the mechanism is shifter-specific — verify before relying on it.
void SolenoidDriver::setShiftLock(bool engaged) {
    if (!ENABLE_RP_LOCK) engaged = false;             // no lock hardware → always released
    bool pin_high = RP_LOCK_ACTIVE_HIGH ? engaged : !engaged;
    digitalWrite(_rp_lock, pin_high ? HIGH : LOW);
}
