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
    crankPulseY3();             // condition the 1-2/4-5 valve during the boot/crank window
}

void SolenoidDriver::update() {
    processRoutingSolenoid(_y3);
    processRoutingSolenoid(_y4);
    processRoutingSolenoid(_y5);

    // End the boot/crank Y3 conditioning pulse after ~400ms (ATSG p.53).
    if (_crank_active &&
        (xTaskGetTickCount() - _crank_start_tick) >= (400 / portTICK_PERIOD_MS)) {
        ledcWrite(_y3.pin, 0);
        _y3.state = STATE_OFF;
        _crank_active = false;
    }
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

// Garage Y4 pulse — pegs the B2 shift valve so the B2 double-piston counter-pressure
// softens N->D/R engagement (ATSG pp.53-54). Pulsed in Park and during the lever-movement
// window only; NOT held forever in N at rest. _y4_garage_owned guards against clobbering
// an active 3-4 shift that legitimately holds Y4.
void SolenoidDriver::setGarageY4(bool pulsing) {
    if (pulsing) {
        if (_y4.state == STATE_OFF) {
            ledcWrite(_y4.pin, 95);     // ~37% duty
            _y4.state = STATE_HOLDING;
            _y4_garage_owned = true;
        }
    } else if (_y4_garage_owned) {
        ledcWrite(_y4.pin, 0);
        _y4.state = STATE_OFF;
        _y4_garage_owned = false;
    }
}

// Standby SPC/MPC duties when NOT shifting (ATSG p.53 solenoid chart).
void SolenoidDriver::setStandbyProfile(StandbyProfile p) {
    if (p == STANDBY_PARK_NEUTRAL) {
        setShiftPressure(67);   // ~33% duty
        setLinePressure(60);    // ~40% duty
    } else {
        setShiftPressure(100);  // de-energized = OEM "OFF". MPC left to the line schedule.
    }
}

// One-shot ~40% duty pulse on Y3 at boot to condition the 1-2/4-5 valve during crank.
void SolenoidDriver::crankPulseY3() {
    ledcWrite(_y3.pin, 102);    // ~40% duty
    _y3.state = STATE_HOLDING;  // block an accidental fire during the pulse
    _y4_garage_owned = false;
    _crank_active = true;
    _crank_start_tick = xTaskGetTickCount();
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
