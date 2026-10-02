// ============================================================================
// FILE: InputManager.cpp
// VERSION: 10.0
// ============================================================================
#include "InputManager.h"
#include "EngineProfile.h"

InputManager::InputManager(uint8_t temp_sensor_pin, uint8_t tps_pin, uint8_t map_pin) {
    _temp_sensor_pin = temp_sensor_pin;
    _tps_pin = tps_pin;
    _map_pin = map_pin;
    _last_known_temp_c = 40.0f;
    _tps_filtered = 0.0f;
    _map_filtered = 100.0f;
    _prnd_candidate_code = 0xFF;
    _prnd_stable_ms = 0;
    _paddle_up_prev = false;
    _paddle_down_prev = false;
    _last_paddle_up_time = 0;
    _last_paddle_down_time = 0;
    _adc_phase = 0;
}

void InputManager::begin() {
    pinMode(_temp_sensor_pin, INPUT);
    pinMode(_tps_pin, INPUT);
    pinMode(_map_pin, INPUT);

    pinMode(PIN_SHIFT_A, INPUT_PULLDOWN);
    pinMode(PIN_SHIFT_B, INPUT_PULLDOWN);
    pinMode(PIN_SHIFT_C, INPUT_PULLDOWN);
    pinMode(PIN_SHIFT_D, INPUT_PULLDOWN);

    pinMode(PIN_PADDLE_UP, INPUT_PULLDOWN);
    pinMode(PIN_PADDLE_DOWN, INPUT_PULLDOWN);

    Serial.println("Input Manager V10.0 Initialized (debounced PRND + edge paddles + staggered ADC).");
}

// ============================================================================
// MAIN UPDATE (called every 1ms). Digital inputs every tick; analog channels
// round-robined one per tick (each sampled at ~333Hz — still 6x faster than the
// EMA time constant, at a third of the per-loop ADC cost).
// ============================================================================
void InputManager::update() {
    if (_lastAtfOnly != telemetry.atf_only_selector) {
        _lastAtfOnly = telemetry.atf_only_selector;
        _prnd_candidate_code = 0xFF;
        _prnd_stable_ms = 0;
        telemetry.prnd_state = '?';
        telemetry.paddle_up_request = telemetry.paddle_down_request = false;
        _paddle_up_prev = digitalRead(PIN_PADDLE_UP) == HIGH;
        _paddle_down_prev = digitalRead(PIN_PADDLE_DOWN) == HIGH;
    }
    decodePRND();
    readPaddles();
    switch (_adc_phase) {
        case 0: readTPS();  break;
        case 1: readMAP();  break;
        case 2: readTemp(); break;
    }
    _adc_phase = (_adc_phase + 1) % 3;
}

void InputManager::decodePRND() {
    bool a = digitalRead(PIN_SHIFT_A);
    bool b = digitalRead(PIN_SHIFT_B);
    bool c = digitalRead(PIN_SHIFT_C);
    bool d = digitalRead(PIN_SHIFT_D);

    uint8_t din = telemetry.io_din & (uint8_t)~0x0F;
    if (a) din |= 0x01;
    if (b) din |= 0x02;
    if (c) din |= 0x04;
    if (d) din |= 0x08;
    telemetry.io_din = din;

    // Bench / circuit-test: dashboard owns the selector. Still sample the pins
    // (shown on the IO panel) but do not overwrite the virtual PRND.
    if (telemetry.test_mode || telemetry.atf_only_selector) return;

    uint8_t code = (d << 3) | (c << 2) | (b << 1) | a;

    // Debounce: only accept a code that has been rock-solid for PRND_STABLE_MS.
    // Lever travel produces transient (sometimes VALID) codes as the contacts
    // make/break asynchronously — without this, a 1ms blip of 'R' while moving
    // forward would trip the reverse-abuse failsafe and dump line pressure.
    if (code == _prnd_candidate_code) {
        if (_prnd_stable_ms < 255) _prnd_stable_ms++;
    } else {
        _prnd_candidate_code = code;
        _prnd_stable_ms = 0;
        return;
    }
    if (_prnd_stable_ms < PRND_STABLE_MS) return;

    switch(code) {
        case 0b0110: telemetry.prnd_state = 'P'; break;
        case 0b0111: telemetry.prnd_state = 'R'; break;
        case 0b1110: telemetry.prnd_state = 'N'; break;
        case 0b1100: telemetry.prnd_state = 'D'; break;
        case 0b1101: telemetry.prnd_state = '4'; break;
        case 0b1001: telemetry.prnd_state = '3'; break;
        case 0b1011: telemetry.prnd_state = '2'; break;
        case 0b1010: telemetry.prnd_state = '1'; break;
        // invalid/between-detent code: keep last known state
    }
}

// Rising-edge latch: one request per physical pull. Holding the paddle does
// nothing further until it is released (no 200ms auto-repeat through the box).
// The debounce window also swallows contact bounce on press AND release.
void InputManager::readPaddles() {
    unsigned long now = millis();

    bool up = (digitalRead(PIN_PADDLE_UP) == HIGH);
    bool down = (digitalRead(PIN_PADDLE_DOWN) == HIGH);
    uint8_t din = telemetry.io_din & (uint8_t)~0x30;
    if (up)   din |= 0x10;
    if (down) din |= 0x20;
    telemetry.io_din = din;

    // In test mode the dashboard paddles command shifts; hardware paddles are
    // displayed only (circuit confirmation).
    if (telemetry.test_mode) {
        _paddle_up_prev = up;
        _paddle_down_prev = down;
        return;
    }

    if (up && !_paddle_up_prev && (now - _last_paddle_up_time > PADDLE_DEBOUNCE_MS)) {
        telemetry.paddle_up_request = true;
        _last_paddle_up_time = now;
    }
    _paddle_up_prev = up;

    if (down && !_paddle_down_prev && (now - _last_paddle_down_time > PADDLE_DEBOUNCE_MS)) {
        telemetry.paddle_down_request = true;
        _last_paddle_down_time = now;
    }
    _paddle_down_prev = down;
}

// ============================================================================
// ANALOG CHANNELS (one per tick, see update()). analogReadMilliVolts() applies
// the ESP32 eFuse ADC calibration → linear up top, where WOT (~2.9 V) and the
// P/N threshold (3.0 V) otherwise sit in the raw ADC's nonlinear region.
// ============================================================================
void InputManager::readTPS() {
    // Calibration from the engine profile so swaps need no recompile.
    float tps_v = analogReadMilliVolts(_tps_pin) / 1000.0f;
    telemetry.tps_v = tps_v;
    // BL-16: a railed reading (open/short) is not a real throttle — ease toward CLOSED
    // (safe: no phantom power-shift or high load) and flag invalid, rather than feed garbage.
    if (tps_v < TPS_VALID_MIN_V || tps_v > TPS_VALID_MAX_V) {
        telemetry.tps_valid = false;
        _tps_filtered += 0.2f * (0.0f - _tps_filtered);
        telemetry.tps_pct = _tps_filtered;
        return;
    }
    telemetry.tps_valid = true;
    float tps_closed = engineProfile.tpsClosedV(), tps_wot = engineProfile.tpsWotV();
    float tps_span = (tps_wot - tps_closed);
    float tps_pct = (tps_span > 0.01f) ? (tps_v - tps_closed) / tps_span * 100.0f : 0.0f;
    tps_pct = constrain(tps_pct, 0.0f, 100.0f);
    // EMA (alpha 0.2 at ~333Hz → ~13ms time constant) to reject ADC jitter
    _tps_filtered += 0.2f * (tps_pct - _tps_filtered);
    telemetry.tps_pct = _tps_filtered;
}

void InputManager::readMAP() {
    float map_v = analogReadMilliVolts(_map_pin) / 1000.0f;
    telemetry.map_v = map_v;
    // BL-16: a railed reading (open/short) is not a real MAP — ease toward ATMOSPHERIC
    // (safe: no phantom boost/high load) and flag invalid.
    if (map_v < MAP_VALID_MIN_V || map_v > MAP_VALID_MAX_V) {
        telemetry.map_valid = false;
        _map_filtered += 0.2f * (100.0f - _map_filtered);
        telemetry.map_kpa = _map_filtered;
        return;
    }
    telemetry.map_valid = true;
    float map_kpa = engineProfile.mapAt0V() + (map_v * engineProfile.mapPerV());
    map_kpa = constrain(map_kpa, 20.0f, 260.0f); // sanity clamp
    _map_filtered += 0.2f * (map_kpa - _map_filtered);
    telemetry.map_kpa = _map_filtered;
}

float InputManager::calculateTemperatureFromResistance(float resistance_ohms) {
    float temp_c = (resistance_ohms - 800.0f) / 10.0f;
    return constrain(temp_c, -20.0f, 150.0f);
}

// ATF temp on pin 39. P/N now comes from the shifter (decodePRND), so this pin is
// purely the thermistor. A reading above ~3.0V means the line is open / pulled to
// the rail — the conductor-plate starter-lockout opening it in P/N, or a
// disconnected sensor — NOT a real temperature, so HOLD the last good value rather
// than computing a bogus very-cold reading. Below ~0.1V is an implausible short.
void InputManager::readTemp() {
    float pin_voltage = analogReadMilliVolts(_temp_sensor_pin) / 1000.0f;
    telemetry.atf_sample_ms = millis();
    telemetry.atf_range_evidence = static_cast<uint8_t>(_atfRange.update(pin_voltage, true, millis()));
    telemetry.atf_v = pin_voltage;
    telemetry.atf_sampled = true;
    telemetry.atf_signal_valid = pin_voltage > 0.1f && pin_voltage < 3.0f;

    if (telemetry.atf_signal_valid) {
        float resistance_ohms = TEMP_PULLUP_RESISTOR_OHMS * (pin_voltage / (ADC_REF_VOLTAGE - pin_voltage));
        _last_known_temp_c = calculateTemperatureFromResistance(resistance_ohms);
        telemetry.atf_has_measurement = true;
        telemetry.atf_last_valid_ms = millis();
    }
    telemetry.atf_temp_c = _last_known_temp_c;   // hold last good value when out of range
}
