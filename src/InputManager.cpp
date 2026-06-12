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
    decodePRND();
    readPaddles();
    switch (_adc_phase) {
        case 0: readTPS();       break;
        case 1: readMAP();       break;
        case 2: readTempAndPN(); break;
    }
    _adc_phase = (_adc_phase + 1) % 3;
}

void InputManager::decodePRND() {
    bool a = digitalRead(PIN_SHIFT_A);
    bool b = digitalRead(PIN_SHIFT_B);
    bool c = digitalRead(PIN_SHIFT_C);
    bool d = digitalRead(PIN_SHIFT_D);

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
    if (up && !_paddle_up_prev && (now - _last_paddle_up_time > PADDLE_DEBOUNCE_MS)) {
        telemetry.paddle_up_request = true;
        _last_paddle_up_time = now;
    }
    _paddle_up_prev = up;

    bool down = (digitalRead(PIN_PADDLE_DOWN) == HIGH);
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
    float map_kpa = engineProfile.mapAt0V() + (map_v * engineProfile.mapPerV());
    map_kpa = constrain(map_kpa, 20.0f, 260.0f); // sanity clamp
    _map_filtered += 0.2f * (map_kpa - _map_filtered);
    telemetry.map_kpa = _map_filtered;
}

float InputManager::calculateTemperatureFromResistance(float resistance_ohms) {
    float temp_c = (resistance_ohms - 800.0f) / 10.0f;
    return constrain(temp_c, -20.0f, 150.0f);
}

// Multiplexed ATF temp + P/N switch (writes pn_switch_raw, NOT a latched state)
void InputManager::readTempAndPN() {
    float pin_voltage = analogReadMilliVolts(_temp_sensor_pin) / 1000.0f;

    if (pin_voltage > 3.0f) {
        telemetry.pn_switch_raw = true;          // raw reading only
        telemetry.atf_temp_c = _last_known_temp_c;
    } else {
        telemetry.pn_switch_raw = false;
        if (pin_voltage > 0.1f) {
            float resistance_ohms = TEMP_PULLUP_RESISTOR_OHMS * (pin_voltage / (ADC_REF_VOLTAGE - pin_voltage));
            _last_known_temp_c = calculateTemperatureFromResistance(resistance_ohms);
            telemetry.atf_temp_c = _last_known_temp_c;
        }
    }
}
