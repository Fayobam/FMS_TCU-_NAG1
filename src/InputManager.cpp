// ============================================================================
// FILE: InputManager.cpp
// VERSION: 9.0
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
    _last_paddle_up_time = 0;
    _last_paddle_down_time = 0;
    _paddle_up_prev = false;
    _paddle_down_prev = false;
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

    Serial.println("Input Manager V9.0 Initialized (PRND + Paddles + TPS + MAP + Temp).");
}

// ============================================================================
// DIGITAL + LOAD (called every 1ms)
// ============================================================================
void InputManager::update() {
    decodePRND();
    readPaddles();
    readThrottleAndBoost();
}

void InputManager::decodePRND() {
    bool a = digitalRead(PIN_SHIFT_A);
    bool b = digitalRead(PIN_SHIFT_B);
    bool c = digitalRead(PIN_SHIFT_C);
    bool d = digitalRead(PIN_SHIFT_D);

    uint8_t code = (d << 3) | (c << 2) | (b << 1) | a;

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

void InputManager::readPaddles() {
    unsigned long current_time = millis();
    bool up = (digitalRead(PIN_PADDLE_UP) == HIGH);
    bool dn = (digitalRead(PIN_PADDLE_DOWN) == HIGH);

    // EDGE-triggered: fire once on the press edge (rising), debounced. The paddle must be
    // released (and re-armed) before it can request another shift — holding does NOT repeat.
    if (up && !_paddle_up_prev && (current_time - _last_paddle_up_time > PADDLE_DEBOUNCE_MS)) {
        telemetry.paddle_up_request = true;
        _last_paddle_up_time = current_time;
    }
    _paddle_up_prev = up;

    if (dn && !_paddle_down_prev && (current_time - _last_paddle_down_time > PADDLE_DEBOUNCE_MS)) {
        telemetry.paddle_down_request = true;
        _last_paddle_down_time = current_time;
    }
    _paddle_down_prev = dn;
}

// ============================================================================
// THROTTLE + BOOST  (the inputs half the controller was missing)
// ============================================================================
void InputManager::readThrottleAndBoost() {
    // --- TPS --- (calibration from the engine profile so swaps need no recompile)
    // analogReadMilliVolts() applies the ESP32 eFuse ADC calibration → linear up top,
    // where WOT (~2.9 V) and the P/N threshold (3.0 V) otherwise sit in the raw ADC's
    // nonlinear region. Returns mV; /1000 → volts.
    float tps_v = analogReadMilliVolts(_tps_pin) / 1000.0f;
    float tps_closed = engineProfile.tpsClosedV(), tps_wot = engineProfile.tpsWotV();
    float tps_span = (tps_wot - tps_closed);
    float tps_pct = (tps_span > 0.01f) ? (tps_v - tps_closed) / tps_span * 100.0f : 0.0f;
    tps_pct = constrain(tps_pct, 0.0f, 100.0f);
    // Exponential moving average (alpha ~0.2) to reject ADC jitter at 1kHz
    _tps_filtered += 0.2f * (tps_pct - _tps_filtered);
    telemetry.tps_pct = _tps_filtered;

    // --- MAP --- (transfer function from the engine profile)
    float map_v = analogReadMilliVolts(_map_pin) / 1000.0f;
    float map_kpa = engineProfile.mapAt0V() + (map_v * engineProfile.mapPerV());
    map_kpa = constrain(map_kpa, 20.0f, 260.0f); // sanity clamp
    _map_filtered += 0.2f * (map_kpa - _map_filtered);
    telemetry.map_kpa = _map_filtered;
}

// ============================================================================
// ANALOG TEMP + MULTIPLEXED P/N  (writes pn_switch_raw, NOT is_park_neutral)
// ============================================================================
float InputManager::calculateTemperatureFromResistance(float resistance_ohms) {
    float temp_c = (resistance_ohms - 800.0f) / 10.0f;
    return constrain(temp_c, -20.0f, 150.0f);
}

void InputManager::updateAnalogSensors() {
    // Calibrated read (linear near the 3.0 V P/N threshold; see readThrottleAndBoost).
    float pin_voltage = analogReadMilliVolts(_temp_sensor_pin) / 1000.0f;

    if (pin_voltage > 3.0f) {
        telemetry.pn_switch_raw = true;          // <-- raw reading only
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
