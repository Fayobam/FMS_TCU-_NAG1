// ============================================================================
// FILE: InputManager.cpp
// VERSION: 8.1
// UPDATES: Fully merged Digital (PRND/Paddles) and Analog (Temp/Multiplex)
// ============================================================================
#include "InputManager.h"

InputManager::InputManager(uint8_t temp_sensor_pin) {
    _temp_sensor_pin = temp_sensor_pin;
    _last_known_temp_c = 40.0f; // Safe default
    _last_paddle_up_time = 0;
    _last_paddle_down_time = 0;
}

void InputManager::begin() {
    // 1. Analog Pin Init
    pinMode(_temp_sensor_pin, INPUT);

    // 2. Digital PRND Init
    pinMode(PIN_SHIFT_A, INPUT_PULLDOWN);
    pinMode(PIN_SHIFT_B, INPUT_PULLDOWN);
    pinMode(PIN_SHIFT_C, INPUT_PULLDOWN);
    pinMode(PIN_SHIFT_D, INPUT_PULLDOWN);

    // 3. Digital Paddle Init
    pinMode(PIN_PADDLE_UP, INPUT_PULLDOWN);
    pinMode(PIN_PADDLE_DOWN, INPUT_PULLDOWN);

    Serial.println("Input Manager V8.1 Initialized (Digital + Analog).");
}

// ============================================================================
// DIGITAL SENSORS (PRND & Paddles)
// ============================================================================
void InputManager::update() {
    decodePRND();
    readPaddles();
}

void InputManager::decodePRND() {
    bool a = digitalRead(PIN_SHIFT_A);
    bool b = digitalRead(PIN_SHIFT_B);
    bool c = digitalRead(PIN_SHIFT_C);
    bool d = digitalRead(PIN_SHIFT_D);

    // Construct the 4-bit binary code: D C B A
    uint8_t code = (d << 3) | (c << 2) | (b << 1) | a;

    // Mercedes 722.6 Shifter Truth Table
    switch(code) {
        case 0b0110: telemetry.prnd_state = 'P'; break;
        case 0b0111: telemetry.prnd_state = 'R'; break;
        case 0b1110: telemetry.prnd_state = 'N'; break;
        case 0b1100: telemetry.prnd_state = 'D'; break;
        case 0b1101: telemetry.prnd_state = '4'; break;
        case 0b1001: telemetry.prnd_state = '3'; break;
        case 0b1011: telemetry.prnd_state = '2'; break;
        case 0b1010: telemetry.prnd_state = '1'; break;
        // If between gears (invalid code), keep the last known state
    }
}

void InputManager::readPaddles() {
    unsigned long current_time = millis();
    
    // Read UP Paddle (with 200ms debounce)
    if (digitalRead(PIN_PADDLE_UP) == HIGH) {
        if (current_time - _last_paddle_up_time > 200) { 
            telemetry.paddle_up_request = true;
            _last_paddle_up_time = current_time;
        }
    }
    
    // Read DOWN Paddle (with 200ms debounce)
    if (digitalRead(PIN_PADDLE_DOWN) == HIGH) {
        if (current_time - _last_paddle_down_time > 200) { 
            telemetry.paddle_down_request = true;
            _last_paddle_down_time = current_time;
        }
    }
}

// ============================================================================
// ANALOG SENSORS (Temp & Multiplexed P/N)
// ============================================================================
float InputManager::calculateTemperatureFromResistance(float resistance_ohms) {
    // 0C = 800 ohms. Adds ~10 ohms per degree C.
    float temp_c = (resistance_ohms - 800.0f) / 10.0f;
    return constrain(temp_c, -20.0f, 150.0f);
}

void InputManager::updateAnalogSensors() {
    int raw_adc = analogRead(_temp_sensor_pin);
    float pin_voltage = (raw_adc / ADC_MAX_TICKS) * ADC_REF_VOLTAGE;

    // Multiplex Logic: Open Switch = Near Max Voltage (Park/Neutral)
    if (pin_voltage > 3.0f) {
        telemetry.is_park_neutral = true;
        telemetry.atf_temp_c = _last_known_temp_c; 
    } 
    else {
        // Switch Closed = In Gear. Voltage drops through Thermistor.
        telemetry.is_park_neutral = false;
        
        if (pin_voltage > 0.1f) { 
            float resistance_ohms = TEMP_PULLUP_RESISTOR_OHMS * (pin_voltage / (ADC_REF_VOLTAGE - pin_voltage));
            _last_known_temp_c = calculateTemperatureFromResistance(resistance_ohms);
            telemetry.atf_temp_c = _last_known_temp_c;
        }
    }
}