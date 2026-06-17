// ============================================================================
// FILE: InputManager.h
// VERSION: 10.0
// UPDATES:
//   - PRND decode now requires the 4-bit code to be STABLE for 20ms before it
//     is accepted — a transiently valid wrong code during lever travel can no
//     longer glitch the state (e.g. momentary 'R' tripping the abuse failsafe).
//   - Paddles latch on the RISING EDGE only (release required to re-arm). A
//     held paddle no longer auto-repeats through the gears.
//   - Analog reads are round-robined (one ADC channel per 1ms tick) instead of
//     3 conversions per tick — frees ~100us/loop of the 1ms physics budget.
//     update() now owns ALL inputs; updateAnalogSensors() is gone.
//   - Analog reads use analogReadMilliVolts() (eFuse-calibrated, linear up top).
// ============================================================================
#pragma once
#include <Arduino.h>
#include "TCU_Data.h"

// --- HARDWARE CALIBRATION ---
#define TEMP_PULLUP_RESISTOR_OHMS 2000.0f   // matches board R9 (2K pull-up to +3.3V)
#define ADC_REF_VOLTAGE 3.3f
// TPS/MAP calibration lives in EngineProfile (NVS, web-editable).

// Analog plausibility bounds (BL-16). A healthy TPS/MAP sits well inside these; a pin
// railed low (open → pulled to GND by the divider) or high (short to supply) is not a
// real reading → substitute a SAFE default instead of a phantom value: TPS→closed (0%),
// MAP→atmospheric (100 kPa). Bounds are generous to never clip a valid WOT/boost reading.
const float TPS_VALID_MIN_V = 0.15f;
const float TPS_VALID_MAX_V = 3.20f;
const float MAP_VALID_MIN_V = 0.15f;
const float MAP_VALID_MAX_V = 3.20f;

const uint8_t  PRND_STABLE_MS     = 20;  // 4-bit code must hold this long to be accepted
const uint16_t PADDLE_DEBOUNCE_MS = 50;  // min gap between accepted paddle edges

class InputManager {
  private:
    uint8_t _temp_sensor_pin;
    uint8_t _tps_pin;
    uint8_t _map_pin;
    float _last_known_temp_c;

    // Light smoothing for noisy analog lines
    float _tps_filtered;
    float _map_filtered;

    // PRND debounce state
    uint8_t _prnd_candidate_code;
    uint8_t _prnd_stable_ms;

    // Paddle edge-latch state
    bool _paddle_up_prev;
    bool _paddle_down_prev;
    unsigned long _last_paddle_up_time;
    unsigned long _last_paddle_down_time;

    uint8_t _adc_phase;   // round-robin: 0=TPS, 1=MAP, 2=temp

    float calculateTemperatureFromResistance(float resistance_ohms);
    void decodePRND();
    void readPaddles();
    void readTPS();
    void readMAP();
    void readTemp();   // ATF temp on pin 39 (P/N now from the shifter, not multiplexed here)

  public:
    InputManager(uint8_t temp_sensor_pin, uint8_t tps_pin, uint8_t map_pin);
    void begin();

    void update();   // PRND + paddles every tick; one analog channel per tick
};
