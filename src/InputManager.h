// ============================================================================
// FILE: InputManager.h
// VERSION: 9.0
// UPDATES: Added TPS + MAP analog acquisition. P/N now writes pn_switch_raw
//          (not is_park_neutral) to stop the scheduler/sensor write fight.
// ============================================================================
#pragma once
#include <Arduino.h>
#include "TCU_Data.h"

// --- HARDWARE CALIBRATION ---
#define TEMP_PULLUP_RESISTOR_OHMS 1000.0f
#define ADC_REF_VOLTAGE 3.3f

// TPS closed/WOT voltages now live in EngineProfile (NVS, web-editable) — see
// engineProfile.tpsClosedV()/tpsWotV(). The old TPS_VOLTS_CLOSED/WOT defines were dead.

// Paddles are EDGE-triggered: one shift per pull, release required before re-arming.
// This debounce rejects contact bounce on the rising edge.
#define PADDLE_DEBOUNCE_MS 40

class InputManager {
  private:
    uint8_t _temp_sensor_pin;
    uint8_t _tps_pin;
    uint8_t _map_pin;
    float _last_known_temp_c;

    // Light smoothing for noisy analog lines
    float _tps_filtered;
    float _map_filtered;

    unsigned long _last_paddle_up_time;
    unsigned long _last_paddle_down_time;
    bool _paddle_up_prev;     // edge-detect: paddle state last tick (require release to re-arm)
    bool _paddle_down_prev;

    float calculateTemperatureFromResistance(float resistance_ohms);
    void decodePRND();
    void readPaddles();
    void readThrottleAndBoost();

  public:
    InputManager(uint8_t temp_sensor_pin, uint8_t tps_pin, uint8_t map_pin);
    void begin();

    void update();              // Digital PRND + Paddles + analog load
    void updateAnalogSensors(); // Fluid temp & raw P/N switch
};
