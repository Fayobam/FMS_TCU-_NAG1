// ============================================================================
// FILE: InputManager.h
// VERSION: 8.1
// UPDATES: Merged Digital PRND/Paddles with Analog Temp/Multiplex logic.
// ============================================================================
#pragma once
#include <Arduino.h>
#include "TCU_Data.h"

// --- HARDWARE CALIBRATION ---
#define TEMP_PULLUP_RESISTOR_OHMS 1000.0f  
#define ADC_MAX_TICKS 4095.0f
#define ADC_REF_VOLTAGE 3.3f

class InputManager {
  private:
    uint8_t _temp_sensor_pin;
    float _last_known_temp_c;

    // Paddle Debouncing Timers
    unsigned long _last_paddle_up_time;
    unsigned long _last_paddle_down_time;

    // Helper Functions
    float calculateTemperatureFromResistance(float resistance_ohms);
    void decodePRND();
    void readPaddles();

  public:
    InputManager(uint8_t temp_sensor_pin);
    void begin();
    
    // Core Updates
    void update();              // Reads Digital PRND & Paddles
    void updateAnalogSensors(); // Reads Fluid Temp & P/N Switch
};