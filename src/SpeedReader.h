// ============================================================================
// FILE: SpeedReader.h
// VERSION: 2.1
// UPDATES: Restored ESP32 Hardware PCNT (Pulse Counter) + Kinematic Math.
// ============================================================================
#pragma once
#include <Arduino.h>
#include "driver/pulse_cnt.h" // NEW: Updated ESP-IDF 5.x Pulse Counter library
#include "TCU_Data.h"

// --- SENSOR CALIBRATION CONSTANTS ---
const float TEETH_N2 = 60.0f;          // OEM 722.6 Internal N2 Drum
const float TEETH_N3 = 60.0f;          // OEM 722.6 Internal N3 Drum
const float TEETH_OUT = 24.0f;         // Custom External Output Shaft Sensor
const float PULSES_PER_REV_ENG = 60.0f; // 60-tooth crank reluctor wheel (M111)

class SpeedReader {
  private:
    uint8_t _pin_n2;
    uint8_t _pin_n3;
    uint8_t _pin_out;
    uint8_t _pin_eng;

    // NEW: Handles for the updated PCNT driver
    pcnt_unit_handle_t _pcnt_n2 = NULL;
    pcnt_unit_handle_t _pcnt_n3 = NULL;
    pcnt_unit_handle_t _pcnt_out = NULL;
    pcnt_unit_handle_t _pcnt_eng = NULL;

    unsigned long _last_read_time;

    // Internal Raw RPMs
    float _raw_n2_rpm;
    float _raw_n3_rpm;

    // Helper functions
    void initPCNT(pcnt_unit_handle_t* unit_handle, uint8_t pin);
    float calculateTurbineRPM(float n2_rpm, float n3_rpm);

  public:
    SpeedReader(uint8_t pin_n2, uint8_t pin_n3, uint8_t pin_out, uint8_t pin_eng);
    void begin();
    
    void update();
};