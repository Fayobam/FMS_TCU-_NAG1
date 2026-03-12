#pragma once
#include <Arduino.h>
#include <driver/pulse_cnt.h> // ESP32 Modern Hardware Pulse Counter

// ----------------------------------------------------------------------------
// TONE RING CONFIGURATION (Adjust these to match your actual hardware!)
// ----------------------------------------------------------------------------
// How many teeth/pulses equal exactly 1 revolution?
const float TEETH_N2  = 30.0f; // Example: 30-tooth trigger wheel for N2
const float TEETH_N3  = 30.0f; // Example: 30-tooth trigger wheel for N3
const float TEETH_OUT = 24.0f; // Custom external driveshaft reluctor
const float TEETH_ENG = 2.0f;  // Example: 4-cyl Tach signal (2 pulses per rev)

class SpeedReader {
  private:
    uint8_t _pin_n2;
    uint8_t _pin_n3;
    uint8_t _pin_out;
    uint8_t _pin_eng;

    // Handles for the new ESP-IDF v5 PCNT driver
    pcnt_unit_handle_t _unit_n2;
    pcnt_unit_handle_t _unit_n3;
    pcnt_unit_handle_t _unit_out;
    pcnt_unit_handle_t _unit_eng;

    // We need to track the exact time of our last reading to calculate RPM
    unsigned long _last_read_time;

    // Helper function to setup the hardware PCNT for a specific pin
    pcnt_unit_handle_t configurePCNT(uint8_t pin);

    // Helper function to read the hardware counter and reset it
    float calculateRPM(pcnt_unit_handle_t unit, float teeth, unsigned long time_delta_ms);

  public:
    SpeedReader(uint8_t pin_n2, uint8_t pin_n3, uint8_t pin_out, uint8_t pin_eng);

    void begin();
    
    // Call this periodically (e.g., every 50ms) to update the global telemetry struct
    void update();
};