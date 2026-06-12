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
// Engine-RPM pulses per rev is now a WEB-EDITABLE engine constant: EngineProfile.eng_ppr
// (NVS-backed), read live via engineProfile.engPpr(). RPM is frequency-counted over a 50 ms
// window, so resolution ≈ 1200/PPR rpm per count: 60 PPR = 20 rpm, 36 = 33, 24 = 50, 4 = 300.
// The TCC slip loop (targets 50 rpm) and overrev both need ≥24 PPR; 2-4 PPR is too coarse.
// SOURCE OPTIONS (set eng_ppr on the dashboard to match whatever you wire to PIN_ENG_SPEED):
//   60  = raw M111 crank sensor — but it's a 60-MINUS-2 wheel; the missing-tooth gap makes
//         this simple counter under-read ~3% and jitter (open item C-11). Level-shift the VR.
//   24-36 = rusEFI tach output configured to a clean, gap-free PPR (recommended; sidesteps
//         the 60-2 gap). VERIFY evenly-spaced on a scope at high rpm, and LEVEL-SHIFT the
//         tach pin to 3.3 V — it is typically 5/12 V open-collector (would damage the GPIO).
// Cleanest long-term: read RPM from rusEFI over CAN and bypass this pulse path entirely.

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