// ============================================================================
// FILE: TCU_Data.h
// VERSION: 9.0
// UPDATES:
//   - Added TPS + MAP analog inputs (PIN_MAP).
//   - Split is_park_neutral (raw sensor) from drive_engaged (scheduler latch).
//   - Added safety thresholds (overrev / lug) and limp-mode reset flag.
//   - Added shift_pressure feedback path so the ramp logic reads real values.
// ============================================================================
#pragma once
#include <Arduino.h>

// ============================================================================
// 1. HARDWARE PIN DEFINITIONS (ESP32 DevKit V1 Mappings)
// ============================================================================
const uint8_t PIN_MPC = 26; // Line Pressure (Y1 modulating)
const uint8_t PIN_SPC = 25; // Shift Pressure (Y2)
const uint8_t PIN_Y3  = 14; // Solenoid 1-2 / 4-5
const uint8_t PIN_Y5  = 19; // Solenoid 2-3
const uint8_t PIN_Y4  = 18; // Solenoid 3-4
const uint8_t PIN_TCC = 27; // Torque Converter Clutch

const uint8_t PIN_TPS      = 36; // Throttle Position Sensor (0-3.3V)
const uint8_t PIN_MAP      = 23; // Manifold Absolute Pressure (0-3.3V)  <-- VERIFY this pin is free on your board
const uint8_t PIN_ATF_TEMP = 39; // Temp Sensor & P/N Switch (multiplexed)

const uint8_t PIN_N2_SPEED  = 34; // N2 Turbine Speed Sensor
const uint8_t PIN_N3_SPEED  = 35; // N3 Turbine Speed Sensor
const uint8_t PIN_OUT_SPEED = 32; // External Output Shaft Speed
const uint8_t PIN_ENG_SPEED = 33; // Engine RPM

const uint8_t PIN_PADDLE_UP   = 21; // Upshift Paddle
const uint8_t PIN_PADDLE_DOWN = 22; // Downshift Paddle

const uint8_t PIN_SHIFT_A = 4;
const uint8_t PIN_SHIFT_B = 16;
const uint8_t PIN_SHIFT_C = 17;
const uint8_t PIN_SHIFT_D = 5;

// ============================================================================
// 2. TRANSMISSION CONSTANTS (W5A330 / Small NAG from W203 C230K)
// ============================================================================
const float RATIO_1ST = 3.932f;
const float RATIO_2ND = 2.408f;
const float RATIO_3RD = 1.486f;
const float RATIO_4TH = 1.000f;
const float RATIO_5TH = 0.830f;
const float RATIO_REV = 3.100f;

// ============================================================================
// 3. SAFETY THRESHOLDS (M111.985 + TVS1320, 6500 rpm ceiling)
// ============================================================================
const float RPM_HARD_CEILING        = 6500.0f; // Never let predicted RPM exceed this
const float RPM_OVERREV_UPSHIFT      = 6300.0f; // Auto-upshift trigger (margin below ceiling)
const float RPM_LUG_THRESHOLD        = 1100.0f; // Below this under load = lugging
const float TPS_LUG_LOAD_PCT         = 25.0f;   // Lug protection only active above this throttle
const float RPM_MAX_SAFE_DOWNSHIFT   = 6000.0f; // Money-shift guard ceiling for downshifts
const unsigned long AUTO_SHIFT_COOLDOWN_MS = 500; // Min gap between auto-safety shifts

// MAP sensor calibration (adjust to YOUR sensor's transfer function)
// Example: 3-bar GM-style sensor, 0.5V=20kPa, 4.5V=304kPa, scaled to 3.3V ADC
const float MAP_KPA_AT_0V   = -10.0f;
const float MAP_KPA_PER_VOLT = 86.0f;

// ============================================================================
// 4. TELEMETRY DATA STRUCTURE (V9.0)
// ============================================================================
struct TCU_Telemetry {
    // --- Speeds & Ratios ---
    float turbine_rpm = 0.0f;
    float output_rpm  = 0.0f;
    float engine_rpm  = 0.0f;
    float live_ratio  = 0.0f;

    // --- Engine Load ---
    float tps_pct = 0.0f;
    float map_kpa = 100.0f;

    // --- States ---
    uint8_t current_gear = 1;
    uint8_t target_gear  = 1;
    char prnd_state      = 'P';

    // --- Limp Mode & Safety ---
    bool is_limp_mode      = false;
    bool limp_reset_request = false;   // Set true (e.g. from web) to attempt recovery
    bool is_slipping       = false;
    unsigned long slip_start_time_ms = 0;
    String limp_mode_reason = "";

    // --- Manual Control ---
    bool paddle_up_request   = false;
    bool paddle_down_request = false;

    // --- Auto-safety bookkeeping ---
    unsigned long last_auto_shift_ms = 0;
    String last_safety_event = "";

    // --- Selector sensing (SEPARATED to fix the fighting-writer bug) ---
    bool pn_switch_raw  = true;   // Raw multiplexed P/N switch reading (InputManager owns this)
    bool drive_engaged  = false;  // Scheduler latch: we have completed garage engagement

    // --- Solenoid Live Data (these are now the single source of truth) ---
    uint8_t line_pressure_pct  = 0;
    uint8_t shift_pressure_pct = 0;
    uint8_t tcc_lockup_pct     = 0;

    // --- TCC Slip Tracking ---
    float tcc_target_slip_rpm = 0.0f;
    float tcc_actual_slip_rpm = 0.0f;

    // --- Sensors ---
    float atf_temp_c = 40.0f;

    // --- Shift Diagnostics ---
    unsigned long last_shift_time_ms = 0;
    bool flare_detected = false;
    bool bind_detected  = false;
};

extern TCU_Telemetry telemetry;

// ----------------------------------------------------------------------------
// SHARED LOAD MODEL  (used by both AdaptiveMemory and ShiftScheduler so the
// load index is computed ONE way everywhere - fixes the mismatch bug)
// ----------------------------------------------------------------------------
#define NUM_LOAD_BINS_SHARED 16
inline float computeLoad(float tps_pct, float map_kpa) {
    // Blend throttle with boost. Above atmospheric, MAP dominates because that
    // is the real torque signal on a supercharged engine.
    float load = tps_pct;
    if (map_kpa > 100.0f) load += (map_kpa - 100.0f) * 0.8f; // 1.2 bar (~220kPa) -> +96
    return load;
}
inline uint8_t loadToBin(float load) {
    // 0..200 load mapped across 16 bins (~12.5 load units each) so 1.2 bar of
    // boost spreads across several cells instead of saturating bin 15.
    int bin = (int)(load / 12.5f);
    return (uint8_t)constrain(bin, 0, NUM_LOAD_BINS_SHARED - 1);
}
