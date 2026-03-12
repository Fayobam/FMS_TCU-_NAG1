// ============================================================================
// FILE: TCU_Data.h
// VERSION: 8.0
// UPDATES: Added Limp Mode slip tracking and diagnostic string.
// ============================================================================
#pragma once
#include <Arduino.h>

// ============================================================================
// 1. HARDWARE PIN DEFINITIONS (ESP32 DevKit V1 Mappings)
// ============================================================================
const uint8_t PIN_MPC = 26; // Line Pressure
const uint8_t PIN_SPC = 25; // Shift Pressure
const uint8_t PIN_Y3  = 14; // Solenoid 1-2 / 4-5
const uint8_t PIN_Y5  = 19; // Solenoid 2-3
const uint8_t PIN_Y4  = 18; // Solenoid 3-4
const uint8_t PIN_TCC = 27; // Torque Converter Clutch

const uint8_t PIN_TPS      = 36; // Throttle Position Sensor 
const uint8_t PIN_ATF_TEMP = 39; // Temp Sensor & P/N Switch 

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
// 2. TRANSMISSION CONSTANTS (W5A330 / Small NAG from C230K)
// ============================================================================
const float RATIO_1ST = 3.932f;
const float RATIO_2ND = 2.408f;
const float RATIO_3RD = 1.486f;
const float RATIO_4TH = 1.000f;
const float RATIO_5TH = 0.830f;
const float RATIO_REV = 3.100f;

// ============================================================================
// 3. TELEMETRY DATA STRUCTURE (V8.0)
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
    
    // --- Limp Mode & Safety (V8.0 Updates) ---
    bool is_limp_mode      = false;
    bool is_slipping       = false;
    unsigned long slip_start_time_ms = 0;
    String limp_mode_reason = "";
    
    // --- Manual Control ---
    bool paddle_up_request   = false;
    bool paddle_down_request = false;
    
    // --- Solenoid Live Data ---
    uint8_t line_pressure_pct  = 0;
    uint8_t shift_pressure_pct = 0;
    uint8_t tcc_lockup_pct     = 0;
    
    // --- TCC Slip Tracking ---
    float tcc_target_slip_rpm = 0.0f;
    float tcc_actual_slip_rpm = 0.0f;
    
    // --- Sensors ---
    float atf_temp_c = 40.0f;      
    bool is_park_neutral = true;   

    // --- Shift Diagnostics ---
    unsigned long last_shift_time_ms = 0;
    bool flare_detected = false;
    bool bind_detected  = false;
};

extern TCU_Telemetry telemetry;