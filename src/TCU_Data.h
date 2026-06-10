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
const uint8_t PIN_RP_LOCK = 13; // Reverse/Park interlock solenoid — digital out (GPIO13,
                                // ADC2 so useless for analog anyway; NOT a strapping pin)

const uint8_t PIN_TPS      = 36; // Throttle Position Sensor    ADC1_CH0
const uint8_t PIN_MAP      = 33; // Manifold Absolute Pressure  ADC1_CH5  <-- REWIRED from 23 (23 has no ADC)
const uint8_t PIN_ATF_TEMP = 39; // Temp Sensor & P/N Switch    ADC1_CH3

// PCNT speed sensors — these only need GPIO, not ADC-capable pins.
// Engine RPM moved to GPIO 23 (frees GPIO 33 for MAP on ADC1).
// HARDWARE: rewire MAP sensor → GPIO 33, engine tach → GPIO 23.
const uint8_t PIN_N2_SPEED  = 34; // N2 Turbine Speed Sensor     PCNT
const uint8_t PIN_N3_SPEED  = 35; // N3 Turbine Speed Sensor     PCNT
const uint8_t PIN_OUT_SPEED = 32; // External Output Shaft Speed  PCNT
const uint8_t PIN_ENG_SPEED = 23; // Engine RPM                   PCNT (moved from 33)

const uint8_t PIN_PADDLE_UP   = 21; // Upshift Paddle
const uint8_t PIN_PADDLE_DOWN = 22; // Downshift Paddle

const uint8_t PIN_SHIFT_A = 4;
const uint8_t PIN_SHIFT_B = 16;
const uint8_t PIN_SHIFT_C = 17;
const uint8_t PIN_SHIFT_D = 5;

// ============================================================================
// 2. TRANSMISSION CONSTANTS
// ============================================================================
// Select the correct set for your gearbox variant.
// The front simple planetary is identical in both variants, so N2_N3_BLEND_K=1.61
// applies to both — only these ratio constants and prefill defaults differ.
//
// W5A330  Small NAG  (330 Nm rated) — C-class 4-cyl/6-cyl, THIS BUILD (W203 C230K)
const float RATIO_1ST = 3.932f;
const float RATIO_2ND = 2.408f;
const float RATIO_3RD = 1.486f;
const float RATIO_4TH = 1.000f;
const float RATIO_5TH = 0.830f;
const float RATIO_REV = 3.100f;
//
// W5A580  Big NAG    (580 Nm rated) — C/E/S-class V6/V8 AMG, swap these in if using Big NAG
// const float RATIO_1ST = 3.595f;
// const float RATIO_2ND = 2.186f;
// const float RATIO_3RD = 1.405f;
// const float RATIO_4TH = 1.000f;
// const float RATIO_5TH = 0.831f;
// const float RATIO_REV = 3.168f;
// NOTE: Big NAG also needs larger prefill timing defaults (K2: ~220ms vs 160ms)
//       due to larger clutch pack volumes — increase prefill_modifiers[2] in
//       AdaptiveMemory::loadUltimateNag52Defaults() if swapping to Big NAG.

// ============================================================================
// 3. SAFETY THRESHOLDS (M111.985 + TVS1320, 6500 rpm ceiling)
// ============================================================================
const float RPM_HARD_CEILING        = 6500.0f; // Never let predicted RPM exceed this
const float RPM_OVERREV_UPSHIFT      = 6300.0f; // Auto-upshift trigger (margin below ceiling)
const float RPM_LUG_THRESHOLD        = 1100.0f; // Below this under load = lugging
const float TPS_LUG_LOAD_PCT         = 25.0f;   // Lug protection only active above this throttle
const float RPM_MAX_SAFE_DOWNSHIFT   = 6000.0f; // Money-shift guard ceiling for downshifts
const unsigned long AUTO_SHIFT_COOLDOWN_MS = 500; // Min gap between auto-safety shifts

// --- Selector-abuse protection (N->D at speed, R while moving) ---
// Output-shaft RPM that counts as "moving". ~150 rpm ≈ 5 km/h with a typical
// 3.5 final drive + 2 m tyre. Below this the car is treated as stopped, so
// reverse / park selection is permitted normally.
const float OUTPUT_RPM_MOVING         = 150.0f;
const float REVERSE_INHIBIT_SPEED_RPM = 150.0f; // above this, R is inhibited / pressure dumped
const uint8_t REVERSE_ABUSE_LINE_PCT  = 15;     // line% if R is forced while moving (slip, not shock)
const uint32_t ENGAGE_GRACE_MS        = 1500;   // suppress slip-limp during D-engagement clutch sync

// RP_LOCK shift-lock solenoid — VERIFY POLARITY ON YOUR SHIFTER before trusting it.
// Fail-safe by design: at boot/reset the pin is LOW = lever released, so a dead
// ESP32 never traps the driver. Engaged only while moving, to block R/P selection.
const bool RP_LOCK_ACTIVE_HIGH = true;  // true: drive HIGH to ENGAGE the lock
const bool ENABLE_RP_LOCK      = true;  // false: never drive the pin (no lock hardware fitted)

// MAP sensor calibration (adjust to YOUR sensor's transfer function)
// Example: 3-bar GM-style sensor, 0.5V=20kPa, 4.5V=304kPa, scaled to 3.3V ADC
const float MAP_KPA_AT_0V   = -10.0f;
const float MAP_KPA_PER_VOLT = 86.0f;

// TPS rate-of-change torque anticipation (supercharger has no boost lag).
// Entry: TPS rising faster than TRIGGER → max line pressure + TCC open immediately.
// Exit: TPS below RELEASE_PCT and ROC below RELEASE for COOLDOWN_MS.
const float    TPS_ROC_TRIGGER_PCT_MS  = 0.15f;  // %/ms — 15%/100ms triggers (fast stab)
const float    TPS_ROC_RELEASE_PCT_MS  = 0.02f;  // %/ms — essentially steady state
const float    TPS_ROC_RELEASE_HOLD    = 40.0f;  // % TPS must be below to start cooldown
const uint32_t TPS_ROC_COOLDOWN_MS     = 2000;   // ms to hold max pressure after settling

// N2/N3 turbine speed blending constant (NAG52 calls this RATIO_2_1, default 1.61).
// turbine = (N2 * K) - (N3 * (K-1))
// Verify: N3=0 (gears 1&5) → N2*1.61;  N2=N3 (3rd gear) → N2*1.0 ✓
// Adjust on bench if turbine RPM doesn't match engine RPM with TCC locked.
const float N2_N3_BLEND_K = 1.61f;

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
    bool reverse_abuse_active = false;   // R selected while moving forward — pressure dumped

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

    // --- TPS Rate-of-Change Torque Mode ---
    bool high_torque_mode = false;

    // --- Current shift phase (broadcast to dashboard for chart) ---
    uint8_t shift_phase = 0; // mirrors ShiftPhase enum: 0=CRUISING … 7=COMPLETION
};

extern TCU_Telemetry telemetry;

// ----------------------------------------------------------------------------
// SHARED LOAD MODEL  (used by both AdaptiveMemory and ShiftScheduler so the
// load index is computed ONE way everywhere - fixes the mismatch bug)
// ----------------------------------------------------------------------------
#define NUM_LOAD_BINS_SHARED 16
inline float computeLoad(float tps_pct, float map_kpa) {
    // TVS supercharger: boost is belt-driven, so torque tracks throttle immediately.
    // MAP lags actual torque by ~100-200ms (manifold fill time). To avoid the holding
    // pressure lagging behind torque delivery, TPS is weighted at 1.25× so the load
    // index responds as fast as the throttle moves, not as fast as MAP settles.
    // At WOT + 1.2 bar boost: load = 125 + (120*0.8) = 221, constrained to 200 via loadToBin.
    float load = tps_pct * 1.25f;
    if (map_kpa > 100.0f) load += (map_kpa - 100.0f) * 0.8f;
    return load;
}
inline uint8_t loadToBin(float load) {
    // 0..200 load mapped across 16 bins (~12.5 load units each) so 1.2 bar of
    // boost spreads across several cells instead of saturating bin 15.
    int bin = (int)(load / 12.5f);
    return (uint8_t)constrain(bin, 0, NUM_LOAD_BINS_SHARED - 1);
}
