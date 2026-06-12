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
const uint8_t PIN_TORQUE_CUT = 15; // rusEFI shift-retard request line (schematic EXTRA_OUT).
                                // NOTE: GPIO15 is a strapping pin (MTDO) — held low only after
                                // boot. Default-disabled until the rusEFI digital input is wired.

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
// Select the correct set for your gearbox variant. N2_N3_BLEND_K is NOT shared — it is
// tooth-derived and differs per variant (1.641 small NAG / 1.630 large NAG; see below).
// Only the ratio constants and prefill defaults change in this block.
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

// (MAP sensor transfer function lives in EngineProfile: map_kpa_at_0v / map_kpa_per_volt,
//  NVS-backed and web-editable. The old TCU_Data MAP_KPA_* constants were dead copies.)

// TPS rate-of-change torque anticipation (supercharger has no boost lag).
// Entry: TPS rising faster than TRIGGER → max line pressure + TCC open immediately.
// Exit: TPS below RELEASE_PCT and ROC below RELEASE for COOLDOWN_MS.
// ROC is averaged over a TPS_ROC_WINDOW_MS ring (not per-1ms deltas), so post-EMA ADC
// noise (~0.4%/ms) can no longer false-trigger the mode — the threshold means what it says.
const float    TPS_ROC_TRIGGER_PCT_MS  = 0.15f;  // %/ms — 15%/100ms triggers (fast stab)
const float    TPS_ROC_RELEASE_PCT_MS  = 0.02f;  // %/ms — essentially steady state
const float    TPS_ROC_RELEASE_HOLD    = 40.0f;  // % TPS must be below to start cooldown
const uint32_t TPS_ROC_COOLDOWN_MS     = 2000;   // ms to hold max pressure after settling
const uint16_t TPS_ROC_WINDOW_MS       = 20;     // ROC averaging window (ring length)

// --- Adaptive harshness metrics (TUNE ON BENCH against real shift traces) ---
// Downshift: flag bind only for the output drop ATTRIBUTABLE TO THE CATCH CLUTCH,
// i.e. beyond the vehicle's pre-catch deceleration trend — so routine braking
// downshifts no longer log false binds and ratchet the trims to their caps.
const float DS_BIND_EXTRA_RPM    = 40.0f; // extra output drop (above decel trend) = real bind

// Flare/bind ratio thresholds must hold CONTINUOUSLY for this long before latching, so a
// single noisy speed sample can never poison the adaptation tables (V10).
const uint16_t RATIO_EVENT_CONFIRM_MS = 10;

// --- Predictive overrev (auto upshift) ---
// The forced upshift only unloads the engine after PREP+FILL+part of TORQUE (~250-400ms);
// at WOT in a low gear the engine gains several hundred rpm in that window. Trigger on
// PREDICTED rpm (now + roc×lead, capped) so the shift beats the limiter (V10).
const uint32_t OVERREV_LEAD_MS = 300;    // expected latency from beginShift to clutch bite

// Speeds now refresh at 200 Hz (MCPWM period capture) but the phase engine runs at 1 kHz, so
// between samples the ratio is still frozen for ~5 ticks. Ratio-DERIVATIVE predicates (sprag
// dRatio/dt collapse) are evaluated only on a NEW sample (speed_sample_seq edge), flatness
// measured across CONSECUTIVE samples. This is the per-sample (~5 ms) flat band: the sprag
// freewheel has caught when the ratio moves less than this between two samples. Bench-tunable.
const float SPRAG_FLAT_RATIO_DELTA = 0.004f;

// N2/N3 turbine speed blending constant — DERIVED FROM TOOTH COUNTS (ATSG p.8, pp.32-33).
// Front simple planetary: n2 = front carrier, n3 = front sun, turbine = front ring.
//   n_turbine = n2*(1 + Zs/Zr) - n3*(Zs/Zr)   so K = 1 + Zs/Zr,  (K-1) = Zs/Zr
//   Small NAG (W5A330): sun 50T, ring 78T → K = 1 + 50/78 = 1.6410  (THIS BUILD)
//   Large NAG (W5A580): sun 58T, ring 92T → K = 1 + 58/92 = 1.6304
// turbine = (N2 * K) - (N3 * (K-1))
// Verify: N3=0 (gears 1&5) → N2*1.641;  N2=N3 (3rd gear direct) → N2*1.0 ✓
// Bench-verify TCC-locked: turbine should equal engine RPM ±20.
const float N2_N3_BLEND_K = 1.641f;

// ============================================================================
// TORQUE ESTIMATION — the master input (ATSG p.77: adaptation is denominated in Nm,
// not raw TPS). Lives in EngineProfile (NVS-backed 8×8 RPM×MAP surface, web-editable,
// bilinear-interpolated). The old linear MAP→Nm fit and its constants (MAP_ZERO_KPA,
// K_T_NM_PER_KPA, T_MAX_NM, estimateTorqueNm/torqueLoadPct/torqueBin) were superseded
// by that surface and removed. The surface is seeded from ~2.43·(map−35), clamp 450.
// ============================================================================

// ============================================================================
// SHIFT CLASSIFICATION (ATSG p.77 categories). Latched at beginShift(), never
// re-evaluated mid-shift. POWER vs COAST keys off torque+throttle with hysteresis.
// ============================================================================
enum ShiftClass : uint8_t { SC_POWER_UP, SC_COAST_UP, SC_POWER_DOWN, SC_COAST_DOWN };
enum PowerDownType : uint8_t { PD_NONE, PD_SPRAG, PD_TIMED };  // 3-2/2-1 sprag-assisted vs 4-3/5-4 timed
const float CLASS_POWER_TPS_PCT  = 8.0f;    // POWER if tps > 8% AND torque > 25 Nm
const float CLASS_POWER_TQ_NM    = 25.0f;
const float CLASS_COAST_TPS_PCT  = 5.0f;    // COAST if tps < 5%; between = keep previous class

// Adaptation gating (ATSG p.78): OEM relearn wants ATF 80-90 °C; 60-105 acceptable.
const float ADAPT_ATF_MIN_C = 60.0f;
const float ADAPT_ATF_MAX_C = 105.0f;

// (Upshift fill calibration — spec §5, indexed by upshift idx — lives in EngineProfile:
//  fill_p[]/fill_t[], NVS-backed and web-editable. The old compile-time copies were dead.)

// 20ms pressure-update quantization (ATSG p.80: ETC changes amplitude once per 20ms).
const uint16_t PRESSURE_TICK_MS = 20;

// --- TCC lockup rate limits (per 20ms ptick) ---
// The clutch must apply gently but release fast: a boost/shift demand to OPEN always
// wins the race. +2%/tick lock = 0->85% in ~850ms; -10%/tick release = full dump in ~200ms.
const int      TCC_LOCK_STEP        = 2;     // % per ptick while locking up
const int      TCC_RELEASE_STEP     = 10;    // % per ptick while opening
const uint16_t TCC_POST_SHIFT_HOLD_MS = 300; // keep TCC fully open this long after a shift ends

// --- Coast-down auto scheduler (spec §4.5) — output-shaft RPM thresholds ---
// Each catch lands at an idle-friendly turbine speed. Floor is 2nd (owner's
// 2nd-gear-launch model); 1st stays the driver's choice. 24-tooth output reluctor.
const float COAST_DN_5_TO_4 = 1900.0f;  // 5->4 below this output rpm
const float COAST_DN_4_TO_3 = 1400.0f;  // 4->3
const float COAST_DN_3_TO_2 =  900.0f;  // 3->2

// --- Kickdown arm (spec §4.6) ---
const float KICKDOWN_TPS_PCT      = 70.0f;   // tps above which a power-down is evaluated
const float KICKDOWN_MAX_ENG_RPM  = 5200.0f; // don't kickdown if already this high (would overrev)

// --- Optional rusEFI torque-cut during power-up INERTIA (spec §9) ---
// Asserting a timing-retard request lets the clutch absorb less energy per shift —
// the single biggest "sharp without sacrificing health" lever. Default OFF until the
// GPIO15 -> rusEFI digital input is wired and the retard is configured in rusEFI.
const bool  ENABLE_TORQUE_CUT     = false;
const float TORQUE_CUT_MIN_LOAD   = 50.0f;   // only on power upshifts above this % load

// ============================================================================
// 4. TELEMETRY DATA STRUCTURE (V9.0)
// ============================================================================
struct TCU_Telemetry {
    // --- Speeds & Ratios ---
    float turbine_rpm = 0.0f;
    float output_rpm  = 0.0f;
    float engine_rpm  = 0.0f;
    float live_ratio  = 0.0f;
    uint32_t speed_sample_seq = 0;   // ++ when a NEW edge advances a ratio channel (N2/N3/OUT);
                                     // lets the 1 kHz phase engine gate ratio-derivative checks (B-4)

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
    // Status strings are fixed buffers (NOT Arduino String) because Core 1 writes
    // them while Core 0 reads them in the telemetry JSON. A heap-backed String can
    // realloc mid-read and corrupt the heap across cores. The seq counter is a
    // seqlock: it is odd while a write is in progress, so the reader can detect and
    // retry a torn read. Write only via setLimpReason()/setSafetyEvent() below.
    char limp_mode_reason[64] = "";
    volatile uint8_t limp_reason_seq = 0;

    // --- Manual Control ---
    bool paddle_up_request   = false;
    bool paddle_down_request = false;

    // --- Auto-safety bookkeeping ---
    unsigned long last_auto_shift_ms = 0;
    char last_safety_event[64] = "";       // fixed buffer + seqlock (see limp_mode_reason note)
    volatile uint8_t safety_event_seq = 0;
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
    uint8_t shift_phase = 0; // mirrors ShiftPhase: 0=CRUISING 1=PREP 2=FILL 3=TORQUE 4=INERTIA
                             //                    5=RELEASE 6=CATCH 7=LOCK 8=END

    // --- Shift classification & torque (ATSG class-based architecture) ---
    float   t_est_nm     = 0.0f;  // estimated input torque (Nm)
    float   load_pct     = 0.0f;  // 0-100% of T_MAX (drives all pressure maps)
    uint8_t shift_class  = 0;     // mirrors ShiftClass for the active/last shift
    uint8_t pd_type      = 0;     // mirrors PowerDownType when class is SC_POWER_DOWN
};

extern TCU_Telemetry telemetry;

// ----------------------------------------------------------------------------
// Cross-core-safe status string writers (seqlock). Call ONLY from Core 1.
// Bump seq odd → copy → bump seq even, so a Core 0 reader can detect a torn read.
// ----------------------------------------------------------------------------
inline void setSafetyEvent(const char* msg) {
    telemetry.safety_event_seq = telemetry.safety_event_seq + 1;   // odd: write in progress
    strncpy(telemetry.last_safety_event, msg, sizeof(telemetry.last_safety_event) - 1);
    telemetry.last_safety_event[sizeof(telemetry.last_safety_event) - 1] = '\0';
    telemetry.safety_event_seq = telemetry.safety_event_seq + 1;   // even: done
}
inline void setLimpReason(const char* msg) {
    telemetry.limp_reason_seq = telemetry.limp_reason_seq + 1;
    strncpy(telemetry.limp_mode_reason, msg, sizeof(telemetry.limp_mode_reason) - 1);
    telemetry.limp_mode_reason[sizeof(telemetry.limp_mode_reason) - 1] = '\0';
    telemetry.limp_reason_seq = telemetry.limp_reason_seq + 1;
}

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
