// ============================================================================
// FILE: AdaptiveMemory.cpp
// VERSION: 5.1
// UPDATES: Added Ultimate-NAG52 / EGS52 baseline calibration injector.
// ============================================================================
#include "AdaptiveMemory.h"

AdaptiveMemory::AdaptiveMemory() {}

void AdaptiveMemory::begin() {
    preferences.begin("tcu_adapt", false);
    
    // Check if the very first table exists. If it doesn't, the memory is blank!
    if (preferences.getBytesLength("P_TBL_0") != sizeof(pressure_modifiers[0])) {
        Serial.println("Blank Flash Detected! Injecting Ultimate-NAG52 Baselines...");
        loadUltimateNag52Defaults();
    } else {
        // Load existing adaptives from Flash
        for (int i = 0; i < NUM_UPSHIFTS; i++) {
            preferences.getBytes(("P_TBL_" + String(i)).c_str(), &pressure_modifiers[i], 256);
            preferences.getBytes(("F_TBL_" + String(i)).c_str(), &prefill_modifiers[i], 256);
            preferences.getBytes(("DP_TBL_" + String(i)).c_str(), &ds_pressure_modifiers[i], 256);
            preferences.getBytes(("DT_TBL_" + String(i)).c_str(), &ds_timing_modifiers[i], 256);
        }
        Serial.println("Adaptive Memory V5.1 Loaded from Flash.");
    }
}

// ----------------------------------------------------------------------------
// ULTIMATE-NAG52 BASELINE CALIBRATION (W5A330 EGS52 PROFILES)
// ----------------------------------------------------------------------------
void AdaptiveMemory::loadUltimateNag52Defaults() {
    // 1. Initialize everything to zero first
    memset(pressure_modifiers, 0, sizeof(pressure_modifiers));
    memset(prefill_modifiers, 0, sizeof(prefill_modifiers));
    memset(ds_pressure_modifiers, 0, sizeof(ds_pressure_modifiers));
    memset(ds_timing_modifiers, 0, sizeof(ds_timing_modifiers));

    // 2. Apply Hardware-Specific Prefill/Timing Offsets
    for (int l = 0; l < NUM_LOAD_BINS; l++) {
        for (int r = 0; r < NUM_RPM_BINS; r++) {
            
            // --- UPSHIFT PREFILLS (Base 60ms + Mod) ---
            // 1->2 (B1 Brake): Small volume. Fast fill needed.
            prefill_modifiers[0][l][r] = -15; // Net: 45ms
            
            // 2->3 (K1 Clutch): Medium volume.
            prefill_modifiers[1][l][r] = -5;  // Net: 55ms
            
            // 3->4 (K2 Clutch): Massive volume! Will flare if not filled long enough.
            prefill_modifiers[2][l][r] = 20;  // Net: 80ms
            
            // 4->5 (B1 Brake again)
            prefill_modifiers[3][l][r] = -15; // Net: 45ms

            // --- UPSHIFT OVERLAP PRESSURES (Base 30% + Mod) ---
            // Scale overlap pressure smoothly based on Load Bin (0 to 15)
            // Light load = soft shift. Heavy load/Boost = harsh clamping to prevent slip.
            int8_t load_pressure_add = (l * 3); // 0% at idle, up to +45% at Max Boost
            pressure_modifiers[0][l][r] = load_pressure_add;
            pressure_modifiers[1][l][r] = load_pressure_add;
            pressure_modifiers[2][l][r] = load_pressure_add + 5; // K2 needs slightly more clamp
            pressure_modifiers[3][l][r] = load_pressure_add;

            // --- DOWNSHIFT REV-MATCH TIMING (Base 80ms + Mod) ---
            // Higher gears (5-4, 4-3) drop RPM slower, need more time to sync
            // Lower gears (3-2, 2-1) sync rapidly
            ds_timing_modifiers[0][l][r] = 20;  // 2->1 (100ms sync)
            ds_timing_modifiers[1][l][r] = 50;  // 3->2 (130ms sync)
            ds_timing_modifiers[2][l][r] = 100; // 4->3 (180ms sync)
            ds_timing_modifiers[3][l][r] = 150; // 5->4 (230ms sync)

            // Downshift clamp pressure (Higher load = grab harder after sync)
            ds_pressure_modifiers[0][l][r] = load_pressure_add;
            ds_pressure_modifiers[1][l][r] = load_pressure_add;
            ds_pressure_modifiers[2][l][r] = load_pressure_add;
            ds_pressure_modifiers[3][l][r] = load_pressure_add;
        }
    }

    // 3. Save these perfect baselines immediately to Flash memory!
    for (int i = 0; i < NUM_UPSHIFTS; i++) {
        saveTable(true, i, true);  // Upshift Pressures
        saveTable(true, i, false); // Upshift Prefills
        saveTable(false, i, true); // Downshift Pressures
        saveTable(false, i, false);// Downshift Timing
    }
    Serial.println("Ultimate-NAG52 Baselines Flashed to Memory!");
}

// ... [Keep existing getLoadBin, getRpmBin, getModifier, evaluateShift, getTablePtr, saveTable unchanged] ...

uint8_t AdaptiveMemory::getLoadBin(float tps_pct, float map_kpa) {
    float load = tps_pct;
    if (map_kpa > 100.0f) load = tps_pct + ((map_kpa - 100.0f) * 1.5f);
    return constrain((int)(load / 10.0f), 0, NUM_LOAD_BINS - 1);
}

uint8_t AdaptiveMemory::getRpmBin(float engine_rpm) {
    return constrain((int)((engine_rpm - 400.0f) / 400.0f), 0, NUM_RPM_BINS - 1);
}

int8_t AdaptiveMemory::getModifier(bool is_upshift, uint8_t shift_idx, bool is_pressure, float tps, float map_kpa, float rpm) {
    uint8_t l = getLoadBin(tps, map_kpa);
    uint8_t r = getRpmBin(rpm);
    if (is_upshift) return is_pressure ? pressure_modifiers[shift_idx][l][r] : prefill_modifiers[shift_idx][l][r];
    return is_pressure ? ds_pressure_modifiers[shift_idx][l][r] : ds_timing_modifiers[shift_idx][l][r];
}

void AdaptiveMemory::evaluateShift(uint8_t from_gear, uint8_t to_gear, float tps_pct, float map_kpa, float engine_rpm, unsigned long shift_time_ms, bool flare, bool bind) {
    if (from_gear == to_gear) return;
    bool is_upshift = (to_gear > from_gear);
    uint8_t shift_idx = is_upshift ? (from_gear - 1) : (to_gear - 1); 
    uint8_t load_idx  = getLoadBin(tps_pct, map_kpa);
    uint8_t rpm_idx   = getRpmBin(engine_rpm);
    
    if (is_upshift) {
        if (flare) { pressure_modifiers[shift_idx][load_idx][rpm_idx] += 2; }
        else if (bind) { pressure_modifiers[shift_idx][load_idx][rpm_idx] -= 2; }
        saveTable(true, shift_idx, true);
    } else {
        if (bind) { ds_timing_modifiers[shift_idx][load_idx][rpm_idx] += 5; } 
        else if (flare) { ds_pressure_modifiers[shift_idx][load_idx][rpm_idx] += 2; } 
        saveTable(false, shift_idx, false);
        saveTable(false, shift_idx, true);
    }
}

int8_t* AdaptiveMemory::getTablePtr(bool is_upshift, uint8_t shift_index, bool is_pressure) {
    if (is_upshift) return is_pressure ? &pressure_modifiers[shift_index][0][0] : &prefill_modifiers[shift_index][0][0];
    return is_pressure ? &ds_pressure_modifiers[shift_index][0][0] : &ds_timing_modifiers[shift_index][0][0];
}

void AdaptiveMemory::saveTable(bool is_upshift, uint8_t shift_index, bool is_pressure) {
    String key = (is_upshift ? (is_pressure ? "P_TBL_" : "F_TBL_") : (is_pressure ? "DP_TBL_" : "DT_TBL_")) + String(shift_index);
    int8_t* data = getTablePtr(is_upshift, shift_index, is_pressure);
    preferences.putBytes(key.c_str(), data, 256);
}