// ============================================================================
// FILE: AdaptiveMemory.cpp
// VERSION: 6.0
// ============================================================================
#include "AdaptiveMemory.h"

AdaptiveMemory::AdaptiveMemory() {}

int8_t AdaptiveMemory::clampMod(int v) {
    if (v < MOD_MIN) v = MOD_MIN;
    if (v > MOD_MAX) v = MOD_MAX;
    return (int8_t)v;
}

void AdaptiveMemory::begin() {
    preferences.begin("tcu_adapt", false);

    if (preferences.getBytesLength("P_TBL_0") != sizeof(pressure_modifiers[0])) {
        Serial.println("Blank Flash Detected! Injecting Ultimate-NAG52 Baselines...");
        loadUltimateNag52Defaults();
    } else {
        for (int i = 0; i < NUM_UPSHIFTS; i++) {
            preferences.getBytes(("P_TBL_"  + String(i)).c_str(), &pressure_modifiers[i],    256);
            preferences.getBytes(("F_TBL_"  + String(i)).c_str(), &prefill_modifiers[i],     256);
            preferences.getBytes(("DP_TBL_" + String(i)).c_str(), &ds_pressure_modifiers[i], 256);
            preferences.getBytes(("DT_TBL_" + String(i)).c_str(), &ds_timing_modifiers[i],   256);
        }
        Serial.println("Adaptive Memory V6.0 Loaded from Flash.");
    }
}

void AdaptiveMemory::loadUltimateNag52Defaults() {
    memset(pressure_modifiers, 0, sizeof(pressure_modifiers));
    memset(prefill_modifiers, 0, sizeof(prefill_modifiers));
    memset(ds_pressure_modifiers, 0, sizeof(ds_pressure_modifiers));
    memset(ds_timing_modifiers, 0, sizeof(ds_timing_modifiers));

    for (int l = 0; l < NUM_LOAD_BINS; l++) {
        for (int r = 0; r < NUM_RPM_BINS; r++) {
            // UPSHIFT PREFILLS (base 60ms + mod)
            prefill_modifiers[0][l][r] = -15; // 1->2 B1 small volume
            prefill_modifiers[1][l][r] = -5;  // 2->3 K1 medium
            prefill_modifiers[2][l][r] = 20;  // 3->4 K2 huge volume (anti-flare)
            prefill_modifiers[3][l][r] = -15; // 4->5 B1

            // UPSHIFT OVERLAP PRESSURES (base + load scaling), clamped
            int8_t load_pressure_add = clampMod(l * 3);
            pressure_modifiers[0][l][r] = load_pressure_add;
            pressure_modifiers[1][l][r] = load_pressure_add;
            pressure_modifiers[2][l][r] = clampMod(load_pressure_add + 5); // K2 extra clamp
            pressure_modifiers[3][l][r] = load_pressure_add;

            // DOWNSHIFT REV-MATCH TIMING (base 80ms + mod)
            ds_timing_modifiers[0][l][r] = 20;  // 2->1
            ds_timing_modifiers[1][l][r] = 50;  // 3->2
            ds_timing_modifiers[2][l][r] = clampMod(100); // 4->3
            ds_timing_modifiers[3][l][r] = clampMod(150); // 5->4 (clamped to 60)

            ds_pressure_modifiers[0][l][r] = load_pressure_add;
            ds_pressure_modifiers[1][l][r] = load_pressure_add;
            ds_pressure_modifiers[2][l][r] = load_pressure_add;
            ds_pressure_modifiers[3][l][r] = load_pressure_add;
        }
    }

    for (int i = 0; i < NUM_UPSHIFTS; i++) {
        saveTable(true, i, true);
        saveTable(true, i, false);
        saveTable(false, i, true);
        saveTable(false, i, false);
    }
    Serial.println("Ultimate-NAG52 Baselines Flashed to Memory!");
}

// Shared load model (matches ShiftScheduler exactly)
uint8_t AdaptiveMemory::getLoadBin(float tps_pct, float map_kpa) {
    return loadToBin(computeLoad(tps_pct, map_kpa));
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

void AdaptiveMemory::evaluateShift(bool is_upshift, uint8_t shift_idx, float tps_pct, float map_kpa,
                                   float engine_rpm, unsigned long shift_time_ms, bool flare, bool bind) {
    if (shift_idx >= NUM_UPSHIFTS) return; // bounds guard
    uint8_t l = getLoadBin(tps_pct, map_kpa);
    uint8_t r = getRpmBin(engine_rpm);

    if (is_upshift) {
        // Flare = clutch slipping too long -> raise pressure. Bind = too harsh -> lower.
        if (flare)      pressure_modifiers[shift_idx][l][r] = clampMod(pressure_modifiers[shift_idx][l][r] + 2);
        else if (bind)  pressure_modifiers[shift_idx][l][r] = clampMod(pressure_modifiers[shift_idx][l][r] - 2);
        saveTable(true, shift_idx, true);
    } else {
        // Downshift: flare (overshoot before catch) -> raise catch pressure.
        // bind (engagement too abrupt) -> lengthen sync time.
        if (flare)      ds_pressure_modifiers[shift_idx][l][r] = clampMod(ds_pressure_modifiers[shift_idx][l][r] + 2);
        else if (bind)  ds_timing_modifiers[shift_idx][l][r]   = clampMod(ds_timing_modifiers[shift_idx][l][r] + 5);
        saveTable(false, shift_idx, true);
        saveTable(false, shift_idx, false);
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
