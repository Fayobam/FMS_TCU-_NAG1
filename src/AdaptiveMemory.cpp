// ============================================================================
// FILE: AdaptiveMemory.cpp
// VERSION: 6.0
// ============================================================================
#include "AdaptiveMemory.h"

AdaptiveMemory::AdaptiveMemory() {}

int8_t AdaptiveMemory::clampPressure(int v) {
    return (int8_t)constrain(v, PRESSURE_MOD_MIN, PRESSURE_MOD_MAX);
}
int8_t AdaptiveMemory::clampTiming(int v) {
    return (int8_t)constrain(v, TIMING_MOD_MIN, TIMING_MOD_MAX);
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
            // UPSHIFT PREFILL TIMING  (base 60ms + mod = total fill time)
            // Starting conservative (long) so early shifts are soft — adaptive tightens.
            // Source: NAG52 empirical data + W5A330 clutch volume ratios.
            prefill_modifiers[0][l][r] = clampTiming(30);  // 1->2 B1 brake:   60+30 = 90ms
            prefill_modifiers[1][l][r] = clampTiming(50);  // 2->3 K1 clutch:  60+50 = 110ms
            prefill_modifiers[2][l][r] = clampTiming(100); // 3->4 K2 clutch:  60+100= 160ms (largest drum, flare-prone)
            prefill_modifiers[3][l][r] = clampTiming(20);  // 4->5 B1 brake:   60+20 = 80ms

            // UPSHIFT OVERLAP PRESSURE  (base 45% SPC + load mod = initial overlap pressure)
            // Performance bias: l*4 gives 0→60% across load bins (was l*3 = 0→45%).
            // K2 gets +10% extra. Adaptive pulls back 2% per bind — start firm.
            int8_t load_p = clampPressure(l * 4); // 0% at idle, 60% at full boost load
            pressure_modifiers[0][l][r] = load_p;
            pressure_modifiers[1][l][r] = load_p;
            pressure_modifiers[2][l][r] = clampPressure(load_p + 10); // K2 extra
            pressure_modifiers[3][l][r] = load_p;

            // DOWNSHIFT RELEASE TIMING  (base 80ms + mod = engine rev-match window)
            // Longer for higher gears: larger ratio delta needs more time for engine
            // inertia to close the gap before the catch clutch applies.
            ds_timing_modifiers[0][l][r] = clampTiming(20);  // 2->1:  80+20 = 100ms
            ds_timing_modifiers[1][l][r] = clampTiming(50);  // 3->2:  80+50 = 130ms
            ds_timing_modifiers[2][l][r] = clampTiming(100); // 4->3:  80+100= 180ms
            ds_timing_modifiers[3][l][r] = clampTiming(120); // 5->4:  80+120= 200ms

            // DOWNSHIFT CATCH PRESSURE  (same performance-biased load scaling)
            ds_pressure_modifiers[0][l][r] = load_p;
            ds_pressure_modifiers[1][l][r] = load_p;
            ds_pressure_modifiers[2][l][r] = clampPressure(load_p + 5); // 4->3 extra
            ds_pressure_modifiers[3][l][r] = clampPressure(load_p + 5); // 5->4 extra
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
        if (flare)     pressure_modifiers[shift_idx][l][r] = clampPressure(pressure_modifiers[shift_idx][l][r] + 2);
        else if (bind) pressure_modifiers[shift_idx][l][r] = clampPressure(pressure_modifiers[shift_idx][l][r] - 2);
        saveTable(true, shift_idx, true);
    } else {
        if (flare)     ds_pressure_modifiers[shift_idx][l][r] = clampPressure(ds_pressure_modifiers[shift_idx][l][r] + 2);
        else if (bind) ds_timing_modifiers[shift_idx][l][r]   = clampTiming(ds_timing_modifiers[shift_idx][l][r] + 5);
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
