// ============================================================================
// FILE: EngineProfile.cpp
// VERSION: 1.0
// ============================================================================
#include "EngineProfile.h"

void EngineProfile::begin() {
    prefs.begin("engine_prof", false);
    if (prefs.getBytesLength("data") == sizeof(d)) {
        prefs.getBytes("data", &d, sizeof(d));
        if (d.magic == EP_MAGIC) { Serial.println("Engine profile loaded from flash."); return; }
    }
    Serial.println("Engine profile: blank/!match — seeding M111+TVS1320 defaults.");
    seedDefaults();
    save();
}

void EngineProfile::save() {
    d.magic = EP_MAGIC;
    prefs.putBytes("data", &d, sizeof(d));
}

void EngineProfile::seedDefaults() {
    // Axes: 0..7000 rpm, 20..240 kPa absolute (full vacuum to ~1.2 bar boost).
    const uint16_t rpm[EP_RPM_BINS] = { 0, 1000, 2000, 3000, 4000, 5000, 6000, 7000 };
    const uint16_t mp [EP_MAP_BINS] = { 20, 50, 80, 110, 140, 170, 205, 240 };
    for (int i = 0; i < EP_RPM_BINS; i++) d.rpm_bp[i] = rpm[i];
    for (int j = 0; j < EP_MAP_BINS; j++) d.map_bp[j] = mp[j];

    // Seed the surface from the M111+TVS1320 linear MAP fit (RPM-flat — a blower is
    // nearly RPM-independent). Re-shape per-RPM for an NA engine via the web tuner.
    //   T = clamp(2.43·(map − 35), 0, 450)
    for (int i = 0; i < EP_RPM_BINS; i++) {
        for (int j = 0; j < EP_MAP_BINS; j++) {
            float t = 2.43f * ((float)mp[j] - 35.0f);
            t = constrain(t, 0.0f, 450.0f);
            d.torque[i][j] = (int16_t)(t + 0.5f);
        }
    }

    d.t_max_ref       = 450;
    d.overrev_rpm     = 6300;
    d.lug_rpm         = 1100;
    d.eng_ppr         = 60;     // M111 60-tooth default; use rusEFI's clean tach out, NOT raw 60-2
    d.out_ppr         = 24;     // current custom output-shaft wheel; 48-60 recommended

    d.tps_closed_v    = 0.50f;
    d.tps_wot_v       = 2.90f;
    d.map_kpa_at_0v   = -10.0f;
    d.map_kpa_per_volt = 86.0f;
    d.fill_p[0] = 80; d.fill_p[1] = 82; d.fill_p[2] = 88; d.fill_p[3] = 78;
    d.fill_t[0] = 140; d.fill_t[1] = 150; d.fill_t[2] = 180; d.fill_t[3] = 130;
}

float EngineProfile::estimateTorque(float rpm, float map_kpa) {
    int i = 0; while (i < EP_RPM_BINS - 1 && rpm >= d.rpm_bp[i + 1]) i++;
    int i2 = (i < EP_RPM_BINS - 1) ? i + 1 : i;
    float r0 = d.rpm_bp[i], r1 = d.rpm_bp[i2];
    float fr = (r1 > r0) ? constrain((rpm - r0) / (r1 - r0), 0.0f, 1.0f) : 0.0f;

    int j = 0; while (j < EP_MAP_BINS - 1 && map_kpa >= d.map_bp[j + 1]) j++;
    int j2 = (j < EP_MAP_BINS - 1) ? j + 1 : j;
    float m0 = d.map_bp[j], m1 = d.map_bp[j2];
    float fm = (m1 > m0) ? constrain((map_kpa - m0) / (m1 - m0), 0.0f, 1.0f) : 0.0f;

    float a = d.torque[i][j]  + (d.torque[i][j2]  - d.torque[i][j])  * fm;   // interp along MAP, low rpm
    float b = d.torque[i2][j] + (d.torque[i2][j2] - d.torque[i2][j]) * fm;   // interp along MAP, high rpm
    return fmaxf(0.0f, a + (b - a) * fr);                                    // interp along RPM
}

float EngineProfile::loadPct(float rpm, float map_kpa) {
    float ref = (d.t_max_ref > 0) ? (float)d.t_max_ref : 1.0f;
    return constrain(100.0f * estimateTorque(rpm, map_kpa) / ref, 0.0f, 100.0f);
}

uint8_t EngineProfile::torqueBin(float rpm, float map_kpa) {
    return (uint8_t)constrain((int)(loadPct(rpm, map_kpa) / 25.0f), 0, 3);
}
