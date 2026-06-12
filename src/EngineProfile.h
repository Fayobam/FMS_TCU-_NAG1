// ============================================================================
// FILE: EngineProfile.h
// VERSION: 1.0
// Per-ENGINE calibration, NVS-backed and web-editable so an engine swap (e.g. the
// M111 Kompressor → an NA M112 V6) is a data change, not a recompile.
//
// Holds:
//   - an 8×8 torque surface (RPM × MAP → Nm), bilinear-interpolated — replaces the
//     single linear MAP→torque fit, which cannot represent a curved NA torque curve;
//   - engine limits (overrev / lug RPM);
//   - sensor calibration (TPS volts, MAP transfer function);
//   - baseline upshift fill tables (pressure %, time ms).
//
// Gearbox-side values (gear ratios, K=1.641) stay compile-time — they are the same
// regardless of engine. The adaptation CELLS (AdaptiveMemory) are learned trims that
// sit ON TOP of the baselines here and correctly start at zero.
// ============================================================================
#pragma once
#include <Arduino.h>
#include <Preferences.h>

#define EP_RPM_BINS 8
#define EP_MAP_BINS 8
#define EP_MAGIC    0x4E414733u   // 'NAG3' — bump if the struct layout changes (v3: + out_ppr)

struct EngineProfileData {
    int16_t  torque[EP_RPM_BINS][EP_MAP_BINS];  // Nm on the RPM×MAP grid
    uint16_t rpm_bp[EP_RPM_BINS];               // RPM breakpoints (ascending)
    uint16_t map_bp[EP_MAP_BINS];               // MAP kPa breakpoints (ascending)
    uint16_t t_max_ref;                         // reference torque for load_pct (≈ clutch capacity)
    uint16_t overrev_rpm;                       // forced-upshift / redline guard
    uint16_t lug_rpm;                           // lug-protection threshold
    uint16_t eng_ppr;                           // engine speed input pulses/rev (tach feed)
    uint16_t out_ppr;                           // output-shaft sensor pulses/rev (custom wheel)
    float    tps_closed_v, tps_wot_v;           // TPS calibration
    float    map_kpa_at_0v, map_kpa_per_volt;   // MAP transfer function
    uint8_t  fill_p[4];                         // baseline upshift fill pressure % (1-2,2-3,3-4,4-5)
    uint16_t fill_t[4];                         // baseline upshift fill time ms
    uint32_t magic;                             // sanity/version tag
};

class EngineProfile {
  private:
    Preferences prefs;
    EngineProfileData d;
    void seedDefaults();

  public:
    void begin();
    void save();

    float   estimateTorque(float rpm, float map_kpa);  // bilinear Nm
    float   loadPct(float rpm, float map_kpa);         // 0..100% of t_max_ref
    uint8_t torqueBin(float rpm, float map_kpa);       // 0..3

    uint16_t overrevRpm() const { return d.overrev_rpm; }
    uint16_t lugRpm()     const { return d.lug_rpm; }
    uint16_t engPpr()     const { return d.eng_ppr ? d.eng_ppr : 60; }  // never 0 (div-by-zero guard)
    uint16_t outPpr()     const { return d.out_ppr ? d.out_ppr : 24; }
    float    tpsClosedV() const { return d.tps_closed_v; }
    float    tpsWotV()    const { return d.tps_wot_v; }
    float    mapAt0V()    const { return d.map_kpa_at_0v; }
    float    mapPerV()    const { return d.map_kpa_per_volt; }
    uint8_t  fillP(uint8_t i)   { return d.fill_p[constrain((int)i,0,3)]; }
    uint16_t fillT(uint8_t i)   { return d.fill_t[constrain((int)i,0,3)]; }

    EngineProfileData* raw() { return &d; }            // for the web tuner
};

extern EngineProfile engineProfile;
