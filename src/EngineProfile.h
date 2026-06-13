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
#include "TCU_Data.h"   // TransVariant / TransSpec / g_trans

#define EP_RPM_BINS 8
#define EP_MAP_BINS 8
#define EP_MAGIC    0x4E414738u   // 'NAG8' — bump if the struct layout changes (v8: UN52 pressure coeffs)

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
    uint8_t  cl_spc_enable;                     // closed-loop SPC in upshift INERTIA (0/1)
    uint16_t cl_spc_kp;                         // P gain: SPC%-trim per unit ratio-schedule error
    uint8_t  trans_variant;                     // TransVariant: 0=small NAG (W5A330), 1=big NAG (W5A580)
    uint16_t tc_stall_mult_x100;                // converter torque multiplication at stall (×100, e.g. 200 = 2.0×)
    uint16_t tc_coupling_sr_x100;               // speed ratio (turbine/engine ×100) at the coupling point (mult→1.0)
    // --- Physical pressure-from-torque model (UN52 backport Phase 3) ---
    // Mirrors UN52 pressure_manager: P_mBar = torque_Nm · friction / coef + return_spring.
    // Coefficients are the authentic UN52 PRM_DEFAULT_SETTINGS; friction numerators + springs
    // are the per-car EGS52 calibration blob (NOT in the open firmware) → reasoned seeds, tune.
    // Per gear-pair idx 0..3 = 1-2, 2-3, 3-4, 4-5 (same physical pair handles the downshift).
    uint8_t  cl_pressure_enable;                // 0 = heuristic % path (default), 1 = torque→pressure model
    uint8_t  coef_stationary;                   // held-clutch coefficient (UN52: 100)
    uint8_t  coef_releasing;                    // off-going (release) coefficient (UN52: 120)
    uint8_t  coef_apply_cold;                   // oncoming coef at 29°C ATF (UN52: 185)
    uint8_t  coef_apply_hot;                    // oncoming coef at 65°C ATF (UN52: 140)
    uint16_t apply_friction[4];                 // oncoming-clutch friction numerator
    uint16_t release_friction[4];               // off-going-clutch friction numerator
    uint16_t apply_spring_mbar[4];              // oncoming return-spring preload (mBar)
    uint16_t release_spring_mbar[4];            // off-going return-spring preload (mBar)
    uint16_t p_full_scale_mbar;                 // line pressure at 100% command (mBar→% solenoid map)
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
    void applyTransVariant();   // load TRANS_SPECS[trans_variant] into g_trans (call after any change)
    uint8_t transVariant() const { return d.trans_variant; }

    float   estimateTorque(float rpm, float map_kpa);  // bilinear ENGINE Nm
    float   loadPct(float rpm, float map_kpa);         // 0..100% of t_max_ref (engine-torque based)
    uint8_t torqueBin(float rpm, float map_kpa);       // 0..3

    // Converter torque multiplication: the clutches see INPUT (turbine) torque, which a
    // fluid converter multiplies at low speed ratio (≈2× at stall → 1.0 at the coupling
    // point). 1.0 when locked/coupled. Phase 3's pressure model is denominated in input Nm.
    float   converterFactor(float engine_rpm, float turbine_rpm) const {
        if (engine_rpm < 1.0f) return 1.0f;
        float sr = turbine_rpm / engine_rpm;                         // speed ratio 0..~1
        float stall = (d.tc_stall_mult_x100 ? d.tc_stall_mult_x100 : 200) / 100.0f;
        float coupling = (d.tc_coupling_sr_x100 ? d.tc_coupling_sr_x100 : 85) / 100.0f;
        if (sr >= coupling) return 1.0f;
        float f = stall + (1.0f - stall) * (sr / coupling);         // stall@sr=0 → 1.0@coupling
        return constrain(f, 1.0f, stall);
    }
    float   inputTorque(float engine_rpm, float turbine_rpm, float map_kpa) {
        return estimateTorque(engine_rpm, map_kpa) * converterFactor(engine_rpm, turbine_rpm);
    }

    // --- Physical pressure model (Phase 3), UN52 form: P = T·friction/coef + spring ---
    bool clPressureEnable() const { return d.cl_pressure_enable != 0; }
    // Oncoming (sliding) coefficient, ATF-temp interpolated 29→65 °C (cold ATF grips more → higher
    // coef → less pressure), clamped to the endpoints — exactly UN52 PressureManager::sliding_coefficient.
    float slidingCoef(float atf_c) const {
        float cold = d.coef_apply_cold ? d.coef_apply_cold : 185;
        float hot  = d.coef_apply_hot  ? d.coef_apply_hot  : 140;
        if (atf_c <= 29.0f) return cold;
        if (atf_c >= 65.0f) return hot;
        return cold + (hot - cold) * ((atf_c - 29.0f) / 36.0f);
    }
    // Oncoming clutch: pressure (mBar) to carry `torque_nm` through gear-pair `idx`.
    float clutchApplyMbar(uint8_t idx, float torque_nm, float atf_c) const {
        idx = (idx > 3) ? 3 : idx;
        float fr = d.apply_friction[idx] ? d.apply_friction[idx] : 4000;
        return fmaxf(0.0f, fmaxf(0.0f, torque_nm) * fr / slidingCoef(atf_c) + (float)d.apply_spring_mbar[idx]);
    }
    // Off-going clutch: line pressure (mBar) to HOLD `torque_nm` during overlap (release coef).
    float clutchReleaseMbar(uint8_t idx, float torque_nm) const {
        idx = (idx > 3) ? 3 : idx;
        float fr = d.release_friction[idx] ? d.release_friction[idx] : 4000;
        float coef = d.coef_releasing ? d.coef_releasing : 120;
        return fmaxf(0.0f, fmaxf(0.0f, torque_nm) * fr / coef + (float)d.release_spring_mbar[idx]);
    }
    // Map a commanded pressure (mBar) to the solenoid's pressure-% command (0..100).
    uint8_t mbarToPct(float mbar) const {
        float fs = (d.p_full_scale_mbar ? d.p_full_scale_mbar : 16000);
        return (uint8_t)constrain(mbar / fs * 100.0f, 0.0f, 100.0f);
    }

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
    bool     clSpcEnable() const { return d.cl_spc_enable != 0; }
    float    clSpcKp()     const { return (float)d.cl_spc_kp; }   // SPC%-trim per unit ratio error

    EngineProfileData* raw() { return &d; }            // for the web tuner
};

extern EngineProfile engineProfile;
