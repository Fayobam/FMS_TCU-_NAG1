// ============================================================================
// FILE: TuneOverlay.cpp
// VERSION: 1.0
// ============================================================================
#include "TuneOverlay.h"
#include "TCU_Data.h"
#include "AutoShiftMap.h"   // AUTO_UPSHIFT_KMH / AUTO_DOWNSHIFT_KMH defaults + autoMapInterp

// ----------------------------------------------------------------------------
// COMPILE-TIME DEFAULTS — the seed source, and the documentation of record.
// Moved here from ShiftScheduler.h so the overlay can re-seed from them.
// ----------------------------------------------------------------------------
//
// Cruise (holding) line pressure, pressure-% (100 = de-energized = max).
//
// REBASED on dueATC's road-tuned #MPC_normalDrive (Reference/DUEATC_CALIBRATION_NOTES.md
// §3): same solenoid, same inverted convention, and actually driven. Its 60 °C row is
// 70/70/73/100/100/100 across load 0/20/40/60/80/100 — a FLOOR OF 70 at light load,
// saturating by ~60 % load. Our previous floor was 20-38, inferred rather than driven,
// and low light-load line is what lets a clutch slip when an unexpected torque spike
// arrives — exactly what a blower delivers off-idle, on a box already at its 330 Nm rating.
//
// Their map has no gear axis; the modest +2/gear step is ours and is kept. Temperature is
// NOT here — cruiseLinePressure() applies the ATF multiplier, which reproduces their cold
// row (70 × 1.30 = 91; their -20 °C row is 100).
// SPORT BIAS: the 73->100 ramp is one load bin earlier than a literal read of their curve.
static const uint8_t DEF_LINE_MAP[TUNE_GEARS][TUNE_LOAD_BINS] = {
    { 70, 70, 70, 72, 75, 90, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 }, // G1
    { 72, 72, 72, 74, 77, 92, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 }, // G2
    { 74, 74, 74, 76, 79, 94, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 }, // G3
    { 76, 76, 76, 78, 81, 96, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 }, // G4
    { 78, 78, 78, 80, 83, 98, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 }  // G5
};

void TuneOverlay::seedDefaults() {
    memcpy(d.line_map, DEF_LINE_MAP, sizeof(d.line_map));
    for (uint8_t i = 0; i < TUNE_LOAD_PTS; i++) d.inertia_target_ms[i] = INERTIA_TARGET_MS[i];
    d.apply_floor              = 52;    // dueATC SPC floor ~50 (+2 sport bias)
    d.apply_slope_x100         = 90;    // 0.90 %/load
    d.inertia_slope_x100       = 200;   // 2.00 %/20ms tick base
    d.inertia_slope_load_x1000 = 20;    // +0.020 %/tick per load%
    d.backstop_hot_ms          = SHIFT_BACKSTOP_HOT_MS;
    d.backstop_cold_ms         = SHIFT_BACKSTOP_COLD_MS;

    memcpy(d.up_kmh, AUTO_UPSHIFT_KMH,   sizeof(d.up_kmh));
    memcpy(d.dn_kmh, AUTO_DOWNSHIFT_KMH, sizeof(d.dn_kmh));

    for (uint8_t i = 0; i < TUNE_MODES; i++) {
        d.dm_firmness_x100[i] = (uint8_t)(DRIVE_MODES[i].firmness       * 100.0f + 0.5f);
        d.dm_shiftpt_x100[i]  = (uint8_t)(DRIVE_MODES[i].shift_pt_scale * 100.0f + 0.5f);
        d.dm_tcc_open_tps[i]  = (uint8_t)DRIVE_MODES[i].tcc_open_tps;
        d.dm_launch_gear[i]   = DRIVE_MODES[i].launch_gear;
        d.dm_auto_shift[i]    = DRIVE_MODES[i].auto_shift ? 1 : 0;
        d.dm_lug_guard[i]     = DRIVE_MODES[i].lug_guard  ? 1 : 0;
        d.dm_torque_cut[i]    = DRIVE_MODES[i].torque_cut ? 1 : 0;
    }
    d.magic = TUNE_MAGIC;
}

// The schedule tables are stored exactly as AutoShiftMap.h lays them out, so the
// same 11-point TPS interpolation applies.
float TuneOverlay::upshiftKmh(uint8_t pair, float tps) const {
    return autoMapInterp(d.up_kmh[pair > (TUNE_PAIRS - 1) ? (TUNE_PAIRS - 1) : pair], tps);
}
float TuneOverlay::downshiftKmh(uint8_t pair, float tps) const {
    return autoMapInterp(d.dn_kmh[pair > (TUNE_PAIRS - 1) ? (TUNE_PAIRS - 1) : pair], tps);
}

// Defined AFTER the default tables above. Those are const PODs with constant
// initialisers, so they live in .rodata and are valid before any constructor runs —
// which is what makes seeding from the constructor safe regardless of init order.
TuneOverlay tuneOverlay;

void TuneOverlay::begin() {
    prefs.begin("tcu_tune", false);
    if (prefs.getBytesLength("data") == sizeof(d)) {
        prefs.getBytes("data", &d, sizeof(d));
        if (d.magic == TUNE_MAGIC) { Serial.println("Tune overlay loaded from flash."); return; }
    }
    Serial.println("Tune overlay: blank/!match — seeding compile-time defaults.");
    seedDefaults();
    save();
}

void TuneOverlay::save() {
    d.magic = TUNE_MAGIC;
    prefs.putBytes("data", &d, sizeof(d));
}

void TuneOverlay::resetToDefaults() { seedDefaults(); save(); }

// Linear interpolation of the 11-point load curve (0,10,...,100 %).
uint16_t TuneOverlay::inertiaTargetMs(float load) const {
    if (load <= 0.0f)   return d.inertia_target_ms[0];
    if (load >= 100.0f) return d.inertia_target_ms[TUNE_LOAD_PTS - 1];
    float pos = load / 10.0f;
    int   i   = (int)pos;
    float fr  = pos - (float)i;
    return (uint16_t)((float)d.inertia_target_ms[i] +
                      ((float)d.inertia_target_ms[i + 1] - (float)d.inertia_target_ms[i]) * fr);
}

// ============================================================================
// PARAMETER REGISTRY
// Order here IS the wire order (index = param id on the wire). Append only.
// ============================================================================
enum : uint8_t {
    P_LINE_MAP = 0, P_INERTIA_TARGET, P_APPLY_FLOOR, P_APPLY_SLOPE,
    P_INERTIA_SLOPE, P_INERTIA_SLOPE_LOAD, P_BACKSTOP_HOT, P_BACKSTOP_COLD,
    P_UP_KMH, P_DN_KMH,
    P_DM_FIRMNESS, P_DM_SHIFTPT, P_DM_TCCOPEN, P_DM_LAUNCH,
    P_DM_AUTO, P_DM_LUG, P_DM_TQCUT,
    P_COUNT
};

static const ParamDesc PARAMS[P_COUNT] = {
    { "line.map", "Cruise line pressure", "Shift feel", PK_TABLE2D, 10, 100, TUNE_GEARS, TUNE_LOAD_BINS, "%",
      "Holding pressure while NOT shifting. Rows=gear 1-5, cols=load bin (0-15, ~12.5 load units each). "
      "100 = solenoid de-energized = maximum pressure. Low light-load values save heat and pump load but "
      "let a clutch slip on a sudden torque spike. Floor of 70 comes from a road-tuned 722.6 (dueATC)." },

    { "inertia.target", "Inertia duration target", "Shift feel", PK_CURVE1D, 120, 900, 1, TUNE_LOAD_PTS, "ms",
      "Target length of the upshift speed-change phase, by load% (0,10,...,100). Drives the closed-loop "
      "ratio schedule and the harshness detector (finishing under 0.6x this = flagged too firm). "
      "Long at light load reads as refined; short at high load reads as sporty." },

    { "apply.floor", "Apply pressure floor", "Shift feel", PK_SCALAR, 0, 100, 1, 1, "%",
      "Torque-phase apply pressure at zero load: apply% = floor + slope x load. This is the ramp's "
      "starting point. Raising it firms up light-throttle shifts; too high is harsh." },

    { "apply.slope", "Apply pressure slope", "Shift feel", PK_SCALAR, 0, 200, 1, 1, "x0.01",
      "How much apply pressure rises per 1% load (value/100). 90 = +0.90%/load, so full load adds 90%." },

    { "inertia.slope", "Inertia ramp base", "Shift feel", PK_SCALAR, 0, 800, 1, 1, "x0.01",
      "Base SPC ramp rate during the inertia phase, %-per-20ms-tick (value/100). Higher pulls the ratio "
      "home faster and firmer." },

    { "inertia.slope.load", "Inertia ramp load gain", "Shift feel", PK_SCALAR, 0, 200, 1, 1, "x0.001",
      "Extra inertia ramp rate per 1% load (value/1000). Makes high-load shifts ramp harder than light ones." },

    { "backstop.hot", "Phase backstop (hot)", "Safety timing", PK_SCALAR, 300, 3000, 1, 1, "ms",
      "How long a shift phase may run at ATF >= 60C before it is abandoned as unverified. NOT a feel knob: "
      "too short and normal shifts get abandoned with a DTC; too long delays fault detection." },

    { "backstop.cold", "Phase backstop (cold)", "Safety timing", PK_SCALAR, 300, 4000, 1, 1, "ms",
      "Same, at ATF <= 0C, interpolated in between. Cold oil fills slowly — a road-tuned 722.6 holds its "
      "shift solenoid up to 2000ms when cold vs 900ms hot, so this must be generous." },

    { "auto.up", "Upshift speeds", "Auto schedule", PK_TABLE2D, 3, 250, TUNE_PAIRS, TUNE_TPS_PTS, "km/h",
      "Road speed at which an automatic upshift fires. Rows = 1-2 / 2-3 / 3-4 / 4-5, cols = TPS% "
      "(0,10,...,100). Scaled by the mode's shift-point factor. Must stay ABOVE the matching "
      "downshift row or the box will hunt between two gears." },

    { "auto.dn", "Downshift speeds", "Auto schedule", PK_TABLE2D, 2, 250, TUNE_PAIRS, TUNE_TPS_PTS, "km/h",
      "Road speed below which an automatic downshift fires. Rows = 2-1 / 3-2 / 4-3 / 5-4, cols = TPS%. "
      "The gap to the upshift row IS the anti-hunting hysteresis — keep a healthy margin. "
      "Auto downshifts floor at 2nd; 1st is reached by the launch drop or a paddle." },

    { "mode.firmness", "Firmness", "Drive modes (D/4/3/2/1)", PK_CURVE1D, 50, 200, 1, TUNE_MODES, "x0.01",
      "Multiplies apply pressure and ramp rate per lever position (value/100). 100 = baseline. "
      "Higher = firmer, faster shifts. Does NOT scale fill — that only seats the piston." },

    { "mode.shiftpt", "Shift-point scale", "Drive modes (D/4/3/2/1)", PK_CURVE1D, 50, 200, 1, TUNE_MODES, "x0.01",
      "Stretches the whole auto schedule per lever position (value/100). Below 100 upshifts early "
      "(economy); above 100 holds gears longer (sport)." },

    { "mode.tccopen", "TCC open above TPS", "Drive modes (D/4/3/2/1)", PK_CURVE1D, 0, 100, 1, TUNE_MODES, "%",
      "Throttle above which the converter clutch is forced open. Lower = unlocks sooner = more slip "
      "and a sportier feel, at the cost of some heat and efficiency." },

    { "mode.launch", "Launch gear", "Drive modes (D/4/3/2/1)", PK_CURVE1D, 1, 2, 1, TUNE_MODES, "gear",
      "Gear a nearly-stopped car drops to before pulling away. 1 = full 3.93 first ratio; "
      "2 = the 722.6 hydraulic default, gentler and better on ice." },

    { "mode.auto", "Auto shifting", "Drive modes (D/4/3/2/1)", PK_CURVE1D, 0, 1, 1, TUNE_MODES, "0/1",
      "1 = the automatic schedule and kickdown run for this lever position; 0 = paddle-only. "
      "Overrev and lug protection stay active either way." },

    { "mode.lug", "Lug protection", "Drive modes (D/4/3/2/1)", PK_CURVE1D, 0, 1, 1, TUNE_MODES, "0/1",
      "1 = auto-downshift when the engine bogs under load. Off in RACE so the driver keeps control." },

    { "mode.tqcut", "Request torque cut", "Drive modes (D/4/3/2/1)", PK_CURVE1D, 0, 1, 1, TUNE_MODES, "0/1",
      "1 = assert the rusEFI shift-retard line during high-load power upshifts. Still gated by the "
      "ENABLE_TORQUE_CUT build flag, so this does nothing until that wire exists." },
};

uint8_t          TuneOverlay::paramCount()          { return P_COUNT; }
const ParamDesc& TuneOverlay::paramDesc(uint8_t i)  { return PARAMS[i < P_COUNT ? i : 0]; }

int16_t TuneOverlay::paramGet(uint8_t i, uint8_t row, uint8_t col) const {
    switch (i) {
        case P_LINE_MAP:
            if (row >= TUNE_GEARS || col >= TUNE_LOAD_BINS) return 0;
            return d.line_map[row][col];
        case P_INERTIA_TARGET:
            if (col >= TUNE_LOAD_PTS) return 0;
            return (int16_t)d.inertia_target_ms[col];
        case P_APPLY_FLOOR:          return d.apply_floor;
        case P_APPLY_SLOPE:          return d.apply_slope_x100;
        case P_INERTIA_SLOPE:        return d.inertia_slope_x100;
        case P_INERTIA_SLOPE_LOAD:   return d.inertia_slope_load_x1000;
        case P_BACKSTOP_HOT:         return (int16_t)d.backstop_hot_ms;
        case P_BACKSTOP_COLD:        return (int16_t)d.backstop_cold_ms;
        case P_UP_KMH:
            if (row >= TUNE_PAIRS || col >= TUNE_TPS_PTS) return 0;
            return (int16_t)d.up_kmh[row][col];
        case P_DN_KMH:
            if (row >= TUNE_PAIRS || col >= TUNE_TPS_PTS) return 0;
            return (int16_t)d.dn_kmh[row][col];
        case P_DM_FIRMNESS:  return (col < TUNE_MODES) ? d.dm_firmness_x100[col] : 0;
        case P_DM_SHIFTPT:   return (col < TUNE_MODES) ? d.dm_shiftpt_x100[col]  : 0;
        case P_DM_TCCOPEN:   return (col < TUNE_MODES) ? d.dm_tcc_open_tps[col]  : 0;
        case P_DM_LAUNCH:    return (col < TUNE_MODES) ? d.dm_launch_gear[col]   : 0;
        case P_DM_AUTO:      return (col < TUNE_MODES) ? d.dm_auto_shift[col]    : 0;
        case P_DM_LUG:       return (col < TUNE_MODES) ? d.dm_lug_guard[col]     : 0;
        case P_DM_TQCUT:     return (col < TUNE_MODES) ? d.dm_torque_cut[col]    : 0;
    }
    return 0;
}

bool TuneOverlay::paramSet(uint8_t i, uint8_t row, uint8_t col, int16_t val) {
    if (i >= P_COUNT) return false;
    const ParamDesc& p = PARAMS[i];
    if (val < p.vmin || val > p.vmax) return false;      // range-checked at the boundary
    switch (i) {
        case P_LINE_MAP:
            if (row >= TUNE_GEARS || col >= TUNE_LOAD_BINS) return false;
            d.line_map[row][col] = (uint8_t)val; return true;
        case P_INERTIA_TARGET:
            if (col >= TUNE_LOAD_PTS) return false;
            d.inertia_target_ms[col] = (uint16_t)val; return true;
        case P_APPLY_FLOOR:          d.apply_floor              = (uint8_t)val;  return true;
        case P_APPLY_SLOPE:          d.apply_slope_x100         = (uint8_t)val;  return true;
        case P_INERTIA_SLOPE:        d.inertia_slope_x100       = (uint8_t)constrain(val, 0, 255); return true;
        case P_INERTIA_SLOPE_LOAD:   d.inertia_slope_load_x1000 = (uint8_t)val;  return true;
        case P_BACKSTOP_HOT:         d.backstop_hot_ms          = (uint16_t)val; return true;
        case P_BACKSTOP_COLD:        d.backstop_cold_ms         = (uint16_t)val; return true;
        case P_UP_KMH:
            if (row >= TUNE_PAIRS || col >= TUNE_TPS_PTS) return false;
            d.up_kmh[row][col] = (uint16_t)val; return true;
        case P_DN_KMH:
            if (row >= TUNE_PAIRS || col >= TUNE_TPS_PTS) return false;
            d.dn_kmh[row][col] = (uint16_t)val; return true;
        case P_DM_FIRMNESS:  if (col >= TUNE_MODES) return false; d.dm_firmness_x100[col] = (uint8_t)val; return true;
        case P_DM_SHIFTPT:   if (col >= TUNE_MODES) return false; d.dm_shiftpt_x100[col]  = (uint8_t)val; return true;
        case P_DM_TCCOPEN:   if (col >= TUNE_MODES) return false; d.dm_tcc_open_tps[col]  = (uint8_t)val; return true;
        case P_DM_LAUNCH:    if (col >= TUNE_MODES) return false; d.dm_launch_gear[col]   = (uint8_t)val; return true;
        case P_DM_AUTO:      if (col >= TUNE_MODES) return false; d.dm_auto_shift[col]    = (uint8_t)val; return true;
        case P_DM_LUG:       if (col >= TUNE_MODES) return false; d.dm_lug_guard[col]     = (uint8_t)val; return true;
        case P_DM_TQCUT:     if (col >= TUNE_MODES) return false; d.dm_torque_cut[col]    = (uint8_t)val; return true;
    }
    return false;
}
