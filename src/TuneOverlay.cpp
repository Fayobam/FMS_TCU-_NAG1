// ============================================================================
// FILE: TuneOverlay.cpp
// VERSION: 1.0
// ============================================================================
#include "TuneOverlay.h"
#include "TCU_Data.h"

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
    d.magic = TUNE_MAGIC;
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
    }
    return false;
}
