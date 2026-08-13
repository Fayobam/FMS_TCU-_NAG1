// ============================================================================
// FILE: TuneOverlay.h
// VERSION: 1.0
//
// FIELD-EDITABLE SHIFT CALIBRATION + a generic parameter registry.
//
// Why this exists: the engine/sensor calibration and the clutch physics model were
// already web-editable (EngineProfile), but the SHIFT-FEEL layer — cruise line
// pressure, apply pressure, inertia ramp and duration, phase backstops — was
// compile-time only. That is the half you actually reach for on a road trip, and
// after the dueATC rebase (Reference/DUEATC_CALIBRATION_NOTES.md) it is also the
// half most likely to need adjusting. Tuning it required a laptop and a recompile.
//
// Three-layer model (mirrors the OPEN8HP approach):
//   compile-time defaults  ->  NVS-backed live overlay  ->  consumers read the overlay
// The defaults keep their provenance comments and stay the single seed source, so a
// blank/`reset` device behaves exactly as the documented calibration.
//
// TO EXPOSE A NEW PARAMETER: add a field to TuneData, seed it in seedDefaults(),
// add one ParamDesc row + one case to paramGet/paramSet. It then appears in the
// dashboard as a typed, range-checked, persisted editor with help text — no
// per-field JSON plumbing in either firmware or JS.
//
// SAFETY BOUNDARY: only feel/comfort values live here. The guards that prevent
// damage — money-shift ceiling, overrev lead, sensor-trust and event-confirm
// windows, ratio-verify tolerance — stay compile-time in TCU_Data.h ON PURPOSE.
// A wrong value there breaks a gearbox; they are not "feel" knobs.
// ============================================================================
#pragma once
#include <Arduino.h>
#include <Preferences.h>

#define TUNE_MAGIC 0x54554E31u   // 'TUN1' — bump to force a re-seed on layout change

#define TUNE_GEARS      5
#define TUNE_LOAD_BINS 16
#define TUNE_LOAD_PTS  11        // 0,10,...,100 % load

struct TuneData {
    // --- Cruise (holding) line pressure, pressure-% [gear][load bin] ---
    uint8_t  line_map[TUNE_GEARS][TUNE_LOAD_BINS];

    // --- Upshift INERTIA phase ---
    uint16_t inertia_target_ms[TUNE_LOAD_PTS];  // target duration by load%
    uint8_t  apply_floor;                       // apply% = floor + slope/100 * load
    uint8_t  apply_slope_x100;
    uint8_t  inertia_slope_x100;                // %/20ms tick = base + k*load
    uint8_t  inertia_slope_load_x1000;

    // --- Phase backstop (ATF-scaled) ---
    uint16_t backstop_hot_ms;
    uint16_t backstop_cold_ms;

    uint32_t magic;
};

// ---- Parameter registry -----------------------------------------------------
enum ParamKind : uint8_t { PK_SCALAR = 0, PK_CURVE1D = 1, PK_TABLE2D = 2 };

struct ParamDesc {
    const char* id;      // stable wire id, e.g. "line.map"
    const char* name;    // human label
    const char* group;   // UI grouping
    ParamKind   kind;
    int16_t     vmin, vmax;
    uint8_t     rows, cols;   // 1,1 for scalar; 1,N for curve
    const char* unit;
    const char* help;    // shown as a tooltip; firmware is the single source
};

class TuneOverlay {
  private:
    Preferences prefs;
    TuneData d;
    void seedDefaults();   // safe pre-main: seeds only from constant-initialised .rodata

  public:
    // Seeds the defaults immediately, so the object is VALID before begin() runs.
    // Without this a zero-initialised overlay means zero line pressure and a zero
    // phase backstop (every shift abandoned on its first tick) for any consumer
    // that runs before setup() gets to it — including the host test build.
    TuneOverlay() { seedDefaults(); }

    void begin();
    void save();
    void resetToDefaults();          // re-seed + persist (dashboard "reset")

    // --- consumers (hot path: plain inline reads, no lookup) ---
    uint8_t  lineMap(uint8_t gear_idx, uint8_t load_idx) const {
        return d.line_map[gear_idx > 4 ? 4 : gear_idx][load_idx > 15 ? 15 : load_idx];
    }
    float    applyPct(float load)      const { return d.apply_floor + (d.apply_slope_x100 / 100.0f) * load; }
    float    inertiaSlope(float load)  const { return (d.inertia_slope_x100 / 100.0f) +
                                                      (d.inertia_slope_load_x1000 / 1000.0f) * load; }
    uint16_t inertiaTargetMs(float load) const;   // interpolates inertia_target_ms
    uint16_t backstopHotMs()  const { return d.backstop_hot_ms; }
    uint16_t backstopColdMs() const { return d.backstop_cold_ms; }

    // --- registry ---
    static uint8_t          paramCount();
    static const ParamDesc& paramDesc(uint8_t i);
    int16_t paramGet(uint8_t i, uint8_t row, uint8_t col) const;
    bool    paramSet(uint8_t i, uint8_t row, uint8_t col, int16_t val);  // range-checked

    TuneData* raw() { return &d; }
};

extern TuneOverlay tuneOverlay;
