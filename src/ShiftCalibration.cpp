// PWM schedules and bounded learned trims. Commands are not measured pressure.
// All methods run on the physics task; see docs/CONTROL_ARCHITECTURE.md.
#include "ShiftScheduler.h"
#include "EngineProfile.h"
#include "DtcManager.h"
#include "AutoShiftMap.h"
#include "TuneOverlay.h"

// ----------------------------------------------------------------------------
// Cruise (holding) line pressure value — per-gear holding map × ATF compensation.
// Returned (not set) so the shift engine can take max(cruise, shift-demand).
float ShiftScheduler::cruiseLinePressure() {
    // Reverse (R1/R2) reuses 2nd-gear holding pressure — no separate reverse cal/adapt
    // (simplicity; R is manual-valve routed and only needs adequate, deterministic line).
    uint8_t g = (telemetry.prnd_state == 'R') ? 2 : telemetry.current_gear;
    uint8_t gear_idx = constrain(g - 1, 0, 4);
    uint8_t load_idx = loadToBin(computeLoad(telemetry.tps_pct, telemetry.map_kpa));
    float p = tuneOverlay.lineMap(gear_idx, load_idx);
    if (telemetry.engine_rpm < 1200.0f) p += 10.0f;
    // Cold ATF is viscous (slow fill); hot ATF leaks past seals — both need MORE pressure.
    float atf = telemetry.atf_temp_c, m = 1.0f;
    if      (atf < 20.0f)  m = 1.30f;
    else if (atf < 40.0f)  m = 1.15f;
    else if (atf < 80.0f)  m = 1.00f;
    else if (atf < 110.0f) m = 1.05f;
    else                   m = 1.20f;
    return constrain(p * m, 10.0f, 100.0f);
}

void ShiftScheduler::calculateLinePressure() {
    if (_current_phase != PHASE_CRUISING) return;   // the phase engine owns MPC during shifts
    // TPS ROC mode: torque is arriving NOW — straight to max line.
    if (telemetry.high_torque_mode) { _solenoids->setLinePressure(100); return; }
    _solenoids->setLinePressure((uint8_t)cruiseLinePressure());
}

// ----------------------------------------------------------------------------
// CLASSIFY + BUILD PROFILE  (ATSG-grounded spec §2-§5). All scalars computed once
// at initiation from the latched torque/load so the phase engine stays branch-light.
// ----------------------------------------------------------------------------
void ShiftScheduler::classifyAndProfile(uint8_t from, uint8_t to, bool is_upshift) {
    float load = _load_at_start;                       // 0-100, torque-based
    uint8_t idx = _active_shift_idx;

    // POWER vs COAST with hysteresis (between thresholds = keep previous).
    bool power;
    if (telemetry.tps_pct > CLASS_POWER_TPS_PCT && telemetry.t_est_nm > CLASS_POWER_TQ_NM) power = true;
    else if (telemetry.tps_pct < CLASS_COAST_TPS_PCT) power = false;
    else power = _prev_was_power;
    _prev_was_power = power;

    _pd_type = PD_NONE;
    if (is_upshift) {
        _sclass = power ? SC_POWER_UP : SC_COAST_UP;
        uint8_t  base_fill_p = engineProfile.fillP(idx);   // baseline from the (tunable) engine profile
        uint16_t base_fill_t = engineProfile.fillT(idx);
        if (power) {
            _fill_p = (uint8_t)constrain((int)base_fill_p, 0, 100);
            _fill_t_ms = base_fill_t;
            // Apply pressure: physical model (pressure to carry input torque) when enabled,
            // else the load-% heuristic. Adaptation apply_trim is added below either way.
            if (engineProfile.clPressureEnable()) {
                float mbar = engineProfile.clutchApplyMbar(idx, _input_at_start, telemetry.atf_temp_c);
                _apply_pct = engineProfile.mbarToPct(mbar);
            } else {
                // Torque-phase apply pressure. REBASED on dueATC's driven per-shift SPC maps
                // (60 C row, load 0/20/40): 1-2 46/69/80, 2-3 66/68/78, 3-4 66/67/88,
                // 4-5 70/81/80 — i.e. a light-load floor near 50 reaching full clamp by ~60 %
                // load. The old 20+0.55·load started at 20, far under anything driven.
                // Theirs is ONE pressure for the whole shift; ours is the torque phase and
                // then ramps through INERTIA, so this is the ramp's starting point.
                // SPORT BIAS: +2 on the floor and a slightly steeper slope than their fit.
                // Live values (floor/slope) come from the web-editable TuneOverlay.
                _apply_pct = (uint8_t)constrain(tuneOverlay.applyPct(load), 0.0f, 100.0f);
            }
            _inertia_slope     = tuneOverlay.inertiaSlope(load);        // %/20ms tick
            _inertia_target_ms = tuneOverlay.inertiaTargetMs(load);
        } else {
            _fill_p = (uint8_t)constrain((int)base_fill_p - 15, 0, 100);
            _fill_t_ms = (base_fill_t > 20) ? (base_fill_t - 20) : 0;
            _apply_pct = 25;                                           // fixed, gentle
            _inertia_slope = 1.0f;
            _inertia_target_ms = 350;
        }
    } else {
        _sclass = power ? SC_POWER_DOWN : SC_COAST_DOWN;
        if (power) {
            // 3-2 / 2-1 are sprag-assisted (freewheel catches at sync); 4-3 / 5-4 are timed.
            _pd_type = (from == 3 || from == 2) ? PD_SPRAG : PD_TIMED;
            if (_pd_type == PD_SPRAG) {
                _release_spc = 10; _release_backstop_ms = 500;
                _catch_start_spc = 30; _catch_slope = 2.0f;
            } else {
                _release_spc = 20; _release_backstop_ms = 450;
                _catch_start_spc = 30; _catch_slope = 3.0f;
            }
        } else {
            _release_spc = 15; _release_backstop_ms = 80;             // coast: no sync wait
            _catch_start_spc = 15; _catch_slope = 1.0f;
        }
    }

    // Apply learned trims for this cell (Adaptation v2). Zero on blank flash.
    AdaptCell cell = _adaptives->getCell((uint8_t)_sclass, idx, _torque_bin);
    if (is_upshift) {
        _fill_p    = (uint8_t)constrain((int)_fill_p + cell.fill_p_trim, 0, 100);
        _fill_t_ms = (uint16_t)constrain((int)_fill_t_ms + cell.fill_t_cycles * 20, 0, 400);
        _apply_pct = (uint8_t)constrain((int)_apply_pct + cell.apply_trim, 0, 100);
    } else {
        _catch_start_spc     = (uint8_t)constrain((int)_catch_start_spc + cell.apply_trim, 0, 100);
        _release_backstop_ms = (uint16_t)constrain((int)_release_backstop_ms + cell.fill_t_cycles * 20, 40, 600);
    }

    // Drive-mode firmness: scale the apply/clamp authority (NOT fill — that just seats the
    // piston). 1.0 = baseline gentle; sport/race scale up for firmer, faster shifts.
    float firm = currentMode().firmness;
    if (is_upshift) {
        _apply_pct     = (uint8_t)constrain((int)(_apply_pct * firm + 0.5f), 0, 100);
        _inertia_slope = _inertia_slope * firm;
    } else {
        _catch_start_spc = (uint8_t)constrain((int)(_catch_start_spc * firm + 0.5f), 0, 100);
        _catch_slope     = _catch_slope * firm;
    }

    telemetry.shift_class = (uint8_t)_sclass;
    telemetry.pd_type     = (uint8_t)_pd_type;
}

void ShiftScheduler::applyShiftMPC() {
    float cruise = cruiseLinePressure();
    float mpc;
    if (_sclass == SC_COAST_UP || _sclass == SC_COAST_DOWN) {
        mpc = cruise;                                   // coast: no boost authority needed
    } else if (engineProfile.clPressureEnable()) {
        // Physical model (Phase 3d): line holds the OFF-GOING clutch at its torque through the
        // overlap (release coefficient + off-going spring) — a clean crossover, vs the flat
        // load-% heuristic. Gear-pair idx selects the off-going element for this shift direction.
        uint8_t idx = constrain(_is_upshift ? (_from_gear - 1) : (telemetry.target_gear - 1), 0, 3);
        float mbar = engineProfile.clutchReleaseMbar(idx, _input_at_start);
        mpc = fmaxf(cruise, (float)engineProfile.mbarToPct(mbar));
    } else {
        float base = (_is_upshift ? 40.0f : 50.0f) + 0.5f * _load_at_start;
        mpc = fmaxf(cruise, base);
        if (_load_at_start > 70.0f) mpc = 100.0f;       // high load: full overlap authority
    }
    _solenoids->setLinePressure((uint8_t)constrain(mpc, 10.0f, 100.0f));
}

// Class-indexed adaptation (Adaptation v2). One update per shift, ATF-gated.
void ShiftScheduler::evaluateAdaptation() {
    if (!_last_adapt_valid || !telemetry.tps_valid || !telemetry.map_valid) return;
    // Don't let the deliberately-firm manual modes (SPORT/RACE firmness >1) poison the
    // learned trims that the gentler auto modes also use — only learn in auto modes.
    if (!currentMode().auto_shift) return;
    // ATSG p.78: only relearn in the valid ATF window (and never in limp).
    if (telemetry.atf_temp_c < ADAPT_ATF_MIN_C || telemetry.atf_temp_c > ADAPT_ATF_MAX_C) return;
    _adaptives->learn((uint8_t)_sclass, _active_shift_idx, _torque_bin,
                      telemetry.flare_detected, _harsh_detected, telemetry.bind_detected);
    // Fill-time from clutch motion (UN52 idea, no OEM tables): late bite → more fill,
    // early bite → less. Deadband one 20 ms cycle. Timer-only exits teach nothing.
    if (_fill_move_ms > 0 && _fill_t_ms > 0) {
        int err_cyc = ((int)_fill_move_ms - (int)_fill_t_ms) / (int)PRESSURE_TICK_MS;
        if (err_cyc > 1) err_cyc = 1;
        if (err_cyc < -1) err_cyc = -1;
        _adaptives->learnFill((uint8_t)_sclass, _active_shift_idx, _torque_bin, err_cyc);
    }
}
