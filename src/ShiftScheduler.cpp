// ============================================================================
// FILE: ShiftScheduler.cpp
// VERSION: 6.0
// UPDATES: Safety auto-shifts, flare detection, P/N latch fix, limp recovery,
//          centralised beginShift() (no more duplicated off-by-one triggers).
// ============================================================================
#include "ShiftScheduler.h"
#include "EngineProfile.h"
#include "DtcManager.h"
#include "AutoShiftMap.h"

ShiftScheduler::ShiftScheduler(SolenoidDriver* solenoids, AdaptiveMemory* adaptives) {
    _solenoids = solenoids;
    _adaptives = adaptives;
    _current_phase = PHASE_CRUISING;
    _active_routing_pin = 0;
    _active_shift_idx = 0;
    _prev_was_power = false;
    _last_pressure_update_ms = 0;
    _spc_cmd = 0.0f;
}

void ShiftScheduler::begin() {
    _current_phase = PHASE_CRUISING;
    telemetry.current_gear = 2;   // 722.6 hydraulic default (was 1 — cosmetic mismatch)
    telemetry.target_gear = 2;
    _prev_pn_raw = true;
    for (int i = 0; i < TPS_ROC_WINDOW_MS; i++) _tps_hist[i] = 0.0f;
    _tps_hist_idx = 0;
    _tps_hist_primed = false;
    _high_torque_mode = false;
    _ht_release_start_ms = 0;
    _engage_grace_until_ms = 0;
    _gear_resync_pending = false;
    _resync_ready_ms = 0;
    _prev_prnd = 'P';
    _legit_reverse = false;
    _slow_since_ms = millis();   // boot presumes stationary (a reboot mid-drive clears
                                 // this on the first tick once output reads > threshold)
    _prev_was_power = false;
    _last_pressure_update_ms = millis();
    _spc_cmd = 0.0f;
    _harsh_detected = false;
    _flare_over_ms = 0;
    _bind_over_ms = 0;
    _eng_rpm_prev_sample = 0.0f;
    _eng_roc_sample_ms = millis();
    _eng_rpm_per_s = 0.0f;
    _n2n3_fault_since_ms = 0;
    telemetry.input_speed_trusted = true;
}

bool ShiftScheduler::isForwardRange() {
    char s = telemetry.prnd_state;
    return (s == 'D' || s == '4' || s == '3' || s == '2' || s == '1');
}

// Lever-selected drive mode (D/4/3/2/1 → COMFORT AUTO … RACE MANUAL). Read live each
// use so moving the lever re-tunes shift behaviour on the fly.
const DriveMode& ShiftScheduler::currentMode() const {
    return DRIVE_MODES[driveModeIndex(telemetry.prnd_state)];
}

// ----------------------------------------------------------------------------
float ShiftScheduler::getTargetRatio(uint8_t gear) {
    switch(gear) {
        case 1: return RATIO_1ST; case 2: return RATIO_2ND; case 3: return RATIO_3RD;
        case 4: return RATIO_4TH; case 5: return RATIO_5TH; default: return 1.0f;
    }
}

uint8_t ShiftScheduler::getRoutingSolenoidForShift(uint8_t from_gear, uint8_t to_gear) {
    if (from_gear == 1 && to_gear == 2) return PIN_Y3;
    if (from_gear == 2 && to_gear == 3) return PIN_Y5;
    if (from_gear == 3 && to_gear == 4) return PIN_Y4;
    if (from_gear == 4 && to_gear == 5) return PIN_Y3;
    if (from_gear == 5 && to_gear == 4) return PIN_Y3;
    if (from_gear == 4 && to_gear == 3) return PIN_Y4;
    if (from_gear == 3 && to_gear == 2) return PIN_Y5;
    if (from_gear == 2 && to_gear == 1) return PIN_Y3;
    return 0;
}

// ----------------------------------------------------------------------------
// Cruise (holding) line pressure value — per-gear holding map × ATF compensation.
// Returned (not set) so the shift engine can take max(cruise, shift-demand).
float ShiftScheduler::cruiseLinePressure() {
    // Reverse (R1/R2) reuses 2nd-gear holding pressure — no separate reverse cal/adapt
    // (simplicity; R is manual-valve routed and only needs adequate, deterministic line).
    uint8_t g = (telemetry.prnd_state == 'R') ? 2 : telemetry.current_gear;
    uint8_t gear_idx = constrain(g - 1, 0, 4);
    uint8_t load_idx = loadToBin(computeLoad(telemetry.tps_pct, telemetry.map_kpa));
    float p = HOLDING_PRESSURE_MAP[gear_idx][load_idx];
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
    if (_high_torque_mode) { _solenoids->setLinePressure(100); return; }
    _solenoids->setLinePressure((uint8_t)cruiseLinePressure());
}

void ShiftScheduler::calculateLiveRatio() {
    if (telemetry.output_rpm > 50.0f) telemetry.live_ratio = telemetry.turbine_rpm / telemetry.output_rpm;
    else telemetry.live_ratio = getTargetRatio(telemetry.current_gear);
}

// ----------------------------------------------------------------------------
// CLUTCH-SPEED MODEL (ported from rnd-ash/ultimate-nag52 models/clutch_speed.cpp).
// Closed-form on-coming / off-going clutch SLIP speeds for the active gear change,
// from the raw shaft speeds we already measure (N2, N3, output) and the live gearbox
// ratios (g_trans — variant-correct for small AND big NAG). These are the signals EGS
// uses for phase transitions: off-clutch slip rising = fill done / off-going releasing;
// on-clutch slip → 0 = synced. Far less noisy than gross turbine/output ratio.
//
// PHASE 1 (now): compute + expose in telemetry/CSV for bench verification against the
// ratio-based behaviour. Wiring these into the phase transitions is the next step,
// once a signal-gen bench run confirms the values + signs per shift.
// ----------------------------------------------------------------------------
void ShiftScheduler::computeClutchSpeeds() {
    // Only meaningful mid-shift; clear when cruising so the dashboard reads 0.
    if (_current_phase == PHASE_CRUISING) {
        telemetry.on_clutch_rpm = 0.0f;
        telemetry.off_clutch_rpm = 0.0f;
        return;
    }
    float n2 = telemetry.n2_rpm, n3 = telemetry.n3_rpm, out = telemetry.output_rpm;
    float r2 = RATIO_2ND, r3 = RATIO_3RD, r4 = RATIO_4TH;
    uint8_t f = _from_gear, t = telemetry.target_gear;
    float on = 0.0f, off = 0.0f;

    if ((f == 1 && t == 2) || (t == 1 && f == 2) ||
        (f == 4 && t == 5) || (t == 4 && f == 5)) {
        // K1/K2 (1-2,5-4) vs B1 (2-1,4-5) — element on the simple front planetary
        float vk1 = n2 - n3;       // K1 (1-2) / K2 (5-4)
        float vb1 = n3;            // B1
        bool on_is_k = (f == 1 && t == 2) || (f == 5 && t == 4);
        on  = on_is_k ? vk1 : vb1;
        off = on_is_k ? vb1 : vk1;
    } else if ((f == 2 && t == 3) || (f == 3 && t == 2)) {
        float vk2 = n3 - (r3 * out);
        float vk3 = (r2 - r3 != 0.0f) ? (r3 * (r2 * out - n3)) / (r2 - r3) : 0.0f;
        bool up = (f == 2 && t == 3);
        on  = up ? vk2 : vk3;
        off = up ? vk3 : vk2;
    } else if ((f == 3 && t == 4) || (f == 4 && t == 3)) {
        float vb2 = (r3 - r4 != 0.0f) ? (r3 * out - n3) / (r3 - r4) : 0.0f;
        float vk3 = n3 - vb2;
        bool up = (f == 3 && t == 4);
        on  = up ? vk3 : vb2;
        off = up ? vb2 : vk3;
    }
    telemetry.on_clutch_rpm  = on;
    telemetry.off_clutch_rpm = off;
}

// ----------------------------------------------------------------------------
// TCC DYNAMIC SLIP CONTROLLER
// Rate-limited and 20ms-quantized (ATSG p.80): lockup moves at most TCC_LOCK_STEP
// %/tick (gentle apply) and opens at TCC_RELEASE_STEP %/tick (fast). TCC is forced
// fully open during any shift phase AND for TCC_POST_SHIFT_HOLD_MS after the shift
// ends, so the converter never locks through a ratio change or the END line-decay.
// ----------------------------------------------------------------------------
void ShiftScheduler::updateTCC(bool ptick) {
    telemetry.tcc_actual_slip_rpm = telemetry.engine_rpm - telemetry.turbine_rpm;
    if (telemetry.tcc_actual_slip_rpm < 0) telemetry.tcc_actual_slip_rpm = 0;

    // Any active shift phase re-arms the post-shift hold; lockup control only resumes
    // once we've been cruising for the full hold window.
    if (_current_phase != PHASE_CRUISING) _tcc_reopen_until_ms = millis() + TCC_POST_SHIFT_HOLD_MS;
    bool hold_open = (millis() < _tcc_reopen_until_ms);

    int current_tcc_pwm = telemetry.tcc_lockup_pct;

    if (_current_phase == PHASE_CRUISING && !hold_open) {
        // Force TCC open if: ROC mode active (fast tip-in), MAP shows boost,
        // or TPS is above moderate demand. ROC mode takes priority — it reacts
        // faster than MAP settling.
        bool force_open = _high_torque_mode ||
                          telemetry.map_kpa > 105.0f ||
                          telemetry.tps_pct > currentMode().tcc_open_tps;  // sportier modes open sooner
        if (force_open) {
            telemetry.tcc_target_slip_rpm = 500.0f;
            if (ptick) current_tcc_pwm -= TCC_RELEASE_STEP;
        } else if (telemetry.engine_rpm < 1400.0f || telemetry.current_gear == 1) {
            telemetry.tcc_target_slip_rpm = 1000.0f;
            if (ptick) current_tcc_pwm -= TCC_RELEASE_STEP;
        } else {
            telemetry.tcc_target_slip_rpm = 50.0f;
            if (ptick) {
                if (telemetry.tcc_actual_slip_rpm > (telemetry.tcc_target_slip_rpm + 20.0f)) {
                    if (current_tcc_pwm < 85) current_tcc_pwm += TCC_LOCK_STEP;
                } else if (telemetry.tcc_actual_slip_rpm < (telemetry.tcc_target_slip_rpm - 10.0f)) {
                    current_tcc_pwm -= TCC_LOCK_STEP;
                }
            }
        }
    } else {
        // Shifting or inside the post-shift hold: drive fully open at the release rate.
        telemetry.tcc_target_slip_rpm = 1000.0f;
        if (ptick) current_tcc_pwm -= TCC_RELEASE_STEP;
    }
    _solenoids->setTCC((uint8_t)constrain(current_tcc_pwm, 0, 100));
}

// ----------------------------------------------------------------------------
// CENTRALISED SHIFT INITIATION  (one code path for paddle AND safety shifts)
// ----------------------------------------------------------------------------
bool ShiftScheduler::beginShift(uint8_t target_gear, bool is_upshift, const char* source) {
    if (_current_phase != PHASE_CRUISING) return false;          // already shifting
    // Gear identity is only a GUESS while a post-engagement ratio resync is pending
    // (engaged at speed / reboot mid-drive / aborted shift). A shift begun from a
    // wrong gear label fires the wrong routing solenoid and can command two clutch
    // packs at once (cross-apply / tie-up). NO shift — paddle, auto, or safety —
    // may start until the label is ratio-verified. Engine overrev inside this
    // <=1.5 s window is the rev limiter's job, not the gearbox's.
    if (_gear_resync_pending) return false;
    if (target_gear < 1 || target_gear > 5) return false;

    // Every legal 722.6 shift is single-step and has exactly one routing solenoid.
    // A pair with none (skip-shift, same-gear) must be refused BEFORE any state is
    // mutated: fireShiftSolenoid(0) is a silent no-op, so the phases would run with
    // nothing energised and end by asserting a gear the gearbox never entered.
    uint8_t routing_pin = getRoutingSolenoidForShift(telemetry.current_gear, target_gear);
    if (routing_pin == 0) return false;

    // Money-shift / overrev guard on ANY downshift (manual or auto).
    // Predict the post-downshift turbine speed TWO independent ways and trust the
    // HIGHER (fail-safe): from the output sensor (output × target ratio) AND from the
    // turbine sensor, which is output-INDEPENDENT (turbine × ratio_target/ratio_current,
    // since turbine = output × ratio_current). So a DEAD output sensor reading 0 can no
    // longer silently defeat the guard — the N2/N3-derived estimate still catches an
    // over-rev downshift. (Both sensors dead = truly blind; nothing can help then.)
    if (!is_upshift) {
        float ratio_t = getTargetRatio(target_gear);
        float ratio_c = getTargetRatio(telemetry.current_gear);
        float pred_out  = telemetry.output_rpm * ratio_t;
        float pred_turb = (ratio_c > 0.01f) ? telemetry.turbine_rpm * (ratio_t / ratio_c) : 0.0f;
        float predicted = fmaxf(pred_out, pred_turb);
        if (predicted > RPM_MAX_SAFE_DOWNSHIFT) {
            Serial.print("DOWNSHIFT BLOCKED ("); Serial.print(source);
            Serial.print(") predicted RPM "); Serial.println(predicted);
            return false;
        }
    }

    _from_gear = telemetry.current_gear;
    telemetry.target_gear = target_gear;
    _is_upshift = is_upshift;
    // Adaptive index: upshift uses lower gear-1, downshift uses target gear-1
    _active_shift_idx = constrain(is_upshift ? (_from_gear - 1) : (target_gear - 1), 0, ADAPT_SHIFTS - 1);

    // Capture the operating cell NOW, at initiation (torque-binned). Adaptation runs
    // in END — by then RPM/torque have moved, so binning there would mis-attribute.
    _load_at_start = telemetry.load_pct;
    _input_at_start = telemetry.t_input_nm;   // input torque the clutches see (pressure model)
    _torque_bin    = engineProfile.torqueBin(telemetry.engine_rpm, telemetry.map_kpa);
    _ratio_old     = getTargetRatio(_from_gear);
    _ratio_target  = getTargetRatio(target_gear);

    // Classify (POWER/COAST, PD_SPRAG/PD_TIMED) and compute the phase profile scalars.
    classifyAndProfile(_from_gear, target_gear, is_upshift);

    telemetry.flare_detected = false;
    telemetry.bind_detected  = false;
    _harsh_detected = false;
    _flare_over_ms = 0;
    _bind_over_ms  = 0;
    _prev_ratio = telemetry.live_ratio;
    _ratio_flat = false;
    _last_speed_seq = telemetry.speed_sample_seq;   // first in-shift sample triggers a fresh delta
    _sync_stable_since_ms = 0;
    _turbine_rpm_at_shift_start = telemetry.turbine_rpm;
    _output_rpm_at_shift_start  = telemetry.output_rpm;
    _active_routing_pin = routing_pin;             // validated above, never 0

    _solenoids->fireShiftSolenoid(_active_routing_pin);
    _shift_stopwatch_start    = millis();
    _last_pressure_update_ms  = millis();
    _phase_start_tick = xTaskGetTickCount();
    _current_phase = PHASE_PREP;     // all classes start in PREP
    applyShiftMPC();                 // lead the gate: set line/overlap authority on THIS tick,
                                     // not the next 20ms ptick (spec PREP intent) — B-7
    _cl_err = 0.0f;

    // Start the high-rate datalog for this shift. Skip if a prior trace is still
    // waiting for Core 0 to send it, so we never clobber an undumped trace.
    if (!shiftTrace.ready) {
        shiftTrace.count = 0;
        shiftTrace.start_ms = millis();
        shiftTrace.last_sample_ms = 0;
        shiftTrace.shift_class = (uint8_t)_sclass;
        shiftTrace.pd_type   = (uint8_t)_pd_type;
        shiftTrace.from_gear = _from_gear;
        shiftTrace.to_gear   = target_gear;
        shiftTrace.capturing = true;
    } else {
        shiftTrace.capturing = false;
    }
    return true;
}

// High-rate per-shift datalog: one compact sample every TRACE_INTERVAL_MS (~500 Hz),
// capturing the COMMANDED pressures + speeds + ratio + closed-loop error. The ring is
// dumped once after the shift by Core 0 (TCU_Data.h ShiftTrace).
void ShiftScheduler::captureTrace() {
    if (!shiftTrace.capturing || shiftTrace.count >= TRACE_MAX) return;
    unsigned long now = millis();
    if (shiftTrace.count > 0 && (now - shiftTrace.last_sample_ms) < TRACE_INTERVAL_MS) return;
    shiftTrace.last_sample_ms = now;
    TraceSample &s = shiftTrace.s[shiftTrace.count];
    s.t_ms  = (uint16_t)constrain((long)(now - _shift_stopwatch_start), 0L, 65535L);
    s.phase = (uint8_t)_current_phase;
    s.spc   = (uint8_t)telemetry.shift_pressure_pct;
    s.mpc   = (uint8_t)telemetry.line_pressure_pct;
    s.flags = (telemetry.flare_detected ? 1 : 0) | (telemetry.bind_detected ? 2 : 0) |
              (_harsh_detected ? 4 : 0) | (telemetry.torque_cut_active ? 8 : 0);
    s.ratio_x1000 = (uint16_t)constrain((int)(telemetry.live_ratio * 1000.0f), 0, 65535);
    s.eng   = (uint16_t)constrain((int)telemetry.engine_rpm,  0, 65535);
    s.turb  = (uint16_t)constrain((int)telemetry.turbine_rpm, 0, 65535);
    s.out   = (uint16_t)constrain((int)telemetry.output_rpm,  0, 65535);
    s.cl_err_x1000 = (int16_t)constrain((int)(_cl_err * 1000.0f), -32768, 32767);
    s.on_clutch  = (int16_t)constrain((int)telemetry.on_clutch_rpm,  -32768, 32767);
    s.off_clutch = (int16_t)constrain((int)telemetry.off_clutch_rpm, -32768, 32767);
    shiftTrace.count = shiftTrace.count + 1;
}

// ----------------------------------------------------------------------------
// CLASSIFY + BUILD PROFILE  (ATSG-grounded spec §2-§5). All scalars computed once
// at initiation from the latched torque/load so the phase engine stays branch-light.
// ----------------------------------------------------------------------------
void ShiftScheduler::classifyAndProfile(uint8_t from, uint8_t to, bool is_upshift) {
    float load = _load_at_start;                       // 0-100, torque-based
    uint8_t idx = constrain(is_upshift ? (from - 1) : (to - 1), 0, 3);

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
                _apply_pct = (uint8_t)constrain(52.0f + 0.9f * load, 0.0f, 100.0f);
            }
            _inertia_slope = 2.0f + 0.02f * load;                       // %/20ms tick
            _inertia_target_ms = (uint16_t)autoMapInterp(INERTIA_TARGET_MS, load);
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

// ----------------------------------------------------------------------------
// AUTO-SAFETY LAYER  (the feature requested: overrev upshift + lug downshift)
// Only fires while CRUISING, in a forward driving state, off cooldown.
// ----------------------------------------------------------------------------
void ShiftScheduler::checkSafetyShifts() {
    if (_current_phase != PHASE_CRUISING) return;
    // Any forward range, including manual limit positions '1'/'2'. Engine-overrev
    // protection deliberately overrides a manual limit detent — hitting the
    // limiter is worse than breaching the driver's requested gear cap.
    if (!isForwardRange()) return;
    if (millis() - telemetry.last_auto_shift_ms < AUTO_SHIFT_COOLDOWN_MS) return;

    // --- OVERREV: force an upshift before the engine hits the limiter ---
    // PREDICTIVE: the shift only unloads the engine after PREP+FILL+part of TORQUE
    // (~OVERREV_LEAD_MS); at WOT in a low gear the engine gains several hundred rpm
    // in that window and a fixed trigger loses to the limiter. Lead is capped at
    // 400 rpm so a high-ROC pull can't fire absurdly early (window: overrev-400 … overrev).
    float lead_rpm = constrain(_eng_rpm_per_s * 0.001f * (float)OVERREV_LEAD_MS, 0.0f, 400.0f);
    if (telemetry.engine_rpm + lead_rpm > engineProfile.overrevRpm() && telemetry.current_gear < 5) {
        if (beginShift(telemetry.current_gear + 1, true, "OVERREV")) {
            telemetry.last_auto_shift_ms = millis();
            dtcManager.trip(DTC_OVERREV);
            char buf[64];
            snprintf(buf, sizeof(buf), "AUTO UPSHIFT (overrev %d +%d/s)",
                     (int)telemetry.engine_rpm, (int)_eng_rpm_per_s);
            setSafetyEvent(buf);
            Serial.println(buf);
        }
        return;
    }

    // --- LUG: sequential downshift back toward 2nd (one shift per cooldown) ---
    // Classic case: driver forgot to downshift from 5th at a traffic light.
    // Floor is 2nd — the hydraulic default — so 1st remains driver's choice.
    // The floor also means this never fires after a D-engagement (which starts in 2nd).
    if (currentMode().lug_guard &&
        telemetry.engine_rpm < engineProfile.lugRpm() &&
        telemetry.tps_pct > TPS_LUG_LOAD_PCT &&
        telemetry.current_gear > 2) {
        if (beginShift(telemetry.current_gear - 1, false, "LUG")) {
            telemetry.last_auto_shift_ms = millis();
            char buf[64];
            snprintf(buf, sizeof(buf), "AUTO DOWNSHIFT (lug %d)", (int)telemetry.engine_rpm);
            setSafetyEvent(buf);
            Serial.println(buf);
        }
    }
}

// ----------------------------------------------------------------------------
// AUTO SHIFT SCHEDULE (full automatic up/down). Runs only in auto drive modes
// (D/4/3). Interpolates the AUTO_SHIFT_MAP km/h thresholds for the current gear by
// TPS, stretches them by the mode's shift_pt_scale (sport holds gears longer), and
// shifts when road km/h crosses. Money-shift (beginShift) + overrev/lug
// (checkSafetyShifts) guards stay on top; AUTO_SHIFT_COOLDOWN_MS + the map's built-in
// up>down hysteresis prevent hunting. A pending paddle request wins this tick.
// The map's closed-throttle column doubles as the coast-down schedule.
// ----------------------------------------------------------------------------
void ShiftScheduler::checkAutoShift() {
    if (_current_phase != PHASE_CRUISING) return;
    if (!isForwardRange()) return;
    if (!currentMode().auto_shift) return;
    if (millis() - telemetry.last_auto_shift_ms < AUTO_SHIFT_COOLDOWN_MS) return;
    if (telemetry.paddle_up_request || telemetry.paddle_down_request) return;   // paddle override wins

    uint8_t g     = telemetry.current_gear;
    float   kmh   = telemetry.road_kmh;
    float   tps   = telemetry.tps_pct;
    float   scale = currentMode().shift_pt_scale;

    // Upshift g→g+1 above the (scaled) threshold. (1st is reachable by paddle in auto.)
    if (g >= 1 && g < 5) {
        float up_kmh = autoMapInterp(AUTO_UPSHIFT_KMH[g - 1], tps) * scale;
        if (kmh > up_kmh && beginShift(g + 1, true, "AUTO")) {
            telemetry.last_auto_shift_ms = millis();
            return;
        }
    }
    // Downshift g→g-1 below the (scaled) threshold. Floor at 2nd (1st = paddle-only).
    if (g >= 3 && g <= 5) {
        float dn_kmh = autoMapInterp(AUTO_DOWNSHIFT_KMH[g - 2], tps) * scale;
        if (kmh < dn_kmh && beginShift(g - 1, false, "AUTO")) {
            telemetry.last_auto_shift_ms = millis();
            return;
        }
    }
}

// ----------------------------------------------------------------------------
// LAUNCH GEAR. A stopped 722.6 in D otherwise sits in its hydraulic-default 2nd —
// lethargic with this car's 3.07 diff. Drop a nearly-stopped car one gear at a time
// toward its mode's launch gear (1st) so pull-away uses the 3.93 first ratio. Runs
// in ALL modes (auto and manual both launch in 1st); the auto schedule / paddles take
// over from 1st as speed builds. Money-shift guard (in beginShift) + cooldown apply.
// ----------------------------------------------------------------------------
void ShiftScheduler::checkLaunchGear() {
    if (_current_phase != PHASE_CRUISING) return;
    if (!isForwardRange() || !telemetry.drive_engaged) return;
    if (millis() < _engage_grace_until_ms) return;                  // let garage engagement settle
    if (millis() - telemetry.last_auto_shift_ms < AUTO_SHIFT_COOLDOWN_MS) return;
    uint8_t lg = currentMode().launch_gear;
    if (telemetry.current_gear > lg && telemetry.output_rpm < LAUNCH_GEAR_MAX_OUTPUT_RPM) {
        if (beginShift(telemetry.current_gear - 1, false, "LAUNCH")) telemetry.last_auto_shift_ms = millis();
    }
}

// ----------------------------------------------------------------------------
// KICKDOWN (spec §4.6). Hard tip-in → request a power-down if the lower gear keeps
// predicted turbine under the money-shift ceiling. Multi-gear kickdowns happen as
// back-to-back single shifts across cooldowns (never skip-shifts).
// ----------------------------------------------------------------------------
void ShiftScheduler::checkKickdown() {
    if (_current_phase != PHASE_CRUISING) return;
    if (!isForwardRange()) return;
    if (!currentMode().auto_shift) return;   // manual modes: the driver paddles for power
    if (millis() - telemetry.last_auto_shift_ms < AUTO_SHIFT_COOLDOWN_MS) return;
    if (telemetry.tps_pct < KICKDOWN_TPS_PCT) return;
    if (telemetry.engine_rpm > KICKDOWN_MAX_ENG_RPM) return;  // already high → don't overrev

    uint8_t g = telemetry.current_gear;
    if (g <= 1) return;
    float predicted = telemetry.output_rpm * getTargetRatio(g - 1);
    if (predicted > RPM_MAX_SAFE_DOWNSHIFT) return;           // money-shift guard
    if (beginShift(g - 1, false, "KICKDOWN")) {
        telemetry.last_auto_shift_ms = millis();
    }
}

// ----------------------------------------------------------------------------
// LIMP MODE  (load-aware threshold + deliberate reset path)
// ----------------------------------------------------------------------------
void ShiftScheduler::checkLimpMode(float target_ratio) {
    // Engagement grace: after selecting D (especially N->D while moving) the
    // oncoming clutch slips for several hundred ms while it drags the turbine up
    // to output*ratio. That transient is NOT a fault — suppress slip detection
    // until the clutch has had time to synchronise. (Bug: N->D at speed tripped
    // instant limp mode on the engagement transient.)
    if (millis() < _engage_grace_until_ms) {
        telemetry.is_slipping = false;
        return;
    }

    // If the N2/N3 plausibility check failed, turbine_rpm is unreliable — a bad speed
    // sensor must never trigger transmission-protection limp (rnd-ash input-shaft trust).
    if (!telemetry.input_speed_trusted) {
        telemetry.is_slipping = false;
        return;
    }

    // Slip detection only while cruising, in D, moving, and NOT at high load
    // (high-boost launches legitimately slip the converter and chirp tyres).
    bool conditions = (_current_phase == PHASE_CRUISING &&
                       isForwardRange() &&            // D and manual limits '4'/'3'/'2'/'1'
                       telemetry.output_rpm > 200.0f &&
                       telemetry.tps_pct < 80.0f &&
                       telemetry.map_kpa < 130.0f);

    if (conditions) {
        float expected_turbine = telemetry.output_rpm * target_ratio;
        float mismatch = fabs(telemetry.turbine_rpm - expected_turbine);

        if (mismatch > 300.0f) {
            if (!telemetry.is_slipping) {
                telemetry.is_slipping = true;
                telemetry.slip_start_time_ms = millis();
            } else if ((millis() - telemetry.slip_start_time_ms) > 400) {
                telemetry.is_limp_mode = true;
                char buf[64];
                snprintf(buf, sizeof(buf), "FATAL SLIP GEAR %d DIFF %d RPM",
                         telemetry.current_gear, (int)mismatch);
                setLimpReason(buf);
                Serial.println("!!! TRANSMISSION PROTECTION ACTIVATED !!!");
                Serial.println(buf);
            }
        } else {
            telemetry.is_slipping = false;
        }
    } else {
        telemetry.is_slipping = false;
    }
}

// ============================================================================
// TPS RATE-OF-CHANGE TORQUE ANTICIPATION
// TVS supercharger delivers torque with throttle, not with MAP settling.
// A fast tip-in means torque is arriving NOW — get ahead of it.
// ROC is averaged over a 20ms ring so single-sample ADC noise (which easily
// exceeded 0.15%/ms tick-to-tick) cannot false-trigger max-pressure mode.
// ============================================================================
void ShiftScheduler::checkTpsROC() {
    // Ring: _tps_hist_idx points at the OLDEST sample (the one being replaced).
    float oldest = _tps_hist[_tps_hist_idx];
    _tps_hist[_tps_hist_idx] = telemetry.tps_pct;
    _tps_hist_idx = (uint8_t)((_tps_hist_idx + 1) % TPS_ROC_WINDOW_MS);
    if (!_tps_hist_primed) {
        if (_tps_hist_idx == 0) _tps_hist_primed = true;
        return;
    }
    float roc = (telemetry.tps_pct - oldest) / (float)TPS_ROC_WINDOW_MS;  // %/ms

    if (!telemetry.drive_engaged) return;

    // --- Entry: fast tip-in detected ---
    if (roc > TPS_ROC_TRIGGER_PCT_MS) {
        _high_torque_mode = true;
        telemetry.high_torque_mode = true;
        _ht_release_start_ms = 0;   // cancel any in-progress cooldown
        return;
    }

    if (!_high_torque_mode) return;

    // --- In high-torque mode: watch for genuine backing-off ---
    // Both conditions must be true together: ROC settled AND TPS actually low.
    // Either condition alone (e.g. momentary plateau at high TPS) keeps the mode alive.
    bool backing_off = (roc < TPS_ROC_RELEASE_PCT_MS &&
                        telemetry.tps_pct < TPS_ROC_RELEASE_HOLD);

    if (!backing_off) {
        _ht_release_start_ms = 0;   // re-arm: still demanding torque
        return;
    }

    // Start cooldown on first tick where backing-off is confirmed
    if (_ht_release_start_ms == 0) {
        _ht_release_start_ms = millis();
        return;
    }

    // Exit when cooldown expires
    if (millis() - _ht_release_start_ms >= TPS_ROC_COOLDOWN_MS) {
        _high_torque_mode = false;
        telemetry.high_torque_mode = false;
        _ht_release_start_ms = 0;
    }
}

// ============================================================================
// REVERSE / PARK INTERLOCK
// Layer 1 (preventive): drive the RP_LOCK solenoid to physically block the lever
//   from leaving the forward range whenever the car is moving. Fail-safe — a dead
//   ESP32 leaves the lever free.
// Layer 2 (reactive failsafe): if R is somehow engaged while still rolling forward
//   (lock not fitted / failed / lever forced), the manual valve has mechanically
//   routed oil to the reverse brake B3 and we cannot stop that. What we CAN do is
//   collapse line pressure so B3 slips and heats instead of shock-loading the
//   driveline, unlock the converter, and warn. Auto-clears once stopped.
// Returns true when the failsafe owns the outputs (caller must skip normal logic).
// ============================================================================
bool ShiftScheduler::checkReverseInhibit() {
    bool moving = telemetry.output_rpm > OUTPUT_RPM_MOVING;
    char now = telemetry.prnd_state;

    // Layer 1: block the lever from LEAVING the forward range while moving. Don't
    // drive the lock while already in R/P/N — its job is the forward→R/P gate only.
    _solenoids->setShiftLock(moving && isForwardRange());

    // The output PCNT sensor has NO direction. We must NOT infer "reverse at speed"
    // purely from prnd=='R' && output>threshold — that also describes a driver simply
    // reversing fast up a driveway, and would wrongly dump pressure and cook B3.
    // Instead latch intent on the EDGE of entering R:
    //   entered R while stopped  -> genuine reverse, allow any subsequent speed
    //   entered R while rolling  -> abuse, run the failsafe
    // Dwell tracker: how long the output shaft has been continuously below the
    // inhibit threshold. One sample below it is not "stopped" — a momentary dip
    // (speed noise, a crest, hard braking blip) at the exact instant R lands must
    // not legitimize reverse at speed.
    if (telemetry.output_rpm <= REVERSE_INHIBIT_SPEED_RPM) {
        if (_slow_since_ms == 0) _slow_since_ms = millis();
    } else {
        _slow_since_ms = 0;
    }

    bool entering_R = (now == 'R' && _prev_prnd != 'R');
    if (entering_R) {
        _legit_reverse = (_slow_since_ms != 0 &&
                          millis() - _slow_since_ms >= REVERSE_LEGIT_STOP_MS);
    }
    if (now != 'R') _legit_reverse = false;   // leaving R clears the latch
    _prev_prnd = now;

    bool abuse = (now == 'R' && !_legit_reverse &&
                  telemetry.output_rpm > REVERSE_INHIBIT_SPEED_RPM);
    if (abuse) {
        _solenoids->stopAllShiftSolenoids();
        _solenoids->setShiftPressure(0);
        _solenoids->setTCC(0);                               // don't transmit rigidly
        _solenoids->setLinePressure(REVERSE_ABUSE_LINE_PCT); // bleed clamp: slip, not shock
        _current_phase = PHASE_CRUISING;
        if (!telemetry.reverse_abuse_active)                 // write the string ONCE, on entry
            setSafetyEvent("REVERSE@SPEED: line pressure dumped (protect B3)");
        telemetry.reverse_abuse_active = true;
        return true;
    }
    telemetry.reverse_abuse_active = false;
    return false;
}

// ============================================================================
// STANDBY + GARAGE (ATSG p.53-54 / spec §7). Called only when NOT shifting.
//   Park or lever-movement window -> pulse Y4 (B2 counter-pressure) + P/N standby duties.
//   N at rest                     -> Y4 off, P/N standby duties.
//   Settled in a driving gear     -> Y4 off, SPC de-energized, MPC on the line schedule.
// The lever-movement window reuses _engage_grace_until_ms (set on the P/N-exit edge).
// ============================================================================
void ShiftScheduler::updateStandbyAndGarage() {
    bool in_park      = (telemetry.prnd_state == 'P');
    bool in_pn        = (telemetry.prnd_state == 'P' || telemetry.prnd_state == 'N');
    bool lever_window = (millis() < _engage_grace_until_ms);

    _solenoids->setGarageY4(in_park || lever_window);

    if (in_pn || lever_window) _solenoids->setStandbyProfile(STANDBY_PARK_NEUTRAL);
    else                       _solenoids->setStandbyProfile(STANDBY_DRIVING);
}

// ============================================================================
// MAIN UPDATE (called every 1ms from core 1)
// ============================================================================
void ShiftScheduler::update() {
    // ---- Reverse/Park interlock + R-while-moving failsafe (HIGHEST PRIORITY) ----
    // Runs above even limp mode: if we're rolling forward and R is selected we must
    // dump line pressure, never let the limp handler clamp B3 at max pressure. Also
    // maintains the RP_LOCK solenoid every loop regardless of any other state.
    if (checkReverseInhibit()) return;

    // ---- Limp-mode enforcement + recovery ----
    if (telemetry.is_limp_mode) {
        // ATSG native failsafe = EVERYTHING de-energized. MPC 100 and SPC 100 are the
        // de-energized (max-pressure / no-current) commands in this API — NOT 0, which
        // would hold SPC at full current.
        _solenoids->setLinePressure(100);
        _solenoids->setShiftPressure(100);
        _solenoids->stopAllShiftSolenoids();
        _solenoids->setTCC(0);
        // p.91: an electrical fault holds the LATCHED gear until stop + ignition cycle.
        // Do not assert 2nd mid-drive — classify from the live ratio instead.
        telemetry.current_gear = classGearFromRatio();
        _current_phase = PHASE_CRUISING;

        // Deliberate recovery: only when stopped, in P/N, and reset requested
        if (telemetry.limp_reset_request &&
            telemetry.output_rpm < 50.0f &&
            (telemetry.prnd_state == 'P' || telemetry.prnd_state == 'N')) {
            telemetry.is_limp_mode = false;
            telemetry.limp_reset_request = false;
            telemetry.is_slipping = false;
            setLimpReason("");
            Serial.println("Limp mode reset.");
        }
        return;
    }

    telemetry.drive_mode = driveModeIndex(telemetry.prnd_state);
    checkTpsROC();
    telemetry.shift_phase = (uint8_t)_current_phase;
    calculateLiveRatio();
    computeClutchSpeeds();   // on/off-clutch slip (clutch-speed model; exposed for bench verify)

    // Input-shaft trust (rnd-ash): in gears 2/3/4 the front planetary is locked, so
    // N2 ≈ N3. A persistent mismatch there ⇒ a dead/wrong N2 or N3 ⇒ turbine_rpm (=f(N2,N3))
    // is unreliable ⇒ checkLimpMode must not fire on it. Only evaluable in 2/3/4 while
    // cruising (in 1/5 N3≈0 by design, and N2≠N3 during a shift); HELD otherwise so a
    // detected fault persists into 5th until it actually recovers back in 2/3/4.
    if (_current_phase == PHASE_CRUISING &&
        telemetry.current_gear >= 2 && telemetry.current_gear <= 4) {
        if (fabsf(telemetry.n2_rpm - telemetry.n3_rpm) > INPUT_TRUST_N2N3_MAX_RPM) {
            if (_n2n3_fault_since_ms == 0) _n2n3_fault_since_ms = millis();
            else if (millis() - _n2n3_fault_since_ms > INPUT_TRUST_CONFIRM_MS)
                telemetry.input_speed_trusted = false;
        } else {
            _n2n3_fault_since_ms = 0;
            telemetry.input_speed_trusted = true;
        }
    }

    // Torque estimate is the master input for all pressure/class decisions (ATSG p.77).
    // From the per-engine torque surface (RPM × MAP), so it ports across engines.
    telemetry.t_est_nm = engineProfile.estimateTorque(telemetry.engine_rpm, telemetry.map_kpa);
    telemetry.load_pct = engineProfile.loadPct(telemetry.engine_rpm, telemetry.map_kpa);
    // Input (turbine) torque = engine × converter multiplication (Phase 2). Exposed for
    // observability and consumed by the Phase 3 pressure model; load_pct/bins stay
    // engine-based for now so the current adaptation mapping is not perturbed pre-Phase 3.
    telemetry.t_input_nm = engineProfile.inputTorque(telemetry.engine_rpm, telemetry.turbine_rpm, telemetry.map_kpa);
    telemetry.road_kmh   = telemetry.output_rpm * engineProfile.kmhPerOutRpm();   // for auto schedule + dash

    // Engine rpm rate (rpm/s, EMA-smoothed over 100ms samples) for predictive overrev.
    if (millis() - _eng_roc_sample_ms >= 100) {
        float dt_s = (millis() - _eng_roc_sample_ms) * 0.001f;
        float inst = (telemetry.engine_rpm - _eng_rpm_prev_sample) / dt_s;
        _eng_rpm_per_s += 0.5f * (inst - _eng_rpm_per_s);
        _eng_rpm_prev_sample = telemetry.engine_rpm;
        _eng_roc_sample_ms = millis();
    }

    TickType_t current_tick = xTaskGetTickCount();
    unsigned long time_in_phase_ms = (current_tick - _phase_start_tick) * portTICK_PERIOD_MS;
    float target_ratio = getTargetRatio(telemetry.target_gear);

    // ---- ABORT a shift if the selector left the forward range mid-shift ----
    // Knocking the lever to N/R/P during a shift hydraulically releases the clutches
    // via the manual valve. Finish cleanly so we never leave a routing solenoid
    // energised, nor hang waiting on a ratio change that can no longer happen.
    bool in_active_shift = (_current_phase != PHASE_CRUISING && _current_phase != PHASE_END);
    if (in_active_shift && !isForwardRange()) {
        _solenoids->stopAllShiftSolenoids();
        _solenoids->setShiftPressure(100);    // de-energized standby (not full current)
        telemetry.current_gear = 2;           // placeholder (hydraulic default); the drive
        telemetry.target_gear  = 2;           // latch re-latches + ratio-resyncs on return to D
        _current_phase = PHASE_CRUISING;
        setSafetyEvent("SHIFT ABORTED (selector left drive)");
    }

    calculateLinePressure();   // CRUISING only; the phase engine owns MPC during shifts
    checkLimpMode(target_ratio);
    checkSafetyShifts();       // auto overrev/lug protection (highest priority)
    checkKickdown();           // power-down on hard tip-in (mutually exclusive w/ coast)
    checkAutoShift();          // full auto up/down schedule (auto drive modes only)
    checkLaunchGear();         // a stopped car drops to 1st (launch gear) in every mode

    // 20ms pressure-update quantizer (ATSG p.80). Sensors + exit checks still run at 1kHz.
    bool ptick = (millis() - _last_pressure_update_ms >= PRESSURE_TICK_MS);
    if (ptick) _last_pressure_update_ms = millis();

    // A genuinely-new 200 Hz speed sample landed this tick? Ratio-derivative predicates
    // (sprag flat) only advance on new samples; between samples the ratio is frozen (B-4).
    bool new_sample = (telemetry.speed_sample_seq != _last_speed_seq);
    if (new_sample) _last_speed_seq = telemetry.speed_sample_seq;

    if (_current_phase == PHASE_CRUISING) {
        updateStandbyAndGarage();   // SPC/MPC standby duties + Y4 garage window

        // Range comes from the 4-bit shifter plate (decodePRND), NOT the multiplexed
        // pin-39 P/N switch. That pin is shared with the ATF temp sensor and reads
        // "in P/N" whenever the temp sensor is open/cold (>3.0 V), which would silently
        // block engagement forever. The plate decodes P/N/R/D directly, so engage off
        // it alone.
        bool in_park_neutral = (telemetry.prnd_state == 'P' || telemetry.prnd_state == 'N');

        // Engagement window: leaving P/N (into D or R) opens the lever window (Y4 pulse
        // + P/N standby duties + slip-limp grace), regardless of whether it lands in D or R.
        bool pn_falling_edge = _prev_pn_raw && !in_park_neutral;
        _prev_pn_raw = in_park_neutral;
        if (pn_falling_edge) {
            _engage_grace_until_ms = millis() + ENGAGE_GRACE_MS;
        }
        // Drive latch: once the debounced plate confirms a FORWARD range. Selecting R
        // does not latch drive_engaged / assert gear 2 (isForwardRange() excludes R).
        if (!telemetry.drive_engaged && isForwardRange()) {
            telemetry.drive_engaged = true;
            telemetry.current_gear  = 2;     // engages at hydraulic-default 2nd; checkLaunchGear() drops to 1st when stopped
            telemetry.target_gear   = 2;
            // Engaged while already rolling (N->D at speed, R->D, or a reboot mid-drive
            // with the shift valves still hydraulically latched in a higher gear):
            // "2nd" is a guess — re-classify from the live ratio after clutch sync.
            _gear_resync_pending = (telemetry.output_rpm > OUTPUT_RPM_MOVING);
            // R->D has no P/N falling edge to open the grace window, so open it here:
            // classification (and slip-limp arming) must wait for the clutches to sync.
            if (_gear_resync_pending) {
                _engage_grace_until_ms = millis() + ENGAGE_GRACE_MS;
                _resync_ready_ms       = _engage_grace_until_ms;
            }
        }
        // Leaving the forward range — P, N, or R, including after a mid-shift abort —
        // drops the latch so the NEXT forward selection re-runs the full engagement
        // above. (R previously never cleared the latch: a D->R->D excursion kept the
        // stale gear label with no resync.) Also clears a pending resync: gear must
        // never be classified from an N/R ratio.
        if (!isForwardRange()) {
            if (telemetry.drive_engaged) _adaptives->requestFlush();  // persist learning at the stop
            telemetry.drive_engaged = false;
            _gear_resync_pending = false;
        }
        if (in_park_neutral) _prev_pn_raw = true;
        // Resync deadline differs by cause: engaging at speed waits the full clutch-sync
        // grace; an unverified shift only needs the driveline to settle (and must resolve
        // BEFORE slip-limp would trip on the mismatch it caused).
        if (_gear_resync_pending && millis() >= _resync_ready_ms) {
            _gear_resync_pending = false;
            uint8_t g = classGearFromRatio();
            if (g != telemetry.current_gear) {
                telemetry.current_gear = g;
                telemetry.target_gear  = g;
                char buf[64];
                snprintf(buf, sizeof(buf), "GEAR RESYNC: ratio says %d", g);
                setSafetyEvent(buf);
                Serial.println(buf);
            }
        }

        if (telemetry.paddle_up_request) {
            telemetry.paddle_up_request = false;
            if (isForwardRange() && telemetry.current_gear < 5)
                beginShift(telemetry.current_gear + 1, true, "PADDLE");
        }
        if (telemetry.paddle_down_request) {
            telemetry.paddle_down_request = false;
            if (isForwardRange() && telemetry.current_gear > 1)
                beginShift(telemetry.current_gear - 1, false, "PADDLE");
        }
    } else {
        runShiftPhases(time_in_phase_ms, ptick, new_sample);
    }

    // High-rate datalog: sample through the shift; finalize (hand to Core 0) when it
    // returns to cruise. runShiftPhases may have ended the shift this tick.
    if (_current_phase != PHASE_CRUISING) {
        captureTrace();
    } else if (shiftTrace.capturing) {
        shiftTrace.capturing = false;
        shiftTrace.ready = true;     // Core 0 serializes + sends the trace
    }

    // rusEFI torque-cut envelope (Phase 5): a phase-aligned WINDOW, not just an INERTIA pulse.
    // Lead-in during TORQUE (ignition retard has latency, so request it before the inertia
    // speed change begins), hold through INERTIA, release at sync (phase leaves INERTIA → LOCK).
    // Aligning the window to the torque-transfer + speed-change event lets the oncoming clutch
    // absorb less energy. A single GPIO controls the window only — amplitude/ramp shaping would
    // need a CAN torque request (out of scope). Still high-load power-upshift only.
    bool tq_cut = (_sclass == SC_POWER_UP) &&
                  (_load_at_start > TORQUE_CUT_MIN_LOAD) &&
                  currentMode().torque_cut &&     // only modes that request it (RACE)
                  (_current_phase == PHASE_TORQUE || _current_phase == PHASE_INERTIA);
    telemetry.torque_cut_active = tq_cut && ENABLE_TORQUE_CUT;
    _solenoids->setTorqueCut(tq_cut);

    updateTCC(ptick);
}

// ============================================================================
// CLASS-AWARE PHASE ENGINE (ATSG-grounded spec §4). Pressure commands move only on
// 20ms ticks (ptick); exit predicates evaluate every 1ms. SPC/MPC are pressure-%
// (de-energized 100 = max apply). _spc_cmd carries fractional ramp across ticks.
// ============================================================================
void ShiftScheduler::setSPC(float pct) {
    _spc_cmd = constrain(pct, 0.0f, 100.0f);
    _solenoids->setShiftPressure((uint8_t)_spc_cmd);
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

void ShiftScheduler::finishShift() {
    telemetry.last_shift_time_ms = millis() - _shift_stopwatch_start;
    _solenoids->stopShiftSolenoid(_active_routing_pin);   // OFF → gear latches hydraulically
    telemetry.current_gear = telemetry.target_gear;
    evaluateAdaptation();
    _current_phase = PHASE_LOCK; _phase_start_tick = xTaskGetTickCount();
}

// Did the ratio actually demonstrate the target gear? Used by both phase backstops to
// decide finish-vs-abandon. Below RATIO_OBSERVABLE_MIN_OUTPUT_RPM there is no real
// ratio to read (see the constant) — trust the command there rather than abandoning a
// shift we simply cannot see, which is the standstill launch case.
// Phase backstop, scaled by ATF temperature. Cold oil fills slowly, so a shift that is
// still unsynced at the hot backstop may be entirely normal when cold — and since F12 a
// backstop hit means abandon + DTC + resync, so a flat value risked spurious abandons on
// a cold morning. Anchors derived from dueATC's driven solenoid-hold map (see
// Reference/DUEATC_CALIBRATION_NOTES.md §1).
uint16_t ShiftScheduler::phaseBackstopMs() const {
    float atf = telemetry.atf_temp_c;
    if (atf <= 0.0f) return SHIFT_BACKSTOP_COLD_MS;
    if (atf >= SHIFT_BACKSTOP_HOT_C) return SHIFT_BACKSTOP_HOT_MS;
    float f = atf / SHIFT_BACKSTOP_HOT_C;                 // 0 at 0 C, 1 at 60 C
    return (uint16_t)(SHIFT_BACKSTOP_COLD_MS +
                      (float)(SHIFT_BACKSTOP_HOT_MS - SHIFT_BACKSTOP_COLD_MS) * f);
}

bool ShiftScheduler::shiftProvedByRatio() const {
    if (telemetry.output_rpm < RATIO_OBSERVABLE_MIN_OUTPUT_RPM) return true;
    return fabsf(telemetry.live_ratio - _ratio_target) <= SHIFT_VERIFY_RATIO_TOL;
}

// A phase backstop expired with the ratio still nowhere near the target: the shift
// did NOT demonstrably happen (weak line pressure, cold ATF, a lazy valve, a failing
// solenoid). Asserting the target gear here is what makes the gear label lie — and the
// label is what picks the routing solenoid for the NEXT shift, so a false one can
// command two clutch packs at once (cross-apply, review item R1).
//
// So: keep the label we had, revert target to it (so the slip-limp check compares
// against the gear we still believe in, not a fiction), and mark the label unverified.
// F1 then blocks every shift until the post-settle ratio resync re-identifies the gear.
// No adaptation — a shift that never proved itself teaches nothing.
void ShiftScheduler::abandonShift(const char* why) {
    telemetry.last_shift_time_ms = millis() - _shift_stopwatch_start;
    _solenoids->stopShiftSolenoid(_active_routing_pin);
    telemetry.target_gear = telemetry.current_gear;
    _gear_resync_pending  = true;
    _resync_ready_ms      = millis() + GEAR_UNVERIFIED_SETTLE_MS;
    dtcManager.trip(DTC_SHIFT_UNVERIFIED);
    setSafetyEvent(why);
    Serial.println(why);
    _current_phase = PHASE_LOCK; _phase_start_tick = xTaskGetTickCount();
}

void ShiftScheduler::runShiftPhases(unsigned long t, bool ptick, bool new_sample) {
    if (ptick && _current_phase != PHASE_END) applyShiftMPC();

    // Ratio derivative is only meaningful across consecutive speed samples. Recompute the
    // "flat" flag (and roll _prev_ratio) ONLY when a fresh sample lands; hold it between
    // samples so the 1 kHz loop reads a stable value instead of a frozen |Δ|=0 (B-4).
    if (new_sample) {
        _ratio_flat = fabsf(telemetry.live_ratio - _prev_ratio) < SPRAG_FLAT_RATIO_DELTA;
        _prev_ratio = telemetry.live_ratio;
    }

    // Flare = ratio rising above the source gear during FILL/TORQUE (under-fill, clutch
    // slipping). Must hold RATIO_EVENT_CONFIRM_MS continuously so one bad speed sample
    // can't latch it and ratchet the adaptation trims (V10).
    if (_current_phase == PHASE_FILL || _current_phase == PHASE_TORQUE) {
        if (telemetry.live_ratio > _ratio_old + 0.10f) {
            if (_flare_over_ms < 60000) _flare_over_ms++;
        } else {
            _flare_over_ms = 0;
        }
        if (_flare_over_ms >= RATIO_EVENT_CONFIRM_MS) telemetry.flare_detected = true;
    }
    if (_current_phase != PHASE_INERTIA) _cl_err = 0.0f;   // closed-loop only sweeps in INERTIA

    switch (_current_phase) {
        case PHASE_PREP:
            if (_is_upshift) setSPC(0);              // about to fill; keep oncoming unclamped
            else             setSPC(_release_spc);   // downshift prep sits at release pressure
            if (t >= PRESSURE_TICK_MS) {
                if (_is_upshift) { setSPC(_fill_p); _current_phase = PHASE_FILL; }
                else             { _current_phase = PHASE_RELEASE; }
                _phase_start_tick = xTaskGetTickCount();
            }
            break;

        case PHASE_FILL:                              // upshift: stroke piston, no ratio movement
            setSPC(_fill_p);                          // (flare detection above, confirm-gated)
            {
                // Fill complete on time, OR (Phase 1b) when the off-going clutch begins to
                // release — the clutch-speed signal EGS uses, far cleaner than gross ratio.
                bool fill_done = (t >= _fill_t_ms) ||
                    (engineProfile.clSpeedTransitions() && telemetry.off_clutch_rpm > CLUTCH_MOVE_RPM);
                if (fill_done) { _current_phase = PHASE_TORQUE; _phase_start_tick = xTaskGetTickCount(); }
            }
            break;

        case PHASE_TORQUE:                            // upshift: oncoming takes torque
            setSPC(_apply_pct);
            if (telemetry.live_ratio < _ratio_old - 0.05f || t >= 250) {
                _spc_cmd = _apply_pct;
                _current_phase = PHASE_INERTIA; _phase_start_tick = xTaskGetTickCount();
            }
            break;

        case PHASE_INERTIA: {                         // upshift: ramp clutch, pull ratio home
            // CLOSED-LOOP SPC (V16): feedforward ramp (_inertia_slope) + proportional trim toward
            // a time-scheduled ratio sweep from _ratio_old → _ratio_target over _inertia_target_ms.
            // Behind schedule (ratio still high) → more clamp; ahead → less. Trim is bounded and
            // applied only on the 20 ms ptick (ATSG quantization). Kp=0 / disabled ⇒ pure ramp.
            float frac = (_inertia_target_ms > 0)
                       ? fminf((float)t / (float)_inertia_target_ms, 1.0f) : 1.0f;
            float scheduled = _ratio_old + (_ratio_target - _ratio_old) * frac;
            _cl_err = telemetry.live_ratio - scheduled;   // upshift: >0 = behind (ratio too high)
            if (ptick) {
                _spc_cmd = constrain(_spc_cmd + _inertia_slope, 0.0f, 100.0f);   // feedforward base
                float trim = engineProfile.clSpcEnable()
                           ? constrain(engineProfile.clSpcKp() * _cl_err, -25.0f, 25.0f) : 0.0f;
                _solenoids->setShiftPressure((uint8_t)constrain(_spc_cmd + trim, 0.0f, 100.0f));
            }
            bool synced = (telemetry.live_ratio <= _ratio_target + 0.03f) ||
                (engineProfile.clSpeedTransitions() && fabsf(telemetry.on_clutch_rpm) <= CLUTCH_SYNC_RPM);
            if (synced) {
                if (t < (unsigned long)(0.6f * _inertia_target_ms)) _harsh_detected = true; // too firm
                finishShift();
            } else if (t >= phaseBackstopMs()) {
                // Backstop: the speed change never happened. Only claim the gear if the
                // ratio actually arrived (tolerance is looser than `synced` — a slow but
                // real shift still counts); otherwise the label stays unverified.
                if (shiftProvedByRatio()) finishShift();
                else abandonShift("UPSHIFT UNVERIFIED (ratio never reached target)");
            }
            break;
        }

        case PHASE_RELEASE: {                         // downshift: off-going exhausts, turbine flares
            setSPC(_release_spc);
            bool go_catch = false;
            if (_sclass == SC_COAST_DOWN) {
                go_catch = (t >= _release_backstop_ms);          // no sync wait at closed throttle
            } else if (_pd_type == PD_SPRAG) {
                // Freewheel catches at sync: ratio reaches target AND its dRatio/dt collapses.
                // Flatness is measured across speed samples (_ratio_flat), not per 1 ms tick.
                bool at_sync = telemetry.live_ratio >= _ratio_target - 0.05f;
                if (at_sync && _ratio_flat) {
                    if (_sync_stable_since_ms == 0) _sync_stable_since_ms = millis();
                    else if (millis() - _sync_stable_since_ms > 40) go_catch = true;
                } else _sync_stable_since_ms = 0;
                if (t >= _release_backstop_ms) go_catch = true;
            } else {                                  // PD_TIMED: clamp after 85% of the ratio change
                float thr = _ratio_old + 0.85f * (_ratio_target - _ratio_old);
                go_catch = (telemetry.live_ratio >= thr || t >= _release_backstop_ms);
            }
            if (go_catch) {
                _output_rpm_at_catch_start = telemetry.output_rpm;
                _catch_start_ms = millis();
                unsigned long pre = _catch_start_ms - _shift_stopwatch_start;
                _ds_baseline_decel_rate = (pre > 0)
                    ? (_output_rpm_at_shift_start - _output_rpm_at_catch_start) / (float)pre : 0.0f;
                _sync_stable_since_ms = 0;
                setSPC(_catch_start_spc);
                _current_phase = PHASE_CATCH; _phase_start_tick = xTaskGetTickCount();
            }
            break;
        }

        case PHASE_CATCH:                             // downshift: clamp as sync approaches
            if (ptick) setSPC(_spc_cmd + _catch_slope);
            // Bind via decel-delta (output decel beyond the pre-catch trend) for ALL
            // downshift classes — power-down learning was unreachable when this was
            // coast-only. Confirm-gated like flare so one bad sample can't latch it.
            {
                float elapsed = (float)(millis() - _catch_start_ms);
                float predicted = _ds_baseline_decel_rate * elapsed;
                float actual = _output_rpm_at_catch_start - telemetry.output_rpm;
                if ((actual - predicted) > DS_BIND_EXTRA_RPM) {
                    if (_bind_over_ms < 60000) _bind_over_ms++;
                } else {
                    _bind_over_ms = 0;
                }
                if (_bind_over_ms >= RATIO_EVENT_CONFIRM_MS) telemetry.bind_detected = true;
            }
            {
                bool at_sync = (telemetry.live_ratio >= _ratio_target - 0.05f &&
                                telemetry.live_ratio <= _ratio_target + 0.05f) ||
                    (engineProfile.clSpeedTransitions() && fabsf(telemetry.on_clutch_rpm) <= CLUTCH_SYNC_RPM);
                if (at_sync) {
                    if (_sync_stable_since_ms == 0) _sync_stable_since_ms = millis();
                    else if (millis() - _sync_stable_since_ms > ((_pd_type == PD_TIMED) ? 60UL : 100UL))
                        finishShift();
                } else _sync_stable_since_ms = 0;
            }
            if (t >= phaseBackstopMs()) {             // backstop — same verify rule as INERTIA
                if (shiftProvedByRatio()) finishShift();
                else abandonShift("DOWNSHIFT UNVERIFIED (ratio never reached target)");
            }
            break;

        case PHASE_LOCK:
            setSPC(100);                              // seat the clutch (de-energized = max apply)
            if (t >= 120) { _current_phase = PHASE_END; _phase_start_tick = xTaskGetTickCount(); }
            break;

        case PHASE_END:                               // decay line to cruise, no thump
            setSPC(100);
            if (ptick) {
                float cur = telemetry.line_pressure_pct;
                float cruise = cruiseLinePressure();
                _solenoids->setLinePressure((uint8_t)((cur > cruise) ? fmaxf(cruise, cur - 5.0f) : cruise));
            }
            if (t >= 200) { _current_phase = PHASE_CRUISING; _phase_start_tick = xTaskGetTickCount(); }
            break;

        default: break;
    }
    // _prev_ratio is rolled at the top, only on a new speed sample (B-4) — not here.
}

// Nearest-ratio gear classifier (ATSG p.91): used by limp so we report the latched
// gear rather than asserting 2nd. At rest the ratio is meaningless → hydraulic default.
uint8_t ShiftScheduler::classGearFromRatio() {
    if (telemetry.output_rpm < 200.0f) return 2;
    float r = telemetry.live_ratio, best_d = 1e9f; uint8_t best = 2;
    for (uint8_t g = 1; g <= 5; g++) {
        float d = fabsf(r - getTargetRatio(g));
        if (d < best_d) { best_d = d; best = g; }
    }
    return best;
}

// Class-indexed adaptation (Adaptation v2). One update per shift, ATF-gated.
void ShiftScheduler::evaluateAdaptation() {
    // Don't let the deliberately-firm manual modes (SPORT/RACE firmness >1) poison the
    // learned trims that the gentler auto modes also use — only learn in auto modes.
    if (!currentMode().auto_shift) return;
    // ATSG p.78: only relearn in the valid ATF window (and never in limp).
    if (telemetry.atf_temp_c < ADAPT_ATF_MIN_C || telemetry.atf_temp_c > ADAPT_ATF_MAX_C) return;
    _adaptives->learn((uint8_t)_sclass, _active_shift_idx, _torque_bin,
                      telemetry.flare_detected, _harsh_detected, telemetry.bind_detected);
}
