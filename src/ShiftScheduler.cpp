// ============================================================================
// FILE: ShiftScheduler.cpp
// VERSION: 6.0
// UPDATES: Safety auto-shifts, flare detection, P/N latch fix, limp recovery,
//          centralised beginShift() (no more duplicated off-by-one triggers).
// ============================================================================
#include "ShiftScheduler.h"
#include "EngineProfile.h"

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
    _prev_prnd = 'P';
    _legit_reverse = false;
    _prev_was_power = false;
    _last_pressure_update_ms = millis();
    _spc_cmd = 0.0f;
    _harsh_detected = false;
    _flare_over_ms = 0;
    _bind_over_ms = 0;
    _eng_rpm_prev_sample = 0.0f;
    _eng_roc_sample_ms = millis();
    _eng_rpm_per_s = 0.0f;
}

bool ShiftScheduler::isForwardRange() {
    char s = telemetry.prnd_state;
    return (s == 'D' || s == '4' || s == '3' || s == '2' || s == '1');
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
    uint8_t gear_idx = constrain(telemetry.current_gear - 1, 0, 4);
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
                          telemetry.tps_pct > 45.0f;
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
    if (target_gear < 1 || target_gear > 5) return false;

    // Money-shift / overrev guard on ANY downshift (manual or auto)
    if (!is_upshift) {
        float predicted = telemetry.output_rpm * getTargetRatio(target_gear);
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
    _active_routing_pin = getRoutingSolenoidForShift(_from_gear, target_gear);

    _solenoids->fireShiftSolenoid(_active_routing_pin);
    _shift_stopwatch_start    = millis();
    _last_pressure_update_ms  = millis();
    _phase_start_tick = xTaskGetTickCount();
    _current_phase = PHASE_PREP;     // all classes start in PREP
    applyShiftMPC();                 // lead the gate: set line/overlap authority on THIS tick,
                                     // not the next 20ms ptick (spec PREP intent) — B-7
    return true;
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
            _apply_pct = (uint8_t)constrain(20.0f + 0.55f * load, 0.0f, 100.0f);
            _inertia_slope = 2.0f + 0.02f * load;                       // %/20ms tick
            _inertia_target_ms = (uint16_t)constrain(400.0f - 1.5f * load, 220.0f, 400.0f);
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
    if (telemetry.engine_rpm < engineProfile.lugRpm() &&
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
// COAST-DOWN SCHEDULER (spec §4.5). While coasting to a stop at closed throttle,
// auto-downshift at output-shaft RPM thresholds so each catch lands at an idle-
// friendly turbine speed. Floor is 2nd. Suppressed if a paddle request is pending.
// ----------------------------------------------------------------------------
void ShiftScheduler::checkCoastDownSchedule() {
    if (_current_phase != PHASE_CRUISING) return;
    if (!isForwardRange()) return;
    if (telemetry.paddle_up_request || telemetry.paddle_down_request) return;
    if (millis() - telemetry.last_auto_shift_ms < AUTO_SHIFT_COOLDOWN_MS) return;
    if (telemetry.tps_pct >= CLASS_COAST_TPS_PCT) return;     // coast only (closed throttle)

    uint8_t g = telemetry.current_gear;
    if (g <= 2) return;                                       // floor at 2nd
    float o = telemetry.output_rpm;
    bool want = (g == 5 && o < COAST_DN_5_TO_4) ||
                (g == 4 && o < COAST_DN_4_TO_3) ||
                (g == 3 && o < COAST_DN_3_TO_2);
    if (want && beginShift(g - 1, false, "COAST")) {
        telemetry.last_auto_shift_ms = millis();
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
    bool entering_R = (now == 'R' && _prev_prnd != 'R');
    if (entering_R) {
        _legit_reverse = (telemetry.output_rpm <= REVERSE_INHIBIT_SPEED_RPM);
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

    checkTpsROC();
    telemetry.shift_phase = (uint8_t)_current_phase;
    calculateLiveRatio();

    // Torque estimate is the master input for all pressure/class decisions (ATSG p.77).
    // From the per-engine torque surface (RPM × MAP), so it ports across engines.
    telemetry.t_est_nm = engineProfile.estimateTorque(telemetry.engine_rpm, telemetry.map_kpa);
    telemetry.load_pct = engineProfile.loadPct(telemetry.engine_rpm, telemetry.map_kpa);

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
        telemetry.current_gear = 2;           // disengaging to N → re-engages at 2nd
        telemetry.target_gear  = 2;
        _current_phase = PHASE_CRUISING;
        setSafetyEvent("SHIFT ABORTED (selector left drive)");
    }

    calculateLinePressure();   // CRUISING only; the phase engine owns MPC during shifts
    checkLimpMode(target_ratio);
    checkSafetyShifts();       // auto overrev/lug protection (highest priority)
    checkKickdown();           // power-down on hard tip-in (mutually exclusive w/ coast)
    checkCoastDownSchedule();  // auto downshift while coasting to a stop

    // 20ms pressure-update quantizer (ATSG p.80). Sensors + exit checks still run at 1kHz.
    bool ptick = (millis() - _last_pressure_update_ms >= PRESSURE_TICK_MS);
    if (ptick) _last_pressure_update_ms = millis();

    // A genuinely-new 200 Hz speed sample landed this tick? Ratio-derivative predicates
    // (sprag flat) only advance on new samples; between samples the ratio is frozen (B-4).
    bool new_sample = (telemetry.speed_sample_seq != _last_speed_seq);
    if (new_sample) _last_speed_seq = telemetry.speed_sample_seq;

    if (_current_phase == PHASE_CRUISING) {
        updateStandbyAndGarage();   // SPC/MPC standby duties + Y4 garage window

        // Engagement window: the FALLING EDGE of the P/N switch means the manual
        // valve is leaving P/N — open the lever window (Y4 pulse + P/N standby
        // duties + slip-limp grace) regardless of whether it lands in D or R.
        bool pn_falling_edge = _prev_pn_raw && !telemetry.pn_switch_raw;
        _prev_pn_raw = telemetry.pn_switch_raw;
        if (pn_falling_edge) {
            _engage_grace_until_ms = millis() + ENGAGE_GRACE_MS;
        }
        // Drive latch: only once the (debounced) decoder confirms a FORWARD range.
        // Selecting R no longer latches drive_engaged / asserts gear 2 (it also
        // enabled TPS-ROC max-line mode in reverse).
        if (!telemetry.pn_switch_raw && !telemetry.drive_engaged && isForwardRange()) {
            telemetry.drive_engaged = true;
            telemetry.current_gear  = 2;     // 722.6 hydraulic default; 1st is paddle-only
            telemetry.target_gear   = 2;
            // Engaged while already rolling (N->D at speed, or a reboot mid-drive
            // with the shift valves still hydraulically latched in a higher gear):
            // "2nd" is a guess — re-classify from the live ratio after clutch sync.
            _gear_resync_pending = (telemetry.output_rpm > OUTPUT_RPM_MOVING);
        }
        if (telemetry.prnd_state == 'P' || telemetry.prnd_state == 'N') {
            if (telemetry.drive_engaged) _adaptives->requestFlush();  // persist learning at the stop
            telemetry.drive_engaged = false;
            _gear_resync_pending = false;
            _prev_pn_raw = true;
        }
        if (_gear_resync_pending && millis() >= _engage_grace_until_ms) {
            _gear_resync_pending = false;
            uint8_t g = classGearFromRatio();
            if (g != telemetry.current_gear) {
                telemetry.current_gear = g;
                telemetry.target_gear  = g;
                char buf[64];
                snprintf(buf, sizeof(buf), "GEAR RESYNC: ratio says %d (engaged at speed)", g);
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

    // rusEFI torque-cut: only during a high-load power-up inertia phase (spec §9).
    _solenoids->setTorqueCut(_current_phase == PHASE_INERTIA &&
                             _sclass == SC_POWER_UP &&
                             _load_at_start > TORQUE_CUT_MIN_LOAD);

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
            if (t >= _fill_t_ms) { _current_phase = PHASE_TORQUE; _phase_start_tick = xTaskGetTickCount(); }
            break;

        case PHASE_TORQUE:                            // upshift: oncoming takes torque
            setSPC(_apply_pct);
            if (telemetry.live_ratio < _ratio_old - 0.05f || t >= 250) {
                _spc_cmd = _apply_pct;
                _current_phase = PHASE_INERTIA; _phase_start_tick = xTaskGetTickCount();
            }
            break;

        case PHASE_INERTIA:                           // upshift: ramp clutch, pull ratio home
            if (ptick) setSPC(_spc_cmd + _inertia_slope);
            if (telemetry.live_ratio <= _ratio_target + 0.03f) {
                if (t < (unsigned long)(0.6f * _inertia_target_ms)) _harsh_detected = true; // too firm
                finishShift();
            } else if (t >= 600) {
                finishShift();                        // ratio backstop
            }
            break;

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
            if (telemetry.live_ratio >= _ratio_target - 0.05f &&
                telemetry.live_ratio <= _ratio_target + 0.05f) {
                if (_sync_stable_since_ms == 0) _sync_stable_since_ms = millis();
                else if (millis() - _sync_stable_since_ms > ((_pd_type == PD_TIMED) ? 60UL : 100UL))
                    finishShift();
            } else _sync_stable_since_ms = 0;
            if (t >= 600) finishShift();
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
    // ATSG p.78: only relearn in the valid ATF window (and never in limp).
    if (telemetry.atf_temp_c < ADAPT_ATF_MIN_C || telemetry.atf_temp_c > ADAPT_ATF_MAX_C) return;
    _adaptives->learn((uint8_t)_sclass, _active_shift_idx, _torque_bin,
                      telemetry.flare_detected, _harsh_detected, telemetry.bind_detected);
}
