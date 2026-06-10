// ============================================================================
// FILE: ShiftScheduler.cpp
// VERSION: 6.0
// UPDATES: Safety auto-shifts, flare detection, P/N latch fix, limp recovery,
//          centralised beginShift() (no more duplicated off-by-one triggers).
// ============================================================================
#include "ShiftScheduler.h"

ShiftScheduler::ShiftScheduler(SolenoidDriver* solenoids, AdaptiveMemory* adaptives) {
    _solenoids = solenoids;
    _adaptives = adaptives;
    _current_phase = PHASE_CRUISING;
    _active_routing_pin = 0;
    _active_shift_idx = 0;
    _ratio_at_overlap_start = 0.0f;
}

void ShiftScheduler::begin() {
    _current_phase = PHASE_CRUISING;
    telemetry.current_gear = 1;
    telemetry.target_gear = 1;
    _prev_pn_raw = true;
    _prev_tps = 0.0f;
    _high_torque_mode = false;
    _ht_release_start_ms = 0;
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
void ShiftScheduler::calculateLinePressure() {
    // TPS ROC mode: torque is arriving NOW — skip the lookup, go straight to max.
    // Only during cruise; during a shift the shift logic already manages pressures.
    if (_high_torque_mode && _current_phase == PHASE_CRUISING) {
        _solenoids->setLinePressure(100);
        return;
    }

    float target_pressure_pct = 0.0f;
    if (_current_phase == PHASE_CRUISING) {
        uint8_t gear_idx = constrain(telemetry.current_gear - 1, 0, 4);
        uint8_t load_idx = loadToBin(computeLoad(telemetry.tps_pct, telemetry.map_kpa));
        target_pressure_pct = HOLDING_PRESSURE_MAP[gear_idx][load_idx];
        if (telemetry.engine_rpm < 1200.0f) target_pressure_pct += 10.0f;
    } else {
        target_pressure_pct = telemetry.tps_pct;
        if (telemetry.map_kpa > 100.0f) target_pressure_pct += (telemetry.map_kpa - 100.0f) * 2.0f;
        target_pressure_pct += 20.0f;
    }
    // Cold ATF is viscous — circuits fill slowly, needs more pressure.
    // Hot ATF leaks past seals — needs more pressure to maintain clamping.
    // Normal operating range (40-80°C) is the baseline.
    float atf = telemetry.atf_temp_c;
    float temp_mult = 1.0f;
    if      (atf < 20.0f)  temp_mult = 1.30f;  // very cold: +30%
    else if (atf < 40.0f)  temp_mult = 1.15f;  // cold: +15%
    else if (atf < 80.0f)  temp_mult = 1.00f;  // normal
    else if (atf < 110.0f) temp_mult = 1.05f;  // warm: minor seal leakage +5%
    else                    temp_mult = 1.20f;  // hot: significant leakage +20%
    target_pressure_pct *= temp_mult;

    _solenoids->setLinePressure((uint8_t)constrain(target_pressure_pct, 10.0f, 100.0f));
}

void ShiftScheduler::calculateLiveRatio() {
    if (telemetry.output_rpm > 50.0f) telemetry.live_ratio = telemetry.turbine_rpm / telemetry.output_rpm;
    else telemetry.live_ratio = getTargetRatio(telemetry.current_gear);
}

// ----------------------------------------------------------------------------
// TCC DYNAMIC SLIP CONTROLLER (unchanged logic, reads real tcc_lockup_pct now)
// ----------------------------------------------------------------------------
void ShiftScheduler::updateTCC() {
    telemetry.tcc_actual_slip_rpm = telemetry.engine_rpm - telemetry.turbine_rpm;
    if (telemetry.tcc_actual_slip_rpm < 0) telemetry.tcc_actual_slip_rpm = 0;

    int current_tcc_pwm = telemetry.tcc_lockup_pct;

    if (_current_phase == PHASE_CRUISING) {
        // Force TCC open if: ROC mode active (fast tip-in), MAP shows boost,
        // or TPS is above moderate demand. ROC mode takes priority — it reacts
        // faster than MAP settling.
        bool force_open = _high_torque_mode ||
                          telemetry.map_kpa > 105.0f ||
                          telemetry.tps_pct > 45.0f;
        if (force_open) {
            telemetry.tcc_target_slip_rpm = 500.0f;
            current_tcc_pwm -= 2;
        } else if (telemetry.engine_rpm < 1400.0f || telemetry.current_gear == 1) {
            telemetry.tcc_target_slip_rpm = 1000.0f;
            current_tcc_pwm -= 3;
        } else {
            telemetry.tcc_target_slip_rpm = 50.0f;
            if (telemetry.tcc_actual_slip_rpm > (telemetry.tcc_target_slip_rpm + 20.0f)) {
                if (current_tcc_pwm < 85) current_tcc_pwm += 1;
            } else if (telemetry.tcc_actual_slip_rpm < (telemetry.tcc_target_slip_rpm - 10.0f)) {
                current_tcc_pwm -= 1;
            }
        }
    } else {
        telemetry.tcc_target_slip_rpm = 1000.0f;
        current_tcc_pwm -= 5;
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

    telemetry.target_gear = target_gear;
    _is_upshift = is_upshift;
    // Adaptive index: upshift uses lower gear-1, downshift uses target gear-1
    _active_shift_idx = is_upshift ? (telemetry.current_gear - 1) : (target_gear - 1);
    _active_shift_idx = constrain(_active_shift_idx, 0, NUM_UPSHIFTS - 1);

    _current_pressure_mod = _adaptives->getModifier(is_upshift, _active_shift_idx, true,
                                telemetry.tps_pct, telemetry.map_kpa, telemetry.engine_rpm);
    _current_timing_mod   = _adaptives->getModifier(is_upshift, _active_shift_idx, false,
                                telemetry.tps_pct, telemetry.map_kpa, telemetry.engine_rpm);

    telemetry.flare_detected = false;
    telemetry.bind_detected  = false;
    _turbine_rpm_at_shift_start = telemetry.turbine_rpm;
    _output_rpm_at_shift_start  = telemetry.output_rpm;
    _active_routing_pin = getRoutingSolenoidForShift(telemetry.current_gear, target_gear);

    _solenoids->fireShiftSolenoid(_active_routing_pin);
    _shift_stopwatch_start = millis();
    _phase_start_tick = xTaskGetTickCount();

    if (is_upshift) {
        _solenoids->setShiftPressure(80);
        _current_phase = PHASE_PREFILL;
    } else {
        _current_phase = PHASE_DS_RELEASE;
    }
    return true;
}

// ----------------------------------------------------------------------------
// AUTO-SAFETY LAYER  (the feature requested: overrev upshift + lug downshift)
// Only fires while CRUISING, in a forward driving state, off cooldown.
// ----------------------------------------------------------------------------
void ShiftScheduler::checkSafetyShifts() {
    if (_current_phase != PHASE_CRUISING) return;
    if (telemetry.prnd_state != 'D' && telemetry.prnd_state != '4' &&
        telemetry.prnd_state != '3' && telemetry.prnd_state != '2') return;
    if (millis() - telemetry.last_auto_shift_ms < AUTO_SHIFT_COOLDOWN_MS) return;

    // --- OVERREV: force an upshift before the engine hits the limiter ---
    if (telemetry.engine_rpm > RPM_OVERREV_UPSHIFT && telemetry.current_gear < 5) {
        if (beginShift(telemetry.current_gear + 1, true, "OVERREV")) {
            telemetry.last_auto_shift_ms = millis();
            telemetry.last_safety_event = "AUTO UPSHIFT (overrev " + String((int)telemetry.engine_rpm) + ")";
            Serial.println(telemetry.last_safety_event);
        }
        return;
    }

    // --- LUG: force a downshift if loaded and below safe RPM ---
    if (telemetry.engine_rpm < RPM_LUG_THRESHOLD &&
        telemetry.tps_pct > TPS_LUG_LOAD_PCT &&
        telemetry.current_gear > 1) {
        // Predictive check: skip if the lower gear would still sit below lug threshold.
        // At crawl speeds no gear can rescue RPM — shifting just burns clutch packs.
        // Require at least 300 RPM headroom above threshold after ratio change.
        float predicted_rpm = telemetry.output_rpm * getTargetRatio(telemetry.current_gear - 1);
        if (predicted_rpm < RPM_LUG_THRESHOLD + 300.0f) {
            telemetry.last_safety_event = "LUG (speed too low, all gears lugging)";
            return;
        }
        if (beginShift(telemetry.current_gear - 1, false, "LUG")) {
            telemetry.last_auto_shift_ms = millis();
            telemetry.last_safety_event = "AUTO DOWNSHIFT (lug " + String((int)telemetry.engine_rpm) + ")";
            Serial.println(telemetry.last_safety_event);
        }
    }
}

// ----------------------------------------------------------------------------
// LIMP MODE  (load-aware threshold + deliberate reset path)
// ----------------------------------------------------------------------------
void ShiftScheduler::checkLimpMode(float target_ratio) {
    // Slip detection only while cruising, in D, moving, and NOT at high load
    // (high-boost launches legitimately slip the converter and chirp tyres).
    bool conditions = (_current_phase == PHASE_CRUISING &&
                       telemetry.prnd_state == 'D' &&
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
                telemetry.limp_mode_reason = "FATAL SLIP GEAR " + String(telemetry.current_gear) +
                                             " DIFF " + String((int)mismatch) + " RPM";
                Serial.println("!!! TRANSMISSION PROTECTION ACTIVATED !!!");
                Serial.println(telemetry.limp_mode_reason);
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
// ============================================================================
void ShiftScheduler::checkTpsROC() {
    if (!telemetry.drive_engaged) {
        _prev_tps = telemetry.tps_pct;
        return;
    }

    float roc = telemetry.tps_pct - _prev_tps;  // %/ms (called at 1kHz)
    _prev_tps = telemetry.tps_pct;

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
// MAIN UPDATE (called every 1ms from core 1)
// ============================================================================
void ShiftScheduler::update() {
    // ---- Limp-mode enforcement + recovery ----
    if (telemetry.is_limp_mode) {
        _solenoids->setLinePressure(100);
        _solenoids->setShiftPressure(0);
        _solenoids->stopAllShiftSolenoids();   // hydraulically defaults to 2nd
        _solenoids->setTCC(0);
        telemetry.current_gear = 2;
        _current_phase = PHASE_CRUISING;

        // Deliberate recovery: only when stopped, in P/N, and reset requested
        if (telemetry.limp_reset_request &&
            telemetry.output_rpm < 50.0f &&
            (telemetry.prnd_state == 'P' || telemetry.prnd_state == 'N')) {
            telemetry.is_limp_mode = false;
            telemetry.limp_reset_request = false;
            telemetry.is_slipping = false;
            telemetry.limp_mode_reason = "";
            Serial.println("Limp mode reset.");
        }
        return;
    }

    checkTpsROC();
    telemetry.shift_phase = (uint8_t)_current_phase;
    calculateLinePressure();
    calculateLiveRatio();

    TickType_t current_tick = xTaskGetTickCount();
    unsigned long time_in_phase_ms = (current_tick - _phase_start_tick) * portTICK_PERIOD_MS;
    float target_ratio = getTargetRatio(telemetry.target_gear);

    checkLimpMode(target_ratio);
    checkSafetyShifts();   // <-- auto overrev/lug protection runs every loop

    switch (_current_phase) {
        case PHASE_CRUISING: {
            // Garage shift: trigger on the FALLING EDGE of the P/N switch.
            // The physical P/N switch (multiplexed on ATF_TEMP pin) opens the moment
            // the manual valve leaves P/N — this is the reliable trigger, independent
            // of whether the 4-bit PRND decoder has settled on 'D' yet.
            bool pn_falling_edge = _prev_pn_raw && !telemetry.pn_switch_raw;
            _prev_pn_raw = telemetry.pn_switch_raw;

            if (pn_falling_edge && !telemetry.drive_engaged) {
                // Acknowledge the hydraulic default: 722.6 sits in 2nd gear with no
                // electronics. Driver selects 1st via paddle-down if they want it.
                telemetry.drive_engaged = true;
                telemetry.current_gear  = 2;
                telemetry.target_gear   = 2;
            }

            // Returning to P/N: re-arm edge detector.
            if (telemetry.prnd_state == 'P' || telemetry.prnd_state == 'N') {
                telemetry.drive_engaged = false;
                _prev_pn_raw = true;
            }

            // MANUAL UPSHIFT
            if (telemetry.paddle_up_request) {
                telemetry.paddle_up_request = false;
                if (telemetry.current_gear < 5)
                    beginShift(telemetry.current_gear + 1, true, "PADDLE");
            }
            // MANUAL DOWNSHIFT (guard inside beginShift)
            if (telemetry.paddle_down_request) {
                telemetry.paddle_down_request = false;
                if (telemetry.current_gear > 1)
                    beginShift(telemetry.current_gear - 1, false, "PADDLE");
            }
            break;
        }

        case PHASE_PREFILL:
            if (time_in_phase_ms > (uint32_t)(60 + _current_timing_mod)) {
                _solenoids->setShiftPressure(constrain(45 + _current_pressure_mod, 0, 100));
                _ratio_at_overlap_start = telemetry.live_ratio;
                _current_phase = PHASE_OVERLAP; _phase_start_tick = current_tick;
                // Stopwatch intentionally NOT reset here — last_shift_time_ms must
                // measure the full shift from trigger (set in beginShift) to completion.
            }
            break;

        case PHASE_OVERLAP:
            if (telemetry.shift_pressure_pct < 90)   // ramp to 90 (was 80)
                _solenoids->setShiftPressure(telemetry.shift_pressure_pct + 1);

            if (telemetry.live_ratio > (_ratio_at_overlap_start + 0.15f)) {
                telemetry.flare_detected = true;
            }
            if (telemetry.live_ratio < (getTargetRatio(telemetry.current_gear) - 0.1f)) {
                _current_phase = PHASE_INERTIA; _phase_start_tick = current_tick;
            }
            break;

        case PHASE_INERTIA:
            if (telemetry.shift_pressure_pct < 100)   // ramp to 100 (was 90)
                _solenoids->setShiftPressure(telemetry.shift_pressure_pct + 1);
            if (telemetry.live_ratio > (_ratio_at_overlap_start + 0.15f))
                telemetry.flare_detected = true;
            if (telemetry.live_ratio <= (target_ratio + 0.05f) || time_in_phase_ms > 600) {
                telemetry.last_shift_time_ms = millis() - _shift_stopwatch_start;
                _solenoids->stopShiftSolenoid(_active_routing_pin);
                _solenoids->setShiftPressure(0);
                telemetry.current_gear = telemetry.target_gear;
                _current_phase = PHASE_COMPLETION; _phase_start_tick = current_tick;
            }
            break;

        case PHASE_DS_RELEASE:
            _solenoids->setShiftPressure(0);
            if (time_in_phase_ms > (uint32_t)(80 + _current_timing_mod)) {
                _current_phase = PHASE_DS_SYNC; _phase_start_tick = current_tick;
            }
            break;

        case PHASE_DS_SYNC:
            // Wait until the engine has rev-matched to within 5% of the target gear
            // ratio before the catch clutch applies. The old 0.2 threshold was
            // satisfied immediately (current ratio already > target-0.2 on entry).
            if (telemetry.live_ratio >= (target_ratio - 0.05f) || time_in_phase_ms > 400) {
                _current_phase = PHASE_DS_CATCH; _phase_start_tick = current_tick;
            }
            break;

        case PHASE_DS_CATCH:
            if (telemetry.shift_pressure_pct < (90 + _current_pressure_mod))
                _solenoids->setShiftPressure(telemetry.shift_pressure_pct + 2);
            if (telemetry.output_rpm < (_output_rpm_at_shift_start - 30.0f))
                telemetry.bind_detected = true;
            // DISTINCT exit from SYNC: tighter ratio tolerance (0.02 vs 0.05)
            // AND minimum 40ms — ensures the pressure ramp has real duration before
            // we declare the gear locked. Previously used target-0.05 (same as SYNC)
            // which meant CATCH exited on its first tick, applying a single +2% step.
            if ((telemetry.live_ratio >= (target_ratio - 0.02f) && time_in_phase_ms >= 40)
                || time_in_phase_ms > 300) {
                telemetry.last_shift_time_ms = millis() - _shift_stopwatch_start;
                _solenoids->stopShiftSolenoid(_active_routing_pin);
                _solenoids->setShiftPressure(0);
                telemetry.current_gear = telemetry.target_gear;
                _current_phase = PHASE_COMPLETION; _phase_start_tick = current_tick;
            }
            break;

        case PHASE_COMPLETION:
            if (time_in_phase_ms > 50) {
                // Explicit, unambiguous index + direction (no gear arithmetic)
                _adaptives->evaluateShift(_is_upshift, _active_shift_idx,
                    telemetry.tps_pct, telemetry.map_kpa, telemetry.engine_rpm,
                    telemetry.last_shift_time_ms, telemetry.flare_detected, telemetry.bind_detected);
                _current_phase = PHASE_CRUISING;
            }
            break;
    }

    updateTCC();
}
