// Active shift phases and evidence required to confirm completion.
// All methods run on the physics task; see docs/CONTROL_ARCHITECTURE.md.
#include "ShiftScheduler.h"
#include "EngineProfile.h"
#include "DtcManager.h"
#include "AutoShiftMap.h"
#include "TuneOverlay.h"

// Clutch-speed is the shift observer we actually have (no current sensing, no OEM
// pressure tables). Only trust it when N2/N3/OUT are real: a bench with every
// sensor at 0 must still run on the fill/backstop timers.
bool ShiftScheduler::clutchSpeedsLive() const {
    return engineProfile.clSpeedTransitions() && ratioFeedbackLive();
}

// No edges is not evidence of synchronisation. This freshness gate complements
// plausibility; it cannot diagnose every individual sensor wiring fault.
bool ShiftScheduler::ratioFeedbackLive() const {
    return !telemetry.test_mode && telemetry.speed_hw_ok &&
        telemetry.n2_signal_recent && telemetry.output_signal_recent &&
        telemetry.input_speed_trusted && _have_ratio_sample &&
        millis() - _last_ratio_sample_ms <= RATIO_SAMPLE_MAX_AGE_MS &&
        telemetry.output_rpm >= RATIO_OBSERVABLE_MIN_OUTPUT_RPM &&
        telemetry.turbine_rpm > 25.0f &&
        isfinite(telemetry.live_ratio);
}

bool ShiftScheduler::targetRatioConfirmed(unsigned long dwell_ms, bool new_sample) {
    if (!ratioFeedbackLive() ||
        fabsf(telemetry.live_ratio - _ratio_target) > SHIFT_SYNC_RATIO_TOL) {
        _target_ratio_tracking = false;
        return false;
    }
    if (!new_sample) return false;
    if (!_target_ratio_tracking) {
        _target_ratio_tracking = true;
        _target_ratio_since_ms = millis();
        return false;
    }
    return millis() - _target_ratio_since_ms >= dwell_ms;
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

void ShiftScheduler::finishShift() {
    telemetry.last_shift_time_ms = millis() - _shift_stopwatch_start;
    _solenoids->stopShiftSolenoid(_active_routing_pin);   // OFF → gear latches hydraulically
    telemetry.current_gear = telemetry.target_gear;
    _last_sclass = (uint8_t)_sclass;
    _last_shift_idx = _active_shift_idx;
    _last_tbin = _torque_bin;
    _last_adapt_valid = ratioFeedbackLive() && !telemetry.is_limp_mode;
    evaluateAdaptation();
    _current_phase = PHASE_LOCK; _phase_start_tick = xTaskGetTickCount();
}

uint16_t ShiftScheduler::phaseBackstopMs() const {
    uint16_t hot = tuneOverlay.backstopHotMs(), cold = tuneOverlay.backstopColdMs();
    float atf = telemetry.atf_temp_c;
    if (atf <= 0.0f) return cold;
    if (atf >= SHIFT_BACKSTOP_HOT_C) return hot;
    float f = atf / SHIFT_BACKSTOP_HOT_C;                 // 0 at 0 C, 1 at 60 C
    return (uint16_t)((float)cold + ((float)hot - (float)cold) * f);
}

bool ShiftScheduler::stationarySequenceAllowed() const {
    // Preserve deliberate stationary/bench sequencing. A rolling shift losing
    // its speed signal must never turn into a successful stationary shift.
    return telemetry.test_mode || (!_shift_started_observable &&
        telemetry.speed_hw_ok &&
        telemetry.output_rpm < RATIO_OBSERVABLE_MIN_OUTPUT_RPM &&
        telemetry.turbine_rpm < RATIO_OBSERVABLE_MIN_OUTPUT_RPM);
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

    // Flare = ratio rising above the source gear during FILL/TORQUE, OR the off-going
    // clutch speed going negative (UN52: sign flip is the cleanest flare detector —
    // do NOT fabs it away). Confirm-gated so one bad sample can't ratchet adaptation.
    if (_current_phase == PHASE_FILL || _current_phase == PHASE_TORQUE) {
        bool ratio_flare  = ratioFeedbackLive() && telemetry.live_ratio > _ratio_old + 0.10f;
        bool clutch_flare = clutchSpeedsLive() && telemetry.off_clutch_rpm < -CLUTCH_MOVE_RPM;
        if (ratio_flare || clutch_flare) {
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
                // Fill complete: off-going clutch starts to move (the EGS signal), else the
                // scheduled fill timer. Timer is the only path on a dead bench.
                if (clutchSpeedsLive() && telemetry.off_clutch_rpm > CLUTCH_MOVE_RPM) {
                    if (_clutch_move_over_ms < 60000) _clutch_move_over_ms++;
                } else {
                    _clutch_move_over_ms = 0;
                }
                bool clutch_fill = clutchSpeedsLive() && _clutch_move_over_ms >= RATIO_EVENT_CONFIRM_MS;
                if (clutch_fill && _fill_move_ms == 0) _fill_move_ms = (uint16_t)constrain(t, 1, 60000);
                if (clutch_fill || t >= _fill_t_ms) {
                    _current_phase = PHASE_TORQUE; _phase_start_tick = xTaskGetTickCount();
                }
            }
            break;

        case PHASE_TORQUE:                            // upshift: oncoming takes torque
            setSPC(_apply_pct);
            if ((ratioFeedbackLive() && telemetry.live_ratio < _ratio_old - 0.05f) || t >= 250) {
                _spc_cmd = _apply_pct;
                _target_ratio_tracking = false;
                _current_phase = PHASE_INERTIA; _phase_start_tick = xTaskGetTickCount();
            }
            break;

        case PHASE_INERTIA: {                         // upshift: ramp clutch, pull ratio home
            // One optional feedback loop: scheduled ratio + bounded proportional
            // trim around the existing PWM ramp. No current/pressure inference.
            float frac = (_inertia_target_ms > 0)
                       ? fminf((float)t / (float)_inertia_target_ms, 1.0f) : 1.0f;
            float scheduled_ratio = _ratio_old + (_ratio_target - _ratio_old) * frac;
            bool feedback = engineProfile.clSpcEnable() && ratioFeedbackLive();
            _cl_err = feedback ? telemetry.live_ratio - scheduled_ratio : 0.0f;
            if (ptick) {
                _spc_cmd = constrain(_spc_cmd + _inertia_slope, 0.0f, 100.0f);
                float kp = feedback ? engineProfile.clSpcKp() : 0.0f;
                float trim = constrain(kp * _cl_err, -25.0f, 25.0f);
                _solenoids->setShiftPressure((uint8_t)constrain(_spc_cmd + trim, 0.0f, 100.0f));
            }
            // Symmetric tolerance rejects undershoot; repeated fresh samples
            // reject a single spike or a frozen reading at the target.
            if (targetRatioConfirmed(UPSHIFT_SYNC_DWELL_MS, new_sample)) {
                if (t < (unsigned long)(0.6f * _inertia_target_ms)) _harsh_detected = true;
                finishShift();
            } else if (t >= phaseBackstopMs()) {
                if (stationarySequenceAllowed()) finishShift();
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
                bool at_sync = ratioFeedbackLive() && telemetry.live_ratio >= _ratio_target - 0.05f;
                if (at_sync && _ratio_flat) {
                    if (_sync_stable_since_ms == 0) _sync_stable_since_ms = millis();
                    else if (millis() - _sync_stable_since_ms > 40) go_catch = true;
                } else _sync_stable_since_ms = 0;
                if (t >= _release_backstop_ms) go_catch = true;
            } else {                                  // PD_TIMED: clamp after 85% of the ratio change
                float thr = _ratio_old + 0.85f * (_ratio_target - _ratio_old);
                go_catch = ((ratioFeedbackLive() && telemetry.live_ratio >= thr) || t >= _release_backstop_ms);
            }
            if (go_catch) {
                _output_rpm_at_catch_start = telemetry.output_rpm;
                _catch_start_ms = millis();
                unsigned long pre = _catch_start_ms - _shift_stopwatch_start;
                _ds_baseline_decel_rate = (pre > 0)
                    ? (_output_rpm_at_shift_start - _output_rpm_at_catch_start) / (float)pre : 0.0f;
                _sync_stable_since_ms = 0;
                _target_ratio_tracking = false;
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
                if (ratioFeedbackLive() && (actual - predicted) > DS_BIND_EXTRA_RPM) {
                    if (_bind_over_ms < 60000) _bind_over_ms++;
                } else {
                    _bind_over_ms = 0;
                }
                if (_bind_over_ms >= RATIO_EVENT_CONFIRM_MS) telemetry.bind_detected = true;
            }
            {
                if (targetRatioConfirmed(_pd_type == PD_TIMED ? 60UL : 100UL, new_sample)) {
                    finishShift();
                    break;
                }
            }
            if (t >= phaseBackstopMs()) {             // backstop — same verify rule as INERTIA
                if (stationarySequenceAllowed()) finishShift();
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
