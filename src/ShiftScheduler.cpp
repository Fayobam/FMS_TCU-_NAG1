// Control-loop orchestration, gear requests, speed observers and TCC.
// All methods run on the physics task; see docs/CONTROL_ARCHITECTURE.md.
#include "ShiftScheduler.h"
#include "EngineProfile.h"
#include "DtcManager.h"
#include "AutoShiftMap.h"
#include "TuneOverlay.h"

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
    _atfCandidate = 0;
    _atfWasEnabled = false;
    _atfSpeedAt = _atfSpeedSeq = 0;
    _have_ratio_sample = false;
    _target_ratio_tracking = false;
    _current_phase = PHASE_CRUISING;
    telemetry.current_gear = 2;   // 722.6 hydraulic default (was 1 — cosmetic mismatch)
    telemetry.target_gear = 2;
    _prev_pn_raw = true;
    for (int i = 0; i < TPS_ROC_WINDOW_MS; i++) _tps_hist[i] = 0.0f;
    _tps_hist_idx = 0;
    _tps_hist_primed = false;
    telemetry.high_torque_mode = false;
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
    _test_out_mask = 0;
    telemetry.test_out_mask = 0;
    _last_adapt_valid = false;
}

bool ShiftScheduler::isForwardRange() {
    char s = telemetry.prnd_state;
    return (s == 'D' || s == '4' || s == '3' || s == '2' || s == '1');
}

// Lever-selected drive mode (D/4/3/2/1 → COMFORT AUTO … RACE MANUAL). Read live each
// use so moving the lever re-tunes shift behaviour on the fly.
// Returned BY VALUE: every knob now comes from the web-editable TuneOverlay, so there is
// no static object to hand back a reference to. DRIVE_MODES supplies the name (and the
// seed values); the overlay supplies the live ones.
DriveMode ShiftScheduler::currentMode() const {
    uint8_t i = driveModeIndex(telemetry.prnd_state);
    DriveMode m      = DRIVE_MODES[i];
    m.auto_shift     = !telemetry.atf_only_selector && tuneOverlay.modeAutoShift(i);
    m.shift_pt_scale = tuneOverlay.modeShiftPt(i);
    m.firmness       = tuneOverlay.modeFirmness(i);
    m.tcc_open_tps   = tuneOverlay.modeTccOpenTps(i);
    m.lug_guard      = tuneOverlay.modeLugGuard(i);
    m.torque_cut     = tuneOverlay.modeTorqueCut(i);
    m.launch_gear    = tuneOverlay.modeLaunchGear(i);
    return m;
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
        bool force_open = !ratioFeedbackLive() || !telemetry.engine_signal_recent ||
                          !telemetry.tps_valid || !telemetry.map_valid ||
                          telemetry.high_torque_mode ||
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

// Predict the post-downshift turbine speed TWO independent ways and return the
// HIGHER (fail-safe): from the output sensor (output × target ratio) AND from the
// turbine sensor, which is output-INDEPENDENT (turbine × ratio_target/ratio_current,
// since turbine = output × ratio_current). So a DEAD output sensor reading 0 cannot
// silently defeat the guard — the N2/N3-derived estimate still catches an over-rev
// downshift. (Both sensors dead = truly blind; nothing can help then.)
//
// beginShift() remains the authority that refuses the shift. This is exposed so the
// automatic layers can decline to REQUEST a downshift they can already see will be
// refused, rather than re-asking every tick. One implementation, two callers.
float ShiftScheduler::predictedDownshiftRpm(uint8_t target_gear) {
    float ratio_t = getTargetRatio(target_gear);
    float ratio_c = getTargetRatio(telemetry.current_gear);
    float pred_out  = telemetry.output_rpm * ratio_t;
    float pred_turb = (ratio_c > 0.01f) ? telemetry.turbine_rpm * (ratio_t / ratio_c) : 0.0f;
    return fmaxf(pred_out, pred_turb);
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
    if (!is_upshift) {
        float predicted = predictedDownshiftRpm(target_gear);
        if (predicted > RPM_MAX_SAFE_DOWNSHIFT) {
            // Rate-limited. The automatic layers pre-screen with
            // predictedDownshiftRpm() so they never arrive here in a loop, but a held
            // paddle or a future caller must not be able to pace this 1 kHz task at
            // the UART's rate: Serial.print blocks once the 256-byte TX ring fills.
            if (millis() - _last_block_log_ms > 1000) {
                _last_block_log_ms = millis();
                Serial.print("DOWNSHIFT BLOCKED ("); Serial.print(source);
                Serial.print(") predicted RPM "); Serial.println(predicted);
            }
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
    _clutch_move_over_ms = 0;
    _fill_move_ms = 0;
    _target_ratio_tracking = false;
    _shift_started_observable = telemetry.output_rpm >= RATIO_OBSERVABLE_MIN_OUTPUT_RPM ||
        telemetry.turbine_rpm >= RATIO_OBSERVABLE_MIN_OUTPUT_RPM;
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
// AUTO SHIFT SCHEDULE (full automatic up/down). Runs only in auto drive modes
// (D/4/3). Interpolates the AUTO_SHIFT_MAP km/h thresholds for the current gear by
// TPS, stretches them by the mode's shift_pt_scale (sport holds gears longer), and
// shifts when road km/h crosses. Money-shift (beginShift) + overrev/lug
// (checkSafetyShifts) guards stay on top; AUTO_SHIFT_COOLDOWN_MS + the map's built-in
// up>down hysteresis prevent hunting. A pending paddle request wins this tick.
// The map's closed-throttle column doubles as the coast-down schedule.
// ----------------------------------------------------------------------------
void ShiftScheduler::checkAutoShift() {
    if (telemetry.test_mode) return;          // bench: dashboard paddles only
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
        float up_kmh = tuneOverlay.upshiftKmh(g - 1, tps) * scale;
        if (kmh > up_kmh && beginShift(g + 1, true, "AUTO")) {
            telemetry.last_auto_shift_ms = millis();
            return;
        }
    }
    // Downshift g→g-1 below the (scaled) threshold. Floor at 2nd (1st = paddle-only).
    if (g >= 3 && g <= 5) {
        float dn_kmh = tuneOverlay.downshiftKmh(g - 2, tps) * scale;
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
    if (telemetry.test_mode) return;          // bench: dashboard paddles only
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
    if (telemetry.test_mode) return;          // bench: dashboard paddles only
    if (_current_phase != PHASE_CRUISING) return;
    if (!isForwardRange()) return;
    if (!currentMode().auto_shift) return;   // manual modes: the driver paddles for power
    if (millis() - telemetry.last_auto_shift_ms < AUTO_SHIFT_COOLDOWN_MS) return;
    if (telemetry.tps_pct < KICKDOWN_TPS_PCT) return;
    if (telemetry.engine_rpm > KICKDOWN_MAX_ENG_RPM) return;  // already high → don't overrev

    uint8_t g = telemetry.current_gear;
    if (g <= 1) return;
    // Do not REQUEST a downshift the money-shift guard will refuse. The kickdown
    // gate is throttle-and-engine-rpm (TPS > 70 %, engine <= 5200), but the guard is
    // on PREDICTED turbine in the lower gear, and the two do not coincide: at WOT in
    // 4th above ~4040 output-equivalent rpm, 3rd would spin the turbine past 6000. A
    // refusal does not arm last_auto_shift_ms (only a successful shift does), so
    // without this pre-screen the request repeated every tick for as long as the
    // throttle stayed down. Checking here rather than arming the shared cooldown on
    // refusal keeps OVERREV protection immediate — that cooldown gates it too.
    if (predictedDownshiftRpm(g - 1) > RPM_MAX_SAFE_DOWNSHIFT) return;
    // beginShift still applies the same guard to every request; this only avoids
    // asking. It remains the sole authority on whether a shift may start.
    if (beginShift(g - 1, false, "KICKDOWN")) {
        telemetry.last_auto_shift_ms = millis();
    }
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
    if (!updateAtfSelector()) return;
    // ---- Reverse/Park interlock + R-while-moving failsafe (HIGHEST PRIORITY) ----
    // Runs above even limp mode: if we're rolling forward and R is selected we must
    // dump line pressure, never let the limp handler clamp B3 at max pressure. Also
    // maintains the RP_LOCK solenoid every loop regardless of any other state.
    if (!telemetry.test_mode && checkReverseInhibit()) return;

    // ---- Bench test mode: consume the dashboard's request, then enforce the exit ----
    // Placed above every driving layer so the fail-safe exit cannot be starved by a
    // shift in progress or by limp-mode's early return below.
    if (telemetry.test_mode_cmd != 0) {
        int8_t cmd = telemetry.test_mode_cmd;
        telemetry.test_mode_cmd = 0;
        if (cmd > 0) enterTestMode();
        else         exitTestMode("TEST MODE OFF");
    }
    // Fail-safe: any genuine road speed ends bench mode at once, whatever the dashboard
    // last said. This is the guard that matters if the toggle is left on in a car.
    if (telemetry.adapt_nudge_cmd != 0) consumeAdaptNudge();

    if (telemetry.test_sol_req != 0) {
        uint8_t id = telemetry.test_sol_req;
        int16_t v  = telemetry.test_sol_req_v;
        telemetry.test_sol_req = 0;
        if (telemetry.test_mode) applyTestIo(id, v);
    }

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

    if (!(_test_out_mask & 0x08)) calculateLinePressure();
    checkLimpMode(target_ratio);
    if (!telemetry.atf_only_selector) {
        checkSafetyShifts();
        checkKickdown();
        checkAutoShift();
        checkLaunchGear();
    } // ATF-only: requests are manual; beginShift still rejects unsafe downshifts.

    // 20ms pressure-update quantizer (ATSG p.80). Sensors + exit checks still run at 1kHz.
    bool ptick = (millis() - _last_pressure_update_ms >= PRESSURE_TICK_MS);
    if (ptick) _last_pressure_update_ms = millis();

    // A genuinely-new 200 Hz speed sample landed this tick? Ratio-derivative predicates
    // (sprag flat) only advance on new samples; between samples the ratio is frozen (B-4).
    bool new_sample = (telemetry.speed_sample_seq != _last_speed_seq);
    if (new_sample) {
        _last_speed_seq = telemetry.speed_sample_seq;
        _last_ratio_sample_ms = millis();
        _have_ratio_sample = true;
    }

    if (_current_phase == PHASE_CRUISING) {
        if (_test_out_mask == 0 && !telemetry.atf_only_selector) updateStandbyAndGarage();   // don't fight latched raw IO

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
        if (!telemetry.atf_only_selector && _gear_resync_pending && millis() >= _resync_ready_ms) {
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

    if (!(_test_out_mask & 0x20)) updateTCC(ptick);
    if (_test_out_mask) applyHeldTestOutputs();
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
