// Independent shift protections, reverse handling and load anticipation.
// All methods run on the physics task; see docs/CONTROL_ARCHITECTURE.md.
#include "ShiftScheduler.h"
#include "EngineProfile.h"
#include "DtcManager.h"
#include "AutoShiftMap.h"
#include "TuneOverlay.h"

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
    if (!telemetry.test_mode && currentMode().lug_guard &&
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
// LIMP MODE  (load-aware threshold + deliberate reset path)
// ----------------------------------------------------------------------------
void ShiftScheduler::checkLimpMode(float target_ratio) {
    if (telemetry.test_mode) { telemetry.is_slipping = false; return; }
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
        telemetry.high_torque_mode = true;
        _ht_release_start_ms = 0;   // cancel any in-progress cooldown
        return;
    }

    if (!telemetry.high_torque_mode) return;

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
