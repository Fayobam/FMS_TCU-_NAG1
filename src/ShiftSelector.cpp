// Optional ATF-only range adapter. Runs exclusively on the physics task.
// It never asserts Reverse or P/N. Open contact and open wire are indistinguishable.
#include "ShiftScheduler.h"
#include <cmath>
#include "EngineProfile.h"

namespace {
uint8_t observedForwardGear() {
    const auto& t = telemetry;
    if (!std::isfinite(t.n2_rpm) || !std::isfinite(t.n3_rpm)
        || !std::isfinite(t.turbine_rpm) || !std::isfinite(t.output_rpm)
        || t.n2_rpm < 100 || t.output_rpm < 200 || t.turbine_rpm < 200) return 0;
    const float ratio = t.turbine_rpm / t.output_rpm;
    uint8_t found = 0;
    for (uint8_t g=1; g<=5; ++g) {
        if (fabsf(ratio-g_trans.ratio[g-1]) > 0.05f) continue;
        // Forward 2/3/4 lock the front planetary; 1/5 have stationary N3.
        const bool signature = (g==1 || g==5)
            ? t.n3_rpm < 60
            : t.n3_signal_recent && fabsf(t.n2_rpm-t.n3_rpm) <= 100;
        if (!signature || found) return 0;
        found = g;
    }
    return found;
}
}

bool ShiftScheduler::updateAtfSelector() {
    const uint32_t now = millis();
    if (!telemetry.atf_only_selector) {
        if (_atfWasEnabled) {
            _atfWasEnabled = false;
            _atfCandidate = 0;
            _gear_resync_pending = false;
            _prev_pn_raw = true;
            _prev_prnd = '?';
            _legit_reverse = false;
        }
        return true;
    }
    if (!_atfWasEnabled) {
        _atfWasEnabled = true;
        _atfCandidate = 0;
        _atfSpeedSeq = telemetry.speed_sample_seq;
        _atfSpeedAt = now;
        telemetry.atf_forward_confirmed = false;
    }
    const bool newSample = _atfSpeedSeq != telemetry.speed_sample_seq;
    if (newSample) {
        if (now-_atfSpeedAt > 20) {
            _atfCandidate = 0;
            telemetry.atf_forward_confirmed = false;
        }
        _atfSpeedSeq = telemetry.speed_sample_seq;
        _atfSpeedAt = now;
    }
    if (telemetry.is_limp_mode && telemetry.limp_reset_request
        && telemetry.engine_rpm < 100 && telemetry.output_rpm < 50
        && telemetry.turbine_rpm < 50 && telemetry.atf_range_evidence == 1
        && now-telemetry.atf_sample_ms <= 20) {
        telemetry.is_limp_mode = false;
        telemetry.limp_reset_request = false;
        telemetry.is_slipping = false;
        setLimpReason("");
    }
    const bool live = telemetry.atf_range_evidence == 2 && telemetry.atf_sampled
        && now-telemetry.atf_sample_ms <= 20 && now-_atfSpeedAt <= 20
        && telemetry.speed_hw_ok && telemetry.n2_signal_recent
        && telemetry.output_signal_recent && telemetry.output_rpm >= 200
        && telemetry.n2_rpm >= 100 && std::isfinite(telemetry.turbine_rpm)
        && std::isfinite(telemetry.n2_rpm) && std::isfinite(telemetry.n3_rpm)
        && std::isfinite(telemetry.output_rpm);
    const bool shifting = _current_phase != PHASE_CRUISING;
    const uint8_t observed = live && !shifting ? observedForwardGear() : 0;
    bool authorized = live && telemetry.atf_forward_confirmed && !_gear_resync_pending
        && (shifting || observed == telemetry.current_gear);
    if (!authorized && live && !shifting && observed && !telemetry.is_limp_mode) {
        if (_atfCandidate != observed) {
            _atfCandidate = observed;
            _atfCandidateSince = now;
        } else if (newSample && now-_atfCandidateSince >= 300) {
            telemetry.current_gear = telemetry.target_gear = observed;
            telemetry.input_speed_trusted = true;
            _n2n3_fault_since_ms = 0;
            _gear_resync_pending = false;
            authorized = true;
        }
    } else if (!authorized) _atfCandidate = 0;

    if (authorized && !telemetry.is_limp_mode) {
        _atfCandidate = 0;
        telemetry.atf_forward_confirmed = true;
        telemetry.drive_engaged = true; // bypass the legacy assumed-second engagement
        telemetry.prnd_state = 'D';
        _prev_pn_raw = false;
        _engage_grace_until_ms = 0;
        // Requests never queue through an active shift; opposing edges cancel.
        if (shifting || (telemetry.paddle_up_request && telemetry.paddle_down_request))
            telemetry.paddle_up_request = telemetry.paddle_down_request = false;
        return true; // Existing phases, pressure maps, downshift guard and TCC.
    }

    // Unknown direction/gear: leave routing and pressure solenoids de-energized.
    // Do not label this as gear 2: a previously latched gear may remain engaged.
    telemetry.atf_forward_confirmed = telemetry.drive_engaged = false;
    telemetry.prnd_state = '?';
    telemetry.current_gear = telemetry.target_gear = 0;
    telemetry.paddle_up_request = telemetry.paddle_down_request = false;
    telemetry.torque_cut_active = false;
    telemetry.shift_phase = 0;
    // update() returns early from here, so anything it would normally derive stops
    // refreshing. Keep the dashboard-visible values live: a frozen load_pct reading
    // 85 % while the valve body sits de-energized reads as a live measurement.
    telemetry.road_kmh   = telemetry.output_rpm * engineProfile.kmhPerOutRpm();
    telemetry.t_est_nm   = engineProfile.estimateTorque(telemetry.engine_rpm, telemetry.map_kpa);
    telemetry.load_pct   = engineProfile.loadPct(telemetry.engine_rpm, telemetry.map_kpa);
    telemetry.t_input_nm = engineProfile.inputTorque(telemetry.engine_rpm, telemetry.turbine_rpm,
                                                    telemetry.map_kpa);
    _current_phase = PHASE_CRUISING;
    _gear_resync_pending = false;
    _last_adapt_valid = false;
    _solenoids->setGarageY4(false);
    _solenoids->stopAllShiftSolenoids();
    _solenoids->setLinePressure(100);
    setSPC(100);
    _solenoids->setTCC(0);
    _solenoids->setTorqueCut(false);
    _solenoids->setShiftLock(false);
    if (shiftTrace.capturing) { shiftTrace.capturing=false; shiftTrace.ready=true; }
    return false;
}
