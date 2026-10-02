#include "ControlBridge.h"
#include "DtcManager.h"
#include "TelemetryConfig.h"

ControlBridge controlBridge;

bool ControlBridge::submit(const ControlCommand& cmd) {
    if (state.load(std::memory_order_acquire) != 0) return false;
    command = cmd;
    state.store(1, std::memory_order_release);
    return true;
}
bool ControlBridge::complete(bool& ok, ControlAction& action) {
    if (state.load(std::memory_order_acquire) != 2) return false;
    ok = accepted;
    action = command.action;
    state.store(0, std::memory_order_release);
    return true;
}
void ControlBridge::consume(AdaptiveMemory& adaptives) {
    if (state.load(std::memory_order_acquire) != 1) return;
    const auto& c = command;
    const bool stopped = telemetry.output_rpm < 50 && telemetry.engine_rpm < 100
        && telemetry.turbine_rpm < 50;
    const bool atfOpen = telemetry.atf_only_selector && telemetry.atf_sampled
        && telemetry.atf_range_evidence == 1 && millis()-telemetry.atf_sample_ms <= 20;
    const bool parked = stopped && (telemetry.prnd_state == 'P' || telemetry.prnd_state == 'N' || atfOpen)
        && telemetry.shift_phase == 0 && !telemetry.test_mode;
    accepted = true;
    switch (c.action) {
    case ControlAction::AtfSelector:
        // No TRRS requirement: mode must be selectable with that harness absent.
        // The operator must secure the vehicle; speed-sensor silence is not proof of rest.
        if (!stopped || telemetry.shift_phase != 0 || telemetry.test_mode
            || !telemetry.atf_sampled || millis() - telemetry.atf_sample_ms > 20
            || telemetry.atf_range_evidence != 1) { accepted = false; break; }
        telemetry.atf_only_selector = c.value != 0;
        telemetry.atf_forward_confirmed = false;
        telemetry.drive_engaged = false;
        telemetry.prnd_state = '?';
        telemetry.paddle_up_request = telemetry.paddle_down_request = false;
        break;
    case ControlAction::Profile:
        if (!parked) { accepted = false; break; }
        *engineProfile.raw() = c.profile;
        g_trans = TRANS_SPECS[c.profile.trans_variant];
        break;
    case ControlAction::Tune:
        if (!parked) { accepted = false; break; }
        *tuneOverlay.raw() = c.tune;
        break;
    case ControlAction::Cells:
        if (!parked) { accepted = false; break; }
        memcpy(adaptives.cellsPtr(), c.cells, sizeof(c.cells));
        break;
    case ControlAction::TestMode:
        if (c.value && (!parked || telemetry.atf_only_selector)) { accepted = false; break; }
        telemetry.test_mode_cmd = c.value ? 1 : -1;
        break;
    case ControlAction::Selector:
        if (!telemetry.test_mode || !stopped) { accepted = false; break; }
        telemetry.prnd_state = (char)c.value;
        break;
    case ControlAction::Paddle:
        if (!telemetry.test_mode) { accepted = false; break; }
        if (c.value > 0) telemetry.paddle_up_request = true;
        else telemetry.paddle_down_request = true;
        break;
    case ControlAction::Output:
        if (!telemetry.test_mode || (c.extra >= 0 && !stopped)) { accepted = false; break; }
        telemetry.test_sol_req_v = c.extra;
        telemetry.test_sol_req = c.value;
        break;
    case ControlAction::Nudge: telemetry.adapt_nudge_cmd = c.value; break;
    case ControlAction::LimpReset:
        if (!stopped || (telemetry.prnd_state != 'P' && telemetry.prnd_state != 'N' && !atfOpen)) {
            accepted = false; break;
        }
        telemetry.limp_reset_request = true;
        break;
    case ControlAction::ClearDtcs: dtcManager.clearAll(); break;
    }
    state.store(2, std::memory_order_release);
}
void ControlBridge::publish(AdaptiveMemory& adaptives) {
    uint32_t now = millis();
    if (now - lastSample < TCU_TELEMETRY_INTERVAL_MS) return;
    lastSample = now;
    // Only bounded POD copies under this lock; readers never serialize while locked.
    portENTER_CRITICAL(&snapshotMux);
    snapshot.data = telemetry;
    memcpy(snapshot.cells, adaptives.cellsPtr(), sizeof(snapshot.cells));
    snapshot.sampledMs = now;
    snapshot.expectedRatio = g_trans.ratio[telemetry.current_gear >= 1 && telemetry.current_gear <= 5
        ? telemetry.current_gear - 1 : 1];
    snapshot.targetRatio = g_trans.ratio[telemetry.target_gear >= 1 && telemetry.target_gear <= 5
        ? telemetry.target_gear - 1 : 1];
    portEXIT_CRITICAL(&snapshotMux);
}
ControlSnapshot ControlBridge::read() {
    ControlSnapshot out;
    portENTER_CRITICAL(&snapshotMux);
    out = snapshot;
    portEXIT_CRITICAL(&snapshotMux);
    return out;
}
