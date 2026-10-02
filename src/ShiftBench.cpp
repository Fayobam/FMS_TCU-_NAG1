// Explicit bench controls. Never enabled or restored automatically.
// All methods run on the physics task; see docs/CONTROL_ARCHITECTURE.md.
#include "ShiftScheduler.h"
#include "EngineProfile.h"
#include "DtcManager.h"
#include "AutoShiftMap.h"
#include "TuneOverlay.h"

void ShiftScheduler::applyTestIo(uint8_t id, int16_t v) {
    if (id < 1 || id > 8) return;
    uint8_t bit = (uint8_t)(1u << (id - 1));
    bool on = (v >= 0);
    if (on) _test_out_mask = (uint8_t)(_test_out_mask | bit);
    else    _test_out_mask = (uint8_t)(_test_out_mask & ~bit);
    if (id == 4) _test_mpc_pct = (uint8_t)constrain(v, 0, 100);
    if (id == 5) _test_spc_pct = (uint8_t)constrain(v, 0, 100);
    if (id == 6) _test_tcc_pct = (uint8_t)constrain(v, 0, 100);
    if (id == 1) { if (on) _solenoids->fireShiftSolenoid(PIN_Y3); else _solenoids->stopShiftSolenoid(PIN_Y3); }
    if (id == 2) { if (on) _solenoids->fireShiftSolenoid(PIN_Y5); else _solenoids->stopShiftSolenoid(PIN_Y5); }
    if (id == 3) { if (on) _solenoids->fireShiftSolenoid(PIN_Y4); else _solenoids->stopShiftSolenoid(PIN_Y4); }
    if (id == 7) _solenoids->setShiftLock(on);
    if (id == 8) _solenoids->setTorqueCut(on);
    telemetry.test_out_mask = _test_out_mask;
}

void ShiftScheduler::releaseAllTestIo() {
    if (_test_out_mask & 0x01) _solenoids->stopShiftSolenoid(PIN_Y3);
    if (_test_out_mask & 0x02) _solenoids->stopShiftSolenoid(PIN_Y5);
    if (_test_out_mask & 0x04) _solenoids->stopShiftSolenoid(PIN_Y4);
    if (_test_out_mask & 0x40) _solenoids->setShiftLock(false);
    if (_test_out_mask & 0x80) _solenoids->setTorqueCut(false);
    if (_test_out_mask & 0x10) _solenoids->setShiftPressure(100);
    if (_test_out_mask & 0x20) _solenoids->setTCC(0);
    _test_out_mask = 0;
    telemetry.test_out_mask = 0;
}

void ShiftScheduler::applyHeldTestOutputs() {
    if (_test_out_mask & 0x08) _solenoids->setLinePressure(_test_mpc_pct);
    if (_test_out_mask & 0x10) _solenoids->setShiftPressure(_test_spc_pct);
    if (_test_out_mask & 0x20) _solenoids->setTCC(_test_tcc_pct);
    if (_test_out_mask & 0x40) _solenoids->setShiftLock(true);
    if (_test_out_mask & 0x80) _solenoids->setTorqueCut(true);
}

void ShiftScheduler::consumeAdaptNudge() {
    int8_t dir = telemetry.adapt_nudge_cmd;
    telemetry.adapt_nudge_cmd = 0;
    if (!_last_adapt_valid || dir == 0) return;
    _adaptives->nudge(_last_sclass, _last_shift_idx, _last_tbin, dir);
    _adaptives->requestFlush();
    setSafetyEvent(dir > 0 ? "FEEL + firmer (last shift)" : "FEEL + softer (last shift)");
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

// ============================================================================
// BENCH TEST MODE
// A TCU on a bench has no gearbox, no engine and no road-speed signal, so every
// sensor reads zero. Normal driving logic then blocks or fights the operator: the
// PRND plate decodes nothing, so isForwardRange() is false and paddles are ignored;
// and once a range IS forced, the automatic layers see 0 km/h and continuously try
// to downshift and drop to the launch gear.
//
// Test mode substitutes the dashboard for the selector and the paddles, and suspends
// the automatic layers plus slip-limp. It deliberately does NOT touch the hydraulics:
// routing solenoids, SPC/MPC ramps and the phase engine run exactly as they do on a
// car, which is the entire point of a bench test.
//
// Safety, in layers:
//   - the dashboard asks before enabling (serious warning),
//   - stays on until the operator turns it off (circuit checks use a signal gen),
//   - never persisted, so a power cycle always returns to normal driving,
//   - on exit the gear label is marked unverified, because what was commanded on a
//     bench says nothing about the gear a real gearbox is now in.
// ============================================================================
void ShiftScheduler::enterTestMode() {
    if (telemetry.test_mode) return;
    telemetry.test_mode = true;
    dtcManager.trip(DTC_TEST_MODE);          // provenance: this unit was bench-driven
    setSafetyEvent("TEST MODE ON - auto layers off, raw IO enabled");
    Serial.println("TEST MODE ON - auto layers off, raw IO enabled");
}

void ShiftScheduler::exitTestMode(const char* why) {
    if (!telemetry.test_mode) return;
    telemetry.test_mode = false;
    telemetry.paddle_up_request = false;     // drop anything the dashboard queued
    telemetry.paddle_down_request = false;
    releaseAllTestIo();
    // The gear label after bench shifting is arbitrary. Force the same re-verify an
    // abandoned shift uses: F1 blocks all dispatch until the live ratio identifies the
    // gear, so the first real shift can never route off a bench-era label.
    _gear_resync_pending = true;
    _resync_ready_ms     = millis() + GEAR_UNVERIFIED_SETTLE_MS;
    setSafetyEvent(why);
    Serial.println(why);
}
