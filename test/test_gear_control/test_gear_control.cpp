// ============================================================================
// FILE: test/test_gear_control/test_gear_control.cpp
// Host tests for the ONE property everything else depends on:
//   the gear the firmware BELIEVES it is in must match the gear the gearbox is
//   ACTUALLY in — because the routing solenoid is chosen from the believed gear.
//
// A wrong gear label is not a cosmetic bug: it fires the wrong routing solenoid
// on the next shift, which can command two clutch packs at once (cross-apply /
// tie-up). That is the R1 hazard the F1 guard exists to prevent.
//
// Run: pio test -e native
// ============================================================================
#include <unity.h>

#include "TCU_Data.h"
#include "SolenoidDriver.h"
#include "AdaptiveMemory.h"
#include "EngineProfile.h"
#include "DtcManager.h"
#include "ShiftScheduler.h"
#include "TuneOverlay.h"
#include <string.h>
#include "InputManager.h"

// Globals normally defined in main.cpp (which the native env excludes).
TCU_Telemetry telemetry;
ShiftTrace    shiftTrace;
EngineProfile engineProfile;

static SolenoidDriver sol(PIN_MPC, PIN_SPC, PIN_TCC, PIN_Y3, PIN_Y4, PIN_Y5, PIN_RP_LOCK);
static AdaptiveMemory adaptives;
static ShiftScheduler sched(&sol, &adaptives);

static const uint16_t KICK_DUTY = 204;   // ~80% snap-open kick (SolenoidDriver)

// ---------------------------------------------------------------------------
// Harness
// ---------------------------------------------------------------------------
static float ratioOf(uint8_t gear) { return g_trans.ratio[gear - 1]; }

// Boot the stack the way main.cpp does.
static void bootStack() {
    g_now_ms = 1000;
    hwResetPins();
    engineProfile.begin();
    adaptives.begin();
    dtcManager.begin();
    sol.begin();
    sched.begin();
}

// Steady state: engaged, in `gear`, rolling at `out_rpm`, part throttle, warm.
// Lever '2' = SPORT MANUAL (auto_shift off) so the automatic schedule cannot
// inject shifts the test did not ask for.
static void setupDriving(uint8_t gear, float out_rpm, char lever = '2') {
    bootStack();
    telemetry.prnd_state    = lever;
    telemetry.drive_engaged = true;          // already engaged: no re-latch, no resync
    telemetry.current_gear  = gear;
    telemetry.target_gear   = gear;
    telemetry.output_rpm    = out_rpm;
    telemetry.turbine_rpm   = out_rpm * ratioOf(gear);
    telemetry.n2_rpm        = telemetry.turbine_rpm;   // N2==N3 keeps input_speed_trusted
    telemetry.n3_rpm        = telemetry.turbine_rpm;
    telemetry.engine_rpm    = 3000.0f;
    telemetry.tps_pct       = 30.0f;
    telemetry.map_kpa       = 100.0f;
    telemetry.atf_temp_c    = 80.0f;
    telemetry.is_limp_mode  = false;
    telemetry.is_slipping   = false;
    telemetry.input_speed_trusted = true;
    telemetry.n2_signal_recent = telemetry.n3_signal_recent = true;
    telemetry.output_signal_recent = telemetry.engine_signal_recent = true;
    telemetry.paddle_up_request   = false;
    telemetry.paddle_down_request = false;
    telemetry.test_mode = false;
    telemetry.test_mode_cmd = 0;
    telemetry.test_sol_req = 0;
    telemetry.last_auto_shift_ms  = 0;
    telemetry.reverse_abuse_active = false;
    telemetry.flare_detected = false;
    telemetry.bind_detected  = false;
    telemetry.speed_sample_seq = 0;
    // Settled in gear, NOT mid lever-movement: without this the first tick sees a
    // P/N falling edge, opens the 1.5 s engagement window and pulses the Y4 garage
    // counter-pressure — real behaviour just after selecting D, but noise for tests
    // that are about routing.
    sched._prev_pn_raw = false;
    hwResetPins();                            // ignore boot-time writes
}

// One 1 kHz physics iteration, in main.cpp's order. Speeds are held FROZEN, so
// the ratio never reaches the target — this is the "shift that never takes" case.
static void tick(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        g_now_ms++;
        sched.update();
        sol.update();
    }
}

// Same, but plays a well-behaved gearbox: once the shift reaches its speed-change
// phase, the ratio arrives at the target gear (what a healthy clutch would do).
static void tickSyncing(uint8_t to_gear, uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        if (sched._current_phase == PHASE_INERTIA || sched._current_phase == PHASE_CATCH) {
            telemetry.turbine_rpm = telemetry.output_rpm * ratioOf(to_gear);
            telemetry.n2_rpm = telemetry.turbine_rpm;
            telemetry.n3_rpm = telemetry.turbine_rpm;
            telemetry.speed_sample_seq++;
        }
        g_now_ms++;
        sched.update();
        sol.update();
    }
}

// Sweep the throttle from `a` to `b` over `ms` ticks. Kickdown now triggers on rate
// of change, so a test that just assigns tps_pct is either an instant stab (a step
// from 30 to 95 is ~3.25 %/ms of ROC) or, held constant, no event at all. Sweep rate
// is what decides which case is under test.
static void tickThrottle(float a, float b, uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        telemetry.tps_pct = a + (b - a) * ((float)(i + 1) / (float)ms);
        g_now_ms++;
        sched.update();
        sol.update();
    }
}

// Run a shift to completion and stop the instant it returns to CRUISING.
// Deliberately NOT "tick for 2 seconds": slip-limp arms 400 ms after a bad shift
// ends, de-energises everything and re-derives the gear from ratio — which would
// mask the very bug under test behind a blunt rescue. Returns ms elapsed.
static uint32_t tickUntilShiftEnds(uint32_t max_ms) {
    uint32_t n = 0;
    while (n < max_ms && sched._current_phase == PHASE_CRUISING) { tick(1); n++; }
    while (n < max_ms && sched._current_phase != PHASE_CRUISING) { tick(1); n++; }
    return n;
}

void setUp(void) {}
void tearDown(void) {}

// ===========================================================================
// 1. DISPATCH — the routing table against the documented 722.6 hydraulics
//    (Y3 = 1-2 & 4-5, Y5 = 2-3, Y4 = 3-4), both directions.
// ===========================================================================
void test_routing_table_matches_722_6_hydraulics(void) {
    bootStack();
    TEST_ASSERT_EQUAL_UINT8(PIN_Y3, sched.getRoutingSolenoidForShift(1, 2));
    TEST_ASSERT_EQUAL_UINT8(PIN_Y5, sched.getRoutingSolenoidForShift(2, 3));
    TEST_ASSERT_EQUAL_UINT8(PIN_Y4, sched.getRoutingSolenoidForShift(3, 4));
    TEST_ASSERT_EQUAL_UINT8(PIN_Y3, sched.getRoutingSolenoidForShift(4, 5));
    TEST_ASSERT_EQUAL_UINT8(PIN_Y3, sched.getRoutingSolenoidForShift(5, 4));
    TEST_ASSERT_EQUAL_UINT8(PIN_Y4, sched.getRoutingSolenoidForShift(4, 3));
    TEST_ASSERT_EQUAL_UINT8(PIN_Y5, sched.getRoutingSolenoidForShift(3, 2));
    TEST_ASSERT_EQUAL_UINT8(PIN_Y3, sched.getRoutingSolenoidForShift(2, 1));
}

// A skip-shift has NO routing solenoid. If one is ever requested it must be
// refused outright — running the phases with no solenoid energised would end in
// finishShift() asserting a gear the gearbox never entered.
void test_skip_shift_has_no_routing_solenoid(void) {
    bootStack();
    TEST_ASSERT_EQUAL_UINT8(0, sched.getRoutingSolenoidForShift(1, 3));
    TEST_ASSERT_EQUAL_UINT8(0, sched.getRoutingSolenoidForShift(5, 2));
    TEST_ASSERT_EQUAL_UINT8(0, sched.getRoutingSolenoidForShift(3, 3));
}

// Behavioural: a paddle request in each gear must actually kick the documented coil.
void test_every_legal_shift_kicks_its_documented_solenoid(void) {
    struct Case { uint8_t from, to, pin; bool up; };
    const Case cases[] = {
        {1, 2, PIN_Y3, true}, {2, 3, PIN_Y5, true}, {3, 4, PIN_Y4, true}, {4, 5, PIN_Y3, true},
        {5, 4, PIN_Y3, false}, {4, 3, PIN_Y4, false}, {3, 2, PIN_Y5, false}, {2, 1, PIN_Y3, false},
    };
    for (const Case& c : cases) {
        setupDriving(c.from, 500.0f);
        if (c.up) telemetry.paddle_up_request = true;
        else      telemetry.paddle_down_request = true;
        tick(2);
        char msg[64];
        snprintf(msg, sizeof(msg), "shift %u->%u did not kick its solenoid", c.from, c.to);
        TEST_ASSERT_EQUAL_UINT16_MESSAGE(KICK_DUTY, g_pwm[c.pin], msg);
        TEST_ASSERT_NOT_EQUAL_MESSAGE(PHASE_CRUISING, sched._current_phase, msg);
    }
}

// ===========================================================================
// 2. GEAR LABEL TRUTH — a shift may only be recorded once the ratio proves it
// ===========================================================================

// Baseline: a healthy shift must still latch normally (guards the fix from
// over-correcting into "never completes a shift").
void test_shift_that_syncs_latches_the_new_gear(void) {
    setupDriving(2, 500.0f);
    telemetry.paddle_up_request = true;
    tickSyncing(3, 2000);
    TEST_ASSERT_EQUAL_UINT8(PHASE_CRUISING, sched._current_phase);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(3, telemetry.current_gear,
        "a shift whose ratio reached the target must latch the new gear");
}

// FINDING 1 (upshift). The clutch never takes — ratio stays at the source gear.
// The INERTIA 600 ms backstop must NOT declare the target gear: the box is still
// in 2nd, and believing "3rd" sends the next shift to the wrong solenoid.
void test_upshift_that_never_syncs_must_not_latch_gear(void) {
    setupDriving(2, 500.0f);
    telemetry.paddle_up_request = true;
    uint32_t ms = tickUntilShiftEnds(3000);      // speeds frozen: ratio never moves
    TEST_ASSERT_LESS_THAN_UINT32_MESSAGE(3000, ms, "shift must terminate, not hang");
    TEST_ASSERT_FALSE_MESSAGE(telemetry.is_limp_mode,
        "limp must not be the thing that catches this");
    TEST_ASSERT_NOT_EQUAL_MESSAGE(3, telemetry.current_gear,
        "FINDING 1: a timed-out upshift was recorded as successful — the gear "
        "label now lies about the gearbox and the next shift routes on it");
}

// FINDING 1 (downshift). Same hole in the CATCH 600 ms backstop — and WORSE here:
// a failed 4-3 leaves turbine at 500 vs an expected 743, a 243 rpm mismatch that
// never reaches limp's 300 rpm threshold. So on close-ratio pairs nothing detects
// the failure at all; the wrong gear label simply persists.
void test_downshift_that_never_syncs_must_not_latch_gear(void) {
    setupDriving(4, 500.0f);
    telemetry.paddle_down_request = true;
    uint32_t ms = tickUntilShiftEnds(3000);
    TEST_ASSERT_LESS_THAN_UINT32_MESSAGE(3000, ms, "shift must terminate, not hang");
    TEST_ASSERT_NOT_EQUAL_MESSAGE(3, telemetry.current_gear,
        "FINDING 1: a timed-out downshift was recorded as successful");
}

// A STOPPED CAR HAS NO OBSERVABLE RATIO. calculateLiveRatio() substitutes the
// BELIEVED gear's ratio below 50 output rpm, so live_ratio never moves and a
// standstill shift can never prove itself. That is absence of evidence, NOT
// evidence of failure — the ratio-verify backstop must not abandon it. If it
// does, the launch 2->1 that precedes EVERY pull-away is abandoned forever, the
// car launches in 2nd on a 3.07 diff, and each attempt trips a DTC.
void test_standstill_downshift_still_latches(void) {
    setupDriving(2, 0.0f);                    // stopped: output and turbine both 0
    telemetry.paddle_down_request = true;
    tickUntilShiftEnds(3000);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(1, telemetry.current_gear,
        "a standstill 2->1 must latch 1st — an unobservable ratio is not a failed shift");
}

// Cold ATF fills slowly. dueATC — a controller that was actually driven — holds its
// shift solenoid up to 2000 ms when cold vs 900 ms hot. Since F12 a backstop hit means
// abandon + DTC + resync, so a flat 600 ms would spuriously abandon normal cold shifts.
// The backstop must therefore stretch when cold and stay tight when hot.
void test_shift_backstop_stretches_when_cold(void) {
    setupDriving(2, 500.0f);
    telemetry.atf_temp_c = 80.0f;               // hot
    telemetry.paddle_up_request = true;
    uint32_t hot_ms = tickUntilShiftEnds(4000);

    setupDriving(2, 500.0f);
    telemetry.atf_temp_c = -10.0f;              // cold
    telemetry.paddle_up_request = true;
    uint32_t cold_ms = tickUntilShiftEnds(4000);

    TEST_ASSERT_LESS_THAN_UINT32_MESSAGE(4000, cold_ms, "cold shift must still terminate");
    TEST_ASSERT_GREATER_THAN_UINT32_MESSAGE(hot_ms + 400, cold_ms,
        "a cold shift must be given materially longer before it is called failed");
}

// Look a parameter up by its wire id rather than hardcoding an index, so reordering
// the registry cannot silently make this test assert against the wrong knob.
static uint8_t paramIdx(const char* id) {
    for (uint8_t i = 0; i < TuneOverlay::paramCount(); i++)
        if (strcmp(TuneOverlay::paramDesc(i).id, id) == 0) return i;
    TEST_FAIL_MESSAGE("parameter id not present in the registry");
    return 0;
}

// The registry has to reach the RUNNING gearbox — a stored number that nothing reads
// is worse than no knob at all, because the dashboard would report a change that never
// happened. Edit the hot backstop through the registry and the same failing shift must
// give up sooner, with no reboot and no re-seed.
void test_registry_edit_changes_live_behaviour(void) {
    const uint8_t idx = paramIdx("backstop.hot");
    const int16_t original = tuneOverlay.paramGet(idx, 0, 0);

    setupDriving(2, 500.0f);
    telemetry.atf_temp_c = 80.0f;                  // hot side of the ATF scale
    telemetry.paddle_up_request = true;
    uint32_t before = tickUntilShiftEnds(4000);

    TEST_ASSERT_TRUE_MESSAGE(tuneOverlay.paramSet(idx, 0, 0, 350),
        "an in-range edit must be accepted");
    TEST_ASSERT_FALSE_MESSAGE(tuneOverlay.paramSet(idx, 0, 0, 5),
        "an out-of-range edit must be rejected at the boundary, not clamped silently");
    TEST_ASSERT_EQUAL_INT16_MESSAGE(350, tuneOverlay.paramGet(idx, 0, 0),
        "the rejected edit must not have disturbed the accepted value");

    setupDriving(2, 500.0f);
    telemetry.atf_temp_c = 80.0f;
    telemetry.paddle_up_request = true;
    uint32_t after = tickUntilShiftEnds(4000);

    TEST_ASSERT_LESS_THAN_UINT32_MESSAGE(before, after,
        "a registry edit must take effect on the live control loop");

    tuneOverlay.paramSet(idx, 0, 0, original);     // leave the rig as we found it
}

// THE HAZARD ITSELF. After an unverified 2-3 the box is still in 2nd. If the label
// says "3rd", the next upshift is dispatched as 3->4 and energises Y4 — the wrong
// clutch pack, on top of the one already applied (cross-apply / tie-up, review R1).
// Correct behaviour: the label is re-derived from the live ratio as 2nd, so the next
// upshift routes 2->3 through Y5.
void test_next_shift_routes_from_the_corrected_gear(void) {
    setupDriving(2, 500.0f);
    telemetry.paddle_up_request = true;
    tickUntilShiftEnds(3000);
    TEST_ASSERT_FALSE_MESSAGE(telemetry.is_limp_mode,
        "precondition: the label must be recovered by resync, not by limp");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(2, telemetry.current_gear,
        "an unverified 2-3 must leave the label at the ratio-derived gear (2nd)");

    hwResetPins();
    telemetry.paddle_up_request = true;
    tick(5);
    TEST_ASSERT_NOT_EQUAL_MESSAGE(KICK_DUTY, g_pwm[PIN_Y4],
        "next shift must NOT route as 3->4 — that is the cross-apply hazard");
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(KICK_DUTY, g_pwm[PIN_Y5],
        "next shift must route as 2->3 from the corrected label");
}

// ===========================================================================
// 3. SOLENOID OWNERSHIP — boot must not stroke the 1-2 valve; garage Y4
//    must still yield to a real 3-4
// ===========================================================================
// OEM pulses Y3 at crank, which can pre-position the 1-2 command valve into
// 1st/R1 before the lever moves. We do not: 1st comes only from the launch
// 2->1 after D is latched.
void test_boot_leaves_y3_off(void) {
    bootStack();
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(0, g_pwm[PIN_Y3],
        "boot must not pulse Y3 — that can latch 1st/R1 before the lever moves");
}

// The already-fixed Y4 case — locked in so it cannot regress.
void test_shift_takes_y4_over_from_the_garage_pulse(void) {
    bootStack();
    sol.setGarageY4(true);                       // garage holds Y4 at ~37%
    hwResetPins();
    sol.fireShiftSolenoid(PIN_Y4);               // a 3-4 shift wants it
    sol.update();
    TEST_ASSERT_EQUAL_UINT16(KICK_DUTY, g_pwm[PIN_Y4]);
}

// ===========================================================================
// 4. MONEY-SHIFT GUARD — must survive a dead output sensor
// ===========================================================================
// The guard predicts post-downshift turbine speed two independent ways and trusts
// the higher, so an output sensor reading 0 cannot silently defeat it.
void test_moneyshift_guard_survives_dead_output_sensor(void) {
    setupDriving(5, 500.0f);
    telemetry.output_rpm  = 0.0f;                // sensor dead
    telemetry.turbine_rpm = 5200.0f;             // 5->4 would predict ~6265 rpm
    telemetry.n2_rpm = telemetry.n3_rpm = 5200.0f;
    hwResetPins();
    telemetry.paddle_down_request = true;
    tick(5);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(5, telemetry.current_gear,
        "a downshift predicted to overrev must be refused even with output_rpm=0");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(PHASE_CRUISING, sched._current_phase,
        "blocked downshift must not start a shift");
}

// A refused kickdown must stay cheap AND must not consume the shared cooldown.
//
// The kickdown gate is throttle + engine rpm (TPS > 70 %, engine <= 5200); the
// money-shift guard is on PREDICTED turbine in the LOWER gear. The two do not
// coincide, so there is a real band where kickdown is evaluated every tick and
// refused every time. In SPORT AUTO (1.20x shift points) 3rd gear at WOT is
// refused from ~95 km/h but does not upshift to 4th until ~121 km/h, so nothing
// relieves it for 26 km/h of wide-open throttle.
//
// The tempting fix — arm last_auto_shift_ms on refusal — would be a safety
// regression: checkSafetyShifts() gates OVERREV on that same cooldown, so a
// refused kickdown would delay engine-overrev protection by up to 500 ms, exactly
// when the throttle is wide open. The pre-screen in checkKickdown() must therefore
// leave the cooldown untouched.
void test_refused_kickdown_never_delays_overrev_protection(void) {
    setupDriving(3, 2600.0f, '3');       // SPORT AUTO @ ~99 km/h: kickdown is live
    telemetry.engine_rpm  = 3864.0f;
    hwResetPins();
    // A real tip-in, not a constant 95 %: with the rate-of-change trigger a held
    // pedal never arms, and this test would pass without exercising kickdown at all.
    // Prime at 85 %: already inside the kickdown range, but a steady pedal means
    // ROC 0, so nothing is armed yet. Then stab. 85-100 % keeps the auto schedule
    // inert at 98.8 km/h (upshift wants >=105, downshift wants <=55), so only the
    // kickdown trigger can explain a shift here.
    tickThrottle(85.0f, 85.0f, 30);
    tickThrottle(85.0f, 100.0f, 40);     // ~0.38 %/ms: a stab
    tickThrottle(100.0f, 100.0f, 10);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(3, telemetry.current_gear,
        "2nd would spin the turbine to ~6261 rpm: the kickdown must be refused");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(PHASE_CRUISING, sched._current_phase,
        "a refused kickdown must not start a shift");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(0, telemetry.last_auto_shift_ms,
        "a refused kickdown must not arm the cooldown that also gates OVERREV");

    // Overrev now, with the refusal still fresh. It must fire on the next tick.
    telemetry.engine_rpm = 6500.0f;      // past overrev_rpm (6300)
    tick(2);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(4, telemetry.target_gear,
        "overrev upshift must be immediate; a refused kickdown must not gate it");
}

// Kickdown is a pedal EVENT, not a throttle position. Squeezing gradually onto the
// throttle up a long hill reaches the same 95 % as a stab, and must NOT be read as a
// request for a lower gear — the old position-only trigger held true continuously
// there. 4th at 3000 output rpm is chosen so the downshift guard would PERMIT 3rd
// (predicted 4458 rpm) and the auto schedule wants neither gear: only the trigger
// itself can explain the difference between these two cases.
void test_slow_throttle_squeeze_is_not_a_kickdown(void) {
    setupDriving(4, 2500.0f, '3');  // 95 km/h: schedule wants neither 3rd nor 5th
    telemetry.engine_rpm = 2500.0f;
    hwResetPins();
    tickThrottle(30.0f, 30.0f, 25);
    tickThrottle(30.0f, 95.0f, 2000);    // ~0.03 %/ms: below the stab threshold
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(4, telemetry.current_gear,
        "a gradual squeeze to full throttle must not request a kickdown");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(PHASE_CRUISING, sched._current_phase,
        "no shift may start from throttle POSITION alone");
}

// The same car, the same final throttle, reached as a stab: now it is a request.
void test_throttle_stab_does_kickdown_when_the_guard_permits(void) {
    setupDriving(4, 2500.0f, '3');  // 95 km/h: schedule wants neither 3rd nor 5th
    telemetry.engine_rpm = 2500.0f;
    hwResetPins();
    tickThrottle(30.0f, 30.0f, 25);
    tickThrottle(30.0f, 95.0f, 40);      // ~1.6 %/ms: a stab
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(3, telemetry.target_gear,
        "a stab into the kickdown range must request the lower gear");
    TEST_ASSERT_TRUE_MESSAGE(sched._current_phase != PHASE_CRUISING,
        "the kickdown must actually start a shift");
}

// The ATF thermistor sits in series with the P/N contact, so an open circuit is
// exactly what P/N looks like and is NOT a fault. Being in a forward range implies
// the contact is closed, so motion in gear with no reading is a sensor or wiring
// fault. The distinction is the whole test: a code that fired on an open contact
// would light up every time the car was parked.
void test_atf_circuit_dtc_distinguishes_a_fault_from_park_neutral(void) {
    setupDriving(3, 500.0f);
    telemetry.atf_last_valid_ms = g_now_ms;          // sensor reading normally
    dtcManager.poll();
    TEST_ASSERT_FALSE_MESSAGE(dtcManager.snapshot().active[DTC_ATF_CIRCUIT],
        "a live ATF reading must not trip the circuit code");

    telemetry.drive_engaged = false;                 // parked, contact open
    telemetry.output_rpm = 0.0f;
    g_now_ms += ATF_MEASUREMENT_TIMEOUT_MS + 100;
    dtcManager.poll();
    TEST_ASSERT_FALSE_MESSAGE(dtcManager.snapshot().active[DTC_ATF_CIRCUIT],
        "an open contact at rest is P/N, not a fault");

    telemetry.drive_engaged = true;                  // moving in gear, still no reading
    telemetry.output_rpm = 500.0f;
    dtcManager.poll();
    TEST_ASSERT_TRUE_MESSAGE(dtcManager.snapshot().active[DTC_ATF_CIRCUIT],
        "motion in gear with no ATF measurement is a sensor or wiring fault");
}

// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// BENCH TEST MODE. The scenario is a TCU on a table wired to nothing but a valve
// body: no speed sensors, no engine, no throttle, no shifter plate.
// ---------------------------------------------------------------------------
static void setupBench() {
    bootStack();
    telemetry.prnd_state    = 'P';   // no plate attached: the decoder holds its boot value
    telemetry.drive_engaged = false;
    telemetry.output_rpm = 0.0f; telemetry.turbine_rpm = 0.0f;
    telemetry.n2_rpm = 0.0f;     telemetry.n3_rpm = 0.0f;
    telemetry.engine_rpm = 0.0f; telemetry.tps_pct = 0.0f;
    telemetry.map_kpa = 100.0f;  telemetry.atf_temp_c = 40.0f;
    telemetry.test_mode = false; telemetry.test_mode_cmd = 0;
    telemetry.test_sol_req = 0; telemetry.test_sol_req_v = 0;
    telemetry.adapt_nudge_cmd = 0;
    telemetry.paddle_up_request = false; telemetry.paddle_down_request = false;
    telemetry.is_limp_mode = false; telemetry.is_slipping = false;
    telemetry.last_auto_shift_ms = 0;
    sched._prev_pn_raw = false;
    hwResetPins();
}

// The feature itself: a commanded shift must reach the hydraulics with every sensor
// dead. Nothing can ever confirm the ratio here, so this also depends on the F12a
// observability rule — verification applies only where the ratio is real.
void test_bench_mode_shifts_with_every_sensor_dead(void) {
    setupBench();
    telemetry.test_mode_cmd = 1;                  // dashboard toggle: enter
    tick(2);
    TEST_ASSERT_TRUE_MESSAGE(telemetry.test_mode, "must enter bench mode while stopped");

    telemetry.prnd_state = 'D';                   // dashboard virtual selector
    tick(2);
    TEST_ASSERT_TRUE_MESSAGE(telemetry.drive_engaged, "a forced range must latch drive");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(2, telemetry.current_gear, "engages at hydraulic default 2nd");

    hwResetPins();
    telemetry.paddle_up_request = true;           // dashboard virtual paddle
    tick(2);
    // Assert the kick WHILE it is live: g_pwm holds the last value written, and the
    // driver drops the coil to its hold duty at 60 ms and to 0 when the shift ends.
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(KICK_DUTY, g_pwm[PIN_Y5],
        "a 2-3 on a dead bench must still kick Y5 - the hydraulics are the point");
    tickUntilShiftEnds(4000);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(3, telemetry.current_gear,
        "the shift must latch even though no ratio could ever confirm it");
}

// Circuit confirmation uses a signal generator, so speed on the pins must NOT
// block entry or cancel the mode. The dashboard asks before enabling.
void test_bench_mode_can_start_even_if_speed_is_present(void) {
    setupBench();
    telemetry.output_rpm = 800.0f;
    telemetry.test_mode_cmd = 1;
    tick(2);
    TEST_ASSERT_TRUE_MESSAGE(telemetry.test_mode, "test mode must arm for wiring checks even with speed");
}

void test_bench_mode_stays_on_when_speed_appears(void) {
    setupBench();
    telemetry.test_mode_cmd = 1; tick(2);
    TEST_ASSERT_TRUE(telemetry.test_mode);
    telemetry.output_rpm = 800.0f;
    tick(2);
    TEST_ASSERT_TRUE_MESSAGE(telemetry.test_mode, "test mode stays until the operator turns it off");
}

void test_bench_mode_can_jog_a_shift_solenoid(void) {
    setupBench();
    telemetry.test_mode_cmd = 1; tick(2);
    TEST_ASSERT_TRUE(telemetry.test_mode);
    hwResetPins();
    telemetry.test_sol_req = 1;               // Y3 latch on
    telemetry.test_sol_req_v = 1;
    tick(2);
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(KICK_DUTY, g_pwm[PIN_Y3],
        "test mode must be able to kick a routing solenoid with no sensors");
    tick(900);
    TEST_ASSERT_NOT_EQUAL_MESSAGE(0, g_pwm[PIN_Y3],
        "a latched output stays on until toggled off (circuit confirmation)");
    telemetry.test_sol_req = 1;
    telemetry.test_sol_req_v = -1;            // off
    tick(2);
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(0, g_pwm[PIN_Y3], "toggling off must drop the coil");
}

void test_fill_exits_when_offgoing_clutch_moves(void) {
    setupDriving(2, 500.0f);
    engineProfile.raw()->cl_speed_transitions = 1;
    hwResetPins();
    telemetry.paddle_up_request = true;
    uint32_t n = 0;
    while (n < 80 && sched._current_phase != PHASE_FILL) { tick(1); n++; }
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(PHASE_FILL, sched._current_phase, "should be filling the 2-3");
    TEST_ASSERT_LESS_THAN_UINT32_MESSAGE(80, n, "must reach FILL well before the fill timer");
    // 2-3 off-going is K3. Drop N3 so vk3 = r3*(r2*out - n3)/(r2-r3) rises above MOVE.
    float out = telemetry.output_rpm;
    float r2 = ratioOf(2), r3 = ratioOf(3);
    telemetry.n3_rpm = r2 * out - 80.0f * (r2 - r3) / r3;
    telemetry.n2_rpm = telemetry.n3_rpm;
    telemetry.speed_sample_seq++;
    tick(20);   // 10 ms confirm + margin
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(PHASE_TORQUE, sched._current_phase,
        "FILL must end when the off-going clutch starts to move, not only on the timer");
}

// Direct phase fixtures isolate feedback from the automatic shift policy.
static void setupInertia() {
    setupDriving(2, 500.0f);
    TEST_ASSERT_TRUE(sched.beginShift(3, true, "TEST"));
    sched._current_phase = PHASE_INERTIA;
    sched._spc_cmd = 40;
    sched._inertia_slope = 0;
    sched._inertia_target_ms = 400;
    sched._have_ratio_sample = true;
    sched._last_ratio_sample_ms = millis();
}

void test_feedback_switch_off_has_no_hidden_clutch_trim() {
    setupInertia();
    engineProfile.raw()->cl_spc_enable = 0;
    engineProfile.raw()->cl_speed_transitions = 1;
    telemetry.on_clutch_rpm = 1000;
    telemetry.live_ratio = ratioOf(2);
    sched.runShiftPhases(200, true, true);
    TEST_ASSERT_EQUAL_UINT8(40, telemetry.shift_pressure_pct);
}

void test_ratio_feedback_is_bounded_and_drops_out_when_stale() {
    setupInertia();
    engineProfile.raw()->cl_spc_enable = 1;
    telemetry.live_ratio = ratioOf(2) + 1;
    sched.runShiftPhases(200, true, true);
    TEST_ASSERT_EQUAL_UINT8(65, telemetry.shift_pressure_pct);
    g_now_ms += 101;
    sched.runShiftPhases(301, true, false);
    TEST_ASSERT_EQUAL_UINT8(40, telemetry.shift_pressure_pct);
}

void test_single_target_sample_and_undershoot_do_not_confirm() {
    setupInertia();
    telemetry.live_ratio = ratioOf(3);
    sched.runShiftPhases(100, false, true);
    g_now_ms += 70;
    sched.runShiftPhases(170, false, false);
    TEST_ASSERT_EQUAL_UINT8(2, telemetry.current_gear);
    telemetry.live_ratio = ratioOf(3) - .3f;
    sched.runShiftPhases(180, false, true);
    TEST_ASSERT_EQUAL_UINT8(2, telemetry.current_gear);
    TEST_ASSERT_FALSE(sched._target_ratio_tracking);
}

void test_rolling_sensor_loss_is_not_stationary_success() {
    setupInertia();
    telemetry.output_rpm = telemetry.turbine_rpm = 0;
    sched.runShiftPhases(sched.phaseBackstopMs(), true, true);
    TEST_ASSERT_EQUAL_UINT8(2, telemetry.current_gear);
    TEST_ASSERT_TRUE(sched._gear_resync_pending);
}

void test_clutch_motion_observer_requires_explicit_enable() {
    setupInertia();
    engineProfile.raw()->cl_speed_transitions = 0;
    TEST_ASSERT_FALSE(sched.clutchSpeedsLive());
    engineProfile.raw()->cl_speed_transitions = 1;
    TEST_ASSERT_TRUE(sched.clutchSpeedsLive());
    telemetry.input_speed_trusted = false;
    TEST_ASSERT_FALSE(sched.clutchSpeedsLive());
}

void test_individual_speed_loss_disables_feedback_and_tcc() {
    setupInertia();
    telemetry.output_signal_recent = false;
    TEST_ASSERT_FALSE(sched.ratioFeedbackLive());
    telemetry.output_signal_recent = true;
    telemetry.n2_signal_recent = false;
    TEST_ASSERT_FALSE(sched.ratioFeedbackLive());
    telemetry.n2_signal_recent = true;
    telemetry.engine_signal_recent = false;
    sched._current_phase = PHASE_CRUISING;
    sched._tcc_reopen_until_ms = 0;
    telemetry.tcc_lockup_pct = 50;
    sched.updateTCC(true);
    TEST_ASSERT_LESS_THAN_UINT8(50, telemetry.tcc_lockup_pct);
}

void test_unobservable_completion_cannot_enable_learning() {
    setupInertia();
    telemetry.test_mode = true;
    sched.finishShift();
    TEST_ASSERT_FALSE(sched._last_adapt_valid);
}

// Fresh sensor acquisitions, independent of the browser / telemetry publication.
static void atfTick(uint32_t ms, bool samples=true) {
    for (uint32_t i=0;i<ms;++i) {
        ++g_now_ms;
        telemetry.atf_sample_ms=g_now_ms;
        if (samples && g_now_ms%5==0) ++telemetry.speed_sample_seq;
        sched.update(); sol.update();
    }
}
static void atfDriving(uint8_t gear=2) {
    telemetry={};
    setupDriving(gear,500,'D');
    telemetry.atf_only_selector=true;
    telemetry.atf_sampled=true;
    telemetry.atf_range_evidence=2;
    if (gear==1 || gear==5) {
        telemetry.n3_rpm=0; telemetry.n3_signal_recent=false;
        telemetry.n2_rpm=telemetry.turbine_rpm/g_trans.blend_k;
    }
}
void test_atf_requires_dwell_and_drops_early_paddles() {
    atfDriving();
    telemetry.paddle_up_request=true;
    atfTick(290);
    TEST_ASSERT_FALSE(telemetry.atf_forward_confirmed);
    TEST_ASSERT_EQUAL(0,telemetry.current_gear);
    atfTick(30);
    TEST_ASSERT_TRUE(telemetry.atf_forward_confirmed);
    TEST_ASSERT_EQUAL(2,telemetry.current_gear);
    TEST_ASSERT_EQUAL(PHASE_CRUISING,sched._current_phase);
    telemetry.paddle_up_request=true; atfTick(1);
    TEST_ASSERT_EQUAL(3,telemetry.target_gear);
    TEST_ASSERT_NOT_EQUAL(PHASE_CRUISING,sched._current_phase);
}
void test_atf_reverse_and_wrong_ratios_never_authorize() {
    atfDriving(); telemetry.n2_rpm=0; telemetry.n2_signal_recent=false;
    telemetry.turbine_rpm=0; atfTick(1000);
    TEST_ASSERT_FALSE(telemetry.atf_forward_confirmed);
    TEST_ASSERT_EQUAL(0,telemetry.tcc_lockup_pct);
    atfDriving(); telemetry.turbine_rpm=telemetry.n2_rpm=telemetry.n3_rpm=850;
    atfTick(1000); TEST_ASSERT_FALSE(telemetry.atf_forward_confirmed);
}
void test_atf_reidentifies_each_gear_without_assumed_second() {
    for (uint8_t gear=1;gear<=5;++gear) {
        atfDriving(gear); atfTick(320);
        TEST_ASSERT_TRUE(telemetry.atf_forward_confirmed);
        TEST_ASSERT_EQUAL(gear,telemetry.current_gear);
    }
}
void test_atf_signal_loss_aborts_shift_and_requires_new_dwell() {
    atfDriving(); atfTick(320);
    telemetry.paddle_up_request=true; atfTick(1);
    telemetry.atf_range_evidence=0; atfTick(1);
    TEST_ASSERT_EQUAL(PHASE_CRUISING,sched._current_phase);
    TEST_ASSERT_FALSE(telemetry.atf_forward_confirmed);
    TEST_ASSERT_EQUAL(100,telemetry.shift_pressure_pct);
    TEST_ASSERT_EQUAL(0,telemetry.tcc_lockup_pct);
    telemetry.atf_range_evidence=2; atfTick(290);
    TEST_ASSERT_FALSE(telemetry.atf_forward_confirmed);
    atfTick(30); TEST_ASSERT_TRUE(telemetry.atf_forward_confirmed);
}
void test_atf_stale_speed_and_stop_revoke_authority() {
    // The speed-sample gap must EXCEED the freshness window to revoke. Written in
    // terms of the constant so widening it again cannot leave this test passing for
    // the wrong reason.
    atfDriving(); atfTick(320); atfTick(ATF_EVIDENCE_FRESH_MS+20,false);
    TEST_ASSERT_FALSE(telemetry.atf_forward_confirmed);
    atfTick(320); TEST_ASSERT_TRUE(telemetry.atf_forward_confirmed);
    telemetry.output_rpm=0; atfTick(1);
    TEST_ASSERT_FALSE(telemetry.atf_forward_confirmed);
    telemetry.paddle_down_request=true; atfTick(100);
    TEST_ASSERT_EQUAL(PHASE_CRUISING,sched._current_phase);
}
void test_atf_opposing_paddles_cancel_and_tcc_remains_available() {
    atfDriving(3); atfTick(320);
    telemetry.paddle_up_request=telemetry.paddle_down_request=true; atfTick(1);
    TEST_ASSERT_EQUAL(PHASE_CRUISING,sched._current_phase);
    atfTick(500);
    TEST_ASSERT_GREATER_THAN(0,telemetry.tcc_lockup_pct);
}
void test_atf_manual_mode_has_no_kickdown_or_auto_shifts() {
    atfDriving(3); atfTick(320);
    telemetry.tps_pct=95; telemetry.map_kpa=150;
    atfTick(1000);
    TEST_ASSERT_EQUAL(PHASE_CRUISING,sched._current_phase);
    TEST_ASSERT_EQUAL(3,telemetry.current_gear);
}

void test_atf_source_switch_does_not_restore_stale_trrs_or_held_paddle() {
    telemetry={}; hwResetPins();
    InputManager inputs(PIN_ATF_TEMP,PIN_TPS,PIN_MAP);
    inputs.begin();
    g_input[PIN_SHIFT_A]=0; g_input[PIN_SHIFT_B]=0;
    g_input[PIN_SHIFT_C]=1; g_input[PIN_SHIFT_D]=1; // D
    g_adc_mv[PIN_ATF_TEMP]=1200;
    for(int i=0;i<130;++i) { ++g_now_ms; inputs.update(); }
    TEST_ASSERT_EQUAL('D',telemetry.prnd_state);
    TEST_ASSERT_EQUAL(2,telemetry.atf_range_evidence);
    telemetry.atf_only_selector=true; inputs.update();
    telemetry.prnd_state='?';
    for(int i=0;i<30;++i) { ++g_now_ms; inputs.update(); }
    TEST_ASSERT_EQUAL('?',telemetry.prnd_state); // TRRS ignored
    g_input[PIN_SHIFT_C]=g_input[PIN_SHIFT_D]=0; // invalid unplugged harness
    g_input[PIN_PADDLE_UP]=1;
    telemetry.atf_only_selector=false; inputs.update();
    for(int i=0;i<30;++i) { ++g_now_ms; inputs.update(); }
    TEST_ASSERT_EQUAL('?',telemetry.prnd_state);
    TEST_ASSERT_FALSE(telemetry.paddle_up_request);
    g_input[PIN_PADDLE_UP]=0;
    g_input[PIN_SHIFT_B]=g_input[PIN_SHIFT_C]=1; // P
    for(int i=0;i<30;++i) { ++g_now_ms; inputs.update(); }
    TEST_ASSERT_EQUAL('P',telemetry.prnd_state);
}

void test_atf_shift_completes_without_losing_authority_or_learning() {
    atfDriving(2); atfTick(320);
    telemetry.paddle_up_request=true; atfTick(1);
    uint32_t elapsed=0;
    while (sched._current_phase!=PHASE_CRUISING && elapsed++<3000) {
        if (sched._current_phase==PHASE_INERTIA) {
            telemetry.turbine_rpm=telemetry.output_rpm*ratioOf(3);
            telemetry.n2_rpm=telemetry.n3_rpm=telemetry.turbine_rpm;
        }
        atfTick(1);
    }
    TEST_ASSERT_LESS_THAN(3000,elapsed);
    TEST_ASSERT_EQUAL(3,telemetry.current_gear);
    TEST_ASSERT_TRUE(telemetry.atf_forward_confirmed);
    TEST_ASSERT_FALSE(sched.currentMode().auto_shift);
}
void test_atf_cannot_resume_on_first_sample_after_scheduler_gap() {
    atfDriving(); atfTick(320);
    g_now_ms+=100; ++telemetry.speed_sample_seq; atfTick(1);
    TEST_ASSERT_FALSE(telemetry.atf_forward_confirmed);
    atfTick(290); TEST_ASSERT_FALSE(telemetry.atf_forward_confirmed);
    atfTick(30); TEST_ASSERT_TRUE(telemetry.atf_forward_confirmed);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_routing_table_matches_722_6_hydraulics);
    RUN_TEST(test_skip_shift_has_no_routing_solenoid);
    RUN_TEST(test_every_legal_shift_kicks_its_documented_solenoid);
    RUN_TEST(test_shift_that_syncs_latches_the_new_gear);
    RUN_TEST(test_upshift_that_never_syncs_must_not_latch_gear);
    RUN_TEST(test_downshift_that_never_syncs_must_not_latch_gear);
    RUN_TEST(test_standstill_downshift_still_latches);
    RUN_TEST(test_shift_backstop_stretches_when_cold);
    RUN_TEST(test_registry_edit_changes_live_behaviour);
    RUN_TEST(test_next_shift_routes_from_the_corrected_gear);
    RUN_TEST(test_boot_leaves_y3_off);
    RUN_TEST(test_shift_takes_y4_over_from_the_garage_pulse);
    RUN_TEST(test_moneyshift_guard_survives_dead_output_sensor);
    RUN_TEST(test_refused_kickdown_never_delays_overrev_protection);
    RUN_TEST(test_slow_throttle_squeeze_is_not_a_kickdown);
    RUN_TEST(test_throttle_stab_does_kickdown_when_the_guard_permits);
    RUN_TEST(test_atf_circuit_dtc_distinguishes_a_fault_from_park_neutral);
    RUN_TEST(test_bench_mode_shifts_with_every_sensor_dead);
    RUN_TEST(test_bench_mode_can_start_even_if_speed_is_present);
    RUN_TEST(test_bench_mode_stays_on_when_speed_appears);
    RUN_TEST(test_bench_mode_can_jog_a_shift_solenoid);
    RUN_TEST(test_fill_exits_when_offgoing_clutch_moves);
    RUN_TEST(test_feedback_switch_off_has_no_hidden_clutch_trim);
    RUN_TEST(test_ratio_feedback_is_bounded_and_drops_out_when_stale);
    RUN_TEST(test_single_target_sample_and_undershoot_do_not_confirm);
    RUN_TEST(test_rolling_sensor_loss_is_not_stationary_success);
    RUN_TEST(test_clutch_motion_observer_requires_explicit_enable);
    RUN_TEST(test_individual_speed_loss_disables_feedback_and_tcc);
    RUN_TEST(test_unobservable_completion_cannot_enable_learning);
    RUN_TEST(test_atf_requires_dwell_and_drops_early_paddles);
    RUN_TEST(test_atf_reverse_and_wrong_ratios_never_authorize);
    RUN_TEST(test_atf_reidentifies_each_gear_without_assumed_second);
    RUN_TEST(test_atf_signal_loss_aborts_shift_and_requires_new_dwell);
    RUN_TEST(test_atf_stale_speed_and_stop_revoke_authority);
    RUN_TEST(test_atf_opposing_paddles_cancel_and_tcc_remains_available);
    RUN_TEST(test_atf_manual_mode_has_no_kickdown_or_auto_shifts);
    RUN_TEST(test_atf_source_switch_does_not_restore_stale_trrs_or_held_paddle);
    RUN_TEST(test_atf_shift_completes_without_losing_authority_or_learning);
    RUN_TEST(test_atf_cannot_resume_on_first_sample_after_scheduler_gap);
    return UNITY_END();
}
