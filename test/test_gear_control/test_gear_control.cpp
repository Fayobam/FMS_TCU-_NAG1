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

// Boot the stack the way main.cpp does, then run the solenoid driver (only) long
// enough for the 400 ms Y3 crank-conditioning pulse to expire — so tests that are
// not ABOUT the crank window start from a quiet valve body.
static void bootStack(bool clearCrankPulse = true) {
    g_now_ms = 1000;
    hwResetPins();
    engineProfile.begin();
    adaptives.begin();
    dtcManager.begin();
    sol.begin();
    sched.begin();
    if (clearCrankPulse) {
        for (int i = 0; i < 450; i++) { g_now_ms++; sol.update(); }
    }
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
    telemetry.paddle_up_request   = false;
    telemetry.paddle_down_request = false;
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
// 3. SOLENOID OWNERSHIP — the boot Y3 crank pulse must not swallow a real shift
// ===========================================================================
// FINDING 3: during the 400 ms conditioning pulse Y3 sits in STATE_HOLDING, and
// fireShiftSolenoid() only acts on STATE_OFF — so a 1-2 / 4-5 / 2-1 / 5-4 shift
// in that window is silently dropped (and update() then forces Y3 off anyway).
// Same class as the Y4 garage-pulse bug that was already fixed.
void test_shift_during_crank_pulse_is_not_swallowed(void) {
    bootStack(/*clearCrankPulse=*/false);       // Y3 is mid conditioning pulse
    hwResetPins();
    sol.fireShiftSolenoid(PIN_Y3);
    sol.update();
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(KICK_DUTY, g_pwm[PIN_Y3],
        "FINDING 3: a real shift must take Y3 over from the boot crank pulse, "
        "not be silently dropped");
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

// ---------------------------------------------------------------------------
int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_routing_table_matches_722_6_hydraulics);
    RUN_TEST(test_skip_shift_has_no_routing_solenoid);
    RUN_TEST(test_every_legal_shift_kicks_its_documented_solenoid);
    RUN_TEST(test_shift_that_syncs_latches_the_new_gear);
    RUN_TEST(test_upshift_that_never_syncs_must_not_latch_gear);
    RUN_TEST(test_downshift_that_never_syncs_must_not_latch_gear);
    RUN_TEST(test_standstill_downshift_still_latches);
    RUN_TEST(test_next_shift_routes_from_the_corrected_gear);
    RUN_TEST(test_shift_during_crank_pulse_is_not_swallowed);
    RUN_TEST(test_shift_takes_y4_over_from_the_garage_pulse);
    RUN_TEST(test_moneyshift_guard_survives_dead_output_sensor);
    return UNITY_END();
}
