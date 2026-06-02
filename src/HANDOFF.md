# FMS 722.6 TCU — Handoff Note (V9)

This note exists so a fresh session (Claude Code in VS Code, or any new chat) can pick
up with full context. It records WHAT was changed, WHY, and — most importantly — what
is still UNVERIFIED and must be checked before this firmware drives a real car.

**Platform:** ESP32 DevKit V1, dual-core FreeRTOS. Arduino framework.
**Vehicle:** Mercedes 190E, M111.985 2.3L + Audi TVS1320 supercharger (target 1.2 bar).
**Transmission:** 722.6 / Small NAG1 (W5A330, from W203 C230 Kompressor).
**Control philosophy:** Manually commanded shifts (paddles) ONLY, except auto-safety
shifts for overrev and lugging. Hard rev ceiling = 6500 rpm.

---

## Architecture (kept — it was sound)

- **Core 1:** 1000 Hz (1 ms) physics loop — inputs, speed math, shift state machine, solenoids.
- **Core 0:** 100 Hz web loop — async WebSocket telemetry + tuning.
- Speed sensing via ESP32 PCNT hardware counters (N2, N3, output, engine).
- Turbine RPM derived from planetary kinematics (no physical input-shaft sensor).
- "Kick and hold" routing solenoid driver (83% kick 60 ms -> 33% hold) to protect 4-ohm coils.
- Inverted-logic pressure solenoids (0% PWM = max pressure).
- 16x16 adaptive memory matrix (load x rpm) persisted to flash.

Reference projects: rnd-ash/ultimate-nag52-fw (mature closed-loop EGS52 reimplementation,
the gold standard) and tkontrol/dueATC (simpler Arduino 722.6 controller). Solenoid routing
confirmed against Transmission Digest: Y3 = 1-2/4-5, Y5 = 2-3, Y4 = 3-4.

---

## Bugs fixed in V9 (vs the previous version)

1. **No safety auto-shifts existed** — the headline requested feature was absent. Added an
   auto-safety layer: overrev -> forced upshift at 6300 rpm; lugging -> forced downshift below
   1100 rpm when throttle > 25%. Both route through the same money-shift guard, so a safety
   downshift can never itself cause an overrev.

2. **TPS and MAP were never read.** tps_pct/map_kpa were used everywhere (pressure maps,
   load bins, money-shift) but nothing populated them — the car would shift as if always at
   idle. InputManager now reads both analog inputs with light EMA filtering.

3. **Pressure ramp was broken.** OVERLAP/INERTIA read telemetry.shift_pressure_pct to decide
   the next step, but setShiftPressure() never wrote that variable back. The solenoid driver
   now writes back line/shift/TCC values as the single source of truth.

4. **Flare was never detected.** Adaptive learning could see bind but never flare — the exact
   failure mode the K2 3-4 shift is prone to. Added ratio-rise flare detection in OVERLAP/INERTIA.

5. **P/N had two fighting writers.** InputManager wrote is_park_neutral every 1 ms and the
   scheduler also wrote it, so they clobbered each other. Split into pn_switch_raw (sensor,
   owned by InputManager) and drive_engaged (latch, owned by scheduler).

6. **Adaptive modifiers could overflow int8_t.** Repeated += with no clamp on the stored value
   could wrap +127 -> -128 (max clamp becomes min clamp). All learning now clamps to +/-60.

7. **Off-by-one shift-index logic was duplicated 3x** with differing conventions. Centralised
   into one beginShift(target_gear, is_upshift, source); evaluateShift now takes an explicit
   index + direction instead of re-deriving from gear arithmetic.

8. **Load model collapsed the powerband.** Old linear bin saturated at bin 15 well before full
   boost, flattening the whole supercharged range into one cell. Unified computeLoad() /
   loadToBin() in TCU_Data.h, used identically by AdaptiveMemory and ShiftScheduler, spreads
   1.2 bar across multiple cells.

9. **Limp mode was a one-way trap and false-triggered under boost.** Threshold is now load-aware
   (disabled above 80% TPS / 130 kPa, where converter slip and tyre chirp are legitimate), and
   there is a deliberate reset path: set limp_reset_request (web "limp_reset" command), and it
   clears only when stopped and in P/N.

10. **Downshift bind/flare metric was measuring the wrong thing** — partially addressed; see
    open items below.

---

## STILL UNVERIFIED — do NOT road-test until these are done

### Calibration constants (currently placeholders)
- PIN_MAP = 23 in TCU_Data.h is a GUESS. Confirm the pin is free AND on ADC1.
  CRITICAL: ESP32 ADC2 pins do not work while WiFi is active, and this firmware runs a WiFi AP.
  Both TPS and MAP MUST be on ADC1 (GPIO 32-39). TPS=36 is fine; verify/relocate MAP accordingly.
- MAP_KPA_AT_0V and MAP_KPA_PER_VOLT are generic 3-bar values — replace with YOUR sensor's
  actual transfer function.
- TPS_VOLTS_CLOSED / TPS_VOLTS_WOT (InputManager.h) — measure with a multimeter at idle and
  wide-open; do not trust the defaults.
- Temperature formula is still the simple linear (R-800)/10 approximation. The project notes
  reference Kovero's segmented interpolation as more accurate — revisit if temp compensation matters.

### Safety thresholds — sanity-check the numbers for your setup
RPM_HARD_CEILING 6500, RPM_OVERREV_UPSHIFT 6300, RPM_LUG_THRESHOLD 1100,
TPS_LUG_LOAD_PCT 25, RPM_MAX_SAFE_DOWNSHIFT 6000, AUTO_SHIFT_COOLDOWN_MS 500.

### Bench test before vehicle test
Drive the four speed signals + TPS + MAP with a signal generator / bench rig and confirm:
- Overrev upshift fires at the right rpm and completes before the limiter.
- Lug downshift fires only under load and is correctly refused when it would overrev.
- Limp mode triggers on a real slip, does NOT false-trigger on a simulated hard launch, and
  resets cleanly only when stopped + P/N.
- Each paddle shift drives the correct routing solenoid (per the Y3/Y4/Y5 map).

---

## Open items / next work
- Downshift bind detection (PHASE_DS_CATCH) keys off a 30-rpm output-shaft drop, which detects
  driveline shock rather than true clutch bind. On a rev-matched downshift output rpm shouldn't
  move at all — revisit this metric.
- No reverse (R) shift logic is implemented; controller assumes forward driving.
- Consider a watchdog / signal-plausibility check (e.g. implausible turbine vs output rpm ->
  safe state) independent of the slip-based limp mode.
- The "<200 ms clean shift locks in" behaviour described in the manual isn't fully reflected in
  evaluateShift — currently it nudges +/-2 on flare/bind only. Decide if you want explicit lock-in.

---

## File inventory (V9)
TCU_Data.h (v9.0), InputManager.h/.cpp (v9.0), SolenoidDriver.h/.cpp (v2.0),
AdaptiveMemory.h/.cpp (v6.0), ShiftScheduler.h/.cpp (v6.0), main.cpp (v8.0),
WebManager.h (v4.0, unchanged) / WebManager.cpp (v7.0), SpeedReader.h/.cpp (unchanged).
