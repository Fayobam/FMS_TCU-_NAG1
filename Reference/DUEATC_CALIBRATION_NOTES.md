# dueATC cross-check — a road-tuned 722.6 calibration in OUR units

Source: `github.com/Fayobam/dueATC-by-Thomas` (fork of `tkontrol/dueATC`), Arduino
Due 722.6 controller. Files read: `SD card contents/ORICONF.CFG`, `src/shiftControl.cpp`,
`src/main.cpp`, `headers/*`.

**Why it transfers.** dueATC drives the same MB solenoids with the same
**inverted pressure convention** — `shiftControl.cpp:95`: *"outside shifts, 100,
because inverse control -> 100 = zero current = full pressure"* — identical to our
`setLinePressure()/setShiftPressure()` (`pct 100 → duty 0 → no current → max
pressure`). Neither controller does current control; both command open-loop PWM duty.
And its core loop is **1 kHz** (`main.cpp`: `Timer1.attachInterrupt(coreLoop).start(1000)`
— 1000 µs), so every millisecond value below is directly comparable to ours.

Its config is a **road-tuned artefact**, not a datasheet: an owner drove this and
adjusted it. That makes it the best independent check we have on our seeded numbers.

**Its architecture is simpler than ours** and that limits what transfers. dueATC has
no phase engine: it picks one MPC and one SPC from a (temp × load) map, energises the
shift solenoid, holds both for a fixed time, then drops everything. No fill/torque/
inertia split, no closed loop, no adaptation. So its *pressure levels* and *durations*
are meaningful to us; its *strategy* is not a model to copy.

---

## 1. Shift-solenoid hold time — the finding that matters

`#Shift_solenoid_time_map` → `2,2, -20,100, 0,100, 2000,1200,1600,900`
(rows = ATF °C, cols = MPC value, data = **milliseconds**):

| ATF | MPC 0 | MPC 100 |
|-----|-------|---------|
| −20 °C | 2000 ms | 1200 ms |
| 100 °C | 1600 ms | 900 ms |

Standstill override (`shiftControl.cpp:111-116`): `MPC = SPC = 100`, hold **600 ms**.

**Our routing solenoid is released at `finishShift()`, i.e. at ratio sync — typically
250–600 ms, hard backstop 600 ms. That maximum sits BELOW dueATC's minimum (900 ms).**

Our early release is not automatically wrong — we close the loop on ratio and they
cannot, so a generous open-loop hold is exactly what a feedback-free controller must
do. The normal path (ratio syncs, release) is defensible. **The problem is the
backstop.** A shift still unsynced at 600 ms is, by their road-tuned numbers, still
well inside normal cold-shift territory — and since F12 that case is now abandoned,
tripping `DTC_SHIFT_UNVERIFIED` and forcing a resync.

**Risk: on a cold morning we could abandon legitimate shifts.** The 600 ms backstop
predates F12, when it merely ended the shift; F12 gave it teeth without revisiting it.

**Proposed fix (not yet applied):** scale the INERTIA/CATCH backstop with ATF temp,
roughly tracking their curve — ~600 ms hot, ~1200–1500 ms cold — and only then apply
`shiftProvedByRatio()`. Cheap to add and directly testable on the native harness.

## 2. Target shift duration by load

`#Shift_time_target_map` (1×11, cols = load 0..100 in 10s), ms:
`800, 800, 800, 800, 700, 600, 600, 300, 200, 200, 200`

Ours: `_inertia_target_ms = 400 − 1.5·load`, clamped [220, 400].

| Load | dueATC | ours |
|------|--------|------|
| 0 % | 800 ms | 400 ms |
| 50 % | 600 ms | 325 ms |
| 100 % | 200 ms | 220 ms |

We converge at WOT and diverge at light throttle, where **our shifts are scheduled
about twice as fast**. Their light-load target is a deliberately lazy, comfort-biased
shift. Worth a bench comparison — if our part-throttle shifts feel abrupt, this is the
first number to move.

## 3. Line pressure at light load

`#MPC_normalDrive` (rows = ATF −20/20/60/100 °C, cols = load 0/20/40/60/80/100):

```
-20 C: 100 100 100 100 100 100
 20 C:  77  77  83 100 100 100
 60 C:  70  70  73 100 100 100
100 C:  70  70  70 100 100 100
```

Two things line up with us and one does not:
- **Cold = max pressure** across the board — same intent as our ATF multiplier (1.30× below 20 °C).
- **Saturates at 100 by ~60 % load** — comparable to our map saturating around bin 9–10.
- **Light-load floor is 70.** Ours is **20 (G1) / 22 / 24 / 28 / 38 (G5)** at bin 0.

That last gap is large and in the direction that matters for your build: the W5A330 is
already being run at/above its 330 Nm rating, and low line pressure at light load is
what lets a clutch slip when an unexpected torque spike arrives (exactly what a
supercharger delivers off-idle). Their 70 is a road-proven floor on a stock-torque car.

**Not a "just raise it" change** — higher light-load line costs efficiency, heat and
shift comfort, and our TPS-ROC anticipation already jumps line to 100 on a fast tip-in,
which is a defence they don't have. But the size of the gap deserves a deliberate
decision rather than an inherited default.

## 4. Per-shift SPC — and the 3-4 confirmation

`#SPC_3to4_load` @ 60 °C: `66, 67, 88, 100, 100, 100`
`#SPC_3to4_coast`: `75, 95, 100, 100, 100, 100` — **identical at every ATF temperature.**
`#MPC_3to4_coast` @ 60 °C: `60, 75, 100, 100, 100, 100` — far higher than the other
coast shifts, which sit at `10, 20, 25, …`.

Every other pair is temperature-compensated; 3-4 alone is pinned firm and hot-line
regardless. That is someone concluding on the road that **the 3-4 needs pressure and
no cleverness** — independent confirmation of the K3-drum weakness in the field notes,
and support for our `fill_p[2] = 88` being the highest of the four (their 88 at mid-load
is the same number).

Their mid-load SPC values at 60 °C for reference: 1-2 `46/69/80`, 2-3 `66/68/78`,
3-4 `66/67/88`, 4-5 `70/81/80`.

## 5. Small corroborations (independent, and pleasing)

- **`Min_vehicle_spd_for_gear_ratio_detection = 10`** km/h — they refuse to trust the
  measured ratio below 10 km/h. Our `RATIO_OBSERVABLE_MIN_OUTPUT_RPM = 200` rpm
  ≈ 7.6 km/h, chosen today from first principles. Same conclusion, arrived at
  independently.
- **`float gap = 0.12`** (`shiftControl.cpp:209`, their commented-out ratio window) is
  *exactly* our `SHIFT_VERIFY_RATIO_TOL = 0.12f`.
- **`accept_measuredGear_as_currentGear_after_delay = 1`, delay `2500` ms** — they
  accept the ratio-measured gear over the commanded one after a timeout. That is F12's
  resync, in a shipped controller. Ours acts in 250 ms; theirs in 2500 ms.
- **`Start_with_1St_gear = 1`** — they also force 1st for launch rather than accepting
  the hydraulic-default 2nd. Same call as our `launch_gear = 1`.
- **Regular-drive SPC = 100** (de-energized) matches our `STANDBY_DRIVING` exactly.

## 6. Where we differ deliberately (no action)

- **Garage/P-N line: theirs 30, ours 60.** Ours also pulses Y4 for B2 counter-pressure,
  which they do not do at all — different engagement strategy, so the numbers are not
  comparable.
- **Sensor PPR: driveshaft 12, engine 2** (vs our 24 / 60). Theirs is a coarser setup;
  ours is period-measured, so this says nothing about us.
- **`Eng_spd_oil_press_correction` is all zeros** — the hook exists, unused. They added
  an engine-speed SPC correction and never needed it.

---

## Actions arising

1. **ATF-scaled shift backstop** (§1) — the one real risk, and it interacts with code
   shipped today. Do this first, with a harness test at cold ATF.
2. **Light-load line pressure** (§3) — bench decision, not a blind change.
3. **Part-throttle shift duration** (§2) — tune against traces if shifts feel abrupt.

Nothing here changes the gear-control logic; it is a calibration and timing cross-check.
