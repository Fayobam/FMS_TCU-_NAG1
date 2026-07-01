# Architecture Fixes — Work Plan & Status

Source: `ARCH_REVIEW_2026-06.md` (full findings, file:line refs). This file is
the **single source of truth for progress** — designed so a fresh session (or a
session reset mid-job) can resume with zero context loss.

Quality-only companion queue: `SIMPLIFICATION_NOTES.md` (S1–S13, declutter +
learning notes; no behavior changes). S1–S4 are safe any time; S5–S8 must not
collide with an in-flight F-item; S11 waits until this queue is done.

## Working protocol (follow this every fix)

1. Pick the next `TODO` item below (top to bottom unless the user redirects).
2. Set it `IN PROGRESS` (commit that status change together with the fix).
3. Implement the fix. **Small and surgical** — one fix, one commit.
4. Build gate: `~/.platformio/penv/Scripts/platformio.exe run` must pass
   (plain `pio` is NOT on the Git Bash PATH).
5. Update this file: status → `DONE`, record the commit hash, note any
   deviations from the design sketch.
6. Commit code + plan together; **push to origin/main immediately**
   (each pushed fix is a checkpoint — a session reset costs at most the
   fix in flight).
7. Anything discovered along the way goes in **Discovered during work** at the
   bottom — never silently expand a fix's scope.

Line numbers in designs are as of commit `48c4b83`; re-locate by symbol name
if the file has drifted.

---

## Fix queue

### F1 — Gate shift dispatch while gear identity is unverified  [R1a]
**Status:** DONE (2026-06-20). Deviation from sketch: implemented as ONE guard
inside beginShift() (`if (_gear_resync_pending) return false;`) instead of
gating each dispatch site — covers all six sources (PADDLE/OVERREV/LUG/
KICKDOWN/AUTO/LAUNCH) incl. the checkLaunchGear() added after the review, and
any future caller. Paddle pulls during the window are dropped (flags cleared
at dispatch), not queued.
**Risk:** Highest — cross-apply / planetary tie-up from a false `current_gear=2`.
**Design:** In `ShiftScheduler::update()` CRUISING block, while
`_gear_resync_pending` is true, skip paddle dispatch AND `checkSafetyShifts()` /
`checkKickdown()` / `checkAutoShift()` (all of them compute routing solenoids
from the unverified gear label). The window is ≤1.5 s after an at-speed
engagement; nothing legitimate needs to shift inside it. Clear paddle request
flags so a pull during the window doesn't fire stale afterwards.
**Files:** `src/ShiftScheduler.cpp` (update(), ~:813-883).
**Accept:** builds; paddle/auto/overrev/lug/kickdown all provably unreachable
while `_gear_resync_pending`; normal dispatch resumes after resync.

### F2 — Mid-shift abort: reclassify gear from ratio instead of asserting 2nd  [R1b]
**Status:** DONE (2026-06-20). Deviation from sketch — root cause was an R-path
hole, not the abort itself: N/P already recovered via the P/N latch-clear +
re-engagement resync, but `drive_engaged` was NEVER cleared for R, so any
D→R→D excursion (abort or not) skipped re-engagement and kept a stale gear-2
label with no resync (F1 guard inert). Fix: (a) latch now clears on ANY
non-forward range (P/N/R) — abort recovery rides the normal re-engagement
path; (b) the drive latch opens the ENGAGE_GRACE window itself when latching
at speed (R→D has no P/N falling edge), which also arms slip-limp suppression
for R→D sync; (c) a pending resync is cleared when leaving forward range, so
gear is never classified from an N/R ratio. Abort keeps gear=2 as an explicit
placeholder only.
**Design:** In the abort handler (`ShiftScheduler.cpp:802-810`), replace the
hard `current_gear=2` with the same pattern as engagement-at-speed: set
`_gear_resync_pending=true` + `_engage_grace_until_ms=millis()+ENGAGE_GRACE_MS`
when `output_rpm > OUTPUT_RPM_MOVING`, else default 2. Combined with F1, no
shift can dispatch until the label is ratio-verified.
**Files:** `src/ShiftScheduler.cpp`.
**Accept:** builds; abort while rolling leads to ratio-classified gear, not 2.

### F3 — Reverse legit-latch requires a dwell, not an instantaneous sample  [R8]
**Status:** TODO
**Design:** Latch `_legit_reverse` only if `output_rpm <= REVERSE_INHIBIT_SPEED_RPM`
has held continuously for ~300 ms before the R edge (track
`_stopped_since_ms` in `checkReverseInhibit()`, `ShiftScheduler.cpp:659-695`).
A momentary dip below ~5 km/h can no longer legitimize R at speed.
**Files:** `src/ShiftScheduler.cpp`, constant in `src/TCU_Data.h`.
**Accept:** builds; brief dip below threshold at the R instant → abuse path still arms.

### F4 — Task watchdog on the physics loop  [R2]
**Status:** TODO
**Design:** `esp_task_wdt_init()` (~250 ms timeout, panic=true) +
`esp_task_wdt_add(NULL)` in `core1PhysicsTask`, `esp_task_wdt_reset()` each
loop. A Core-1 stall then resets the chip into the safe boot state (all
solenoids de-energized, `SolenoidDriver::begin()` boot profile) instead of
holding a routing coil at 80 % kick and SPC mid-shift forever. Reboot-mid-drive
recovery already exists (drive latch + gear resync). Do NOT add the dashboard
task to the WDT (a WiFi stall must not reboot the car).
**Files:** `src/main.cpp`.
**Accept:** builds; deliberate `while(1)` test on the bench (never in car)
resets within ~250 ms; normal operation never trips it (check against the
existing loop-overrun DTC).

### F5 — Overrev upshift plausibility + firm profile  [R6]
**Status:** TODO
**Design:** In `beginShift()` add an upshift-side sanity: if
`telemetry.engine_rpm > RPM_HARD_CEILING` AND turbine/output disagree wildly
with a live engine (sensor-implausible), log + still allow (upshift is the
correct overrev response) but clamp to the firm profile: for `source=="OVERREV"`
force `_apply_pct`/`_inertia_slope` to the firm end so the slip window is short.
Small change; the point is bounded clutch-energy, not blocking the shift.
**Files:** `src/ShiftScheduler.cpp` (beginShift/classifyAndProfile).
**Accept:** builds; OVERREV-sourced shifts get the firm profile; normal shifts unchanged.

### F6 — Speed-sensor plausibility + fail-closed guards  [R3]
**Status:** TODO — needs a short design pass before coding
**Design sketch:**
- New telemetry flags `out_speed_trusted`, `eng_speed_trusted` + DTCs.
- OUT implausible: turbine > ~1500 rpm sustained (X00 ms) while output < 50 in
  a forward gear with drive engaged → either real catastrophic slip or dead OUT
  sensor; both justify protective action (limp or pressure-safe hold).
- ENG implausible: engine_rpm < 300 while turbine > 800 (converter cannot be
  that far over-driven with engine stopped) → distrust ENG.
- Lug guard: require `eng_speed_trusted && engine_rpm > 400`.
- Money-shift guard: if BOTH out+turbine read ~0 while `drive_engaged` and MAP/TPS
  show the engine under load → refuse the downshift (fail closed) instead of
  predicting 0. Careful: must not break stopped-car 2→1 launch selection —
  gate the refusal on engine load, not just speeds.
**Files:** `src/ShiftScheduler.cpp`, `src/TCU_Data.h`, `src/DtcManager.{h,cpp}`.
**Accept:** builds; simulated dead-OUT (unplug on bench) trips DTC + protection
instead of silently disabling limp; stopped-car launch shift still works.

### F7 — Double-buffered web writes to live calibration  [R4]
**Status:** TODO — needs a short design pass before coding
**Design sketch:**
- `set_profile` writes into a staging `EngineProfileData` + sets an atomic
  `profile_pending` flag; Core 1 applies staging→active at a safe point
  (PHASE_CRUISING, start of tick) then clears the flag. Same generation-counter
  pattern for `g_trans` (or simply: variant switch requires stopped + P/N —
  switching physical gearbox variant while driving is meaningless anyway).
- `set_cells`: reuse `_dirtyMux` to guard cell writes AND `getCell()` reads
  (3-byte struct copy under the mux is nanoseconds; zero impact on the 1 kHz loop).
- NVS `save()` serializes from the staging copy, not the live one.
**Files:** `src/EngineProfile.{h,cpp}`, `src/WebManager.cpp`,
`src/AdaptiveMemory.{h,cpp}`, `src/ShiftScheduler.cpp` (apply point).
**Accept:** builds; a `set_profile` mid-shift cannot change any value the
in-flight shift reads; variant switch refused unless stopped in P/N.

### F8 — Limp trust-flag recovery + arming windows  [R7]
**Status:** TODO
**Design:** (a) `input_speed_trusted` recovery: also re-evaluate in 5th when
TCC is locked (engine≈turbine cross-check gives an independent trust signal);
(b) DTC the trust-flag trip (already exists: DTC_SPEED_N2N3_MISMATCH — verify
it edges correctly on the held flag); (c) document (not change) the tps<80 /
map<130 gates — they're deliberate (boost launches slip the converter).
**Files:** `src/ShiftScheduler.cpp`.
**Accept:** builds; a transient N2/N3 glitch latched in 2-4 clears itself on a
TCC-locked 5th cruise instead of persisting forever.

### F9 — Comms hygiene batch  [R10 + honorable mentions]
**Status:** TODO
**Design (batch of small independent edits, one commit):**
- `SPIFFS.begin(false)` + on failure retry once, then serve the stub and set a
  telemetry flag — never auto-format the dashboard assets.
- `buf.reserve(24*1024)` + alloc check before the shift-trace serialize; skip
  send on failure (`WebManager.cpp:283-285`).
- `DtcManager::clearAll()` + `processFlush()` around the arrays: wrap in a
  portMUX (same pattern as AdaptiveMemory `_dirtyMux`).
- Rate-limit / remove the blocking `Serial.println` calls on the shift path
  (`ShiftScheduler.cpp:233-234` "DOWNSHIFT BLOCKED" is the spammy one — gate to
  1/s like the overrun log).
- Loop-overrun: additionally count 1001–1500 µs ticks (soft overrun) into a
  separate counter surfaced in telemetry.
- Delete dead `checkCoastDownSchedule()` (confirm with user first).
- Fix the `computeLoad()` comment (`TCU_Data.h:419`).
**Accept:** builds; each edit independently revertable in review.

### F10 — Torque-cut enablement  [R5]
**Status:** BLOCKED (hardware) — wire GPIO15 → rusEFI digital input, configure
retard in rusEFI, bench-verify, then flip `ENABLE_TORQUE_CUT=true` and decide
which modes request it (currently RACE only). Firmware support already exists.
No code change until the wire exists.

### F11 — Limp-entry mid-shift behavior  [R9]
**Status:** EVALUATE — de-energized (SPC=MPC=100) IS the ATSG-native failsafe;
the "fix" may be to accept OEM behavior. Decide after F6 (sensor trust) lands,
since spurious mid-shift limp entries become rarer then. Do not change without
bench traces.

---

## Done

(nothing yet)

## Discovered during work

- `pio` is not on the Git Bash PATH — use
  `~/.platformio/penv/Scripts/platformio.exe`. A `pio run | tail -5` pipe
  masks the exit code; always echo `$?` separately.
- Baseline (48c4b83): RAM 16.8 %, **Flash 86.3 % of 1.25 MB** — flash headroom
  is thin; watch it per fix, and consider a custom partition table before any
  feature that adds libraries.
