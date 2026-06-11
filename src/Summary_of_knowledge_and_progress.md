# FMS TCU — Master Engineering & Firmware Manual
**Document Version:** 4.0
**Firmware Version:** V12.1 (git `f50b8b4`, branch `main`)
**Application:** Mercedes-Benz 722.6 (W5A330 Small NAG)
**Powertrain:** M111 2.3L + TVS1320 Supercharger (≈1.2 bar)
**Status:** Bench-ready. Road test pending sensor calibration + hardware-assumption validation (see §16).

> **For external reviewers:** §16 (“Handoff Notes & Open Questions”) is written specifically
> for a second AI/engineer to scrutinise. It lists every load-bearing assumption, the known
> concurrency concerns, and the residual limitations. Start there if you are auditing, then
> use §1–§15 as the reference for *what the code currently does*.

---

## 1. System Architecture

Dual-core ESP32 DevKit V1 (240 MHz):

- **Core 1 — Physics Engine (1000 Hz):** Solenoid PWM, shift state machine, speed sensors, planetary kinematics, safety layers, TPS-ROC tracking. Owns **all** writes to the shared telemetry struct.
- **Core 0 — Web Studio (100 Hz):** Async WebSocket server; streams live telemetry JSON, receives adaptive-table edits, and performs the **deferred NVS flash writes** offloaded from Core 1 (see §11).

The two cores share a single global `TCU_Telemetry telemetry` struct. Core 1 writes, Core 0 reads — **with two unsynchronised exceptions flagged in §16.2.**

### Update-loop priority order (Core 1, every 1 ms)

The order is deliberate — higher items can seize the outputs and `return` before lower items run:

```
1.  checkReverseInhibit()      // HIGHEST — R-while-moving failsafe + RP_LOCK solenoid
2.  limp-mode enforcement      // forced 2nd, max line pressure, recovery gate
3.  checkTpsROC()              // supercharger torque anticipation
4.  calculateLiveRatio()
5.  mid-shift ABORT            // selector knocked out of forward range
6.  calculateLinePressure()
7.  checkLimpMode()            // slip detection (engagement-grace gated)
8.  checkSafetyShifts()        // overrev / lug auto-shifts
9.  phase state machine        // PREFILL/OVERLAP/INERTIA or DS_RELEASE/SYNC/CATCH
10. updateTCC()
```

---

## 2. Hardware Sensors

### Speed Sensors (PCNT Hardware)

All four use the ESP32 `pulse_cnt.h` hardware peripheral — zero missed pulses at 7000+ RPM.

| Signal | Pin | Teeth | Notes |
|---|---|---|---|
| N2 (internal) | 34 | 60 | K1 clutch drum |
| N3 (internal) | 35 | 60 | K2 clutch drum |
| Output shaft | 32 | 24 | Custom external reluctor |
| Engine RPM | 23 | 60 | M111 crank reluctor (moved off GPIO 33 to free ADC1 for MAP) |

Cross-referenced with Thomas’s dueATC (6 teeth × ÷10 PCB divider = 60 effective) and NAG52.

### ATF Temperature / P/N Switch Multiplex

Single wire, two signals on `PIN_ATF_TEMP` (pin 39):

- **Voltage > 3.0 V** → P/N switch open → `pn_switch_raw = true`
- **Voltage ≤ 3.0 V** → through PTC thermistor → temperature from resistance

`temp_c = (resistance_ohms − 800) / 10`, clamped −20…150 °C.

### TPS / MAP

Both on **ADC1** pins (36 / 33) — ADC2 is unusable while WiFi AP is up. EMA α = 0.2 at 1 kHz (τ ≈ 5 ms).

---

## 3. Planetary Kinematics — CONFIRMED FORMULA

The 722.6 has no physical input-shaft sensor. Turbine RPM is derived from N2 and N3.

```
turbine_rpm = (N2 × K) − (N3 × (K − 1))     K = 1.61  (N2_N3_BLEND_K)
```

- N3 = 0 (gears 1 & 5, N3 drum parked) → `N2 × 1.61` ✓
- N2 = N3 (gear 3, both drums locked) → `N2 × 1.0` ✓

Single formula, all gears, no gear-position branch. K is shared across Small/Big NAG (common front planetary). **Bench-verify:** with TCC locked, turbine should equal engine RPM ± 20.

**Prior bug:** formula had N2/N3 swapped (`(N3×1.48) − (N2×0.48)`) → negative turbine in 1st → clamped to 0 → false limp on every startup. Also the 5th-gear constant `0.83` was the *gear ratio*, not the sensor constant.

---

## 4. Transmission Gear Ratios

### W5A330 Small NAG (THIS BUILD)

| Gear | Ratio | | Gear | Ratio |
|---|---|---|---|---|
| 1st | 3.932 | | 4th | 1.000 |
| 2nd | 2.408 | | 5th | 0.830 |
| 3rd | 1.486 | | Rev | 3.100 |

### W5A580 Big NAG (reference — commented block in TCU_Data.h)

1st 3.595 · 2nd 2.186 · 3rd 1.405 · 4th 1.000 · 5th 0.831. If swapping: also raise K2 prefill (~220 ms vs 160 ms) for the larger clutch volume; `K = 1.61` unchanged.

---

## 5. Solenoid Routing — CONFIRMED CORRECT

Cross-referenced against dueATC (`SOL_12_45 / SOL_23 / SOL_34`):

| Solenoid | Pin | Handles |
|---|---|---|
| Y3 | 14 | 1↑2, 4↑5, 5↓4, 2↓1 |
| Y5 | 19 | 2↑3, 3↓2 |
| Y4 | 18 | 3↑4, 4↓3 |
| TCC | 27 | Converter lock-up |
| MPC | 26 | Line/holding pressure (inverted) |
| SPC | 25 | Shift pressure (inverted) |
| **RP_LOCK** | **13** | **Reverse/Park interlock solenoid (digital, new in V12)** |

### Kick-and-Hold Driver

4-ohm coils overheat at continuous 12 V. Firmware: **83 % PWM for 60 ms** (snap open) → **33 % PWM** hold. Shift solenoids behave as on/off hydraulic switches.

### Pressure Solenoids (normally-high / fail-safe)

MPC and SPC are **normally-high** regulator solenoids: **zero current = maximum
pressure** (fail-safe). The API takes an **actual-pressure %** and inverts to PWM:

- `setLinePressure(100)` → duty 0 → ~0 current → **max pressure** (limp/fail-safe)
- `setLinePressure(0)`   → duty 255 → max current → **min pressure**

"Commanded %" *is* pressure %. (The earlier "255 duty = MAX pressure" wording was
backwards — current and pressure are **inversely** related here.) This is why §16.3/H2
flags resting SPC at 0 %: full coil current for *minimum* shift pressure. TCC is
normal logic (more duty = more lock-up).

---

## 6. Drive Engagement Sequence

**In P/N:** Y4 continuously pulsed at 37 % (garage jiggle) to buffer the 3-4 valve body.

**Shifting to D:** garage logic fires on the **falling edge of `pn_switch_raw`** (manual valve leaving P/N), independent of the 4-bit PRND decoder settling.

The 722.6 hydraulic default is **2nd gear**, and the firmware acknowledges it:
- `current_gear = 2`, `target_gear = 2` on engagement
- Y3 is **not** fired (no auto-shift to 1st) — this is a manual-paddle box; the driver picks 1st via paddle-down
- **NEW (V12):** an **engagement-sync grace window** (`ENGAGE_GRACE_MS = 1500`) is armed (see §12.3)

Returning to P/N: `drive_engaged` resets, jiggle resumes, edge detector re-arms.

---

## 7. Shift Execution Pipeline

### Upshift State Machine

| Phase | SPC | Exit condition |
|---|---|---|
| CRUISING | 0 % | Paddle-up OR auto-safety |
| PREFILL | 80 % (fast fill) | `60 + timing_mod` ms elapsed |
| OVERLAP | 45 % → ramp to 90 % | `live_ratio < current_ratio − 0.1` **OR 800 ms backstop** |
| INERTIA | 90 % → ramp to 100 % | `live_ratio ≤ target_ratio + 0.05` OR 600 ms timeout |
| COMPLETION | 0 % | 50 ms settle → adaptive evaluate |

The **800 ms OVERLAP backstop (V12)** guarantees the machine can’t hang if the ratio signal is noisy/absent; it falls through to INERTIA, which has its own completion timeout.

### Downshift State Machine

| Phase | SPC | Exit condition |
|---|---|---|
| DS_RELEASE | 0 % | `80 + timing_mod` ms (rev-match window) |
| DS_SYNC | 0 % | `live_ratio ≥ target_ratio − 0.05` OR 400 ms |
| DS_CATCH | ramp to `90 + pressure_mod` | `(live_ratio ≥ target_ratio − 0.02 AND ≥ 40 ms)` OR 300 ms |
| COMPLETION | 0 % | 50 ms settle → adaptive evaluate |

**Two fixes baked in:**
- DS_SYNC threshold was `target − 0.2` (true immediately on entry → rev-match skipped). Now `target − 0.05`, requiring real ratio climb.
- DS_CATCH formerly shared DS_SYNC’s exact exit → it completed on its first tick (single +2 % step, no real ramp). Now a **distinct, tighter** exit (`target − 0.02` AND minimum 40 ms) so the catch pressure ramp has genuine duration.

### Money-Shift Guard

`predicted_rpm = output_rpm × target_ratio`; if > **6000 RPM**, the downshift is refused regardless of source (paddle or auto-safety).

---

## 8. Hydraulic Pressure Strategy

**Holding pressure (`HOLDING_PRESSURE_MAP[5][16]`):** performance-biased — floor 20–38 %, boost-zone bins (4–8) pushed hard. The blower makes real torque even at light throttle.

**Line pressure during shifts:** `target = tps_pct + (map_kpa − 100)×2.0 + 20`, applied across all shift phases.

**ATF temperature compensation (corrected — both extremes need MORE pressure):**

| ATF | × | ATF | × |
|---|---|---|---|
| < 20 °C | 1.30 | 80–110 °C | 1.05 |
| 20–40 °C | 1.15 | > 110 °C | 1.20 |
| 40–80 °C | 1.00 | | |

---

## 9. Supercharger-Specific Load Model

`load = (tps_pct × 1.25) + max(0, map_kpa − 100) × 0.8`

The ×1.25 TPS weighting front-runs MAP lag (~100–200 ms fill). On a TVS1320 torque tracks throttle mechanically, so TPS is the faster torque proxy. WOT + 1.2 bar ≈ 221 → bin 15 via `loadToBin()` (16 bins, ~12.5 load units each).

### TPS Rate-of-Change (ROC) Torque Anticipation

- **Trigger:** `dTPS/dt > 0.15 %/ms`
- **Effect:** 100 % line pressure + TCC forced open immediately
- **Hold:** while TPS > 40 % OR ROC positive
- **Release:** TPS < 40 % AND ROC < 0.02 %/ms → 2 s cooldown → normal control
- Dashboard: **Torque Mode** badge (orange = active)

---

## 10. TCC Strategy

| Condition | Target slip | Behaviour |
|---|---|---|
| High-torque (ROC) | 500 | Force open preemptively |
| MAP > 105 kPa OR TPS > 45 % | 500 | Open for boost/demand |
| RPM < 1400 OR gear 1 | 1000 | Open to prevent stall |
| Normal cruise | 50 | Integral lock-up (max 85 %) |
| Any shift phase | 1000 | Always open during shifts |

TPS open threshold lowered 75 → 45 % because the blower makes peak torque from any throttle above ~45 %.

---

## 11. Adaptive Memory System

Four `int8_t[4][16][16]` tables (`pressure / prefill / ds_pressure / ds_timing`), persisted to NVS via `Preferences`. Loaded on boot; defaults injected only on blank flash.

| Type | Min | Max | Rationale |
|---|---|---|---|
| Pressure mods | −30 % | +60 % | Small corrections only |
| Timing mods | −30 ms | +120 ms | K2 needs up to 180 ms fill |

**Learning:** flare → `pressure += 2`; upshift bind → `pressure −= 2`; downshift bind → `timing += 5`. Small nudges; ~20–50 shifts per cell to converge.

### NVS write offload (NEW — V11)

`evaluateShift()` used to call `preferences.putBytes()` **directly on the 1 kHz Core-1 loop** — an NVS write can block 1–10 ms and jitter shift timing. Now `evaluateShift()` only sets bits in a `volatile uint16_t _dirty_mask`; **`processDirtyTables()` runs on Core 0** (from `broadcastTelemetry`, 100 Hz) and flushes **one** table per call. Max flush latency for all 16 tables ≈ 160 ms — fine for adaptive learning. *(Concurrency caveat in §16.2.)*

> **Bench note:** new defaults load **only on blank flash**. Run *Erase Flash* before first bench test, or stale tables from a prior version persist.

---

## 12. Safety & Abuse-Case Systems

### 12.1 Auto-Safety Shifts (both pass the money-shift guard)

- **Overrev upshift:** engine > 6300 RPM → force upshift (unless 5th). Now gated on `isForwardRange()`, so it also fires in manual limit positions `'1'`/`'2'` — **engine protection overrides a manual gear cap** (V12.1).
- **Lug downshift:** RPM < 1100 AND TPS > 25 AND **gear > 2** → force downshift. **Floor is 2nd** (V11): 1st stays the driver’s choice, and because D-engagement starts in 2nd this can never fire from a standstill. Sequential — one shift per 500 ms cooldown — so a driver who forgot to drop from 5th at a light gets 5→4→3→2.
- **Cooldown:** 500 ms between auto-shifts.

> A *predictive* lug check was tried and **removed** (V11): at a light in 5th, `output_rpm ≈ 0` made predicted RPM ≈ 0, which blocked the exact downshift the feature exists for. The 2nd-gear floor is the correct guard instead.

### 12.2 Limp Mode

Active while cruising at moderate load (`tps < 80, map < 130, output_rpm > 200`).
`mismatch = |turbine − output×target_ratio|`; > 300 RPM sustained 400 ms → limp:
- 100 % line, 0 % shift, all routing solenoids off (→ hydraulic 2nd), TCC off, dashboard FATAL.
- **Reset:** web `limp_reset`, accepted only when `output_rpm < 50` AND in P/N.

### 12.3 Selector-Abuse Protection (NEW — V12 / V12.1)

Three real abuse cases found in the standalone walkthrough:

**(a) N→D while moving — was: *instant limp*.** Root cause is the engagement-sync transient: engaging 2nd at road speed slips the clutch for several hundred ms while dragging the turbine to `output×2.408`. `checkLimpMode()` now suppresses slip detection while `millis() < _engage_grace_until_ms` (1500 ms, armed on every D engagement). After sync the box genuinely *is* in 2nd, so the ratio math matches and detection resumes. Residual: engaging 2nd at very high speed is mechanically harsh; above ~160 km/h it can momentarily touch overrev before the OVERREV net climbs out (inherent to “coast in N at speed, slam D”).

**(b) Selector knocked to N/R/P mid-shift — was: *stuck solenoid + OVERLAP hang*.** The manual valve hydraulically releases the clutches, but firmware kept the routing solenoid energised and OVERLAP waited forever on a ratio change that could no longer happen. Now an **abort block** (`in_active_shift && !isForwardRange()`) stops all shift solenoids, drops SPC to 0, sets gear→2, returns to CRUISING. The 800 ms OVERLAP backstop (§7) is the second line of defence.

**(c) R while moving forward — was: *no inhibit at all*.** Two layers:
- **Layer 1 (preventive):** `RP_LOCK` solenoid (GPIO13) driven via `setShiftLock(moving)` every loop — physically blocks the lever from leaving the forward range while `output_rpm > 150` (~5 km/h). **Fail-safe:** released at boot/reset, so a dead ESP32 never traps the driver.
- **Layer 2 (reactive failsafe):** if R is engaged above 150 RPM anyway, `checkReverseInhibit()` (top of loop, **above limp mode**) collapses line pressure to `REVERSE_ABUSE_LINE_PCT = 15 %` so the reverse brake B3 slips/heats instead of shock-loading the driveline; SPC 0, TCC 0, all shift solenoids off; `reverse_abuse_active` warns the dashboard (`revAbuse`). Auto-clears below 150 RPM.

**Priority:** reverse failsafe runs **before** limp mode specifically so an already-limp car (which forces line pressure to 100 %) can’t slam B3 at max clamp when R is yanked while rolling (V12.1).

Paddle requests are also ignored outside a forward range (`isForwardRange()` gate).

---

## 13. Calibration Values — VERIFY BEFORE ROAD TEST

| Item | File | Status |
|---|---|---|
| `PIN_MAP = 33`, `PIN_ENG = 23` | TCU_Data.h | **HARDWARE REWIRE** — GPIO 23 has no ADC; MAP→33 (ADC1_CH5), engine tach→23 (PCNT) |
| `PIN_RP_LOCK = 13`, `RP_LOCK_ACTIVE_HIGH`, `ENABLE_RP_LOCK` | TCU_Data.h | **VERIFY shifter mechanism + polarity** before trusting Layer 1; set `ENABLE_RP_LOCK=false` if no lock fitted |
| `MAP_KPA_AT_0V`, `MAP_KPA_PER_VOLT` | TCU_Data.h | **Placeholder** — measure sensor transfer function |
| `TPS_VOLTS_CLOSED/WOT` | InputManager.h | **Placeholder** — measure at pedal stops |
| `N2_N3_BLEND_K = 1.61` | TCU_Data.h | **Bench-verify** vs engine RPM, TCC locked |
| `RPM_OVERREV_UPSHIFT = 6300`, `RPM_LUG_THRESHOLD = 1100` | TCU_Data.h | Sanity-check for M111 + blower |
| `OUTPUT_RPM_MOVING / REVERSE_INHIBIT_SPEED_RPM = 150` | TCU_Data.h | Tune to your final-drive/tyre (≈5 km/h) |
| `ENGAGE_GRACE_MS = 1500` | TCU_Data.h | Long enough for clutch sync at speed |

### Bench Test Protocol

1. **Erase flash** → confirm new adaptive defaults load.
2. Signal-gen all 4 speed sensors → verify RPM math + turbine formula (§3).
3. P→D transition → confirm garage edge fires (gear=2, **no Y3**).
4. Fast TPS ramp → Torque Mode indicator activates.
5. Sustained 300+ RPM turbine mismatch 400 ms → limp fires; reset via dashboard in P/N.
6. Overrev upshift at 6300 simulated.
7. **N→D with output spinning** → confirm **no** false limp (grace).
8. **Selector→N mid-shift** → confirm solenoid drops, no hang.
9. **R with output spinning** → confirm RP_LOCK asserts + line pressure dumps + `revAbuse`.
10. Watch Serial for “Telemetry JSON truncated” — should never appear.

---

## 14. Web Dashboard

- **AP:** `FMS_TCU` / `shiftfast` → `http://192.168.4.1`
- 100 Hz WebSocket JSON. Calibration Studio: fetch/flash any of the 8 tables as a live 16×16 heatmap.
- **Shift View** tab: rolling 6 s canvas chart (pressures / ratio / RPM) with phase colour bands and gear markers.
- New telemetry fields: `phase`, `htMode`, `revAbuse`.
- **Update index.html:** PlatformIO → *Upload Filesystem Image* (separate from firmware). Close Serial Monitor first.

---

## 15. Change Log (recent)

| Ver | Highlights |
|---|---|
| V9 | NAG52 turbine formula fix; TPS/MAP inputs; P/N latch split |
| V10 | Performance pressure tuning; MAP pin→33; DS_CATCH/DS_SYNC distinct exits; flare-ratio + stopwatch fixes; Shift View tab |
| V11 | Lug floor at 2nd (predictive check removed); **NVS write deferred to Core 0** |
| V12 | Engagement-sync grace; mid-shift abort + OVERLAP backstop; **RP_LOCK interlock** + reverse-at-speed failsafe; paddle forward-range gate |
| V12.1 | Reverse failsafe outranks limp; overrev covers manual `'1'`/`'2'` |
| V13 | External-review fixes: reverse trigger now edge/intent-latched (C1); adaptive learns the initiation cell + upshift down-path + braking-aware DS bind (C2/C3/C4); telemetry 10→100 Hz (C5); `char[]`+seqlock status strings (A2); limp in all forward ranges (H4); web-ingest clamp (M1); bounded WS parse (M3); `_dirtyMux` (M6) |

---

## 16. Handoff Notes & Open Questions (FOR EXTERNAL REVIEW)

This section is the audit surface. Everything below is either an **assumption that could be wrong**, a **known concurrency concern**, or a **residual limitation**. A reviewer’s time is best spent here.

### 16.1 Load-bearing assumptions that need expert/hardware validation

1. **Pulse-latch gear-hold model (BIGGEST ASSUMPTION).** The firmware fires a shift solenoid momentarily to *initiate* a shift, then **releases it** in COMPLETION, assuming the valve body hydraulically **latches** the new gear. If the real 722.6 instead holds gears via a **continuous solenoid energisation pattern**, this architecture is fundamentally wrong (gears would drop back to the hydraulic default the moment the solenoid releases). *Question for reviewer: is momentary-pulse-then-latch correct for the 722.6 valve body, or must shift solenoids be held per-gear?*

2. **Hydraulic default = 2nd gear, achieved by all shift solenoids OFF.** Limp mode and every abort rely on this. *Verify the 722.6 with all three shift solenoids de-energised actually sits in 2nd (some sources say the limp/default behaviour is 2nd or 5th depending on valve body / failure mode).*

3. **Solenoid routing map** (Y3=1-2/4-5, Y5=2-3, Y4=3-4). Cross-checked against dueATC but not on this physical valve body.

4. **N2/N3 blend K = 1.61.** Taken from NAG52; bench-verify against engine RPM with TCC locked.

5. **RP_LOCK (GPIO13) existence, mechanism, and polarity.** We *assume* an energise-to-lock interlock that blocks lever travel out of the forward range. The actual shifter may have only a park-shift-lock (energise-to-*release*), or none. Currently `ENABLE_RP_LOCK = true`, `RP_LOCK_ACTIVE_HIGH = true`. **Layer 2 (pressure dump) protects regardless**, so a wrong Layer-1 assumption is not catastrophic, but verify before relying on the lever block.

6. **MAP rewire to GPIO 33 / engine tach to GPIO 23.** Pure ADC2-vs-WiFi hardware constraint. Without the rewire, MAP reads garbage and all boost compensation is dead.

### 16.2 Concurrency concerns (genuine — please scrutinise)

1. **✅ FIXED (V13) — `String` telemetry fields read across cores.** `last_safety_event` and `limp_mode_reason` are now fixed `char[64]` buffers (no heap, no realloc, no cross-core crash). Writes go through `setSafetyEvent()/setLimpReason()` which bump a `seq` counter odd→even (seqlock); Core 0 reads via `readStatusString()` and retries a torn read. The external reviewer also flagged that `checkReverseInhibit()` was assigning the string **every 1 ms** during abuse — now it writes only on the abuse-entry transition.

2. **✅ FIXED (V13) — `_dirty_mask` RMW.** Both the Core-1 `|=` and the Core-0 `&= ~bit` are now wrapped in a shared `portMUX_TYPE _dirtyMux` (`portENTER/EXIT_CRITICAL`). As a bonus the downshift path now dirties only the *one* table it changed (was dirtying both ds_pressure and ds_timing every event).

3. **Scalar telemetry (float/uint8) cross-core reads** are unsynchronised but 32-bit aligned word access is atomic on Xtensa, so individual fields are fine. The risk is only *tearing across multiple related fields* read at slightly different instants (e.g. ratio vs gear) — cosmetic on the dashboard, not used for control on Core 0.

### 16.2b External review — DEFERRED items (intentionally open, with rationale)

The first external review (722.6/EGS52 knowledge, no ATSG) raised these beyond the ones
fixed in V13. They are deferred deliberately — each needs the ATSG PDF, a hardware fact,
a bench trace, or conflicts with the owner's stated intent. **Do not silently "fix" these
without resolving the gating question first.**

| # | Item | Why deferred / gating question |
|---|---|---|
| A1 | Re-derive gear after abort/limp instead of asserting 2nd | The reviewer confirmed pulse-latch holds the *current* gear mid-drive, so all-off ≠ 2nd while rolling. BUT the fix hinges on **whether cycling the manual valve through N resets the latch to 2nd** — an ATSG hydraulic question. Guessing wrong is worse than the current behaviour. |
| H1 | TCC apply/release rate-limit (±1–5 %/ms at 1 kHz is 10–50× too fast) | Real, but a **shift-feel change**; do it together with A3 and re-tune. Low risk to defer (TCC is opened during all shifts anyway). |
| H2 | Rest SPC de-energized between shifts (currently full current at 0 %) | Correct *if* standby SPC pressure is hydraulically blocked with no shift valve stroked — **ATSG hydraulic-diagram question**. Wrong assumption = max shift pressure at rest. |
| H3 | Delete the Y4 garage jiggle (5 W continuous in P/N, unverified benefit) | **Owner design decision** — was added deliberately. Neither EGS52 nor ultimate-nag52 pulses the 3-4 solenoid in P/N; confirm the source before removing. |
| A3 | Real ramp intervals (ramps are near-steps at 1 kHz tick) | Partly **conflicts with the owner's explicit "firm shifts over smooth"** mandate. Needs a deliberate decision on how firm, then a ramp-interval (e.g. step every 5–10 ms). Couples with C3 — adaptive should be re-validated after. |
| H5 | `analogReadMilliVolts()` + divide TPS so WOT ≤ 2.5 V | ESP32 ADC nonlinearity is real; this is a **calibration-time** improvement, best done on the car with the actual sensors. |
| H6 | Confirm M111 trigger is 60 (vs 60−2) and VR-sensor conditioning | **Hardware fact** — cross-referenced as 60 via dueATC, but verify the physical wheel + whether a VR conditioner (MAX9926/LM1815) feeds the GPIO. |
| M2 | Edge-detect paddles (held paddle currently machine-guns shifts every 200 ms) | Genuine; deferred only because it's an **input-semantics change** worth confirming the owner wants (vs intentional repeat). Quick to do. |
| M4 | `ws.textAll()` called from two tasks; `saveTable()` inside WS callback | Needs a small mailbox refactor (service web cmds from `broadcastTelemetry`). Larger change; AsyncWebSocket usually survives but isn't documented thread-safe. |
| M5 | NVS wear (per-shift 256-byte writes) | Partially addressed in V13 (downshift no longer rewrites the unchanged table). Full fix = persist on a timer / on entry to P/N. Defer until learning cadence is measured. |
| H2/SPC + limp | Limp currently commands SPC 0 (= max current); native hydraulic limp is everything de-energized | Same ATSG gate as H2. |

### 16.3 Residual functional limitations (accepted, not bugs)

| Item | Detail | Severity |
|---|---|---|
| Reverse holding pressure | R borrows the 2nd-gear `HOLDING_PRESSURE_MAP` row (gear label = 2). ~22 % line at idle may be soft for a boosted reverse launch. No dedicated reverse clamp target. | Low |
| Reverse re-engage speed | Layer-2 failsafe releases at 150 RPM (~5 km/h); B3 then fully applies while still creeping. Survivable, not silky. Tunable. | Low |
| N→D at extreme speed | Engages 2nd (harsh); OVERREV net climbs out. No speed-matched gear pre-selection (the pulse-latch model can’t statically hold an arbitrary gear without running the shift sequence). | Low |
| Open-loop load | No input-shaft torque sensor; load is inferred from TPS/MAP only. Adaptive learning is the only closed-loop correction. | Medium |
| Adaptive convergence | ±2 nudges with single flare/bind flags per shift — assumed to converge, not formally proven stable; could hunt in a noisy cell. | Medium |
| Flare/bind thresholds | Flare = ratio rise > 0.15; bind = output drop > 30 RPM. Plausible but unvalidated on a real shift trace. | Medium |

### 16.4 Suggested review questions to pose to the external model

- Is the **pulse-latch hold model** valid for the 722.6, or must shift solenoids be held continuously per gear? (§16.1.1 — highest impact.)
- Does the **`String` cross-core access** (§16.2.1) pose a real crash risk here, and what is the lightest correct fix?
- Are the **DS_SYNC / DS_CATCH ratio thresholds and timeouts** physically sensible for the W5A330, or will they cut shifts short / drag them out?
- Is **collapsing line pressure** the right reaction to R-at-speed, or does it risk a different failure (e.g. losing forward clutch control if the lever is mid-travel)?
- Is **1500 ms** an adequate engagement-sync grace at the worst-case N→D speed delta, without masking a genuine clutch failure for too long?
- Does the **performance-biased pressure map** risk clutch shock that could, over time, damage the planetary or the output thrust bearing?
