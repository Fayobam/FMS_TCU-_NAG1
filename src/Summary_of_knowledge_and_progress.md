# FMS TCU — Master Engineering & Firmware Manual
**Document Version:** 6.0
**Firmware Version:** V14+ (ATSG shift-class architecture; post second-pass external review)
**Application:** Mercedes-Benz 722.6 (W5A330 Small NAG)
**Powertrain:** M111 2.3L + TVS1320 Supercharger (~1.2 bar), engine on rusEFI
**Status:** Compiles clean; bench-validation pending. Firmware logic now grounded in the
ATSG 722.6 Technical Service Information (2004) via the architecture spec in
`Reference/722.6_SHIFT_CLASS_ARCHITECTURE_SPEC.md.pdf`.

> **This is the single source of truth.** The old split between this manual and a separate
> `HANDOFF.md` is retired — `HANDOFF.md` was deleted at v6.0; everything (architecture +
> live status + open items) lives here now. The latest external review (second pass,
> post-V14) is archived at `Reference/FMS 722.6 TCU — Handoff Note (V14).pdf`; its findings
> are folded into §16 as the working open-items tracker.

> **For reviewers:** §1 is the executive architecture. §12 (safety) and §16 (open
> questions) are the audit surface. The design goal is **sharp, confident shifts without
> sacrificing transmission health** — achieved through correct fill + short inertia +
> freewheel-synchronised downshifts, NOT brute pressure (clutch thermal energy =
> torque × slip-speed × slip-time; a correctly-filled short shift is *gentler* than a
> long soft one).

---

## 1. Executive Architecture (v5.0)

Dual-core ESP32. **Core 1 (1 kHz):** sensors, kinematics, the class-based shift engine,
safety, standby/garage. **Core 0 (100 Hz):** WebSocket telemetry + deferred NVS writes.

The shift engine is **one class-parameterised phase machine**. Every shift is classified
once at `beginShift()` from latched **torque** (not raw TPS) and runs the profile for its
class. Pressure commands are quantised to **20 ms** (ATSG p.80); exit predicates run at 1 kHz.

Torque is read from a **per-engine `EngineProfile`** (8×8 RPM×MAP surface, bilinear, NVS-backed
and web-editable — `EngineProfile.h/.cpp`), so an engine swap is a data change, not a recompile.
Gearbox-side constants (ratios, K=1.641) stay compile-time; the learned adaptation trims sit on
top of the profile's baselines. See §2.

**Four shift classes** (ATSG p.77 adaptation categories):

| Class | Trigger | Phase path | Character |
|---|---|---|---|
| `SC_POWER_UP` | tps>8% & torque>25 Nm, upshift | PREP→FILL→TORQUE→INERTIA→LOCK→END | filled fast, short inertia, optional torque-cut |
| `SC_COAST_UP` | closed throttle, upshift | same skeleton, gentle numbers | pull engine down, avoid clunk |
| `SC_POWER_DOWN` | power, downshift | PREP→RELEASE→CATCH→LOCK→END | sprag or timed (below) |
| `SC_COAST_DOWN` | closed throttle, downshift | PREP→RELEASE→CATCH→END | no sync wait, imperceptible |

**Power-down sub-types (the key 722.6 insight, ATSG p.44–49):**
- **`PD_SPRAG` (3→2, 2→1):** release the off-going clutch, let engine torque flare the
  turbine up; the **freewheel (F1/F2) catches at exactly synchronous speed** — drive
  restored with *zero commanded clamp* through the speed change. Sharp **and** zero-wear.
  Detection: ratio reaches target and dRatio/dt collapses for ~40 ms.
- **`PD_TIMED` (4→3, 5→4):** no sprag assist → controlled release-flare-catch with ratio
  feedback; clamp once ~85% of the ratio change is done.

---

## 2. Hardware Sensors

PCNT speed sensors (zero missed pulses at 7000+ rpm):

| Signal | Pin | Teeth |
|---|---|---|
| N2 (front carrier) | 34 | 60 |
| N3 (front sun) | 35 | 60 |
| Output shaft | 32 | 24 |
| Engine RPM | 23 | 60 (M111 crank) |

ATF temp + P/N switch multiplexed on pin 39 (>3.0 V = P/N open). TPS pin 36, MAP pin 33
(both ADC1 — ADC2 dies with WiFi). EMA α=0.2 at 1 kHz.

### Turbine kinematics — derived from tooth counts (ATSG p.8, 32–33)
```
turbine = N2·(1 + Zs/Zr) − N3·(Zs/Zr) = (N2·K) − (N3·(K−1))
Small NAG: sun 50T, ring 78T → K = 1.6410   (Large NAG 58/92 → 1.6304)
```
N3=0 (gears 1&5) → N2·1.641; N2=N3 (3rd) → N2·1.0. **Bench-verify TCC-locked: ±20 rpm.**

### Torque estimation — the master input (ATSG p.77)

Torque now comes from the **`EngineProfile` 8×8 RPM×MAP surface** (bilinear interpolation),
which can represent a curved NA or boosted torque curve — superseding the old single linear
`K_T·(map−MAP_ZERO)` fit. `load_pct = 100·T_est / t_max_ref`, where `t_max_ref` ≈ clutch
capacity (the reference all pressure maps scale against).

The surface is **seeded** from the linear model in `TCU_Data.h` —
`MAP_ZERO=35 kPa, K_T=2.43 Nm/kPa, T_MAX=450 Nm` (calibrated to the real ~450 Nm @ 1.2 bar;
NOT the old 1.55/330). Note the W5A330 is rated 330 Nm, so ~450 Nm is ~36% over rating — the
firm holding-pressure floor and high line authority at load are deliberate; treat ATF service
as part of the strategy. **`T_MAX=450` is unvalidated — confirm against a datalog/dyno (item 8).**

TPS is kept only for **intent** (kickdown, ROC). Torque bins (0-25/25-50/50-75/75-100%)
key the adaptation. Per-engine limits (overrev/lug RPM), sensor cal (TPS volts, MAP transfer
function) and baseline fill tables also live in `EngineProfile`, not `InputManager.h`.

---

## 3. Gear Ratios

W5A330: 1=3.932, 2=2.408, 3=1.486, 4=1.000, 5=0.830, R=3.100.
(W5A580 commented in TCU_Data.h: 3.595/2.186/1.405/1.000/0.831.)

---

## 4. Solenoids & Pressure Model (ATSG pp.25–26, 52–58)

| Solenoid | Pin | Role |
|---|---|---|
| MPC | 26 | Modulating (line/working) pressure; **also feeds the overlap valves** — governs how firmly the off-going element holds during overlap. No current = max line. |
| SPC | 25 | Shift pressure for the oncoming element; PWM **during shifts only**, hydraulically blocked between shifts. No current = max. |
| Y3 / Y5 / Y4 | 14 / 19 / 18 | On/off shift valves (1-2·4-5 / 2-3 / 3-4). ON = initiate shift; OFF in gear (the command valves **latch the gear hydraulically**). |
| TCC | 27 | Converter lock-up (normal logic). |
| RP_LOCK | 13 | Reverse/Park interlock solenoid. |
| TORQUE_CUT | 15 | Optional rusEFI shift-retard request (default OFF). |

**API:** `setLinePressure/Shift(pct)` take **actual pressure %**; coil duty = 100−pct.
These are **normally-high** (no current = max pressure). So `SPC 100%` = de-energized =
OEM "OFF" = also the end-of-shift seat command = in-gear standby — conveniently one value.

**Kick-and-hold** routing drive: 83% for 60 ms, then 33% hold (protects the 4 Ω coils).

### Solenoid state reference (OEM p.53) — implemented
| Condition | Y3 | Y5 | Y4 | MPC | SPC |
|---|---|---|---|---|---|
| Ignition/crank | pulsed ~400 ms | off | off | PWM | off |
| P/N idle | off | off | **pulsed** (Park + lever window) | ~40% duty (60%) | ~33% duty (67%) |
| In gear, driving | off | off | off | line schedule | **OFF (de-energized)** |
| During a shift | ON (whole shift) | ↳ | ↳ | apply/overlap profile | apply profile |

---

## 5. Standby & Garage Strategy (ATSG pp.53–54)

- **Park** or the **lever-movement window** (P/N-exit edge → engagement-grace window):
  pulse Y4 (~37%) to peg the B2 shift valve so its double-piston counter-pressure softens
  N→D/R engagement; SPC 67 / MPC 60 standby duties.
- **N at rest:** Y4 off, SPC 67 / MPC 60.
- **Settled in a driving gear:** Y4 off, SPC de-energized (100), MPC on the line schedule.
- **Boot/crank:** ~400 ms Y3 conditioning pulse.

Engagement to D acknowledges the hydraulic default (2nd); 1st is paddle-only. A 1500 ms
engagement-sync grace masks the N→D-at-speed clutch slip from limp detection.

---

## 6. Shift Phase Profiles (spec §4 — the core)

All pressures quantised to 20 ms; `_spc_cmd` carries fractional ramp. MPC during a shift =
`max(cruise, 40/50 + 0.5·load)`, load>70 → 100 (power); cruise (coast). END decays line at
5%/20 ms.

**Power upshift:** FILL `FILL_P[idx]` (80/82/88/78%) for `FILL_T[idx]` (140/150/180/130 ms)
→ TORQUE apply `20 + 0.55·load` until ratio departs old gear (or 250 ms) → INERTIA ramp
`2 + 0.02·load` %/tick to `target+0.03`, target_ms `clamp(400−1.5·load, 220, 400)` → LOCK
120 ms → END. **Flare** = ratio >0.10 above source during fill/torque. **Harsh** = inertia
< 0.6·target.

**Coast upshift:** fill −15%/−20 ms, apply 25% fixed, ramp 1%/tick, target 350 ms, MPC cruise.

**Power-down PD_SPRAG:** RELEASE SPC 10 (oncoming unclamped) until sprag catch (ratio at
target + dRatio/dt collapse ~40 ms, or 500 ms) → CATCH 30 +2%/tick to lock → END.

**Power-down PD_TIMED:** RELEASE SPC 20 until ratio covers 85% of the change (or 450 ms) →
CATCH 30 +3%/tick closed on sync (±0.05 held 60 ms) → LOCK → END.

**Coast-down:** no sync wait — RELEASE 15 for ~80 ms → CATCH 15 +1%/tick to sync (or 600 ms).
Bind via **decel-delta** (output decel during catch minus the pre-catch braking trend).

---

## 7. Auto Scheduling & Safety

- **Money-shift guard** on every downshift request: `output_rpm × target_ratio ≤ 6000`
  (per single shift in multi-gear kickdowns).
- **Overrev** forced upshift >6300 (gears 1–4, overrides manual limit positions).
- **Lug** forced downshift (RPM<1100, tps>25%, gear>2 floor).
- **Coast-down scheduler:** output-rpm thresholds 5→4<1900, 4→3<1400, 3→2<900; floor 2nd;
  suppressed while a paddle request is pending.
- **Kickdown arm:** tps>70% & engine<5200 → power-down if predicted turbine ≤6000;
  sequential single shifts, never skip-shifts.
- **500 ms cooldown** between auto-shifts.

---

## 8. Adaptation v2 (spec §6)

`AdaptCell{fill_t_cycles ±5, fill_p_trim ±15%, apply_trim ±15%}` indexed
`[ShiftClass(4)][shift_idx(4)][torque_bin(4)]`. Bins captured at `beginShift()`.

- **Power-up (two-sided, no ratchet):** flare → fill_t +1 cycle (primary) + fill_p +2%;
  harsh → apply −2%. **Coast-up:** fill_t only. **Power-down:** catch shock → apply −2%.
  **Coast-down:** decel-delta bind → fill_t +1.
- Deadband: a clean shift writes nothing.
- **ATF-gated** (60–105 °C, p.78); frozen in limp.
- Persist on Core 0: dirty bit per class under `portMUX`, flushed on a 60 s timer or forced
  on entry to P/N (not per shift → NVS wear). Namespace `tcu_adapt2`.
- Web tuner protocol: `get_cells` / `set_cells` (flat int8 stream, clamped on ingest).

---

## 9. TCC Strategy

Force open: ROC mode, MAP>105, TPS>45%, RPM<1400/gear-1, and **any shift phase**. Cruise:
integral lock to 50 rpm slip (max 85%). Forced open before every power-down.

---

## 10. rusEFI Torque-Cut (spec §9, optional)

GPIO15 → rusEFI digital input (shift retard). Asserted only during `SC_POWER_UP` INERTIA at
load>50%. `ENABLE_TORQUE_CUT=false` until wired. Lets the clutch absorb less energy → the
single biggest "sharp without sacrificing health" lever.

---

## 11. Calibration to verify before road test

| Item | File | Status |
|---|---|---|
| `K = 1.641` | TCU_Data.h | bench-verify TCC-locked ±20 rpm |
| MAP rewire (33), engine tach (23) | TCU_Data.h | **hardware** |
| `MAP_ZERO/K_T/T_MAX` torque model | TCU_Data.h | calibrate to engine/sensor |
| `tps_closed_v / tps_wot_v` | EngineProfile (NVS) | measure; switch reads to `analogReadMilliVolts` (item 9) |
| `FILL_P/FILL_T`, profile scalars | TCU_Data.h / ShiftScheduler | bench-tune per class |
| Coast-down rpm thresholds | TCU_Data.h | tune to final-drive/tyre |
| RP_LOCK / TORQUE_CUT polarity+enable | TCU_Data.h | verify hardware before enabling |

---

## 12. Safety Invariants

- Money-shift guard on every downshift; overrev/lug unchanged.
- Limp in **all forward ranges**; on limp: all shift solenoids off, MPC 100 (de-energized
  = native max line), **SPC 100 (de-energized too)**, TCC off, **classify gear from live
  ratio** (don't assert 2nd — p.91: electrical fault holds the latched gear).
- Reverse abuse: transition-based trigger + legitimate-reverse latch (direction-blind
  sensor fix); RP_LOCK blocks the forward→R/P gate while moving; collapse line pressure if
  R is forced at speed.
- Reverse failsafe outranks limp. Mid-shift abort if selector leaves the forward range.
- TCC forced open before power-downs and during all shifts.

---

## 13. Web Dashboard

AP `FMS_TCU`/`shiftfast` → `http://192.168.4.1`. 100 Hz telemetry (gate corrected from 10 Hz).

Phase 9b is **done**: the tuner now speaks `get_cells`/`set_cells` (Adaptation v2, 4×4×4 cells ×
3 fields), there's an **Engine Profile** tab (`get_profile`/`set_profile` — 8×8 torque surface +
limits + sensor cal), and the Shift View phase bands are remapped to the 9-phase enum.

Class/torque telemetry (item A-2, **done**): `buildAndSendTelemetryJSON()` emits `tEstNm`,
`loadPct`, `shiftClass`, `pdType` alongside `phase`/`revAbuse`/`htMode`; the Adaptive Metrics
panel shows a **Shift Class** badge (incl. PD_SPRAG/PD_TIMED) and a **Load / Torque** readout.

---

## 14. Validation Plan (bench → road, spec §11)

1. Output free: TCC-locked turbine check (K=1.641); scope standby duties (P/N 33/40%,
   in-gear SPC flat-OFF); confirm 20 ms command quantisation on the SPC trace.
2. Garage: N→D / N→R < 800 ms, no clunk; verify Y4 window on the scope; B2 firmness after Y4 drop.
3. Coast upshifts first (lowest energy), then power upshifts at 25/50/75% load — flare
   detector silent, inertia near targets, adaptation moves ≤2 steps then settles.
4. Coast downshifts to a stop — imperceptible; decel-delta writes ~nothing under braking.
5. Power downshifts: 3→2 part-throttle first (watch the sprag signature — ratio snaps to
   target with SPC still ~10%); then enable 4→3/5→4 timed; full kickdown last.
6. Log every shift (class, bins, per-phase durations, SPC/MPC, flare/harsh) at 100 Hz.

---

## 15. Change Log

| Ver | Highlights |
|---|---|
| V9–V12.1 | NAG52 turbine fix; perf tuning; selector-abuse protection; reverse interlock |
| V13 | External-review fixes: reverse edge-trigger, adaptation cell-capture/two-sided/decel-delta, 100 Hz gate, char[]+seqlock, limp in all ranges, ingest clamp, dirty-mask mux |
| **V14** | **ATSG shift-class architecture**: K=1.641 (tooth-derived); torque model; standby/garage/crank (p.53-54); 20 ms quantizer (p.80); class-based phase engine (POWER/COAST × UP/DOWN, PD_SPRAG/PD_TIMED); Adaptation v2 (class×idx×torque-bin); coast-down scheduler + kickdown; limp ratio-classifier (p.91); optional rusEFI torque-cut |
| **V14.1** | Dashboard rework (Phase 9b): `get_cells`/`set_cells`, 9-phase enum remap; **EngineProfile** 8×8 NVS torque surface + Engine Profile tab; torque recal to 2.43/450 (real ~450 Nm @ 1.2 bar) |
| **V14.2** | **(this doc, v6.0)** Second-pass external review folded in; HANDOFF.md retired into §16 open-items tracker. |
| **V14.3** | **Review-2 fixes implemented (A+B+C9/C10+D):** TCC rate-limit/quantize + 300 ms post-shift hold; class/torque telemetry on dashboard; Y4 garage→shift takeover; 20 Hz-sample-aware ratio detectors; power-down (PD_TIMED) bind learning; boot de-energized; MPC leads the gate; `analogReadMilliVolts` ADC; edge-triggered paddles; stale-comment/dead-code cleanup. Remaining: C-8/11/12 (bench/dyno/hardware). |

---

## 16. OPEN ITEMS — live tracker (second-pass review, priority order)

> Source: `Reference/FMS 722.6 TCU — Handoff Note (V14).pdf`. This replaces the old
> HANDOFF.md. Status legend: ☐ open · ◐ in progress · ✅ done. Update inline as items close.

### A — fix before bench
- ✅ **1. TCC not rate-limited.** *(done — commit pending)* `updateTCC(ptick)` now moves only on
  the 20 ms quantizer: `TCC_LOCK_STEP=2`%/tick lock (0→85% ≈ 850 ms), `TCC_RELEASE_STEP=10`%/tick
  release (≈200 ms dump). Forced fully open during any shift phase **and** for
  `TCC_POST_SHIFT_HOLD_MS=300` ms after via `_tcc_reopen_until_ms`. *(last leftover of review-1 H1)*
- ✅ **2. Class/torque telemetry now broadcast.** *(done — commit pending)*
  `buildAndSendTelemetryJSON()` emits `tEstNm`/`loadPct`/`shiftClass`/`pdType`; dashboard shows a
  **Shift Class** badge (POWER UP/COAST UP/POWER DN ·SPRAG/·TIMED/COAST DN) + **Load / Torque**
  readout in the Adaptive Metrics panel.
- ✅ **3. Y4 garage-vs-shift collision.** *(done — commit pending)* `fireShiftSolenoid()` now takes
  over a garage-owned Y4 (`STATE_HOLDING` → kick) and clears `_y4_garage_owned`, so a 3→4 inside
  the 1500 ms lever window fires hydraulically instead of silently no-op'ing.

### B — correctness / robustness
- ✅ **4. 20 Hz ratio vs 1 kHz detectors.** *(done — commit pending)* SpeedReader now bumps
  `speed_sample_seq` on each 50 ms refresh; the phase engine computes a `new_sample` edge and rolls
  `_prev_ratio`/`_ratio_flat` **only on a new sample**, measuring flatness across consecutive
  samples (`SPRAG_FLAT_RATIO_DELTA=0.02`). The PD_SPRAG catch reads the held `_ratio_flat` instead
  of the old per-1 ms |Δ|<0.002 (which was trivially true 49/50 iters). PCNT window unchanged
  (20 ms); level-based exits stay at 1 kHz with the accepted ≤50 ms inherent latency.
- ✅ **5. Power-down bind learning revived.** *(done — commit pending)* The decel-delta bind metric
  in `PHASE_CATCH` now fires for `SC_POWER_DOWN && PD_TIMED` as well as coast-down, so
  `learn()`'s power-down path (apply_trim −2, softer catch) is reachable. PD_SPRAG stays excluded
  (freewheel-synced, zero clamp = no catch shock to learn).
- ✅ **6. Boot pressure de-energized.** *(done — commit pending)* `SolenoidDriver::begin()` now
  commands `setLinePressure(100)`/`setShiftPressure(100)` (inverted-logic = no coil current) so
  the coils aren't held at full current/min pressure through the ~1-2 s SPIFFS+WiFi/crank window.
- ✅ **7. MPC leads the gate.** *(done — commit pending)* `beginShift()` calls `applyShiftMPC()`
  on the same tick the routing solenoid fires, so line/overlap authority no longer lags to the
  next 20 ms ptick (spec PREP intent).

### C — calibration / hardware (mostly bench/dyno — nothing more to code unless noted)
- ☐ **8. Torque model validation.** *(hardware/dyno — no code)* `K_T=2.43 / T_MAX=450` makes full
  boost ≈100% load (right shape) but 450 Nm is unvalidated — confirm against datalog/dyno; the SPC
  apply line and all four torque bins scale from it. W5A330 rated 330 Nm; ~450 is 36% over — firm
  holding-pressure floor is deliberate, keep line authority high at load, treat ATF service as
  strategy. (All web-editable in EngineProfile, no recompile.)
- ✅ **9. ADC linearity.** *(done — commit pending)* TPS/MAP/temp reads switched from
  `analogRead/4095·3.3` to `analogReadMilliVolts()` (eFuse-calibrated, linear up top where WOT
  ~2.9 V and the P/N 3.0 V threshold sit). **Still TODO on the bench:** re-measure TPS closed/WOT
  with the calibrated read and write them into EngineProfile.
- ✅ **10. Paddles edge-triggered.** *(done — commit pending; owner chose one-shift-per-pull)*
  `readPaddles()` now fires once on the rising edge, requires release to re-arm, `PADDLE_DEBOUNCE_MS=40`.
  Holding no longer walks gears.
- ☐ **11. M111 crank wheel:** *(hardware — no code)* confirm 60 vs 60-2 (missing teeth under-read
  RPM ~3% + add jitter through the gap); VR sensor conditioning before PCNT remains a hardware item.
  `PULSES_PER_REV_ENG` assumes a clean 60.
- ☐ **12. Hardware rewires already encoded in pins:** *(hardware — no code)* MAP→GPIO33 (ADC1),
  engine tach→GPIO23. RP_LOCK polarity and the GPIO15 torque-cut line (strapping pin; default
  disabled) must be verified before enabling (`RP_LOCK_ACTIVE_HIGH` / `ENABLE_*` flags exist).

### D — cosmetics / doc corrections
- ✅ **13. Stale code comments cleared.** *(done — commit pending)* Fixed: TCU_Data W5A330 header
  ("K is tooth-derived per variant: 1.641 small / 1.630 large", not "1.61 both"); SpeedReader
  kinematics comment (1.641); `telemetry.shift_phase` comment (full 9-entry enum). Removed dead
  code: `_flare_peak_ratio` (member + assignments), the leftover `y4_garage_owned` clear in
  `crankPulseY3()`, and the dead `TPS_VOLTS_CLOSED/WOT` defines in InputManager.h.
- ✅ **14. This manual (v6.0) corrected:** §2 torque section updated to the EngineProfile 8×8
  surface @ 2.43/450; §13 "Phase 9b OPEN" closed (telemetry-broadcast gap re-filed as A-2);
  §11 TPS cal relocated to EngineProfile.

### Resolved by the ATSG spec (historical — do not re-litigate)
- ✅ Pulse-latch hold model correct; ✅ "all-off = 2nd" only for fresh P/N engagement (mid-drive
  holds latched gear → limp classifies from ratio, p.91); ✅ SPC standby de-energized in gear /
  33% in P/N; ✅ Y4 garage pulse OEM-correct (kept + windowed); ✅ K=1.641 (tooth-count).
- Review-2 verified-correct (don't re-audit): class engine + 20 ms quantizer + `_spc_cmd`
  fractional ramp, PD_SPRAG/PD_TIMED split, coast-down decel-delta bind, kickdown + money-shift
  guard, limp ratio-classifier, standby/garage/crank duties, reverse edge-latch failsafe,
  mid-shift abort on leaving forward range, Adaptation v2 (bins-at-init, two-sided, deadband,
  mux dirty mask, one-class flush on Core 0), all V13 fixes (10 ms gate, `char[64]`+seqlock,
  bounded JSON parse, ingest clamps).

### Carried-over concurrency note
- Status strings are `char[64]` + seqlock; scalar telemetry cross-core reads are atomic word
  reads. Adaptation NVS writes are Core-0-only under `portMUX`.

### Validation order (spec §11 / §14, unchanged)
Bench: K=1.641 TCC-locked ±20 rpm → scope standby duties + 20 ms SPC quantisation → garage
N→D/N→R. Road: coast upshifts → power upshifts 25/50/75% load → coast downshifts to a stop →
3→2 part-throttle (watch the sprag signature: ratio snaps to target with SPC still ~10%) → timed
4→3 / 5→4 → full kickdown last. Log class/bins/phase-durations/SPC/MPC per shift (class/load now
broadcast — A-2 done).
