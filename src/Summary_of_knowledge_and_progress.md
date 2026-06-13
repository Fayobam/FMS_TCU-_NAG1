# FMS TCU — Master Engineering & Firmware Manual
**Document Version:** 7.1
**Firmware Version:** V16 (V15 + closed-loop SPC, high-rate shift datalogger, auto-reconnect UI)
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

**Speed sensing = MCPWM period measurement (V15), NOT pulse counting.** Each channel
hardware-timestamps rising edges at 80 MHz (MCPWM capture; group 0 = N2/N3/OUT, group 1 = ENG)
and averages intervals over up to one full revolution (tooth-spacing error cancels exactly),
capped at a 100 ms span. ISR glitch rejection (implausibly short intervals dropped), zero-speed
timeout, open-interval decel clamp. Telemetry refreshes at **200 Hz** (was 20 Hz). This is
**sub-rpm at all speeds** — it eliminates the old ±50 rpm counting quantization that poisoned
the 0.10 flare threshold and the 50 rpm TCC slip target.

| Signal | Pin | Pulses/rev |
|---|---|---|
| N2 (front carrier) | 34 | 60 (gearbox internal, fixed) |
| N3 (front sun) | 35 | 60 (gearbox internal, fixed) |
| Output shaft | 32 | `out_ppr` (default 24; **web-editable**) |
| Engine RPM | 23 | `eng_ppr` (default 60; **web-editable**) |

`eng_ppr`/`out_ppr` are EngineProfile fields set on the dashboard and **hot-applied** by
SpeedReader without reboot (math-only reconfig) — a rusEFI tach output or new trigger wheel
needs no recompile. ATF temp + P/N switch multiplexed on pin 39 (>3.0 V = P/N open). TPS pin 36,
MAP pin 33 (both ADC1 — ADC2 dies with WiFi); analog reads use `analogReadMilliVolts()`
(eFuse-calibrated) and are round-robined one channel per 1 ms tick. EMA α=0.2.

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
- **Overrev** forced upshift, **predictive** (V15): triggers on `engine_rpm + roc×OVERREV_LEAD_MS`
  (lead capped +400 rpm) so the shift beats the limiter at WOT in a low gear; gears 1–4, overrides
  manual limit positions.
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

**Rate-limited + 20 ms-quantized (V14.3):** lockup moves only on the pressure ptick —
`TCC_LOCK_STEP=2`%/tick to apply (0→85% ≈ 850 ms), `TCC_RELEASE_STEP=10`%/tick to release
(≈200 ms; release always outruns apply so a boost/shift demand to open wins). Held fully open
for `TCC_POST_SHIFT_HOLD_MS=300` ms after a shift ends (`_tcc_reopen_until_ms`) before lockup
control resumes — never locks through a ratio change or the END line-decay.

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
| `eng_ppr` (engine-RPM pulses/rev) | EngineProfile (NVS) | set to wired source; **≥24**; scope-verify clean (item 11) |
| MAP rewire (33), engine tach (23) | TCU_Data.h pins | **hardware** |
| Torque surface (8×8) + `t_max_ref` | EngineProfile (NVS) | seeded 2.43/450; **validate vs dyno** (item 8) |
| `tps_closed_v / tps_wot_v` | EngineProfile (NVS) | reads now `analogReadMilliVolts` ✓; **re-measure closed/WOT** |
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
3 fields), there's an **Engine Profile** tab (`get_profile`/`set_profile` — 8×8 torque surface,
limits incl. **Engine PPR**, overrev/lug, and sensor cal), and the Shift View phase bands are
remapped to the 9-phase enum.

Class/torque telemetry (item A-2, **done**): `buildAndSendTelemetryJSON()` emits `tEstNm`,
`loadPct`, `shiftClass`, `pdType` alongside `phase`/`revAbuse`/`htMode`; the Adaptive Metrics
panel shows a **Shift Class** badge (incl. PD_SPRAG/PD_TIMED) and a **Load / Torque** readout.

---

## 14. Validation Plan (bench → road, spec §11)

0. **Engine-RPM signal source** (do this first — everything else needs trustworthy RPM): scope
   whatever feeds `PIN_ENG_SPEED` at idle→redline. Confirm clean, evenly-spaced pulses and a
   3.3 V logic level. If using the rusEFI tach output, verify the configured PPR holds its
   spacing at 7000 rpm (60 PPR = 7 kHz — may jitter; drop to 24–36 PPR if so) and set
   `PULSES_PER_REV_ENG` to match. Cross-check the displayed RPM against rusEFI's own RPM. Need
   ≥24 PPR for the TCC slip loop / overrev to resolve (≤50 rpm/count).
1. Output free: TCC-locked turbine check (K=1.641); scope standby duties (P/N 33/40%,
   in-gear SPC flat-OFF); confirm 20 ms command quantisation on the SPC trace.
2. Garage: N→D / N→R < 800 ms, no clunk; verify Y4 window on the scope; B2 firmness after Y4 drop.
3. Coast upshifts first (lowest energy), then power upshifts at 25/50/75% load — flare
   detector silent, inertia near targets, adaptation moves ≤2 steps then settles.
4. Coast downshifts to a stop — imperceptible; decel-delta writes ~nothing under braking.
5. Power downshifts: 3→2 part-throttle first (watch the sprag signature — ratio snaps to
   target with SPC still ~10%); then enable 4→3/5→4 timed; full kickdown last.
6. Log every shift (class, bins, per-phase durations, SPC/MPC, flare/harsh) at 100 Hz.

### Future architecture work — Ultimate-NAG52 backports
A study of rnd-ash's `ultimate-nag52-fw` (C++ engine, CAN/diag excluded) and a phased backport
plan live in **`Reference/UN52_BACKPORT_PLAN.md`**. Headline candidates, value-per-effort order:
(1) **clutch-speed model** — on/off-clutch slip from N2/N3/output for robust phase transitions
(cheap, our sensors already cover it); (2) **physical pressure-from-torque clutch model**
(`P = trq·k/coef + spring`) replacing the % heuristics; (3) **MOD computed to hold the off-going
clutch** during overlap; (4) clutch-speed-driven split adaptation; (5) ramped/phase-aligned
torque-cut; (6) converter input-torque factor. Their **speed sensing is inferior to ours** (PCNT
counting; engine RPM via CAN only) — not a backport target.

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
| **V14.4** | **Engine PPR is web-editable** (EngineProfile `eng_ppr`, default 60; EP_MAGIC→'NAG2'): change the engine-RPM pulses/rev from the Engine Profile tab to match a rusEFI tach output or other source without recompiling. SpeedReader reads `engineProfile.engPpr()` live. **Note: flashing this re-seeds the EngineProfile NVS to defaults** (struct grew) — re-enter any custom torque-surface/cal values after the update. |
| **V20** | **Pressure model: authentic UN52 coefficients + off-going MPC (Phase 3 cont., flag-gated OFF).** Refactored to UN52's exact form `P = T·friction/coef + spring` with the real `PRM_DEFAULT_SETTINGS` coefficients (stationary 100, releasing 120, apply cold 185 / hot 140, ATF-temp interpolated). Friction numerators + springs (per-car EGS52 blob, not open) seeded ~4000 (≈27 mBar/Nm) and web-tunable. **Off-going MPC (3d):** line now holds the *releasing* clutch via `clutchReleaseMbar` (release coef + off-going spring) instead of the apply-clutch proxy. Dashboard exposes the 4 coefficients + per-shift apply/off friction & spring. EP_MAGIC→'NAG8' (re-seeds NVS). Flash 86.0%. |
| **V19** | **Physical pressure-from-torque model started (UN52 backport Phase 3, flag-gated OFF).** EngineProfile gains the clutch model (`clutch_k_x100[4]`, `release_spring_mbar[4]`, `p_full_scale_mbar`; EP_MAGIC→'NAG7', re-seeds NVS) + math: `clutchApplyMbar(idx,T,atf) = k·T·atfMult + spring`, `mbarToPct`. When `cl_pressure_enable` is set, the **power-upshift apply SPC** and the **shift MPC** are derived from input torque (Phase 2's `t_input_nm`) instead of the `20+0.55·load` / `40+0.5·load` heuristics; default OFF keeps the heuristic path in control until bench-validated. Dashboard cal (enable, line@100%, per-shift k/spring) added. **TODO:** off-going-clutch-specific MPC, downshift-catch from model, real mBar→% solenoid curve, then tune `clutch_k` + A/B. Flash 85.9%. |
| **V18** | **Converter input-torque factor (UN52 backport Phase 2).** `EngineProfile.converterFactor(engine,turbine)` = stall multiplication (~2.0×) ramped to 1.0 by the coupling speed ratio (~0.85), both web-tunable (`tcStall`/`tcCoupSr`, EP_MAGIC→'NAG6' — re-seeds NVS). `t_input_nm = engine × factor` exposed in telemetry (`tInput`, on the Load/Torque readout) — the clutches see this, not engine torque, most at launch/low gears. `t_est_nm`/`load_pct`/bins kept engine-based for now so the adaptation mapping isn't perturbed before Phase 3 consumes input torque. Flash 85.8%. |
| **V17** | **Runtime transmission variant + clutch-speed model (UN52 backport Phase 0–1a).** (1) **Variant is now web-selectable** (`EngineProfile.trans_variant`, EP_MAGIC→'NAG5'): small NAG (W5A330) ↔ big NAG (W5A580) switches gear ratios + tooth-blend K live via `g_trans`/`TRANS_SPECS[]` from a dashboard dropdown — no recompile/reflash. `RATIO_*`/`N2_N3_BLEND_K` are now macros over `g_trans`, so all downstream math is variant-correct. **Flashing re-seeds EngineProfile NVS** (struct grew) — re-enter custom cal. (2) **Clutch-speed model**: on/off-clutch slip derived from N2/N3/output + live ratios (`ShiftScheduler::computeClutchSpeeds`), exposed in telemetry (`onClutch`/`offClutch`) and the Shift CSV. **Phase 1a = compute + expose for bench verification**; wiring it into phase transitions (Phase 1b) is gated on a signal-gen check of values/signs per shift. Compiles clean (RAM 16.8%, Flash 85.7%). |
| **V16.1** | **WebUI flapping fix.** Live telemetry was flooding at 100 Hz; a slow/throttled client (PC on WiFi power-save or a background tab) overflowed its 32-deep async TX queue, the stream stalled, and the client's watchdog reconnected → "offline/online" flapping while a faster phone stayed live. Fix: telemetry rate 100 Hz→~33 Hz (30 ms); **back-pressure send** — iterate `ws.getClients()` and `client.text()` only when `canSend()` (a backed-up client skips a frame instead of being force-dropped); client staleness watchdog 2.5 s→4 s. Gauges/chart unchanged; per-shift `shift_trace` unaffected. |
| **V16** | **Bigger wins (closed-loop + observability).** (1) **Closed-loop SPC** in upshift INERTIA: feedforward ramp + proportional trim toward a time-scheduled ratio sweep (`_ratio_old`→`_ratio_target` over `_inertia_target_ms`); trim bounded ±25%, applied on the 20 ms ptick; web-tunable `cl_spc_enable`/`cl_spc_kp` (EngineProfile, default on/Kp=80). Kp=0 or disabled ⇒ pure open-loop (safe fallback). (2) **High-rate shift datalogger**: Core 1 captures a compact sample every 2 ms (~500 Hz) through each shift into a `ShiftTrace` ring; Core 0 dumps it once as a `shift_trace` message → dashboard **⬇ Shift CSV** button (t/phase/spc/mpc/ratio/eng/turb/out/cl_err/flags). (3) **WebUI auto-reconnect**: WS reconnects on drop + a staleness watchdog, no manual refresh after a flash/reboot. EP_MAGIC→'NAG4' (re-seeds NVS). Compiles clean (RAM 16.3%, Flash 85.6%). **CL gain needs bench tuning** — the CSV's `cl_err` column is the tuning signal. |
| **V15.1** | **SpeedReader recompute every 1 ms** (was 200 Hz-gated): the open-interval decel clamp now tracks hard deceleration / low-speed shafts at full loop resolution. `speed_sample_seq` is now **edge-driven** — bumps only when a real new edge advances a ratio channel (N2/N3/OUT), via per-channel `edge_count`, so engine-only edges can't falsely trip the B-4 sprag-flat gate. Steady-state averaging unchanged. |
| **V15** | **Cloud "V10 refinement pass" integrated** (was authored on the f9420dd snapshot; semantically re-applied over V14.4 since `git am` would conflict). Headline: **SpeedReader → MCPWM period measurement** (sub-rpm at all speeds, 200 Hz, hot-applied `eng_ppr`/`out_ppr`, ISR glitch reject / zero-speed timeout / open-interval clamp). Plus: predictive overrev (rpm + roc×lead); flare/bind need 10 ms continuous before latching (`RATIO_EVENT_CONFIRM_MS`); bind detection for **all** downshift classes (supersedes V14.3's PD_TIMED-only); PRND 20 ms debounce; staggered ADC (1 channel/tick); engagement latch only on confirmed forward range + gear-resync-from-ratio when engaged rolling; TPS-ROC over a 20 ms window; `out_ppr` web-editable; `begin()` gear=2; TCU_Data dead-code removal (linear torque fit, `MAP_KPA_*`, `UPSHIFT_FIRM_MS`, `FILL_P_PCT/FILL_T_MS`). EP_MAGIC→'NAG3' (re-seeds NVS). Kept from V14: `analogReadMilliVolts`, TCC 300 ms post-shift hold, B-4 sample-gated sprag flat (now driven at 200 Hz). Compiles clean (RAM 14.3%, Flash 85.5%). |

---

## 16. OPEN ITEMS — live tracker (second-pass review, priority order)

> Source: `Reference/FMS 722.6 TCU — Handoff Note (V14).pdf`. This replaces the old
> HANDOFF.md. Status legend: ☐ open · ◐ in progress · ✅ done. Update inline as items close.

### V15 (cloud "V10 pass") — integrated, NEEDS BENCH VERIFICATION
The MCPWM speed-sensing rewrite compiles clean but **cannot be behavior-verified without a
signal generator** — its capture/averaging/timeout logic is the new foundation everything reads.
Bench before road:
- ☐ Verify MCPWM capture rpm vs a signal generator on **all 4 channels** (N2/N3/OUT/ENG),
  including the **zero-speed timeout** (kill the signal → reading drops to 0 within ~0.25 s).
- ☐ Verify a **web PPR change** (e.g. OUT 24→48) rescales rpm immediately, no reboot.
- ☐ Confirm `K=1.641` turbine check still holds TCC-locked ±20 rpm with period measurement.
- ☐ Verify **predictive overrev** fires in the (overrev−400 … overrev) window under a simulated
  3000+ rpm/s pull and completes before 6500.
- ☐ Re-verify TCC ramp (my V14.3 rates: +2/−10 per tick + 300 ms post-shift hold) on a scope.
- ☐ Sanity-check that `SPRAG_FLAT_RATIO_DELTA=0.004` (now a ~5 ms per-sample band) still detects
  a real 3→2 sprag catch — retuned from 0.02 when samples went 50 ms→5 ms.

### V16 (closed-loop SPC) — tune on the bench using the new datalog
- ☐ **Tune `cl_spc_kp`** (Engine Profile tab, live): pull a power upshift, hit **⬇ Shift CSV**, plot
  `cl_err` vs `t_ms`. Goal: `cl_err` stays small through INERTIA with no SPC oscillation. Too low =
  sluggish (ratio lags schedule, `cl_err` stays positive); too high = SPC hunts / `cl_err` rings.
  Start 80, walk up; if it oscillates, halve it. `clEn` off = pure open-loop A/B reference.
- ☐ Confirm the ±25% trim clamp is never the limiting factor at sane Kp (if it saturates, the
  feedforward `_inertia_slope`/fill is wrong, not the gain).
- ☐ Closed-loop is **upshift INERTIA only** for now; downshift CATCH stays open-loop (future item).
- ☐ Verify the datalogger never drops a needed trace: back-to-back shifts skip capture if the prior
  trace hasn't been sent (`shiftTrace.ready`) — expected, but confirm it's not masking shifts you
  care about (the `trace-info` line shows the last captured shift).

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
  samples. **V15 update:** speeds are now MCPWM-period-measured (sub-rpm) at 200 Hz, so the seq
  bumps every 5 ms; the per-sample flat band was retuned `SPRAG_FLAT_RATIO_DELTA 0.02→0.004`.
  The sample-gating is still needed because the 1 kHz loop still outruns the 200 Hz refresh.
- ✅ **5. Power-down bind learning revived.** *(done; V15 widened)* The decel-delta bind metric in
  `PHASE_CATCH` now fires for **all** downshift classes (V15, with a 10 ms confirm window),
  superseding V14.3's PD_TIMED-only restriction — `learn()`'s power-down path is reachable.
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
- ✅ **10. Paddles edge-triggered.** *(done; V15 debounce 50 ms)* `readPaddles()` fires once on the
  rising edge, requires release to re-arm, `PADDLE_DEBOUNCE_MS=50`. Holding no longer walks gears.
- ☐ **11. Engine-RPM signal source (decision pending).** *(hardware; PPR web-editable)* **V15 note:**
  speed sensing is now **period-measured (MCPWM)**, which is sub-rpm even at low PPR — so the old
  "≥24 PPR" resolution floor **no longer applies**; more teeth just average a full rev faster at
  crawl. `eng_ppr`/`out_ppr` are live on the Engine Profile tab. Remaining source decision:
  - **Raw M111 crank** = 60-**minus-2** wheel; the missing-tooth gap is still **not handled** (the
    period averager assumes even teeth) → feed a clean gap-free signal, not the raw crank.
  - **rusEFI tach output (recommended)** — clean, gap-free; sidesteps the 60-2 gap AND the
    VR-conditioning item. Any PPR works now; **level-shift 5/12 V → 3.3 V.**
  - **CAN RPM (cleanest)** — see item 15.
- ☐ **12. Hardware rewires already encoded in pins:** *(hardware — no code)* MAP→GPIO33 (ADC1),
  engine tach→GPIO23. RP_LOCK polarity and the GPIO15 torque-cut line (strapping pin; default
  disabled) must be verified before enabling (`RP_LOCK_ACTIVE_HIGH` / `ENABLE_*` flags exist).
- ☐ **15. CAN RPM path (future, robust).** *(new — code + hardware)* Add a 3.3 V CAN transceiver
  (e.g. SN65HVD230) on a spare pin-pair and read RPM from rusEFI's broadcast instead of counting
  pulses — gap-free, jitter-free, no resolution ceiling, and opens the door to pulling coolant/
  other channels too. Replaces the `PIN_ENG_SPEED` pulse path; keep the pulse counter as fallback.
  Effort: new CAN reader module + a telemetry source-select. Not started.

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
