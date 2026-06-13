# Ultimate-NAG52 → FMS_TCU — Method Comparison & Backport Plan

Source studied: `rnd-ash/ultimate-nag52-fw` (the **C++/ESP-IDF** firmware, not the Rust rewrite),
focused on the shift engine and excluding everything CAN/EGS-bus and diagnostics (KWP2000), per
scope. UN52 is a full EGS52 re-implementation (CAN torque requests to the ECU, OEM clutch
calibration tables, diagnostics, multiple shifter types). We share the **gearbox** (W5A330 small
NAG) and the **ESP32**, so the *mechanical* methods port even though the *integration* does not.

> TL;DR ranking of what's worth taking, best value-per-effort first:
> 1. **Clutch-speed model** (on/off-clutch slip from N2/N3/output) — cheap, low-risk, high value.
> 2. **Physical pressure-from-torque clutch model** (`P = trq·k/coef + spring − centrifugal`) — the big one.
> 3. **MOD pressure computed to hold the off-going clutch during overlap** (falls out of #2).
> 4. **Clutch-speed-driven split adaptation** (fill-time / apply-torque / spc offsets).
> 5. **Ramped, phase-aligned torque-cut** (shapes our rusEFI GPIO like their CAN trq request).
> 6. **Converter input-torque factor** (turbine torque ≠ engine torque when TCC open).

---

## 1. How the two engines line up

| Concern | UN52 (C++) | FMS_TCU (today) | Verdict |
|---|---|---|---|
| Shift skeleton | `ShiftingAlgorithm` base + **Crossover** (overlap) and **Release** (freewheel/sprag) strategies, each a sub-phase machine (BLEED→FILL→OVERLAP→OVERLAP2→MAXP→END) | Class engine POWER/COAST × UP/DOWN, PD_SPRAG/PD_TIMED; phases PREP→FILL→TORQUE→INERTIA→LOCK→END | **Parity at the skeleton.** Their Crossover≈our power overlap; their Release≈our sprag/coast down. |
| Phase transitions | **Clutch slip speed** (off-clutch starts to move ⇒ fill done; on-clutch→0 ⇒ synced) | Gross **live_ratio** (turbine/output) departure/arrival | **Theirs is better.** Direct clutch slip is far less noisy than gross ratio. **Backport #1.** |
| Pressure command | **Physical:** clutch torque capacity → pressure via friction map + temp coefficients + return spring + centrifugal, then pressure→solenoid current map | **Heuristic:** `FILL_P[]` tables, `apply=20+0.55·load`, `HOLDING_PRESSURE_MAP` %; SPC/MPC as inverted‑% | **Theirs is far more principled. Backport #2/#3.** |
| Line (MOD) during shift | Computed to **hold the off-going clutch** at exactly its torque during overlap | `max(cruise, 40/50 + 0.5·load)`, →100 over 70% load | **Backport #3.** |
| Input torque | Engine torque × **converter multiplication** factor(speed ratio); separate pump-load model | Engine torque from MAP×RPM surface used **directly** as input torque | Backport #6 (small). |
| Adaptation | **Three separate** NVS maps: prefill-time, applying-torque offset, spc offset — fill-time learned from the **clutch-speed** transition cycle | Adaptation v2: `[class][idx][torque-bin]` fill_t/fill_p/apply trims, learned from flare/harsh/bind flags | Backport #4 (depends on #1). |
| Engine torque-reduction during inertia | Ramped CAN **torque request** (down at overlap, back up at sync), sized by torque & target shift time | Binary rusEFI **torque-cut GPIO**, asserted flat during POWER_UP inertia | Backport #5 (shape it). |
| Speed sensing | **PCNT pulse-count** over a variable window; **engine RPM only from CAN** | **MCPWM period capture** @80 MHz, full-rev averaging, all 4 shafts incl. engine | **We are ahead — keep ours, do NOT backport.** |
| TCC | Slip target map + pressure model + power/energy (Joule) accounting | Slip target + rate-limited duty + 300 ms post-shift hold | Parity; their energy accounting is a nice-to-have, low priority. |

**Do NOT backport:** CAN layers (egs51/52/53, hfm), KWP2000/diagnostics, shifter variants,
and the hard dependency on OEM `egs_calibration` (`MECH_PTR`/`VEHICLE_CONFIG`) tables — we will
keep our own web-tunable calibration in `EngineProfile`/NVS and *adapt* the numbers.

---

## 2. The two crown-jewel methods, in detail

### 2.1 Clutch-speed model (`models/clutch_speed.cpp`)
For each gear change, the **on-coming** and **off-going** clutch slip speeds are computed in closed
form from the raw shaft speeds we already measure (N2, N3, output, turbine) and the gear ratios:

- **1-2 / 5-4:** on = `n2 − n3` (K1/K2 path), off = `n3` (B1)
- **2-1 / 4-5:** swapped
- **2-3:** on = `n3 − r3·output` (K2), off = `r3·(r2·output − n3)/(r2 − r3)` (K3)
- **3-4 / 4-3:** off/on = `(r3·output − n3)/(r3 − r4)` (B2) and `n3 − that` (K3)

(`r2,r3,r4` = 2nd/3rd/4th ratios — we have these.) The phase machine then uses, instead of gross
ratio:
- **Fill complete** = off-clutch speed rises above ~`clutch_stationary_rpm` (the off-going element
  has begun to release / oncoming has begun to bite).
- **Synced / end** = on-clutch speed falls below a torque-derived `threshold_rpm` → 0.
- **Flare** = on-clutch speed *increasing* when it should be falling (true clutch slip, not a ratio
  blip). **Bind** = on-clutch speed collapsing faster than commanded.

Why we want it: our 0.10 flare-ratio threshold is noise-prone on low-speed 1-2 shifts; clutch slip
is the signal EGS actually uses and it's robust. We have every input already. **Pure math, no new
calibration, no hardware.**

### 2.2 Physical pressure-from-torque model (`pressure_manager.cpp`)
The whole pressure schedule is derived from torque:

```
pressure_mBar = (clutch_torque_Nm * friction_k[gear][clutch]) / coefficient
clutch_torque_Nm = pressure_mBar * coefficient / friction_k[gear][clutch]   // inverse
P_apply_commanded = pressure_mBar + return_spring_mBar − centrifugal_mBar
```
- `coefficient` ∈ {static, sliding, release}; **sliding is ATF-temp interpolated** (cold→hot).
- **Apply (SPC):** during TORQUE/OVERLAP, commanded clutch torque = input torque (+ adapt offset
  + correction), converted to pressure → the clutch carries exactly engine torque, no more.
- **Release (MOD):** during overlap, line is set so the **off-going** clutch holds exactly its
  share and releases cleanly — this is what produces a flare-free, bind-free crossover.
- Final step: `pressure_mBar → solenoid duty` via the solenoid's pressure/current characteristic.

Why we want it: replaces every magic % with a torque-correct command that holds across the whole
supercharged load range, and makes adaptation meaningful (we'd adapt in Nm/mBar, not %). This is
the single biggest quality lever, and the reason their shifts are sharp without being harsh.

What it needs from us (all web-tunable in `EngineProfile`, seeded then adapted):
- `friction_k[shift_idx]` (mBar per Nm) per clutch/shift — 8 values.
- 3 coefficients (static, release, sliding-cold, sliding-hot) — 4 numbers.
- `return_spring_mBar[shift_idx]` — 8 values.
- A `mBar → SPC%/MPC%` solenoid map (our solenoids are commanded in %; one curve each).
- (Optional, phase later) centrifugal term = f(clutch rpm²).

---

## 3. Phased implementation plan (mapped to our files)

Each phase compiles clean and is independently shippable + bench-testable. Risk rises with phase
number; **gate every pressure-model phase behind a feature flag** so we can fall back to the
current heuristic path on the bench instantly.

### Phase 0 — Runtime transmission variant — ✅ DONE (V17)
Small/big NAG ratios + tooth-blend K are a web-selectable `TRANS_SPECS[]` preset (`g_trans`,
`EngineProfile.trans_variant`, NVS). The clutch-speed model and all ratio math read `g_trans`, so
big NAG is a dropdown change, not a code change. (Prefill seeds per variant noted in `TRANS_SPECS`;
fill_t stays user/adapt-owned.)

### Phase 1 — Clutch-speed model (foundation) — 1a ✅ DONE (V17), 1b PENDING BENCH
- New `ClutchSpeeds.{h,cpp}` (or fold into `SpeedReader`): `computeClutchSpeeds(n2,n3,out,turbine,
  from,to)` → `{on_clutch_rpm, off_clutch_rpm}` using the §2.1 equations + our ratio constants.
- Add `on_clutch_rpm` / `off_clutch_rpm` to telemetry + the Shift CSV (they're the best tuning trace).
- In `ShiftScheduler::runShiftPhases`, add clutch-speed-based transitions **alongside** the ratio
  ones (use whichever fires first at first; then switch over once verified): FILL→TORQUE on
  off-clutch movement; INERTIA/CATCH end on on-clutch→threshold; flare/bind from on-clutch slip.
- **Validate on bench** (signal-gen the 4 channels): confirm on/off clutch speeds match hand-calc
  for each shift, including sign. *Blocks nothing else; do this first.*

### Phase 2 — Converter input-torque factor — ✅ DONE (V18)
`EngineProfile.converterFactor(engine,turbine)` = stall mult (≈2.0×) ramped to 1.0 by the coupling
speed ratio (≈0.85), both web-tunable (`tc_stall_mult`, `tc_coupling_sr`). `t_input_nm = engine ×
factor` is exposed in telemetry (`tInput`, shown on the Load/Torque readout). **Deliberately not yet
repurposing `t_est_nm`/`load_pct`/bins** — kept engine-based so the current adaptation mapping isn't
perturbed before Phase 3 consumes input torque.
Original note:
- In `EngineProfile`: add a 2-point converter multiplication curve (factor vs turbine/engine ratio).
- `t_est` becomes engine torque × factor when TCC is open (≈1.0 locked). Improves the torque value
  feeding load bins, money-shift, and (later) the pressure model — most at launch/low gears.

### Phase 3 — Physical pressure model (the big lever) — MEDIUM risk, HIGHEST value
Incremental, flag-gated (`cl_pressure_model_enable`):
- **3a.** `EngineProfile` calibration block: `friction_k[8]`, coefficients, `return_spring[8]`,
  and the `mBar→%` solenoid curves. Seed `friction_k`/springs from UN52 small-NAG values where
  mappable; otherwise reasonable guesses (they're adapted later).
- **3b.** `PressureModel`: `pForClutchTorque(idx,trq,coef)` + inverse `trqForP(idx,p,coef)`;
  ATF-temp sliding coef.
- **3c.** Apply path: in TORQUE/INERTIA, `SPC = mBarToPct(pForClutchTorque(idx, t_est + adapt,
  Sliding) + spring)`. Replaces `apply=20+0.55·load` when the flag is on.
- **3d.** Release path: during overlap, `MPC = mBarToPct(pForClutchTorque(off_idx, holding_trq,
  Release) + spring)` so line holds the off-going clutch — replaces `40/50+0.5·load`.
- **3e.** Keep `HOLDING_PRESSURE_MAP`/`FILL_*` as the fallback when the flag is off.
- **Validate:** scope SPC/MPC vs commanded mBar; tune `friction_k` so a known torque holds without
  slip; compare shift feel A/B against the heuristic path.

### Phase 4 — Clutch-speed-driven split adaptation — MEDIUM risk
- Replace/augment Adaptation v2 with three offset maps (indexed by shift_idx): **prefill-time**,
  **apply-torque**, **spc** — mirroring `shift_adaptation.cpp`.
- Fill-time learning: record the cycle at which off-clutch first moves (Phase 1 signal) vs expected
  fill cycles; nudge prefill ±1 (their `calc_t_adapt_offset_adv` uses √(pressure) scaling — port it).
- ATF-gated, deadband, Core-0 NVS flush as today.

### Phase 5 — Ramped, phase-aligned torque-cut — LOW/MED risk
- Turn the binary rusEFI GPIO into a timed envelope: assert lead-in before INERTIA, hold through the
  speed change, release at sync — sized by `t_est` and `target_shift_time` (their `linear_ramp_with
  _timer` shape). Even as a single GPIO, aligning the *window* to the clutch-speed sync improves the
  inertia phase materially.

### Sequencing
Phase 1 → 2 can land now (low risk, both feed everything). Phase 3 is the multi-session effort and
depends on 1. Phase 4 depends on 1 (+3 ideally). Phase 5 is independent and can slot anytime.

---

## 4. Notes / risks
- **Calibration provenance:** their friction/spring numbers live in `egs_calibration` (OEM EGS
  flash blobs) keyed to clutch geometry. We can seed from the small-NAG values but must treat them
  as starting points and lean on adaptation + bench traces — same posture as our existing open-item
  C-8 (torque-model validation).
- **Solenoid characteristic:** the `mBar→%` step assumes our MPC/SPC solenoids behave like the OEM
  ones. Our inverted-logic % is a rough proxy; Phase 3c/3d will want a measured pressure/duty curve
  (bench item) to be truly closed-loop. Until then, a tunable linear map is fine.
- **Keep our wins:** MCPWM period sensing and the edge-driven sample sequence stay — they are
  better than UN52's PCNT path and the clutch-speed model rides on top of them.
- Clone studied at `…/Temp/un52fw` (shallow); not committed here.
