# rnd-ash (ultimate-nag52) video insights — cross-checked vs our TCU

Running log of insights from rnd-ash's TCU dev videos, each bucketed against OUR firmware.
**Our hard constraints (differ from his rig):** (1) **no CANBUS** — anything CAN-dependent is N/A;
(2) **no solenoid current sensing** — closed-loop pressure *by coil current* will NOT port; our
substitute is clutch-speed feedback (which we CAN compute). (3) small NAG W5A330, paddle-first.

---

## Video 1 — Clutch-speed calculation & why it beats ratio monitoring

**Thesis:** monitoring ratio gives the *midpoint* of the change, but what you feel is the **torque
transfer between two clutches**, which does NOT line up with the ratio midpoint. Clutch speeds let
you see: clutch bite, torque-transfer point, full engagement, and flares → feed pressure ramps,
torque requests, and adaptation.

**Structure he uses (matches us):** 722.6 shifts sequentially → collapses to **3 shift groups**:
(1-2/2-1/4-5/5-4), (2-3/3-2), (3-4/4-3). Inputs: output speed Q, input-shaft speed I (from N2/N3),
N2 = front carrier, N3 = front sun. Key kinematic fact: **in gears 2/3/4 the front planetary is
locked, so N2 ≈ N3 ≈ I**; in 1/5 the front sun is held (N3 = 0, I = N2·K).

### ✅ Already aligned (this video largely validates us)
- **3 shift groups** — our [computeClutchSpeeds()](../src/ShiftScheduler.cpp) has exactly these 3 cases.
- **Our `n3`-as-input-shaft proxy is VALID.** Our 2-3 / 3-4 equations use raw `n3` where he uses
  computed `I`. Because the front planetary is locked through those shifts, **N3 = I**, so the
  formulas are structurally correct. This *downgrades* the code-review's "clutch-speed algebra
  unverified / sign risk" worry — the structure checks out against the authority.
- **TCC opened during every shift to dampen shock** — our `updateTCC()` forces TCC open through all
  shift phases + post-shift hold. He confirms the **stock TCU does this too.** Good validation.
- **Pre-fill early-exit on off-clutch movement** — our FILL exits on `off_clutch_rpm > CLUTCH_MOVE_RPM`.
  Same mechanism he describes.

### ⚠️ Refine (discrepancies / better-value tweaks)
- **Negative clutch speed = FLARE, not a bug.** When the clutch comes off before it should, the
  equation output goes negative — he uses that as the cleanest flare detector. So the review's
  "off_clutch could read negative" is a *feature*. Action: when we enable clutch-speed transitions,
  do NOT `fabsf()` the sign away during FILL/overlap — a sign flip is a flare. (We currently
  `fabsf(on_clutch_rpm)` in the sync checks.)
- **Pre-fill threshold:** his early-shift detect = **20 RPM**; ours `CLUTCH_MOVE_RPM = 50`. Lower
  toward 20–25 once bench-verified.
- **Use computed input-shaft speed** (`turbine_rpm = n2·K − n3·(K−1)`) instead of raw `n3` in the
  K2/K3/B2 equations — identical when the front set is locked, but more robust if it momentarily isn't.

### 🆕 Gaps worth adding
- **Input-shaft trust flag (HIGH VALUE — directly fixes a review finding).** In gears 2/3/4 while not
  shifting, N2 and N3 should match; if `|N2 − N3| > ~100 RPM` → a speed sensor is bad → mark input
  speed untrusted. This is the clean guard against the **false-limp-from-bad-turbine-sensing** risk.
  Low-risk diagnostic; propose to implement.
- **TCC pre-fill-to-lockup once torque transfer completes** (off-clutch speed passes on-clutch speed)
  — earlier, cleaner TCC re-apply than our fixed `TCC_POST_SHIFT_HOLD_MS` timer.
- **Overlap pressure ramp = interpolate vs applied-clutch speed** for an ease-out that dampens the
  final shock (feeds future adaptation). We have feedforward + P-trim in INERTIA; this is the
  clutch-speed variant to adopt once cl_speed is verified.
- **Torque RESTORE ramped on clutch speed, not time** (torque-cut decrease static, increase in sync
  with transfer). We only have an on/off GPIO (amplitude ramp = CAN territory, N/A), but we can at
  least gate the cut RELEASE on clutch sync rather than a pure timer.

### 🚫 N/A / forward-looking
- Output speed from ABS wheels + coded diff ratio → we use a dedicated output sensor. Fine.
- "Divide N2/N3 by 2 (both edges)" → our MCPWM captures one (rising) edge; no divide needed.
- **Next video = pressure management via full valve-body simulation.** Expect it to lean on solenoid
  **current** feedback, which we DON'T have. Our path: drive pressure adaptation from clutch-speed
  feedback instead. Flagging now so we don't try to port a current-based loop.

### Action items
- [ ] (Safe, proposed now) Add the N2/N3 input-shaft trust flag → guard limp from sensor faults.
- [ ] (After bench-verifying cl_speed) lower `CLUTCH_MOVE_RPM`→~20–25; swap `n3`→`turbine_rpm` in the
      equations; add negative-clutch-speed flare detection.
- [ ] Keep `cl_speed_transitions` OFF until bench-verified.

---

## Video 2 (processed) — Progress overview [chronologically the EARLIER video]
Covers: hydraulic shift mechanism, current-based pressure control, egs51/analog-shifter support.

### ✅✅ RESOLVES the code-review's #1 blocker — the gear-latch model is CONFIRMED
He spells out the 722.6 shift hydraulics: there is **no direct link between any solenoid and a
clutch pack**. A shift solenoid brings one of the 3 **shift groups** online (Y3 → B1/K1 = 1↔2 & 4↔5,
etc.). Sequence: **bleed → fill → overlap → max-pressure**, then **"the shift solenoid DEACTIVATES
and the command valve switches control of the new clutches to the MAIN PRESSURE RAIL."**
→ The gear is **held hydraulically by line pressure (MPC) via the command/latch valve, NOT by a held
shift solenoid.** This is exactly our **pulse-and-release** model: `beginShift()` fires the routing
solenoid, `finishShift()` turns it OFF, gear latches. **The gear-trace agent's "OEM holds solenoids
per-gear" premise was wrong** — our model is authority-confirmed. (Still worth one bench sanity check,
but no longer a design risk.) Also confirms sequential-only shifting = our ±1 design.

**Corollary:** because the held gear rides on the **main rail (MPC)**, our `HOLDING_PRESSURE_MAP` /
line-pressure cal is **gear-holding-critical**, not just shift-feel — MPC too low ⇒ a settled gear
can slip. Reinforces "don't run line pressure low."

### ✅ Confirms our direction (no change needed)
- **4 pressure phases** (bleed/fill/overlap/max) map onto our PREP→FILL→TORQUE/INERTIA→LOCK. Our PREP
  `setSPC(0)` = his bleed; overlap = MPC relaxes while SPC rises (our `applyShiftMPC` + SPC ramp).
- **Per-torque, per-variant (small/large NAG) working pressures** = our `cl_pressure` model direction.
- **Torque reduction during shift** ("ask the engine to reduce power") = our torque-cut GPIO (he does
  it over CAN with amplitude; ours is on/off — principle same).
- **Flare cause = worn overlap valve** leaking MPC → off-clutch released too early → momentary neutral.
  Mechanical, but validates our flare→more-fill adaptation as the right compensation.
- **egs51 / W210 analog shifter + TRRS** is the parallel (non-CAN) selector path — i.e. our **4-bit
  shifter** approach is the correct one for a standalone. He reverse-engineered the same signal set
  (shifter, pedal, kickdown, reverse/park solenoid, profile switch, starter-lockout). Validation.
- **We're AHEAD on safety:** the features he lists as still-TODO (N→D at speed, overspeed handling,
  limp engage) we already have. Our safety layer is a genuine strength.

### 🚫 Current-based pressure control — the big thing that does NOT port (we have no current sensing)
His core method: **pressure is derived from solenoid CURRENT, not duty cycle.** He runs a
**constant-current driver** (adjusts PWM every 5 ms to hold a target current via shunt-resistor
readings, i2s peripheral reading all 6 solenoids @600 kHz), uses the **OEM EGS52 pressure-vs-current
table**, and closes the loop (armature position ⇒ current draw ⇒ real-time pressure feedback).
**We have none of this.** Implications for us:
- Our **open-loop duty→pressure drifts with battery voltage and coil temperature** — exactly what his
  constant-current loop compensates. This is a real accuracy limitation to design around.
- Partial mitigation if we can sense **VIN**: feed-forward duty compensation for battery voltage
  (hold pressure roughly constant vs supply). See BL-13. (Coil-temp drift stays uncompensated.)
- The OEM pressure-vs-current table isn't directly usable without current sensing (we'd need
  pressure-vs-duty at nominal V/temp). Our substitute remains **clutch-speed feedback** for adaptation.

### Action items (added to backlog)
- [ ] BL-13: battery-voltage feed-forward on pressure-solenoid duty (needs a VIN sense — HW check).
- [ ] Mark the gear-latch concern RESOLVED in the backlog / review verdict.

---

## Video 3 (processed) — Reverse-engineering the OEM EGS-52 firmware (Ghidra)
OEM-internals deep dive. Mostly validation + a couple of concrete refinements.

### ✅ Validates our approach
- **20 ms cycle / "tick" quantization.** OEM runs the whole gearbox app (`MB_layer_update`) on one
  thread **every 20 ms**; all its timers are countdown counters in **20 ms ticks** ("cycles" in DAS).
  rnd-ash even built a ms↔20ms-tick compatibility layer to port OEM logic. → Our `PRESSURE_TICK_MS=20`
  is OEM-correct. **Note for importing any OEM numbers:** times are in 20 ms ticks; all values are
  **unsigned** with a **−50 °C temp offset** (0 = −50 °C).
- **Generalized cals + adaptation beats chasing the exact blob.** The OEM is a full Simulink-generated
  *mathematical model of the gearbox* — change car weight or a clutch pack and the whole calibration
  model must change; that's why one EGS doesn't fit all cars. rnd-ash **generalizes values while
  reversing them** and "some people report that's helped quite a bit." → Our physics-reasoned seeds +
  adaptation is the **same pragmatic path he uses** — we are not doing it wrong by not having the blob.
- **Per-cycle SPC/MPC pressure/current command** at the end of each shift cycle → matches our
  per-tick SPC/MPC computation in `runShiftPhases`.

### 🆕 Concrete refinements (added to backlog)
- **Two-stage pre-fill (BL-15).** OEM fills the applying clutch in **two ramps — high pressure then
  low pressure — *before* the clutch starts moving on**, then engages. Ours is single `_fill_p` for
  `_fill_t_ms`. A high burst (seat piston fast) → low settle (avoid slam) → apply is a real
  shift-quality win. Bench-gated.
- **Signal-validity → safe substitution (BL-16).** OEM marks a lost signal invalid → after a timeout
  *substituted* → injects a **safe default** (e.g. pedal % → 25% if unavailable). Port the *pattern*
  to our sensors: implausible TPS/MAP/speed ⇒ substitute a moderate safe value, not garbage. Extends
  the BL-1 trust flag into a general input-plausibility layer.

### ⚠️ Tuning note (no code)
- **Expect to special-case 5→4 and 3→4.** The OEM has explicit **5→4 patch cases** in the fill
  high-time function (generalized cases under-performed), and our field notes flag **3→4 as the weak
  shift** ([[ofgear-722-6-field-notes]]). So both 5-4 and 3-4 are known-problem shifts — plan to hand-tune
  their fill/overlap rather than rely on the generic profile.

### 🚫 N/A (OEM-internal)
- SCN coding / 10 calibration pools / EPROM blocks / runtime-stat histograms / Ghidra / C166 quirks.
  (The variant-pool concept ≈ our `trans_variant`; the oil-temp histogram ≈ a possible minor logging
  feature, folds into BL-14.)
