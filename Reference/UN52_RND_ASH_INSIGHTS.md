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
