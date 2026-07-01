# Simplification & Declutter Notes

Companion to `ARCH_FIXES_PLAN.md` (safety fixes F1–F11). This queue is
**quality-only** — nothing here changes behavior; every item either deletes
dead weight, merges duplicated state, or improves comments. Same working
protocol as the fix plan: one item = one commit = one push, build-gated
(`~/.platformio/penv/Scripts/platformio.exe run`), status updated in the same
commit. Line numbers as of commit `1b6d272`.

Each item has a **Learning note** — the general principle behind it, because
the point is not just a cleaner repo but knowing *why* next time.

**Ordering vs the fix queue:** apply S1–S4 (pure deletions) any time — they
cannot conflict with anything. Hold S5–S8 (code reshaping) until they don't
collide with an in-flight F-item touching the same lines. S11 is explicitly
deferred until F1–F11 are done.

---

## A. Dead code — delete it (verified unused by grep, 2026-06-20)

### S1 — Delete `checkCoastDownSchedule()` and the `COAST_DN_*` constants
**Status:** TODO
**Where:** `ShiftScheduler.h:116`, `ShiftScheduler.cpp:453-475`,
`TCU_Data.h:208-216`; also the "Supersedes checkCoastDownSchedule" comment at
`ShiftScheduler.cpp:484` once its referent is gone.
**What:** Fully implemented, never called. `checkAutoShift()`'s closed-throttle
column *is* the coast schedule now. (Also listed in F9 — tick both when done.)

> **Learning note — dead code is a tax, not a reserve.** Unreachable code
> feels harmless ("might need it later"), but every future reader — including
> you in six months — must first *figure out* it's dead before they can
> understand the live path. It shows up in searches, it looks like it matters,
> and comments claiming "superseded" go stale silently. Git remembers deleted
> code forever: `git log -p --follow -- src/ShiftScheduler.cpp` can resurrect
> it any time. Deleting is not losing.

### S2 — Delete the three dead safety constants
**Status:** TODO
**Where:** `TCU_Data.h:88-90` — `RPM_HARD_CEILING`, `RPM_OVERREV_UPSHIFT`,
`RPM_LUG_THRESHOLD`. Zero uses in `src/`.
**What:** All three were superseded by the NVS-backed, web-editable
`engineProfile.overrevRpm()` / `lugRpm()`. The live values come from the
dashboard's Engine Profile tab, not this header.

> **Learning note — the most dangerous clutter is authoritative-looking dead
> config.** These read like the safety limits of the TCU. Someone (you, on a
> late night) edits `RPM_HARD_CEILING` to change the rev ceiling, reflashes,
> and *nothing changes* — the real value lives in NVS. That failure mode is
> worse than a crash, because it's silent. When a value migrates to runtime
> config, delete the compile-time twin in the same commit, or leave a
> one-line tombstone comment saying where it went (this file already does
> that pattern well elsewhere, e.g. the MAP_KPA note at `TCU_Data.h:111`).

### S3 — Remove the write-only `pn_switch_raw` field + its false comment
**Status:** TODO
**Where:** `TCU_Data.h:317-318` (field), `InputManager.cpp:92` (only write),
`ShiftScheduler.cpp:832-834` (comment claiming "pn_switch_raw is still
telemetered for diagnostics" — it is **not** in the JSON; nothing reads it).
**What:** Delete the field, the write, and fix the comment. If you actually
want it on the dashboard someday, adding a JSON field takes one line then.

> **Learning note — a comment that asserts something false is worse than no
> comment.** Readers trust comments *more* than code when skimming. This one
> would send someone hunting through the dashboard JS for a field that was
> never sent. When you delete or change behavior, grep for comments naming it:
> `grep -rn "pn_switch_raw" src/` before *and* after the change.

### S4 — Move `Summary_of_knowledge_and_progress.md` out of `src/`
**Status:** TODO
**Where:** `src/Summary_of_knowledge_and_progress.md` → `Reference/`.
**What:** `src/` is the compiler's directory; docs live in `Reference/` with
the rest (AUTO_SHIFT_MAP, UN52 insights, this file). Pure `git mv`.

---

## B. Duplicated state & logic — one source of truth

### S5 — Collapse the `_high_torque_mode` / `telemetry.high_torque_mode` twin
**Status:** TODO
**Where:** `ShiftScheduler.h:92` (private `_high_torque_mode`),
`ShiftScheduler.cpp:614-615, 641-643` (both written in lockstep),
readers at `ShiftScheduler.cpp:105` (line pressure) and `:185` (TCC).
**What:** Two copies of one fact, updated together at every site. Drop the
private member; read/write `telemetry.high_torque_mode` everywhere. (The
telemetry copy must stay — the dashboard reads it.)

> **Learning note — duplicated state always diverges eventually.** Today the
> two writes sit on adjacent lines. One day a new code path sets one and not
> the other, and you get a bug where the *dashboard* says high-torque mode but
> the *line pressure* doesn't reflect it (or vice versa) — near-impossible to
> spot in a log because each copy looks internally consistent. The rule:
> a fact lives in exactly one place; everything else reads it. Mirrors are
> only justified across a boundary (e.g. latching a value at shift start so
> mid-shift changes can't corrupt an in-flight decision — that's why
> `_load_at_start` at `ShiftScheduler.cpp:247` is correct and NOT clutter:
> it's a deliberate snapshot, not a mirror).

### S6 — Delete the kickdown pre-guard that duplicates `beginShift()`'s guard
**Status:** TODO
**Where:** `ShiftScheduler.cpp:531-532` (`predicted = output_rpm * ratio;
if (predicted > RPM_MAX_SAFE_DOWNSHIFT) return;`) vs the centralized two-way
guard inside `beginShift()` at `:226-237`.
**What:** `checkKickdown()` pre-computes a *weaker* version (output-only) of
the money-shift check that `beginShift()` will run anyway (two-way,
output+turbine). Delete the pre-check; let `beginShift()` decide. The file
header even states the design intent: "Centralised shift initiation in
beginShift() to remove duplicated, off-by-one trigger code."

> **Learning note — guards belong at the choke point.** When several callers
> funnel into one function, put the safety check inside that function, once.
> Scattered caller-side pre-checks drift: this one already has — it checks
> only the output-sensor prediction, so it *passes* cases the real guard
> blocks, and a future editor "fixing" the pre-check wouldn't help because
> the authoritative check is elsewhere. One gate, one place, no drift.

### S7 — Compute the shift/adaptation index once
**Status:** TODO
**Where:** `ShiftScheduler.cpp:243` (`_active_shift_idx = constrain(...)` in
`beginShift`) and `:326` (identical `idx = constrain(...)` recomputed inside
`classifyAndProfile`), used again via `_active_shift_idx` in
`evaluateAdaptation` (`:1130`).
**What:** Same derivation, two places. Pass `_active_shift_idx` into
`classifyAndProfile` (or have it read the member) so the "which gear-pair is
this" rule exists once.

> **Learning note — duplicated *derivations* are subtler than duplicated
> state.** Both lines are correct today because they encode the same rule
> (upshift → from-gear−1, downshift → target−1). Edit one someday — say a
> 6-speed variant changes the indexing — and the adaptation table reads cells
> the profile builder didn't write. Nothing crashes; shifts just slowly learn
> garbage. Rule of thumb: if two expressions must stay identical for the
> program to be correct, they're one function/variable wearing two coats.

### S8 — In-class member initializers; slim the constructor/begin() overlap
**Status:** TODO
**Where:** `ShiftScheduler.cpp:12-21` (constructor) vs `begin()` (`:23-48`) —
overlapping resets (`_prev_was_power`, `_last_pressure_update_ms`, `_spc_cmd`,
`_current_phase`...). Same pattern smaller in `InputManager.cpp:8-22`.
**What:** Give every member its default at the point of declaration in the
header (`bool _prev_was_power = false;` — C++11, already used elsewhere, e.g.
`ShiftScheduler.h:63`). Constructor then only stores the injected pointers.
`begin()` keeps ONLY what genuinely must re-arm on a restart (phase, gear
latch, timestamps that must be "now").

> **Learning note — why Arduino code has both a constructor and `begin()`.**
> Global objects construct *before* `setup()` runs, when hardware (Serial,
> LEDC, NVS, even `millis()`) isn't ready — so hardware setup must wait for an
> explicit `begin()`. That split is for *hardware*, not for plain values.
> Plain values belong on the declaration line: one place to read a member's
> default, impossible to forget one in a ctor, and new members added later
> can't silently start uninitialized (a classic C++ footgun — uninitialized
> members contain garbage, and it "works" until it doesn't).

---

## C. Comment hygiene — comments carry the WHY

### S9 — Drop the per-file `VERSION / UPDATES` changelog headers
**Status:** TODO
**Where:** Top of every file, e.g. `ShiftScheduler.cpp:1-6`, `main.cpp:1-7`,
`InputManager.h:1-14`, `SpeedReader.h:1-19`.
**What:** Replace each with 2–5 lines describing what the module IS and its
non-obvious invariants (keep gems like "pressure-% API is inverted: 100 =
de-energized = max pressure" — that's exactly what a header is for). The
what-changed-in-V6 history is git's job: `git log --oneline -- src/File.cpp`.

> **Learning note — git is the changelog; headers are the orientation.** Hand-
> maintained version banners rot (nobody bumps them consistently — half these
> files say V1.0 with dozens of commits behind them) and they duplicate what
> `git log`/`git blame` answer better, with dates, diffs, and no discipline
> required. What git *cannot* tell a new reader is the module's contract:
> units, inversions, ownership ("Core 0 only"), threading rules. Spend the
> header lines there.

### S10 — Anchor or expand the floating bug-ID references
**Status:** TODO
**Where:** Comments citing `B-4`, `BL-1`, `BL-16`, `V10`, `V16`, `spec §4`…
(e.g. `ShiftScheduler.cpp:71,80`, `InputManager.cpp:125,144`,
`TCU_Data.h:275,296`). These IDs point at documents that aren't in the repo.
**What:** Two options: (a) create `Reference/DESIGN_DECISIONS.md`, move each
ID there with its one-paragraph story, keep the ID in code as a pointer; or
(b) where the story is one sentence, write the sentence inline and drop the
ID. Do NOT delete the reasoning — these comments encode real field lessons
(the N->D-at-speed limp bug, ADC noise false-triggering ROC mode).

> **Learning note — a comment's job is the WHY the code can't show.** "BL-16"
> is a why with the actual explanation stored in a place readers can't reach
> (an old chat, your head). The habit worth keeping from these comments is
> excellent — you consistently record *why* thresholds exist. The upgrade is
> making the reference resolvable from a fresh clone of the repo alone. Test:
> could someone with only this repo reconstruct the reason? If not, the
> knowledge isn't saved yet.

---

## D. Structure — mostly "leave it", with reasons

### S11 — ShiftScheduler.cpp is 1,132 lines — split? **DEFER, then maybe**
**Status:** DEFERRED (until F1–F11 are done)
**What:** The file holds five concerns: the phase engine
(`runShiftPhases`), the policy layer (auto/kickdown/TPS-ROC scheduling),
protection overlays (limp/reverse), the TCC controller, and adaptation glue.
A textbook split would be one `.cpp` per concern (same class — C++ allows
member functions across files), e.g. `ShiftPhases.cpp`, `ShiftPolicy.cpp`,
`TccController.cpp`.
**Why defer:** the F-queue edits these exact lines; refactoring underneath a
safety-fix queue guarantees merge pain and makes every fix diff noisier to
review. And the file is currently *navigable* — good section banners, one
concern per function, clear names.

> **Learning note — refactors have a *when*, not just a *whether*.** "Working
> code you can navigate" beats "beautiful churn". Split a file when you
> actually feel navigation pain or when two people/agents keep colliding in
> it — not because a line count crossed a round number. When you do split,
> split by *concern* (who changes together) not by *size* (arbitrary halves),
> and do it in a commit that changes NOTHING else, so the diff is pure moves
> and trivially reviewable.

### S12 — WebManager's 90-line profile marshalling wall — **keep as is**
**Status:** WONTFIX (deliberate)
**Where:** `WebManager.cpp:96-189` (`get_profile`/`set_profile`).
**What:** Ninety lines of `doc["x"] = p->x` / `if (doc["x"].is<int>()) p->x =
constrain(...)` look like a DRY violation begging for a clever table/macro.
Recommendation: leave it. Each line is independently greppable, has its own
clamp range, and reads top-to-bottom with zero indirection. The "clean"
alternatives (X-macros, field-descriptor tables, templates) would be harder
for you to debug than the repetition is to scroll past.

> **Learning note — DRY has a price called indirection.** The rule "Don't
> Repeat Yourself" is about not duplicating *knowledge* (like S6/S7), not
> about making code visually short. Here each line holds a unique fact (this
> field, this key, this valid range) — there's no shared rule to extract,
> just shared *shape*. Collapsing shape without shared knowledge produces
> abstraction that must be mentally unfolded on every read. The saying:
> "duplication is far cheaper than the wrong abstraction."

### S13 — Small batch: NVS key building + minor tidies
**Status:** TODO
**What (one commit):**
- `AdaptiveMemory.cpp:13,21,38`: `String key = "CLS_" + String(c)` allocates
  heap for a 5-char key on every flush; a `char key[8]; snprintf(key, sizeof
  key, "CLS_%u", c);` does it on the stack. (Matters little here — it runs on
  Core 0, rarely — but heap-free habits are worth building for the 1 kHz side,
  and the review flagged Core-0 heap fragmentation as a slow killer.)
- Fix the `computeLoad()` comment (`TCU_Data.h:414-421`): the "constrained to
  200" is an emergent effect of `loadToBin`'s bin clamp, not a real clamp on
  the returned value. Say what actually happens.

> **Learning note — Arduino `String` vs C arrays.** `String` concatenation
> heap-allocates; on a long-running embedded system, many small alloc/free
> cycles fragment the heap until one day a big allocation (your 30 KB shift
> trace JSON) fails even though "free heap" looks fine — the free space is
> confetti, not a slab. Habit: on hot or long-running paths, prefer
> fixed-size `char` buffers + `snprintf` (which never overflows the buffer
> you give it). `String` is fine in setup-time or rare code.

---

## What was checked and deliberately NOT flagged

Reviewed and judged *earned* complexity — don't "simplify" these:

- **SpeedReader's ISR + ring-buffer machinery** — irreducible; period capture
  with glitch rejection is what makes 0.10-ratio flare detection possible at
  all. The complexity buys a capability.
- **The 20 ms ptick quantizer + 1 kHz exit predicates split** — looks like two
  overlapping timing systems; it's ATSG-correct behavior (pressure amplitude
  changes each 20 ms; detection must not wait 20 ms).
- **Latched at-shift-start snapshots** (`_load_at_start`, `_ratio_old`,
  `_torque_bin`…) — mirrors-with-a-reason: they freeze the decision inputs so
  a mid-shift change can't corrupt an in-flight shift (see S5 note).
- **`TCU_Data.h` as one big shared header** — a purist would split
  pins/thresholds/telemetry into three headers; with one consumer team (you)
  and this size, the single well-sectioned file is easier to search. Revisit
  only if compile times or merge conflicts start to hurt.
- **Seqlock string writers** — the odd/even counter dance looks arcane but is
  the correct minimal tool for cross-core strings without a mutex; the
  comments explain it well. This is a pattern worth *understanding*, not
  removing (look up "seqlock" — you implemented a real kernel technique).

## Done

(nothing yet)
