# Implementation Backlog — candidate changes (DO NOT code yet)

Single durable list of things we *might* implement, gathered from the multi-agent code review,
rnd-ash videos, and the auto-shift sheet. **Holding pattern: gathering all transcripts before coding.**
Nothing here is committed to code until explicitly green-lit.

Status: 🔵 proposed (ready) · ⏸ deferred (bench-gated) · 🟡 decision needed · ✅ done · 🚫 dropped
Risk: how much it touches load-bearing/boot/control paths.

## A. Safety / robustness
| ID | Item | Source | Risk | Status |
|----|------|--------|------|--------|
| BL-1 | **Input-shaft trust flag** — in gears 2/3/4 not shifting, `\|N2−N3\| > ~100 RPM` ⇒ bad speed sensor ⇒ suppress false limp | rnd-ash V1 + review | low | ✅ |
| BL-2 | **output==0 downshift guard** — a dead output sensor reads "stopped"; block high-speed downshift instead of allowing it | review | low | ✅ (V23, turbine-independent prediction) |
| BL-3 | **MCPWM init graceful-fail** — replace `ESP_ERROR_CHECK` (reboot loop) with a degraded-mode flag so a capture-alloc failure doesn't brick boot | review (completeness) | med (boot) | ✅ (V23, `_hw_ok`) |
| BL-4 | **RP_LOCK default OFF for first drive** — `ENABLE_RP_LOCK=false` (V23) + polarity CONFIRMED by owner (HIGH=lock) | review | trivial | ✅ |
| BL-5 | **cl_spc first-drive default** — `cl_spc_enable=0` (pure feedforward); EP_MAGIC bumped NAG9→NAGA to re-seed | review | low | ✅ |
| BL-13 | **Battery-voltage feed-forward** on pressure-solenoid duty — we have no current sensing, so open-loop duty→pressure drifts with VIN/coil-temp; compensate duty for measured VIN if a VIN sense exists | rnd-ash V2 | med | 🟡 (needs VIN sense) |
| BL-14 | **Structured DTC logging** — DtcManager (NVS), 8 codes, web get/clear, telemetry health flags | rnd-ash V2 | low | ✅ |

## B. Clutch-speed & shift quality (rnd-ash) — all gated on bench-verifying the clutch-speed model first
| ID | Item | Source | Risk | Status |
|----|------|--------|------|--------|
| BL-6 | **Negative-clutch-speed = flare** — don't `fabsf()` the sign away in fill/overlap; a sign flip is the cleanest flare detector | rnd-ash V1 | low | ⏸ |
| BL-7 | Lower `CLUTCH_MOVE_RPM` 50 → ~20–25 (his early-shift detect = 20) | rnd-ash V1 | low | ⏸ |
| BL-8 | Use computed `turbine_rpm` instead of raw `n3` in the K2/K3/B2 equations (robust if front set unlocks) | rnd-ash V1 | low | ⏸ |
| BL-9 | **TCC pre-fill-to-lockup** once torque transfer completes (off-clutch speed passes on-clutch) — earlier than the fixed post-shift timer | rnd-ash V1 | med | ⏸ |
| BL-10 | Overlap pressure ramp = interpolate vs applied-clutch speed (ease-out shock damping) | rnd-ash V1 | med | ⏸ |
| BL-11 | Torque-cut RELEASE gated on clutch sync, not a pure timer (amplitude ramp = CAN, N/A for us) | rnd-ash V1 | low | ⏸ |
| BL-15 | **Two-stage pre-fill** — high-pressure ramp then low-pressure ramp before the clutch moves on (OEM method), vs our single `_fill_p`/`_fill_t_ms` | rnd-ash V3 | med | ⏸ |
| BL-16 | **Signal-validity → safe substitution** — railed TPS→closed, railed MAP→atmospheric, `tps_valid`/`map_valid` flags | rnd-ash V3 | low | ✅ |

## C. Auto mode
| ID | Item | Source | Risk | Status |
|----|------|--------|------|--------|
| BL-12 | **Full auto upshift/downshift** + **drive-mode gate** (D/4/3 auto, 2/1 manual; lever=mode). Needs per-car `kmh_per_outrpm` set on the dashboard before relying on auto | auto-shift sheet | high | ✅ (bench-tune) |

## D. Done (traceability)
| Item | Commit |
|------|--------|
| P/N from the 4-bit shifter (InputManager side) | 6fe79aa |
| Temp pull-up constant 1K→2K (matches board R9) | 78bb441 |
| Y3/Y4/Y5 kick-and-hold to OEM 80% / 60ms / 37% | 9e1113c |
| Engagement reads the plate range directly (single-sensor) | (pre-existing in ShiftScheduler) |
| BL-1 input-shaft trust flag (N2/N3 mismatch in 2/3/4 → suppress limp) | this batch |
| BL-16 TPS/MAP rail → safe-default substitution + valid flags | this batch |
| BL-2 / BL-3 (output-independent downshift guard / MCPWM fail-soft) | already in V23 (3d35d29) |
| BL-14 DTC logging (DtcManager + web + health flags) | 7bfa179 |
| BL-12 drive-mode gate (lever D/4/3/2/1 = modes) | f6b40c7 |
| BL-12 auto-shift schedule (AutoShiftMap + road km/h) | b695f8e |

---
*Detailed reasoning lives in `UN52_RND_ASH_INSIGHTS.md` (per-video) and the code-review verdict.
Append new candidates here as transcripts are processed.*
