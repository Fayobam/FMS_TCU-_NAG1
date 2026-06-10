# FMS TCU — Master Engineering & Firmware Manual
**Version:** 3.0  
**Application:** Mercedes-Benz 722.6 (W5A330 Small NAG)  
**Powertrain:** M111 2.3L + TVS1320 Supercharger  
**Status:** Bench-ready. Road test pending sensor calibration verification.

---

## 1. System Architecture

Dual-core ESP32 DevKit V1 (240 MHz):

- **Core 1 — Physics Engine (1000 Hz):** Solenoid PWM, shift state machine, speed sensors, planetary kinematics, safety layers, TPS ROC tracking.
- **Core 0 — Web Studio (100 Hz):** Async WebSocket server; streams live telemetry JSON, receives adaptive table updates from dashboard.

The two cores share a single `TCU_Telemetry` struct. Core 1 owns all writes; Core 0 reads only.

---

## 2. Hardware Sensors

### Speed Sensors (PCNT Hardware)

All four sensors use the ESP32 `pulse_cnt.h` hardware peripheral — zero missed pulses at 7000+ RPM.

| Signal | Pin | Teeth | Notes |
|---|---|---|---|
| N2 (internal) | 34 | 60 | K1 clutch drum |
| N3 (internal) | 35 | 60 | K2 clutch drum |
| Output shaft | 32 | 24 | Custom external reluctor |
| Engine RPM | 33 | 60 | M111 crank reluctor wheel (60-tooth) |

**Confirmed by cross-reference with Thomas's dueATC (6 teeth × ÷10 PCB divider = 60 physical) and NAG52.**

### ATF Temperature / P/N Switch Multiplex

Single wire, two signals on `PIN_ATF_TEMP` (pin 39):

- **Voltage > 3.0 V** → P/N switch open → `pn_switch_raw = true`
- **Voltage ≤ 3.0 V** → switch closed, running through PTC thermistor → temperature calculated from resistance

Temperature formula: `temp_c = (resistance_ohms - 800) / 10` — simple linear approximation, clamped −20 to 150°C.

### TPS / MAP

Both on ADC1 pins only — WiFi disables ADC2 entirely.  
Both filtered with EMA α=0.2 at 1kHz (τ ≈ 5ms).

---

## 3. Planetary Kinematics — CONFIRMED FORMULA

**The 722.6 has no physical input shaft sensor.** Turbine RPM is derived from N2 and N3.

### Authoritative Formula (NAG52 `RATIO_2_1`)

```
turbine_rpm = (N2 × K) − (N3 × (K − 1))     K = 1.61  (N2_N3_BLEND_K in TCU_Data.h)
```

**Verification:**
- N3 = 0 (gears 1 & 5 — N3 drum parked): collapses to `N2 × 1.61` ✓  
- N2 = N3 (gear 3 — K1+K2 lock both drums to shaft): collapses to `N2 × 1.0` ✓

**Single formula, all gears, no gear-position dependency.**  
K = 1.61 is a calibration constant (same for Small NAG and Big NAG — shared front planetary).  
Adjust `N2_N3_BLEND_K` on bench if turbine RPM ≠ engine RPM with TCC locked.

**What was wrong before:** The formula was written as `(N3 × 1.48) − (N2 × 0.48)` — N2 and N3 swapped. In 1st gear (N3 ≈ 0) this gave a negative turbine RPM, clamped to zero, so the limp mode detector saw a 300+ RPM mismatch on every startup.

The gear 5 constant `N2 × 0.83` was also wrong — `0.83` is the gear ratio, not the sensor constant.

---

## 4. Transmission Gear Ratios

### W5A330 Small NAG (THIS BUILD)

| Gear | Ratio |
|---|---|
| 1st | 3.932 |
| 2nd | 2.408 |
| 3rd | 1.486 |
| 4th | 1.000 |
| 5th | 0.830 |
| Rev | 3.100 |

### W5A580 Big NAG (for reference — swap in TCU_Data.h if needed)

| Gear | Ratio |
|---|---|
| 1st | 3.595 |
| 2nd | 2.186 |
| 3rd | 1.405 |
| 4th | 1.000 |
| 5th | 0.831 |

**If swapping to Big NAG:** also increase prefill timing defaults (K2 needs ~220ms vs 160ms) due to larger clutch volume. `N2_N3_BLEND_K = 1.61` stays the same.

---

## 5. Solenoid Routing — CONFIRMED CORRECT

Cross-referenced against Thomas's dueATC (`SOL_12_45 / SOL_23 / SOL_34`):

| Solenoid | Pin | Handles |
|---|---|---|
| Y3 | 14 | 1↑2, 4↑5, 5↓4, 2↓1 |
| Y5 | 19 | 2↑3, 3↓2 |
| Y4 | 18 | 3↑4, 4↓3 |

### Kick-and-Hold Driver

4-ohm coils burn up at continuous 12V. The firmware blasts **83% PWM for 60ms** (snap open), then drops to **33% PWM** (hold without overheating). Simple `digitalWrite`-equivalent solenoids — hydraulically on or off.

### Pressure Solenoids (Inverted Logic)

`0% commanded = 255 PWM duty = maximum hydraulic pressure`  
`100% commanded = 0 PWM duty = zero hydraulic pressure`

MPC (pin 26): line/holding pressure. SPC (pin 25): shift pressure. TCC (pin 27): normal logic.

---

## 6. Drive Engagement Sequence

### In P/N

Y4 continuously pulsed at **37% PWM** (garage shift jiggle) — buffers the 3-4 valve body so Drive engagement is smooth.

### Shifting to D

The garage shift fires on the **falling edge of `pn_switch_raw`** — the instant the physical P/N switch opens as the manual valve moves. This is independent of the 4-bit PRND decoder settling.

**The 722.6 hydraulic default is 2nd gear.** The firmware acknowledges this reality:
- `current_gear = 2`, `target_gear = 2` on engagement
- Y3 is NOT fired — no automatic shift to 1st
- The driver selects 1st via paddle-down if desired

This is correct for a manual-paddle transmission. Attempting to force 1st at standstill via the downshift pipeline has ratio-based exit conditions that don't function at zero output RPM.

Returning to P/N: `drive_engaged` resets, Y4 jiggle resumes, edge detector re-arms.

---

## 7. Shift Execution Pipeline

### Upshift State Machine

| Phase | SPC | Line (MPC) | Exit condition |
|---|---|---|---|
| PHASE_CRUISING | 0% | HOLDING_MAP | Paddle-up OR auto-safety |
| PHASE_PREFILL | 80% (fill fast) | Elevated | `60 + timing_mod` ms elapsed |
| PHASE_OVERLAP | 45% → ramp to 90% | Elevated | `live_ratio < current_ratio − 0.1` (clutch biting) |
| PHASE_INERTIA | 90% → ramp to 100% | Elevated | `live_ratio ≤ target_ratio + 0.05` OR 600ms timeout |
| PHASE_COMPLETION | 0% | HOLDING_MAP | 50ms settle → adaptive evaluate |

### Downshift State Machine

| Phase | SPC | Exit condition |
|---|---|---|
| PHASE_DS_RELEASE | 0% | `80 + timing_mod` ms — engine rev-match window |
| PHASE_DS_SYNC | 0% | `live_ratio ≥ target_ratio − 0.05` OR 400ms timeout |
| PHASE_DS_CATCH | Ramp to `90 + pressure_mod` | `live_ratio ≥ target_ratio − 0.05` OR 400ms timeout |
| PHASE_COMPLETION | 0% | 50ms settle → adaptive evaluate |

**Key fix:** DS_SYNC threshold was `target − 0.2` which was satisfied immediately on entry, skipping the entire rev-match wait. Fixed to `target − 0.05`.

### Money-Shift Guard

`predicted_rpm = output_rpm × target_gear_ratio`  
If predicted > **6000 RPM** → downshift refused, regardless of source (paddle or auto-safety).

---

## 8. Hydraulic Pressure Strategy

### Holding Pressure (HOLDING_PRESSURE_MAP)

Performance-biased for M111 + TVS1320. Floor raised to minimum 20–38%, boost-zone bins (4–8) pushed hard. The supercharger makes significant torque even at moderate throttle — even light-load bins carry real torque risk.

### Line Pressure During Shifts

```
target = tps_pct + (map_kpa − 100) × 2.0 + 20
```

Applied across all shift phases to ensure adequate clamping during the transition.

### ATF Temperature Compensation (CORRECTED)

The old code incorrectly *reduced* pressure when cold. Cold ATF is thick and slow-filling; hot ATF leaks past seals. Both directions need MORE pressure, not less.

| ATF temp | Multiplier | Reason |
|---|---|---|
| < 20°C | × 1.30 | Very viscous, slow fill |
| 20–40°C | × 1.15 | Cold |
| 40–80°C | × 1.00 | Normal operating range |
| 80–110°C | × 1.05 | Minor seal leakage |
| > 110°C | × 1.20 | Significant leakage |

---

## 9. Supercharger-Specific Load Model

### computeLoad()

```
load = (tps_pct × 1.25) + max(0, map_kpa − 100) × 0.8
```

The `× 1.25` TPS weighting compensates for MAP sensor lag (~100–200ms manifold fill time). On the TVS1320, torque tracks throttle mechanically — the MAP reading lags the actual torque delivery. TPS is forward-weighted so the load index responds as fast as the throttle moves.

At WOT + 1.2 bar boost: load ≈ 125 + 96 = 221 → constrained to bin 15 by `loadToBin()`.

### TPS Rate-of-Change (ROC) Torque Anticipation

The TVS1320 has no spool lag — torque arrives with throttle, not with MAP settling. A fast tip-in means torque is arriving NOW.

**Trigger:** `dTPS/dt > 0.15 %/ms` (≈ full throttle in under 700ms)  
**Effect:** Immediately → 100% line pressure + TCC forced open  
**Hold:** While TPS > 40% OR ROC still positive  
**Release:** TPS < 40% AND ROC < 0.02 %/ms → start 2-second cooldown  
**Exit:** Cooldown expires → normal MAP/TPS-based control resumes

The cooldown prevents pressure from dropping immediately after the tip-in while the clutches are still loaded.

Dashboard indicator: **Torque Mode** (orange = ACTIVE) in the Adaptive Metrics panel.

Calibration constants in `TCU_Data.h`:
- `TPS_ROC_TRIGGER_PCT_MS = 0.15f`  
- `TPS_ROC_RELEASE_HOLD = 40.0f`  
- `TPS_ROC_COOLDOWN_MS = 2000`

---

## 10. TCC (Torque Converter Clutch) Strategy

| Condition | Target slip | Behaviour |
|---|---|---|
| High-torque mode (ROC) | 500 RPM | Force open preemptively |
| MAP > 105 kPa OR TPS > 45% | 500 RPM | Open for boost/demand |
| RPM < 1400 OR gear 1 | 1000 RPM | Open to prevent stall |
| Normal cruise | 50 RPM | PID lock-up control (max 85%) |
| During any shift phase | 1000 RPM | Always open during shifts |

**TPS threshold reduced from 75% → 45%** because the supercharger makes peak torque from any throttle position above ~45%. At 75% the TCC would stay locked through the most dangerous part of a throttle transition.

---

## 11. Adaptive Memory System

### Structure

Four `int8_t` tables, each `[4 shifts][16 load bins][16 RPM bins]`:

| Table | Key | Content |
|---|---|---|
| `P_TBL_x` | `pressure_modifiers` | Upshift overlap SPC % offset |
| `F_TBL_x` | `prefill_modifiers` | Upshift fill time ms offset |
| `DP_TBL_x` | `ds_pressure_modifiers` | Downshift catch SPC % offset |
| `DT_TBL_x` | `ds_timing_modifiers` | Downshift rev-match window ms offset |

Persisted to ESP32 NVS flash via `Preferences`. Loaded on boot; re-flashed after every learning event.

### Clamp Ranges

| Type | Min | Max | Rationale |
|---|---|---|---|
| Pressure modifiers | −30% | +60% | Tight range; small corrections only |
| Timing modifiers | −30ms | +120ms | Wide range; K2 needs up to 180ms fill |

### Default Baselines (W5A330 / performance build)

**Upshift prefill timing** (base 60ms + modifier):

| Shift | Modifier | Total | Clutch |
|---|---|---|---|
| 1→2 | +30ms | 90ms | B1 brake band (small volume) |
| 2→3 | +50ms | 110ms | K1 clutch (medium volume) |
| 3→4 | +100ms | 160ms | K2 clutch (large volume, flare-prone) |
| 4→5 | +20ms | 80ms | B1 brake band |

**Downshift rev-match timing** (base 80ms + modifier):

| Shift | Modifier | Total |
|---|---|---|
| 2→1 | +20ms | 100ms |
| 3→2 | +50ms | 130ms |
| 4→3 | +100ms | 180ms |
| 5→4 | +120ms | 200ms |

**Pressure defaults:** `load_bin × 4` (0→60% across 16 bins). K2 upshift and 4↓3 / 5↓4 downshifts get +10% and +5% extra respectively.

### Learning Logic

- **Flare detected:** `pressure_mod += 2` (clutch slipped — more pressure needed)
- **Bind detected (upshift):** `pressure_mod −= 2` (too harsh — less pressure)
- **Bind detected (downshift):** `timing_mod += 5` (catch applied too early — more rev-match time)
- Nudge amounts are small; convergence takes 20–50 shifts per operating cell.

**Important:** New defaults only load on blank flash. If adaptive tables exist in NVS from a previous version, run **Erase Flash** in PlatformIO before first bench test to load the updated baselines.

---

## 12. Safety Systems

### Auto-Safety Shifts

Both pass through the money-shift guard before executing.

- **Overrev upshift:** engine RPM > 6300 → force upshift (unless already 5th gear)
- **Lug downshift:** RPM < 1100 AND TPS > 25% AND gear > 1 → force downshift
- **Cooldown:** 500ms between consecutive auto-safety shifts

### Limp Mode

Monitors while cruising at moderate load (`tps < 80%, map < 130 kPa, output_rpm > 200`).  
`mismatch = |turbine_rpm − (output_rpm × target_ratio)|`  
If mismatch > 300 RPM sustained for 400ms → limp mode:
- 100% line pressure
- 0% shift pressure  
- All routing solenoids off (hydraulic default = 2nd gear)
- TCC off
- Dashboard FATAL code

**Reset:** Web command `limp_reset` accepted only when `output_rpm < 50` AND in P/N.

---

## 13. Calibration Values — VERIFY BEFORE ROAD TEST

| Item | File | Status |
|---|---|---|
| `PIN_MAP = 33`, `PIN_ENG = 23` | TCU_Data.h | **HARDWARE REWIRE REQUIRED** — GPIO 23 has no ADC channel; MAP moved to GPIO 33 (ADC1_CH5), engine tach moved to GPIO 23 (PCNT only needs GPIO) |
| `MAP_KPA_AT_0V`, `MAP_KPA_PER_VOLT` | TCU_Data.h | **Placeholder** — measure your sensor's transfer function |
| `TPS_VOLTS_CLOSED`, `TPS_VOLTS_WOT` | InputManager.h | **Placeholder** — measure at pedal stops on the car |
| `N2_N3_BLEND_K = 1.61` | TCU_Data.h | **Verify on bench** — with TCC locked, turbine should equal engine RPM ±20 RPM |
| `RPM_OVERREV_UPSHIFT = 6300` | TCU_Data.h | **Sanity check** — appropriate for M111 + TVS1320 redline |
| `RPM_LUG_THRESHOLD = 1100` | TCU_Data.h | **Sanity check** — confirm stall speed with blower |
| Prefill timing defaults | AdaptiveMemory.cpp | Conservative starting point — adaptive will tune |
| Pressure map values | ShiftScheduler.h | Performance-biased — adaptive will pull back if bind |

### Bench Test Protocol (before any vehicle test)

1. Erase flash → confirms new adaptive defaults load
2. Signal generator on all 4 speed sensors → verify RPM math and turbine formula
3. Signal generator through P→D transition → confirm P/N edge detection fires garage acknowledgement correctly (gear=2, no Y3 fire)
4. Simulate fast TPS ramp → confirm Torque Mode indicator activates on dashboard
5. Simulate 300+ RPM turbine mismatch sustained 400ms → confirm limp mode fires
6. Test limp reset via dashboard while stationary and in P/N
7. Verify overrev upshift fires at 6300 RPM simulated engine speed
8. Monitor Serial for "WARNING: Telemetry JSON truncated" — should never appear

---

## 14. Web Dashboard

- **WiFi AP:** `FMS_TCU` / `shiftfast`
- **Address:** `http://192.168.4.1`
- **Telemetry:** 100Hz WebSocket JSON stream
- **Calibration Studio:** Fetch/flash any of the 8 adaptive tables (4 upshift × 4 downshift, pressure or timing) as a live 16×16 heatmap grid

**To update index.html:** PlatformIO → Upload Filesystem Image (separate from firmware flash). Always close Serial Monitor first.
