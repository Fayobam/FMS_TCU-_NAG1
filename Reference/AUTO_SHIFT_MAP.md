# Auto-Shift Point Map (for the future full-automatic mode)

Source: user's "POTENTIAL AUTOSHIFT POINTS" sheet (OF Gear-style TPS×speed schedule).
Status: **design data, not yet implemented.** The firmware today is paddle-first with
auto-*safety* only (overrev / lug / coast-down / kickdown in `ShiftScheduler`). This map is
the schedule for a future full-auto upshift/downshift mode. Numbers are sensible **starting
points** — tune on the road.

## Units & conversion
Values are **road speed in km/h** (the OF Gear convention). Our firmware works in
**output-shaft rpm**, so an auto implementation converts once with a single per-car constant:

```
output_rpm = kmh * K          K = (final_drive * 1000) / (tyre_circ_m * 60)
kmh        = output_rpm / K
```

For the 190E: measure `final_drive` (≈3.07–3.27 typical) and tyre rolling circumference
(≈1.9 m for 195/65R15), compute `K`, expose it as a calibration value. The existing
`COAST_DN_*` thresholds (output-rpm) would be superseded by the downshift table below when
auto mode is on.

## TPS shaping curve (nonlinear throttle weighting)
Anchors: TPS 25%→7, 50%→25, 75%→50. Interpolated factor (0–100) per TPS breakpoint:

| TPS% | 0 | 10 | 20 | 30 | 40 | 50 | 60 | 70 | 80 | 90 | 100 |
|------|---|----|----|----|----|----|----|----|----|----|-----|
| factor | 0 | 3 | 6 | 11 | 18 | 25 | 35 | 45 | 60 | 80 | 100 |

The per-shift curves below already bake in this shaping (gentle rise to ~50% TPS, steep above).

## UPSHIFT — shift UP when road speed exceeds (km/h)
| TPS% | 1→2 | 2→3 | 3→4 | 4→5 |
|------|-----|-----|-----|-----|
| 0   | 6  | 20 | 40  | 75  |
| 10  | 7  | 21 | 42  | 77  |
| 20  | 8  | 23 | 44  | 79  |
| 30  | 10 | 25 | 47  | 83  |
| 40  | 13 | 28 | 52  | 89  |
| 50  | 16 | 32 | 57  | 95  |
| 60  | 20 | 36 | 64  | 103 |
| 70  | 24 | 41 | 71  | 111 |
| 80  | 29 | 48 | 81  | 122 |
| 90  | 37 | 58 | 94  | 138 |
| 100 | 45 | 67 | 108 | 154 |

## DOWNSHIFT — shift DOWN when road speed drops below (km/h)
| TPS% | 2→1 | 3→2 | 4→3 | 5→4 |
|------|-----|-----|-----|-----|
| 0   | 4  | 14 | 35 | 65  |
| 10  | 5  | 15 | 36 | 67  |
| 20  | 5  | 16 | 38 | 68  |
| 30  | 6  | 17 | 40 | 71  |
| 40  | 7  | 20 | 43 | 76  |
| 50  | 9  | 22 | 46 | 80  |
| 60  | 10 | 25 | 51 | 86  |
| 70  | 12 | 28 | 55 | 92  |
| 80  | 15 | 33 | 62 | 101 |
| 90  | 18 | 40 | 71 | 113 |
| 100 | 22 | 46 | 80 | 125 |

**Hysteresis (anti-hunt):** for every gear pair the downshift speed sits below the upshift
speed, and the gap widens with TPS (e.g. 1↔2 at WOT: up 45 / down 22). Preserve this when
tuning — narrowing it causes gear hunting.

## Ready-to-paste C table
```c
// Auto-shift schedule, km/h vs TPS%. Index 0=1-2/2-1, 1=2-3/3-2, 2=3-4/4-3, 3=4-5/5-4.
static const uint8_t  AUTO_TPS_BP[11]      = {0,10,20,30,40,50,60,70,80,90,100};
static const uint16_t AUTO_UPSHIFT_KMH[4][11] = {
  {  6,  7,  8, 10, 13, 16, 20, 24, 29, 37, 45 }, // 1->2
  { 20, 21, 23, 25, 28, 32, 36, 41, 48, 58, 67 }, // 2->3
  { 40, 42, 44, 47, 52, 57, 64, 71, 81, 94,108 }, // 3->4
  { 75, 77, 79, 83, 89, 95,103,111,122,138,154 }, // 4->5
};
static const uint16_t AUTO_DOWNSHIFT_KMH[4][11] = {
  {  4,  5,  5,  6,  7,  9, 10, 12, 15, 18, 22 }, // 2->1
  { 14, 15, 16, 17, 20, 22, 25, 28, 33, 40, 46 }, // 3->2
  { 35, 36, 38, 40, 43, 46, 51, 55, 62, 71, 80 }, // 4->3
  { 65, 67, 68, 71, 76, 80, 86, 92,101,113,125 }, // 5->4
};
```

## Integration notes (when we build auto mode)
- Gate behind a runtime `auto_mode` flag; paddle requests still override.
- Reuse `beginShift()` — feed it `current_gear±1` when the interpolated threshold is crossed.
- Keep the existing `AUTO_SHIFT_COOLDOWN_MS` (OF Gear field value is ~2 s for auto — anti-hunt;
  see [[ofgear-722-6-field-notes]]).
- Money-shift / overrev guards in `beginShift()` already protect downshifts.
- Lug + overrev safety auto-shifts stay active above this schedule.
- Bilinear: interpolate the chosen gear's row across `AUTO_TPS_BP` by live TPS, compare to road
  speed (output_rpm/K).
