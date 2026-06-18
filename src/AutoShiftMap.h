// ============================================================================
// FILE: AutoShiftMap.h
// VERSION: 1.0
// Automatic shift schedule: road speed (km/h) × TPS% → shift point.
// Source/derivation: Reference/AUTO_SHIFT_MAP.md (OF Gear-style schedule).
// Index 0..3 = gear pair 1-2/2-1, 2-3/3-2, 3-4/4-3, 4-5/5-4.
// TPS breakpoints: 0,10,20,...,100 (11 columns). Up > down for each pair = hysteresis.
// Values are starting points — tune per car. The per-mode shift_pt_scale stretches
// the whole map (sport = hold gears, comfort = early upshift).
// ============================================================================
#pragma once
#include <Arduino.h>

// Upshift: shift UP when road km/h exceeds the value for the current gear.
const uint16_t AUTO_UPSHIFT_KMH[4][11] = {
  {  6,  7,  8, 10, 13, 16, 20, 24, 29, 37, 45 }, // 1->2
  { 20, 21, 23, 25, 28, 32, 36, 41, 48, 58, 67 }, // 2->3
  { 40, 42, 44, 47, 52, 57, 64, 71, 81, 94,108 }, // 3->4
  { 75, 77, 79, 83, 89, 95,103,111,122,138,154 }, // 4->5
};

// Downshift: shift DOWN when road km/h drops below the value for the current gear.
const uint16_t AUTO_DOWNSHIFT_KMH[4][11] = {
  {  4,  5,  5,  6,  7,  9, 10, 12, 15, 18, 22 }, // 2->1
  { 14, 15, 16, 17, 20, 22, 25, 28, 33, 40, 46 }, // 3->2
  { 35, 36, 38, 40, 43, 46, 51, 55, 62, 71, 80 }, // 4->3
  { 65, 67, 68, 71, 76, 80, 86, 92,101,113,125 }, // 5->4
};

// Linear interpolation of an 11-point row by TPS% (0..100).
inline float autoMapInterp(const uint16_t row[11], float tps) {
    if (tps <= 0.0f)   return (float)row[0];
    if (tps >= 100.0f) return (float)row[10];
    float pos = tps / 10.0f;          // 0..10
    int   i   = (int)pos;             // 0..9
    float fr  = pos - (float)i;
    return (float)row[i] + ((float)row[i + 1] - (float)row[i]) * fr;
}
