// ============================================================================
// FILE: AdaptiveMemory.cpp
// VERSION: 7.0  (Adaptation v2 — class-indexed, torque-binned)
// ============================================================================
#include "AdaptiveMemory.h"

AdaptiveMemory::AdaptiveMemory() {}

void AdaptiveMemory::begin() {
    preferences.begin("tcu_adapt2", false);   // new namespace — v1 tables are obsolete
    bool blank = false;
    for (uint8_t c = 0; c < ADAPT_CLASSES; c++) {
        String key = "CLS_" + String(c);
        if (preferences.getBytesLength(key.c_str()) != sizeof(_cells[c])) { blank = true; break; }
    }
    if (blank) {
        Serial.println("Adaptation v2: blank/!match flash — loading zero baselines.");
        loadDefaults();
    } else {
        for (uint8_t c = 0; c < ADAPT_CLASSES; c++) {
            String key = "CLS_" + String(c);
            preferences.getBytes(key.c_str(), &_cells[c], sizeof(_cells[c]));
        }
        Serial.println("Adaptation v2 loaded from flash.");
    }
    _last_flush_ms = millis();
}

void AdaptiveMemory::loadDefaults() {
    // Zero = run the baseline phase profiles un-trimmed. The profiles already encode
    // the ATSG starting fill/apply numbers; adaptation only trims from there.
    memset(_cells, 0, sizeof(_cells));
    for (uint8_t c = 0; c < ADAPT_CLASSES; c++) saveClass(c);
}

void AdaptiveMemory::saveClass(uint8_t sclass) {
    if (sclass >= ADAPT_CLASSES) return;
    String key = "CLS_" + String(sclass);
    preferences.putBytes(key.c_str(), &_cells[sclass], sizeof(_cells[sclass]));
}

AdaptCell AdaptiveMemory::getCell(uint8_t sclass, uint8_t shift_idx, uint8_t tbin) {
    sclass    = constrain(sclass,    0, ADAPT_CLASSES - 1);
    shift_idx = constrain(shift_idx, 0, ADAPT_SHIFTS - 1);
    tbin      = constrain(tbin,      0, ADAPT_TBINS - 1);
    return _cells[sclass][shift_idx][tbin];
}

// One update per completed shift. Step sizes ±1 cycle / ±2 % (spec §6). Deadband =
// no detector fired → write nothing. Two-sided on power upshifts so it can't ratchet.
void AdaptiveMemory::learn(uint8_t sclass, uint8_t shift_idx, uint8_t tbin,
                           bool flare, bool harsh, bool bind) {
    if (sclass >= ADAPT_CLASSES || shift_idx >= ADAPT_SHIFTS || tbin >= ADAPT_TBINS) return;
    AdaptCell &cell = _cells[sclass][shift_idx][tbin];
    bool changed = false;

    switch ((ShiftClass)sclass) {
        case SC_POWER_UP:
            if (flare) {                                    // under-filled / slipped → too soft
                cell.fill_t_cycles = clampFillT(cell.fill_t_cycles + 1);   // primary: more fill time
                cell.fill_p_trim   = clampFillP(cell.fill_p_trim   + 2);   // secondary: more fill pressure
                changed = true;
            } else if (harsh) {                             // inertia too short / decel spike → too firm
                cell.apply_trim = clampApply(cell.apply_trim - 2);
                changed = true;
            }
            break;
        case SC_COAST_UP:
            if (flare) { cell.fill_t_cycles = clampFillT(cell.fill_t_cycles + 1); changed = true; }
            break;
        case SC_POWER_DOWN:
            if (bind) { cell.apply_trim = clampApply(cell.apply_trim - 2); changed = true; } // catch shock → softer
            break;
        case SC_COAST_DOWN:
            if (bind) { cell.fill_t_cycles = clampFillT(cell.fill_t_cycles + 1); changed = true; }
            break;
    }

    if (changed) {
        portENTER_CRITICAL(&_dirtyMux);
        _dirty = (uint8_t)(_dirty | (1u << sclass));
        portEXIT_CRITICAL(&_dirtyMux);
    }
}

// Core 0 only (NVS can block 1-10ms). Flush dirty classes on the 60s timer or when
// forced (P/N entry). One class write per call to bound latency.
void AdaptiveMemory::processFlush() {
    bool timer = (millis() - _last_flush_ms > 60000UL);
    if (!_flush_now && !timer) return;

    portENTER_CRITICAL(&_dirtyMux);
    uint8_t mask = _dirty;
    portEXIT_CRITICAL(&_dirtyMux);

    if (mask == 0) { _last_flush_ms = millis(); _flush_now = false; return; }

    for (uint8_t c = 0; c < ADAPT_CLASSES; c++) {
        if (mask & (1u << c)) {
            saveClass(c);
            portENTER_CRITICAL(&_dirtyMux);
            _dirty = (uint8_t)(_dirty & ~(1u << c));
            portEXIT_CRITICAL(&_dirtyMux);
            return;   // one class per call
        }
    }
    _last_flush_ms = millis();
    _flush_now = false;
}

void AdaptiveMemory::markAllDirtyAndFlush() {
    portENTER_CRITICAL(&_dirtyMux);
    _dirty = 0x0F;
    portEXIT_CRITICAL(&_dirtyMux);
    _flush_now = true;
}
