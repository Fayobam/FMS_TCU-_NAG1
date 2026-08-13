// ============================================================================
// FILE: ShiftScheduler.h
// VERSION: 6.0
// UPDATES:
//   - Added auto-safety layer (overrev upshift + lug downshift).
//   - Added flare detection during upshift inertia phase.
//   - Centralised shift initiation in beginShift() to remove duplicated,
//     off-by-one trigger code.
//   - Limp mode now has a deliberate reset path and load-aware threshold.
// ============================================================================
#pragma once
#include <Arduino.h>
#include "TCU_Data.h"
#include "SolenoidDriver.h"
#include "AdaptiveMemory.h"

// Generic phase set shared by all four shift classes (ATSG-grounded spec §4).
// Upshift path:   PREP → FILL → TORQUE → INERTIA → LOCK → END
// Downshift path: PREP → RELEASE → CATCH → LOCK → END  (PD_SPRAG skips LOCK timing-wise)
enum ShiftPhase {
    PHASE_CRUISING,
    PHASE_PREP,
    PHASE_FILL,       // upshift: stroke oncoming piston (ratio must not move)
    PHASE_TORQUE,     // upshift: oncoming takes torque while off-going releases via overlap
    PHASE_INERTIA,    // upshift: ramp clutch, pull ratio to target
    PHASE_RELEASE,    // downshift: off-going exhausts, engine flares turbine toward sync
    PHASE_CATCH,      // downshift: clamp oncoming as sync approaches (sprag = secure brake)
    PHASE_LOCK,       // both: seat the clutch at full apply
    PHASE_END         // both: decay line to cruise, adapt, latch
};

class ShiftScheduler {
// Host tests (pio test -e native) drive the internals directly so pure helpers
// — the routing table, the ratio classifier — can be asserted without faking a
// whole driving scenario. UNIT_TEST is defined ONLY by the native env; the
// firmware build is unchanged.
#ifdef UNIT_TEST
  public:
#else
  private:
#endif
    SolenoidDriver* _solenoids;
    AdaptiveMemory* _adaptives;

    ShiftPhase _current_phase;
    TickType_t _phase_start_tick;
    uint8_t _active_routing_pin;
    bool _is_upshift;
    uint8_t _active_shift_idx;      // adaptive table index for THIS shift (upshift idx / target-1)

    // --- Active shift classification & profile (computed at beginShift) ---
    ShiftClass    _sclass;
    PowerDownType _pd_type;
    bool          _prev_was_power;  // class hysteresis between the POWER/COAST thresholds
    uint8_t  _from_gear;
    float    _load_at_start;        // load_pct (0-100, torque-based) captured at initiation
    float    _input_at_start;       // input/turbine torque (Nm) at initiation — pressure model (Phase 3)
    uint8_t  _torque_bin;           // 4-wide torque bin captured at initiation (adaptation key)
    float    _ratio_old;            // source-gear ratio
    float    _ratio_target;         // target-gear ratio
    // profile scalars (filled by classifyAndProfile)
    uint8_t  _fill_p;  uint16_t _fill_t_ms;
    uint8_t  _apply_pct;
    float    _inertia_slope;  uint16_t _inertia_target_ms;
    uint8_t  _release_spc;  uint16_t _release_backstop_ms;
    uint8_t  _catch_start_spc;  float _catch_slope;

    // --- 20ms pressure-update quantizer (ATSG p.80) ---
    unsigned long _last_pressure_update_ms;
    float    _spc_cmd;              // fractional SPC accumulator so ramps are smooth across 20ms ticks
    unsigned long _tcc_reopen_until_ms = 0; // hold TCC fully open until this time (post-shift)

    // --- Detection state ---
    unsigned long _shift_stopwatch_start;
    float    _turbine_rpm_at_shift_start;
    float    _output_rpm_at_shift_start;
    float    _prev_ratio;              // ratio at the PREVIOUS speed sample (sprag dRatio/dt)
    bool     _ratio_flat = false;      // |Δratio| over the last sample < flat band (held between samples)
    uint32_t _last_speed_seq = 0;      // last speed_sample_seq the engine acted on (B-4)
    unsigned long _sync_stable_since_ms; // when ratio first parked at target (sprag/timed catch)
    float    _output_rpm_at_catch_start; // coast-down decel-delta metric baseline
    unsigned long _catch_start_ms;
    float    _ds_baseline_decel_rate;
    bool     _harsh_detected;          // upshift inertia too short / decel spike
    uint16_t _flare_over_ms;           // consecutive ms the flare condition has held
    uint16_t _bind_over_ms;            // consecutive ms the bind condition has held
    float    _cl_err = 0.0f;           // closed-loop SPC schedule error (INERTIA; 0 elsewhere)
    unsigned long _n2n3_fault_since_ms = 0; // when |N2−N3| first exceeded the trust band in 2/3/4 (BL-1)

    bool  _prev_pn_raw;             // edge-detect for the engagement (lever) window
    unsigned long _engage_grace_until_ms; // suppress slip-limp during D-engagement sync
    bool  _gear_resync_pending;     // gear label unverified → re-classify from ratio
    unsigned long _resync_ready_ms; // when the pending resync may run (settle deadline)
    char  _prev_prnd;              // edge-detect for reverse selection
    bool  _legit_reverse;         // R was selected while stopped → genuine reverse, allow any speed
    unsigned long _slow_since_ms = 0; // when output dropped below REVERSE_INHIBIT_SPEED_RPM (0 = above)

    // TPS rate-of-change torque anticipation (20ms windowed ROC)
    float         _tps_hist[TPS_ROC_WINDOW_MS];
    uint8_t       _tps_hist_idx;         // points at the OLDEST sample
    bool          _tps_hist_primed;
    bool          _high_torque_mode;
    unsigned long _ht_release_start_ms;  // 0 = not in cooldown

    // Engine rpm rate-of-change (rpm/s, EMA-smoothed) for predictive overrev
    float         _eng_rpm_prev_sample;
    unsigned long _eng_roc_sample_ms;
    float         _eng_rpm_per_s;

    // Cruise (holding) line pressure now lives in the NVS-backed, web-editable
    // TuneOverlay (tuneOverlay.lineMap(gear_idx, load_idx)). Its compile-time
    // defaults — and the dueATC provenance for them — are in TuneOverlay.cpp.

    void calculateLinePressure();             // CRUISING line pressure (holding map + ATF)
    float cruiseLinePressure();               // the cruise MPC value, for max(cruise, …) during shifts
    void calculateLiveRatio();
    void computeClutchSpeeds();   // UN52 clutch-speed model: on/off-clutch slip from N2/N3/out
    void updateTCC(bool ptick);
    void checkSafetyShifts();
    void checkAutoShift();                    // full auto up/down schedule (AUTO_SHIFT_MAP × road km/h)
    void checkLaunchGear();                   // drop a stopped car to its launch gear (1st), all modes
    void checkKickdown();                     // power-down request on hard tip-in
    void checkLimpMode(float target_ratio);
    void checkTpsROC();                       // TPS rate-of-change torque anticipation
    bool checkReverseInhibit();               // RP_LOCK + R-while-moving failsafe (returns true if it owns outputs)
    bool isForwardRange();                    // prnd is one of D/4/3/2/1
    void updateStandbyAndGarage();            // SPC/MPC standby duties + Y4 garage window (not shifting)
    bool beginShift(uint8_t target_gear, bool is_upshift, const char* source);
    void classifyAndProfile(uint8_t from, uint8_t to, bool is_upshift);  // class + profile scalars
    void runShiftPhases(unsigned long t_ms, bool ptick, bool new_sample);  // the class-aware phase engine
    void captureTrace();                      // high-rate datalog sample (bench tuning)
    void setSPC(float pct);                   // write _spc_cmd + command solenoid
    void applyShiftMPC();                     // MPC rule during a shift (per class/load)
    uint16_t phaseBackstopMs() const;         // INERTIA/CATCH timeout, ATF-scaled
    bool shiftProvedByRatio() const;          // did live_ratio actually demonstrate the target?
    void finishShift();                       // ratio proved the target: latch gear, adapt
    void abandonShift(const char* why);       // backstop hit unproven: keep label, force resync
    void evaluateAdaptation();                // class-indexed learning (Phase 5)
    uint8_t classGearFromRatio();             // nearest-ratio gear classifier (limp/abort)
    float getTargetRatio(uint8_t gear);
    uint8_t getRoutingSolenoidForShift(uint8_t from_gear, uint8_t to_gear);
    DriveMode currentMode() const;   // by value: knobs come from the NVS overlay     // lever-selected drive-mode knob bundle

  public:
    ShiftScheduler(SolenoidDriver* solenoids, AdaptiveMemory* adaptives);
    void begin();
    void update();
};
