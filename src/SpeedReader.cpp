// ============================================================================
// FILE: SpeedReader.cpp
// VERSION: 3.0
// UPDATES: MCPWM-capture period measurement (see header). ISR work is minimal:
//          one delta, one plausibility compare, one ring write. All averaging
//          happens in update() on Core 1 task context.
// ============================================================================
#include "SpeedReader.h"
#include "EngineProfile.h"
#include "esp_timer.h"

SpeedReader::SpeedReader(uint8_t pin_n2, uint8_t pin_n3, uint8_t pin_out, uint8_t pin_eng) {
    _pin_n2 = pin_n2;
    _pin_n3 = pin_n3;
    _pin_out = pin_out;
    _pin_eng = pin_eng;
}

// ----------------------------------------------------------------------------
// Capture ISR — hardware-timestamped rising edge. Glitch policy: an interval
// shorter than the channel's plausibility floor is discarded WITHOUT advancing
// last_cap, so a noise edge splits one real interval into (rejected tiny) +
// (remainder = real) instead of injecting a huge rpm spike.
// ----------------------------------------------------------------------------
static bool IRAM_ATTR onCaptureISR(mcpwm_cap_channel_handle_t chan,
                                   const mcpwm_capture_event_data_t *edata,
                                   void *user_ctx) {
    SpeedChannel *ch = (SpeedChannel *)user_ctx;
    uint32_t now = edata->cap_value;

    if (!ch->has_last) {
        ch->last_cap = now;
        ch->has_last = true;
        ch->last_edge_us = esp_timer_get_time();
        return false;
    }

    uint32_t delta = now - ch->last_cap;          // uint32 wrap-safe
    if (delta < ch->min_period_ticks) return false;  // glitch — drop the edge

    ch->ring[ch->head] = delta;
    ch->head = (ch->head + 1) & SR_RING_MASK;
    if (ch->count < SR_RING_N) ch->count = ch->count + 1;
    ch->last_cap = now;
    ch->last_edge_us = esp_timer_get_time();
    ch->edge_count++;                              // signal a genuinely-new edge landed
    return false;
}

void SpeedReader::initChannel(mcpwm_cap_timer_handle_t timer, SpeedChannel &ch, uint8_t pin) {
    mcpwm_capture_channel_config_t cconf = {};
    cconf.gpio_num = pin;
    cconf.prescale = 1;
    cconf.flags.pos_edge = true;
    cconf.flags.neg_edge = false;
    cconf.flags.pull_up  = false;
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(timer, &cconf, &ch.cap_chan));

    mcpwm_capture_event_callbacks_t cbs = {};
    cbs.on_cap = onCaptureISR;
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(ch.cap_chan, &cbs, &ch));
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(ch.cap_chan));
}

// PPR-dependent math: glitch floor, full-rev window, zero-speed timeout.
// Cheap (no hardware touch) so web PPR edits hot-apply from update().
void SpeedReader::configChannel(SpeedChannel &ch, float ppr, float max_rpm) {
    ch.ppr = (ppr >= 1.0f) ? ppr : 1.0f;
    ch.full_rev_intervals = (uint16_t)constrain((int)(ch.ppr + 0.5f), 1, SR_RING_N);
    // floor: period of one tooth at max plausible rpm
    float max_tooth_hz = (max_rpm / 60.0f) * ch.ppr;
    ch.min_period_ticks = (uint32_t)((float)_cap_res_hz / max_tooth_hz);
    // timeout: period of one tooth at the zero-speed floor, capped
    float floor_tooth_hz = (SR_ZERO_FLOOR_RPM / 60.0f) * ch.ppr;
    int64_t to = (int64_t)(1000000.0f / floor_tooth_hz);
    ch.zero_timeout_us = (to > (int64_t)SR_ZERO_TIMEOUT_MAX_US) ? SR_ZERO_TIMEOUT_MAX_US : to;
}

void SpeedReader::begin() {
    // One capture timer per MCPWM group; ESP32 classic = 2 groups × 3 channels.
    // Group 0 carries N2/N3/OUT, group 1 carries ENG.
    mcpwm_capture_timer_config_t tconf = {};
    tconf.group_id = 0;
    tconf.clk_src  = MCPWM_CAPTURE_CLK_SRC_DEFAULT;
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&tconf, &_cap_timer_g0));
    tconf.group_id = 1;
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&tconf, &_cap_timer_g1));
    ESP_ERROR_CHECK(mcpwm_capture_timer_get_resolution(_cap_timer_g0, &_cap_res_hz));

    initChannel(_cap_timer_g0, _n2,  _pin_n2);
    initChannel(_cap_timer_g0, _n3,  _pin_n3);
    initChannel(_cap_timer_g0, _out, _pin_out);
    initChannel(_cap_timer_g1, _eng, _pin_eng);

    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(_cap_timer_g0));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(_cap_timer_g0));
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(_cap_timer_g1));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(_cap_timer_g1));

    _last_eng_ppr = engineProfile.engPpr();
    _last_out_ppr = engineProfile.outPpr();
    configChannel(_n2,  TEETH_N2,      SR_MAX_RPM_N2);
    configChannel(_n3,  TEETH_N3,      SR_MAX_RPM_N3);
    configChannel(_out, _last_out_ppr, SR_MAX_RPM_OUT);
    configChannel(_eng, _last_eng_ppr, SR_MAX_RPM_ENG);

    Serial.printf("Speed Reader V3.0 (MCPWM period capture @ %lu Hz; ENG %u PPR, OUT %u PPR).\n",
                  (unsigned long)_cap_res_hz, _last_eng_ppr, _last_out_ppr);
}

// ----------------------------------------------------------------------------
// Average the newest intervals: walk back from head until we have a full
// revolution (tooth-spacing error cancels exactly — same tooth set every rev)
// OR the span exceeds SR_AVG_WINDOW_US (bounds latency during hard decel/crawl).
// The still-open interval (time since the last edge) clamps the result downward
// so a hard stop reads down between edges instead of holding the last speed.
// ----------------------------------------------------------------------------
float SpeedReader::readChannelRPM(SpeedChannel &ch) {
    int64_t now_us = esp_timer_get_time();

    // Zero-speed: no edge for longer than one tooth at the floor rpm.
    if (!ch.has_last || (now_us - ch.last_edge_us) > ch.zero_timeout_us) {
        ch.has_last = false;     // restart cleanly; stale intervals must not resurface
        ch.count = 0;
        return 0.0f;
    }

    // Snapshot ISR state. The ISR runs on this same core, so it can only run
    // between our instructions — copy indices first, then the entries.
    uint16_t head, count;
    portDISABLE_INTERRUPTS();
    head  = ch.head;
    count = ch.count;
    portENABLE_INTERRUPTS();
    if (count == 0) return 0.0f;

    uint32_t window_ticks = (uint32_t)(((uint64_t)SR_AVG_WINDOW_US * _cap_res_hz) / 1000000ULL);
    uint64_t span = 0;
    uint16_t k = 0;
    while (k < count && k < ch.full_rev_intervals) {
        uint16_t idx = (uint16_t)((head - 1 - k) & SR_RING_MASK);
        uint32_t iv = ch.ring[idx];
        if (k > 0 && span + iv > window_ticks) break;   // keep ≥1 interval, cap the span
        span += iv;
        k++;
    }
    if (k == 0 || span == 0) return 0.0f;

    float avg_ticks = (float)span / (float)k;
    float rpm = (60.0f * (float)_cap_res_hz) / (ch.ppr * avg_ticks);

    // Open-interval clamp: if the gap since the last edge already exceeds the
    // average interval, the shaft has slowed — believe the open gap instead.
    float open_us = (float)(now_us - ch.last_edge_us);
    float avg_us  = avg_ticks * (1000000.0f / (float)_cap_res_hz);
    if (open_us > avg_us) {
        float rpm_open = 60.0f * 1000000.0f / (ch.ppr * open_us);
        if (rpm_open < rpm) rpm = rpm_open;
    }
    return rpm;
}

// ============================================================================
// 722.6 PLANETARY KINEMATICS
// ============================================================================
float SpeedReader::calculateTurbineRPM(float n2_rpm, float n3_rpm) {
    // NAG52 unified formula — one expression, all gears, no gear-position dependency:
    //   turbine = (N2 * K) - (N3 * (K-1))   K = N2_N3_BLEND_K (1.641, tooth-derived, small NAG)
    // Verify: N3=0 (gears 1&5) → N2*K; N2=N3 (3rd gear direct) → N2*1.0 ✓
    float turbine = (n2_rpm * N2_N3_BLEND_K) - (n3_rpm * (N2_N3_BLEND_K - 1.0f));
    return fmaxf(0.0f, turbine);
}

void SpeedReader::update() {
    // Called every 1ms. We recompute every tick so the open-interval clamp in
    // readChannelRPM() tracks hard deceleration / low-speed shafts (sparse edges)
    // at full loop resolution — the steady-state average only changes when a new
    // edge lands, but the between-edge decel estimate refreshes each tick.

    // Hot-apply PPR edits from the web tuner (math-only reconfig, no hardware touch).
    uint16_t eng_ppr = engineProfile.engPpr(), out_ppr = engineProfile.outPpr();
    if (eng_ppr != _last_eng_ppr) { configChannel(_eng, eng_ppr, SR_MAX_RPM_ENG); _last_eng_ppr = eng_ppr; }
    if (out_ppr != _last_out_ppr) { configChannel(_out, out_ppr, SR_MAX_RPM_OUT); _last_out_ppr = out_ppr; }

    float n2 = readChannelRPM(_n2);
    float n3 = readChannelRPM(_n3);
    telemetry.output_rpm  = readChannelRPM(_out);
    telemetry.engine_rpm  = readChannelRPM(_eng);
    telemetry.turbine_rpm = calculateTurbineRPM(n2, n3);

    // Bump the sample sequence ONLY when a real new edge advanced a RATIO channel
    // (N2/N3/OUT — they drive live_ratio). Engine-only edges must NOT trip the phase
    // engine's ratio-derivative gate (B-4): a frozen ratio would read "flat" trivially.
    // So speed_sample_seq means "new ratio data", not "recomputed".
    uint32_t ratio_edges = _n2.edge_count + _n3.edge_count + _out.edge_count;
    if (ratio_edges != _last_ratio_edges) {
        _last_ratio_edges = ratio_edges;
        telemetry.speed_sample_seq++;
    }
}
