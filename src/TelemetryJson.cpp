#include "TelemetryJson.h"
#include "TelemetryConfig.h"

void fillTelemetryJson(JsonDocument& doc, const ControlSnapshot& snap,
                       uint32_t clients, uint32_t freeHeap, bool filesystemOk) {
    const auto& telemetry = snap.data;
    char prnd[2] = { telemetry.prnd_state, 0 };
    doc["type"]      = "telemetry";
    doc["prnd"]      = JsonString(prnd);
    doc["atfOnly"] = telemetry.atf_only_selector;
    doc["forwardConfirmed"] = telemetry.atf_forward_confirmed;
    doc["atfRange"] = telemetry.atf_range_evidence;
    doc["gear"]      = telemetry.current_gear;
    doc["tgt"]       = telemetry.target_gear;
    doc["mode"]      = telemetry.drive_mode;
    doc["modeName"]  = telemetry.atf_only_selector ? "ATF ONLY / MANUAL" : DRIVE_MODES[telemetry.drive_mode <= 4 ? telemetry.drive_mode : 0].name;
    doc["engRpm"]    = telemetry.engine_rpm;
    doc["turbRpm"]   = telemetry.turbine_rpm;
    doc["outRpm"]    = telemetry.output_rpm;
    doc["tps"]       = telemetry.tps_pct;
    doc["map"]       = telemetry.map_kpa;
    doc["mpc"]       = telemetry.line_pressure_pct;
    doc["spc"]       = telemetry.shift_pressure_pct;
    doc["shiftTime"] = telemetry.last_shift_time_ms;
    doc["ratio"]     = telemetry.live_ratio;
    doc["kmh"]       = telemetry.road_kmh;
    doc["flare"]     = telemetry.flare_detected;
    doc["bind"]      = telemetry.bind_detected;
    doc["tccPwm"]    = telemetry.tcc_lockup_pct;
    doc["tccTarget"] = telemetry.tcc_target_slip_rpm;
    doc["tccActual"] = telemetry.tcc_actual_slip_rpm;
    doc["limp"]      = telemetry.is_limp_mode;
    // ArduinoJson treats const char[N] as a string literal and may borrow it.
    // These are runtime snapshot buffers: explicitly request owned string copies.
    doc["limpReason"] = JsonString(telemetry.limp_mode_reason);
    doc["safety"] = JsonString(telemetry.last_safety_event);
    doc["atfTemp"]   = telemetry.atf_temp_c;
    doc["htMode"]    = telemetry.high_torque_mode;
    doc["phase"]     = telemetry.shift_phase;
    doc["revAbuse"]  = telemetry.reverse_abuse_active;
    doc["testMode"]  = telemetry.test_mode;
    doc["tpsV"]      = telemetry.tps_v;
    doc["mapV"]      = telemetry.map_v;
    doc["atfV"]      = telemetry.atf_v;
    doc["din"]       = telemetry.io_din;
    doc["tout"]      = telemetry.test_out_mask;
    doc["n2"]        = telemetry.n2_rpm;
    doc["n3"]        = telemetry.n3_rpm;
    doc["tEstNm"]    = telemetry.t_est_nm;
    doc["loadPct"]   = telemetry.load_pct;
    doc["shiftClass"]= telemetry.shift_class;
    doc["pdType"]    = telemetry.pd_type;
    doc["onClutch"]  = (int)telemetry.on_clutch_rpm;
    doc["offClutch"] = (int)telemetry.off_clutch_rpm;
    doc["tInput"]    = (int)telemetry.t_input_nm;
    doc["tqCut"]     = telemetry.torque_cut_active;
    doc["dtcN"]      = telemetry.dtc_active_count;
    doc["spdHwOk"]   = telemetry.speed_hw_ok;
    doc["inTrust"]   = telemetry.input_speed_trusted;
    doc["tpsOk"]     = telemetry.tps_valid;
    doc["mapOk"]     = telemetry.map_valid;
    doc["clients"] = clients;
    doc["sampledMs"] = snap.sampledMs;
    doc["snapshotAgeMs"] = uint32_t(millis() - snap.sampledMs);
    doc["expectedRatio"] = snap.expectedRatio;
    doc["targetRatio"] = snap.targetRatio;
    if (telemetry.atf_only_selector && !telemetry.atf_forward_confirmed) {
        doc["gear"] = nullptr; doc["tgt"] = nullptr;
        doc["expectedRatio"] = nullptr; doc["targetRatio"] = nullptr;
    }
    doc["loopOverrunSoft"] = telemetry.loop_overrun_soft;
    doc["loopOverrunHard"] = telemetry.loop_overrun_hard;
    doc["loopMaxUs"] = telemetry.loop_max_us;
    doc["intervalMs"] = TCU_TELEMETRY_INTERVAL_MS;
    doc["heap"] = freeHeap;
    doc["fsOk"] = filesystemOk;
    doc["assets"] = "embedded";
    doc["atfSignalOk"] = telemetry.atf_signal_valid;
    doc["atfSource"] = telemetry.atf_signal_valid ? "live" : telemetry.atf_has_measurement ? "held" : "default";
    doc["atfMeasuredC"] = nullptr;
    if (telemetry.atf_signal_valid) doc["atfMeasuredC"] = telemetry.atf_temp_c;
    doc["atfLastValidMs"] = telemetry.atf_last_valid_ms;
    doc["atfCircuit"] = !telemetry.atf_sampled ? "not_sampled" : telemetry.atf_signal_valid ? "temperature"
        : telemetry.atf_v >= 3.0f ? "pn_or_open" : "low_or_short";
    doc["n2Recent"] = telemetry.n2_signal_recent;
    doc["n3Recent"] = telemetry.n3_signal_recent;
    doc["outRecent"] = telemetry.output_signal_recent;
    doc["engRecent"] = telemetry.engine_signal_recent;
    // No voltage divider/current sensor is configured on this hardware.
    // Null means unavailable; zero would falsely claim a measured value.
    doc["batteryV"] = nullptr;
    doc["solenoidCurrentA"] = nullptr;
    doc["batterySupported"] = false;
    doc["currentSupported"] = false;

}
