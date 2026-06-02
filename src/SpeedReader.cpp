// ============================================================================
// FILE: SpeedReader.cpp
// VERSION: 2.1
// UPDATES: Merged 722.6 Kinematics with zero-overhead ESP32 PCNT drivers.
// ============================================================================
#include "SpeedReader.h"

SpeedReader::SpeedReader(uint8_t pin_n2, uint8_t pin_n3, uint8_t pin_out, uint8_t pin_eng) {
    _pin_n2 = pin_n2;
    _pin_n3 = pin_n3;
    _pin_out = pin_out;
    _pin_eng = pin_eng;
}

void SpeedReader::initPCNT(pcnt_unit_handle_t* unit_handle, uint8_t pin) {
    // 1. Configure the PCNT Unit
    pcnt_unit_config_t unit_config = {};
    unit_config.high_limit = 32767;
    unit_config.low_limit = -1;
    pcnt_new_unit(&unit_config, unit_handle);

    // 2. Configure the Hardware Glitch Filter (~1.2us)
    pcnt_glitch_filter_config_t filter_config = {};
    filter_config.max_glitch_ns = 1200;
    pcnt_unit_set_glitch_filter(*unit_handle, &filter_config);

    // 3. Configure the PCNT Channel
    pcnt_chan_config_t chan_config = {};
    chan_config.edge_gpio_num = pin;
    chan_config.level_gpio_num = -1; // Not used
    pcnt_channel_handle_t pcnt_chan = NULL;
    pcnt_new_channel(*unit_handle, &chan_config, &pcnt_chan);

    // 4. Set Edge Actions: Count UP on rising edge, ignore falling edge
    pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);

    // 5. Enable and Start
    pcnt_unit_enable(*unit_handle);
    pcnt_unit_clear_count(*unit_handle);
    pcnt_unit_start(*unit_handle);
}

void SpeedReader::begin() {
    // Initialize the 4 hardware PCNT units using the new API
    initPCNT(&_pcnt_n2, _pin_n2);
    initPCNT(&_pcnt_n3, _pin_n3);
    initPCNT(&_pcnt_out, _pin_out);
    initPCNT(&_pcnt_eng, _pin_eng);

    _last_read_time = millis();
    Serial.println("Speed Reader V2.2 Initialized (ESP-IDF 5.x PCNT + Kinematics).");
}

// ============================================================================
// 722.6 PLANETARY KINEMATICS (The "Magic" Math)
// ============================================================================
float SpeedReader::calculateTurbineRPM(float n2_rpm, float n3_rpm, uint8_t current_gear) {
    // 722.6 Front Planetary Gearset Constants
    const float PLANETARY_MULTIPLIER_N3 = 1.48f; // (Ring + Sun) / Ring
    const float PLANETARY_MULTIPLIER_N2 = 0.48f; // Sun / Ring

    switch (current_gear) {
        case 1:
        case 2:
        case 3:
        case 4:
            // 1st-4th: Input Shaft is combination of Sun Gear (N2) & Carrier (N3)
            return (n3_rpm * PLANETARY_MULTIPLIER_N3) - (n2_rpm * PLANETARY_MULTIPLIER_N2);

        case 5:
            // 5th Gear: N3 drum is locked to 0 RPM. Calculate using only N2.
            return n2_rpm * 0.83f; 

        default:
            // Fallback (Park/Neutral/Reverse)
            return n3_rpm; 
    }
}

void SpeedReader::update(uint8_t current_gear) {
    unsigned long current_time = millis();
    unsigned long delta_time = current_time - _last_read_time;

    if (delta_time >= 50) { // Update every 50ms
        int pulses_n2 = 0, pulses_n3 = 0, pulses_out = 0, pulses_eng = 0;

        // 1. Grab hardware counts and instantly clear them
        pcnt_unit_get_count(_pcnt_n2, &pulses_n2);  pcnt_unit_clear_count(_pcnt_n2);
        pcnt_unit_get_count(_pcnt_n3, &pulses_n3);  pcnt_unit_clear_count(_pcnt_n3);
        pcnt_unit_get_count(_pcnt_out, &pulses_out); pcnt_unit_clear_count(_pcnt_out);
        pcnt_unit_get_count(_pcnt_eng, &pulses_eng); pcnt_unit_clear_count(_pcnt_eng);

        // 2. Convert pulses to Raw RPM using Calibration Constants
        // Formula: (Pulses / Teeth) * (60000ms / delta_time_ms)
        _raw_n2_rpm = (pulses_n2 / TEETH_N2) * (60000.0f / delta_time); 
        _raw_n3_rpm = (pulses_n3 / TEETH_N3) * (60000.0f / delta_time);
        
        telemetry.output_rpm = (pulses_out / TEETH_OUT) * (60000.0f / delta_time); 
        telemetry.engine_rpm = (pulses_eng / PULSES_PER_REV_ENG)  * (60000.0f / delta_time);  

        // 3. Run planetary kinematics to find true Turbine Input Speed
        telemetry.turbine_rpm = calculateTurbineRPM(_raw_n2_rpm, _raw_n3_rpm, current_gear);

        // Constrain to prevent random math drops below 0
        if (telemetry.turbine_rpm < 0) telemetry.turbine_rpm = 0;

        _last_read_time = current_time;
    }
}