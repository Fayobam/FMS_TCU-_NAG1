#include "SpeedReader.h"
#include "TCU_Data.h" // We need this to write to the global telemetry struct

SpeedReader::SpeedReader(uint8_t pin_n2, uint8_t pin_n3, uint8_t pin_out, uint8_t pin_eng) {
    _pin_n2  = pin_n2;
    _pin_n3  = pin_n3;
    _pin_out = pin_out;
    _pin_eng = pin_eng;
    _last_read_time = 0;
}

void SpeedReader::begin() {
    // We assign each sensor to a new hardware pulse counter unit
    _unit_n2  = configurePCNT(_pin_n2);
    _unit_n3  = configurePCNT(_pin_n3);
    _unit_out = configurePCNT(_pin_out);
    _unit_eng = configurePCNT(_pin_eng);

    _last_read_time = millis();
}

pcnt_unit_handle_t SpeedReader::configurePCNT(uint8_t pin) {
    // 1. Create a new PCNT unit
    pcnt_unit_config_t unit_config = {
        .low_limit  = -30000,
        .high_limit = 30000,
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    pcnt_new_unit(&unit_config, &pcnt_unit);

    // 2. Set up a glitch filter (10,000 ns = 10 microseconds debounce)
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 10000, 
    };
    pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config);

    // 3. Create a channel for this unit attached to our sensor pin
    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = pin,
        .level_gpio_num = -1, // No control pin needed
    };
    pcnt_channel_handle_t pcnt_chan = NULL;
    pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan);

    // 4. Configure the channel to count UP on the rising edge, and do nothing on falling
    pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);

    // 5. Enable and start the counter
    pcnt_unit_enable(pcnt_unit);
    pcnt_unit_clear_count(pcnt_unit);
    pcnt_unit_start(pcnt_unit);

    return pcnt_unit;
}

void SpeedReader::update() {
    unsigned long current_time = millis();
    unsigned long time_delta_ms = current_time - _last_read_time;

    // Prevent divide-by-zero if called too rapidly
    if (time_delta_ms == 0) return; 

    // Calculate RPM and instantly write it to our global telemetry struct!
    telemetry.turbine_rpm = calculateRPM(_unit_n2, TEETH_N2,  time_delta_ms);
    // Note: To get the true turbine RPM, the physics engine will later decide whether
    // to look at N2 or N3 based on the current gear's planetary math.
    // For now, we will store N3 separately in a local/future variable if needed, 
    // or you can add `n3_rpm` to your telemetry struct. Let's just calculate them all:
    
    float raw_n3_rpm      = calculateRPM(_unit_n3, TEETH_N3,  time_delta_ms);
    telemetry.output_rpm  = calculateRPM(_unit_out, TEETH_OUT, time_delta_ms);
    telemetry.engine_rpm  = calculateRPM(_unit_eng, TEETH_ENG, time_delta_ms);

    _last_read_time = current_time;
}

float SpeedReader::calculateRPM(pcnt_unit_handle_t unit, float teeth, unsigned long time_delta_ms) {
    int pulse_count = 0;
    
    // 1. Ask the hardware how many pulses it saw
    pcnt_unit_get_count(unit, &pulse_count);
    
    // 2. Clear the hardware counter immediately so it starts counting the next batch
    pcnt_unit_clear_count(unit);

    // 3. Math: (Pulses / Teeth) gives revolutions. Multiply by (60,000 / ms) to get RPM.
    float revolutions = (float)pulse_count / teeth;
    float rpm = revolutions * (60000.0f / (float)time_delta_ms);

    return rpm;
}