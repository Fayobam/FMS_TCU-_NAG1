// ============================================================================
// FILE: ShiftScheduler.cpp
// VERSION: 5.3
// UPDATES: Added Predictive "Money Shift" Guard to downshift logic.
// ============================================================================
#include "ShiftScheduler.h"

ShiftScheduler::ShiftScheduler(SolenoidDriver* solenoids, AdaptiveMemory* adaptives) {
    _solenoids = solenoids;
    _adaptives = adaptives;
    _current_phase = PHASE_CRUISING;
    _active_routing_pin = 0;
}

void ShiftScheduler::begin() {
    _current_phase = PHASE_CRUISING;
    telemetry.current_gear = 1;
    telemetry.target_gear = 1;
}

// ... [Keep existing calculateLinePressure and calculateLiveRatio unchanged] ...
void ShiftScheduler::calculateLinePressure() {
    float target_pressure_pct = 0.0f;
    if (_current_phase == PHASE_CRUISING) {
        uint8_t gear_idx = constrain(telemetry.current_gear - 1, 0, 4);
        float load = telemetry.tps_pct;
        if (telemetry.map_kpa > 100.0f) load = telemetry.tps_pct + ((telemetry.map_kpa - 100.0f) * 1.5f);
        uint8_t load_idx = constrain((int)(load / 10.0f), 0, 15);
        target_pressure_pct = HOLDING_PRESSURE_MAP[gear_idx][load_idx];
        if (telemetry.engine_rpm < 1200.0f) target_pressure_pct += 10.0f; 
    } else {
        target_pressure_pct = telemetry.tps_pct;
        if (telemetry.map_kpa > 100.0f) target_pressure_pct += (telemetry.map_kpa - 100.0f) * 2.0f; 
        target_pressure_pct += 20.0f; 
    }
    if (telemetry.atf_temp_c < 40.0f) target_pressure_pct *= 0.8f;
    else if (telemetry.atf_temp_c > 100.0f) target_pressure_pct += 15.0f;

    telemetry.line_pressure_pct = (uint8_t)constrain(target_pressure_pct, 10.0f, 100.0f);
    _solenoids->setLinePressure(telemetry.line_pressure_pct);
}

void ShiftScheduler::calculateLiveRatio() {
    if (telemetry.output_rpm > 50.0f) telemetry.live_ratio = telemetry.turbine_rpm / telemetry.output_rpm;
    else telemetry.live_ratio = getTargetRatio(telemetry.current_gear);
}

// ============================================================================
// TCC DYNAMIC SLIP CONTROLLER
// ============================================================================
void ShiftScheduler::updateTCC() {
    // ... (Keep existing updateTCC logic from Version 6.0) ...
    telemetry.tcc_actual_slip_rpm = telemetry.engine_rpm - telemetry.turbine_rpm;
    if (telemetry.tcc_actual_slip_rpm < 0) telemetry.tcc_actual_slip_rpm = 0; // Ignore negative slip during engine braking

    int current_tcc_pwm = telemetry.tcc_lockup_pct;

    // 2. Determine our target state based on driving conditions
    if (_current_phase == PHASE_CRUISING) {
        
        if (telemetry.map_kpa > 105.0f || telemetry.tps_pct > 75.0f) {
            // HIGH LOAD / BOOST: Protect the clutch! Ramp open to allow torque multiplication.
            telemetry.tcc_target_slip_rpm = 500.0f; // High target acts as "Open"
            current_tcc_pwm -= 2; // Fast release
            
        } else if (telemetry.engine_rpm < 1400.0f || telemetry.current_gear == 1) {
            // LOW SPEED: Keep TCC fully open to prevent stalling the engine
            telemetry.tcc_target_slip_rpm = 1000.0f;
            current_tcc_pwm -= 3;
            
        } else {
            // LIGHT CRUISING: Chase the OEM ~50 RPM slip target for fuel economy & smoothness
            telemetry.tcc_target_slip_rpm = 50.0f;
            
            // The Ramping Logic (P-Controller)
            // Note: TCC updates at 1000Hz, so small +/- 1 adjustments create smooth ramps
            if (telemetry.tcc_actual_slip_rpm > (telemetry.tcc_target_slip_rpm + 20.0f)) {
                // Too much slip -> Ramp pressure UP to clamp harder
                if (current_tcc_pwm < 85) current_tcc_pwm += 1; 
            } else if (telemetry.tcc_actual_slip_rpm < (telemetry.tcc_target_slip_rpm - 10.0f)) {
                // Too tightly locked -> Ramp pressure DOWN to relax
                current_tcc_pwm -= 1; 
            }
        }
    } else {
        // TRANSIENT STATE (Shifting): Rapidly unlock the TCC to absorb shift shock
        telemetry.tcc_target_slip_rpm = 1000.0f;
        current_tcc_pwm -= 5; // Very aggressive release during shifts
    }

    // 3. Constrain safety limits and command hardware
    telemetry.tcc_lockup_pct = (uint8_t)constrain(current_tcc_pwm, 0, 100);
    _solenoids->setTCC(telemetry.tcc_lockup_pct);
}

void ShiftScheduler::update() {
    // ========================================================================
    // LIMP MODE ENFORCEMENT
    // ========================================================================
    if (telemetry.is_limp_mode) {
        _solenoids->setLinePressure(100);    // Max line pressure to grip failing clutches
        _solenoids->setShiftPressure(0);     // Exhaust shift pressure
        _solenoids->stopAllShiftSolenoids(); // Default to 2nd Gear hydraulically
        _solenoids->setTCC(0);               // Unlock TCC to prevent stalling
        telemetry.current_gear = 2;          // Tell dashboard we are in 2nd
        return;                              // HALT ALL OTHER LOGIC
    }

    calculateLinePressure();
    calculateLiveRatio();

    TickType_t current_tick = xTaskGetTickCount();
    unsigned long time_in_phase_ms = (current_tick - _phase_start_tick) * portTICK_PERIOD_MS;
    float target_ratio = getTargetRatio(telemetry.target_gear);

    // ========================================================================
    // V8.0 LIMP MODE SLIP-DETECTION ALGORITHM
    // ========================================================================
    // We only check for slip while CRUISING, in DRIVE, at speeds over ~10mph.
    // (We expect slip during shifts, and sensors can be noisy near 0mph).
    if (_current_phase == PHASE_CRUISING && telemetry.prnd_state == 'D' && telemetry.output_rpm > 200.0f) {
        
        // Mathematically predict what the Turbine RPM *should* be
        float expected_turbine_rpm = telemetry.output_rpm * target_ratio;
        
        // Calculate the absolute difference between reality and theory
        float rpm_mismatch = abs(telemetry.turbine_rpm - expected_turbine_rpm);

        // If the mismatch is > 300 RPM, the clutch pack is physically slipping!
        if (rpm_mismatch > 300.0f) {
            if (!telemetry.is_slipping) {
                telemetry.is_slipping = true;
                telemetry.slip_start_time_ms = millis();
            } else if ((millis() - telemetry.slip_start_time_ms) > 400) {
                // It has slipped continuously for > 400ms. TRIGGER LIMP MODE!
                telemetry.is_limp_mode = true;
                telemetry.limp_mode_reason = "FATAL SLIP: GEAR " + String(telemetry.current_gear) + " | DIFF: " + String(rpm_mismatch) + " RPM";
                Serial.println("!!! TRANSMISSION PROTECTION PROTOCOL ACTIVATED !!!");
                Serial.println(telemetry.limp_mode_reason);
            }
        } else {
            // Mismatch resolved itself (e.g. hitting a pothole caused a microsecond wheel spin)
            telemetry.is_slipping = false;
        }
    } else {
        // Reset the slipping flag during shifts or while stopped
        telemetry.is_slipping = false;
    }

    // ========================================================================
    // MAIN STATE MACHINE
    // ========================================================================
    switch (_current_phase) {
        case PHASE_CRUISING:
            if (telemetry.prnd_state == 'D' && telemetry.is_park_neutral) {
                telemetry.is_park_neutral = false;
                _solenoids->fireShiftSolenoid(PIN_Y3); _solenoids->setShiftPressure(20); 
                _current_phase = PHASE_PREFILL; _phase_start_tick = current_tick;
            }

            // UPSHIFT TRIGGER
            if (telemetry.paddle_up_request && telemetry.current_gear < 5) {
                telemetry.paddle_up_request = false; 
                telemetry.target_gear = telemetry.current_gear + 1;
                _is_upshift = true;
                _current_pressure_mod = _adaptives->getModifier(true, telemetry.current_gear-1, true, telemetry.tps_pct, telemetry.map_kpa, telemetry.engine_rpm);
                _current_timing_mod  = _adaptives->getModifier(true, telemetry.current_gear-1, false, telemetry.tps_pct, telemetry.map_kpa, telemetry.engine_rpm);
                
                telemetry.flare_detected = false; telemetry.bind_detected = false;
                _turbine_rpm_at_shift_start = telemetry.turbine_rpm; _output_rpm_at_shift_start = telemetry.output_rpm;
                _active_routing_pin = getRoutingSolenoidForShift(telemetry.current_gear, telemetry.target_gear);
                
                _solenoids->fireShiftSolenoid(_active_routing_pin); 
                _solenoids->setShiftPressure(80); 
                _current_phase = PHASE_PREFILL; _phase_start_tick = current_tick;
            }

            // NEW: DOWNSHIFT TRIGGER WITH MONEY SHIFT GUARD
            if (telemetry.paddle_down_request && telemetry.current_gear > 1) {
                telemetry.paddle_down_request = false; // Always clear the request so it doesn't get stuck
                
                // 1. Predict the engine RPM if we execute this downshift
                float lower_gear_ratio = getTargetRatio(telemetry.current_gear - 1);
                float predicted_engine_rpm = telemetry.output_rpm * lower_gear_ratio;
                
                // 2. The Performance Guard (Leaves ~1200+ RPM of supercharged pull)
                const float MAX_SAFE_DOWNSHIFT_RPM = 5000.0f;
                
                if (predicted_engine_rpm < MAX_SAFE_DOWNSHIFT_RPM) {
                    // Safe to shift!
                    telemetry.target_gear = telemetry.current_gear - 1;
                    _is_upshift = false;
                    
                    _current_pressure_mod = _adaptives->getModifier(false, telemetry.target_gear-1, true, telemetry.tps_pct, telemetry.map_kpa, telemetry.engine_rpm);
                    _current_timing_mod   = _adaptives->getModifier(false, telemetry.target_gear-1, false, telemetry.tps_pct, telemetry.map_kpa, telemetry.engine_rpm);
                    
                    telemetry.flare_detected = false; telemetry.bind_detected = false;
                    _turbine_rpm_at_shift_start = telemetry.turbine_rpm; _output_rpm_at_shift_start = telemetry.output_rpm;
                    _active_routing_pin = getRoutingSolenoidForShift(telemetry.current_gear, telemetry.target_gear);
                    
                    _solenoids->fireShiftSolenoid(_active_routing_pin);
                    _current_phase = PHASE_DS_RELEASE; 
                    _phase_start_tick = current_tick;
                    _shift_stopwatch_start = millis(); 
                } else {
                    // MONEY SHIFT GUARD ACTIVATED
                    Serial.print("MONEY SHIFT PREVENTED! Predicted RPM: ");
                    Serial.println(predicted_engine_rpm);
                    // The request is cleared and the TCU remains in PHASE_CRUISING.
                }
            }
            break;

        // ... [Keep PHASE_PREFILL, OVERLAP, INERTIA, DS_RELEASE, DS_SYNC, DS_CATCH, COMPLETION exact same] ...
        case PHASE_PREFILL:
            if (time_in_phase_ms > (60 + _current_timing_mod)) {
                _solenoids->setShiftPressure(constrain(30 + _current_pressure_mod, 0, 100)); 
                _current_phase = PHASE_OVERLAP; _phase_start_tick = current_tick; _shift_stopwatch_start = millis(); 
            }
            break;
        case PHASE_OVERLAP:
            if (time_in_phase_ms % 2 == 0 && telemetry.shift_pressure_pct < 80) _solenoids->setShiftPressure(telemetry.shift_pressure_pct + 1);
            if (telemetry.live_ratio < (getTargetRatio(telemetry.current_gear) - 0.1f)) { _current_phase = PHASE_INERTIA; _phase_start_tick = current_tick; }
            break;
        case PHASE_INERTIA:
            if (time_in_phase_ms % 2 == 0 && telemetry.shift_pressure_pct < 90) _solenoids->setShiftPressure(telemetry.shift_pressure_pct + 1);
            if (telemetry.live_ratio <= (target_ratio + 0.05f) || time_in_phase_ms > 600) {
                telemetry.last_shift_time_ms = millis() - _shift_stopwatch_start;
                _solenoids->stopShiftSolenoid(_active_routing_pin); _solenoids->setShiftPressure(0); 
                telemetry.current_gear = telemetry.target_gear; _current_phase = PHASE_COMPLETION; _phase_start_tick = current_tick;
            }
            break;
        case PHASE_DS_RELEASE:
            _solenoids->setShiftPressure(0); 
            if (time_in_phase_ms > (80 + _current_timing_mod)) { _current_phase = PHASE_DS_SYNC; _phase_start_tick = current_tick; }
            break;
        case PHASE_DS_SYNC:
            if (telemetry.live_ratio >= (target_ratio - 0.2f) || time_in_phase_ms > 400) { _current_phase = PHASE_DS_CATCH; _phase_start_tick = current_tick; }
            break;
        case PHASE_DS_CATCH:
            if (time_in_phase_ms % 2 == 0 && telemetry.shift_pressure_pct < (80 + _current_pressure_mod)) _solenoids->setShiftPressure(telemetry.shift_pressure_pct + 2);
            if (telemetry.output_rpm < (_output_rpm_at_shift_start - 30.0f)) telemetry.bind_detected = true;
            if (telemetry.live_ratio >= (target_ratio - 0.05f) || time_in_phase_ms > 400) {
                telemetry.last_shift_time_ms = millis() - _shift_stopwatch_start;
                _solenoids->stopShiftSolenoid(_active_routing_pin); _solenoids->setShiftPressure(0);
                telemetry.current_gear = telemetry.target_gear; _current_phase = PHASE_COMPLETION; _phase_start_tick = current_tick;
            }
            break;
        case PHASE_COMPLETION:
            if (time_in_phase_ms > 50) {
                _adaptives->evaluateShift(_is_upshift ? (telemetry.current_gear - 1) : (telemetry.current_gear + 1), telemetry.current_gear, telemetry.tps_pct, telemetry.map_kpa, telemetry.engine_rpm, telemetry.last_shift_time_ms, telemetry.flare_detected, telemetry.bind_detected);
                _current_phase = PHASE_CRUISING;
            }
            break;
    }

    // Update the TCC at the very end of every 1ms loop
    updateTCC();
}

uint8_t ShiftScheduler::getRoutingSolenoidForShift(uint8_t from_gear, uint8_t to_gear) {
    if (from_gear == 1 && to_gear == 2) return PIN_Y3;
    if (from_gear == 2 && to_gear == 3) return PIN_Y5;
    if (from_gear == 3 && to_gear == 4) return PIN_Y4;
    if (from_gear == 4 && to_gear == 5) return PIN_Y3;
    if (from_gear == 5 && to_gear == 4) return PIN_Y3;
    if (from_gear == 4 && to_gear == 3) return PIN_Y4;
    if (from_gear == 3 && to_gear == 2) return PIN_Y5;
    if (from_gear == 2 && to_gear == 1) return PIN_Y3;
    return 0;
}

float ShiftScheduler::getTargetRatio(uint8_t gear) {
    switch(gear) { case 1: return RATIO_1ST; case 2: return RATIO_2ND; case 3: return RATIO_3RD; case 4: return RATIO_4TH; case 5: return RATIO_5TH; default: return 1.0f; }
}