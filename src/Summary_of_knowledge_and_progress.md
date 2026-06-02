FMS TCU: The Master 722.6 Engineering & Firmware ManualVersion: 2.0 (Unified Hardware & Software Specification)Application: Mercedes-Benz 722.6 (W5A330 Small NAG)Powertrain: M111 2.3L + Audi TVS1320 Supercharger1. System Architecture OverviewThe FMS (Fayobam's Manual Sporty) TCU is a high-performance, ESP32-based standalone controller. It completely severs the 722.6 transmission from the restrictive OEM engine ECU, allowing for granular control over hydraulic line pressures, clutch fill volumes, and shift synchronizations.The dual-core ESP32 architecture divides labor:Core 1 (Physics Engine): Runs a strict 1000Hz (1ms) FreeRTOS loop handling high-frequency PWM solenoids, state machines, and planetary kinematics.Core 0 (Web Studio): Hosts an asynchronous WebSockets server to stream live telemetry and receive tuning updates without interrupting the physical shift logic.2. Hardware Sensors & Signal ProcessingTo control the transmission safely, the TCU reconstructs a live model of the powertrain using direct hardware interrupts and precise analog conversions.A. Speed Sensors & PCNT HardwareThe transmission relies on four speed sensors, tracked using the ESP32's pulse_cnt.h (PCNT) hardware peripheral to ensure zero missed pulses at 7000+ RPM.Output Speed (External): 24-tooth reluctor.Engine Speed (External): Tachometer signal (V6=3 pulses/rev, I4=2 pulses/rev).N2 Drum (Internal - 60 teeth): Front sun gear speed.N3 Drum (Internal - 60 teeth): Front planet carrier speed.B. The Temperature & PRND MultiplexThe 722.6 uses a single wire to communicate both the gear selector position and the ATF temperature.Park/Neutral: The physical switch opens, sending a full 3.3V (5V logic pulled down) to the TCU.In Gear (D, R, 4, 3, 2, 1): The switch closes, running the signal through a PTC thermistor.Non-Linear Temp Calculation: Using Kovero's field-tested atfSensorMap, the TCU translates resistance directly to temperature using segmented linear interpolation (e.g., $1109\Omega = 40^\circ\text{C}$, $1450\Omega = 80^\circ\text{C}$). This allows precise pressure scaling to compensate for oil viscosity changes.3. Planetary Kinematics (The "Magic Math")Because the 722.6 does not have a physical input shaft speed sensor, the TCU must mathematically derive the Turbine RPM using the Ravigneaux planetary gear set ratios (K1 = 1.48, K2 = 0.48).In 1st, 2nd, 3rd, and 4th Gear:$RPM_{turbine} = (N3 \times 1.48) - (N2 \times 0.48)$In 5th Gear (Overdrive):A clutch locks the N3 carrier to the casing. Since N3 is 0 RPM, the formula shifts:$RPM_{turbine} = N2 \times 0.83$This calculated Turbine RPM is the backbone of the TCU. By continuously comparing it to the Output RPM, the TCU calculates the Live Mechanical Ratio, dictating exactly when a shift is completed.4. Hydraulic Control & Solenoid StrategyA. Routing Solenoids (Y3, Y4, Y5)These solenoids act as train-track switches to route fluid to specific clutches. They possess 4-ohm coils that will burn up if fed 12V continuously.The "Kick and Hold" Software Driver: When a shift is triggered, the firmware blasts the solenoid with 83% PWM for exactly 60ms to snap the valve open against hydraulic line pressure. At millisecond 61, the firmware automatically drops the PWM to 33% to safely hold the valve open without overheating.B. Pressure Solenoids (MPC & SPC)Operating at 1000Hz, these use Inverted Logic (0% PWM = Maximum Pressure, 100% PWM = Zero Pressure).MPC (Main Pressure Control): Controls overall system clamping force.SPC (Shift Pressure Control): Acts as the digital "clutch pedal" during a gear change.5. The Shift Execution PipelineThe firmware breaks every gear change into an extremely precise, multi-phase state machine.Table: Upshift State Machine (e.g., 1st to 2nd)PhaseFirmware ActionTransition ConditionCruisingY3 OFF, SPC at 0% pressure.UP Paddle pulled.Pre-FillY3 Kicked. SPC hits 80% to rapidly fill the empty clutch drum.Timer: 60ms + Adaptive Modifier.OverlapSPC dropped to 30%, slowly ramping up. Clutches are crossing over.Live ratio begins dropping from 3.93.InertiaSPC ramps aggressively. Engine RPM is dragged down.Live ratio hits 2.40 (Target Ratio).CompletionShift is complete. Y3 turned OFF, SPC bled to 0.50ms settling timer.Downshift Guard (The "Money Shift" Preventer)Before executing a downshift, the firmware mathematically predicts the resulting engine speed:Predicted RPM = Output RPM * Lower Gear RatioIf this value exceeds 5000 RPM, the TCU ignores the paddle pull, protecting the M111 from catastrophic valve float.6. Supercharger Tuning: The Exponential Pressure MapBecause the Audi TVS1320 supercharger creates massive instantaneous torque, linear pressure increases are insufficient. The FMS TCU utilizes an Exponential Pressure Curve for the MPC line pressure and Shift Pressure baselines.The Formula: $Pressure = \frac{Load_{bin}^2}{2.25}$The Result: At 40% throttle (cruising), the TCU adds a gentle 7% extra clamping force. At 150% load (full boost), the math commands 100% maximum clamping force, locking the clutches like a vise to prevent glazing.7. The 16x16 Adaptive Memory MatrixTo refine the supercharged powerband, the TCU employs a 256-cell 3D matrix (Load vs. Engine RPM).The Metrics: For every upshift and downshift, the TCU stores specific Pressure Modifiers (%) and Timing Modifiers (ms).The Evaluation Logic: If a shift executes cleanly (no flare, no bind) within a sub-200ms target, the TCU actively "locks in" those parameters, preventing the OEM behavior of softening the pressure after a hard pull.K2 Clutch Quirk: The 3-4 shift utilizes the massive K2 clutch. The firmware is hardcoded to grant this specific shift an extra 20ms of pre-fill timing and +5% base pressure to prevent the notorious 722.6 "3-4 flare."8. Safety & Standstill ProtocolsA. TCC Dynamic PID Slip ControllerThe Torque Converter Clutch operates on a separate PID loop:Cruising: Targets 50 RPM of slip to absorb NVH (Noise, Vibration, Harshness).Under Boost: Ramps PWM open to allow the torque converter to multiply TVS1320 torque.Low Speed (<1400 RPM): Unlocks completely to prevent stalling the engine.B. V8.0 Slip-Detection (Limp Mode)While cruising, the TCU constantly compares the mathematically derived Turbine RPM to the Target Expected RPM (Output * Gear Ratio).If a physical clutch pack fails and the mismatch exceeds 300 RPM for 400ms, the TCU:Maxes Line Pressure (100%).Exhausts Shift Pressure.Kills all routing solenoids (mechanically forcing 2nd gear).Emits a FATAL SLIP telemetry code to the Web Dashboard.C. The Garage Shift "Jiggle"If the transmission sits in Park or Neutral, the regulating valves drain. Dropping into Drive causes a massive clunk. To prevent this, the firmware pulses the Y4 solenoid at 37% continuously while in P/N, buffering the hydraulics so the Drive engagement is buttery smooth.

====================================================================
9. V9 FIRMWARE UPDATE — CHANGELOG & CURRENT STATE
====================================================================
(Appended after a full code review against ultimate-nag52-fw and dueATC.
The architecture above was kept; the items below are corrections and additions.)

A. NEW: Auto-Safety Shift Layer (the previously-missing requested feature)
The controller is manual-paddle-commanded EXCEPT for two protective auto-shifts:
- OVERREV: forces an upshift at 6300 rpm (200 rpm below the 6500 ceiling) so the
  shift completes before the limiter. Skipped if already in 5th.
- LUGGING: forces a downshift below 1100 rpm when throttle > 25%, in gears 2-5.
Both pass through the money-shift guard, so a safety downshift can never overrev.
A 500 ms cooldown prevents hunting. Money-shift guard ceiling raised to 6000 rpm.

B. FIXED: Engine load inputs (TPS + MAP) are now actually read.
Previously tps_pct and map_kpa were used by every load decision but never populated,
so the car effectively always thought it was at idle. InputManager now reads both
analog channels (filtered). NOTE: both MUST be on ADC1 pins because WiFi disables ADC2.

C. FIXED: Shift pressure ramp. The solenoid driver now writes the commanded pressure
back into telemetry, so the OVERLAP/INERTIA ramp reads real values instead of stale ones.

D. FIXED: Flare detection. Upshift flare (clutch slipping, ratio climbing instead of
dropping) is now detected, so adaptive learning can correct the K2 3-4 flare it was
designed for. Previously only bind was detectable.

E. FIXED: P/N selector double-write. Split into pn_switch_raw (sensor) and
drive_engaged (scheduler latch) so they no longer overwrite each other every 1 ms.

F. FIXED: Adaptive modifier overflow. Stored modifiers now clamp to +/-60 to prevent
int8_t wraparound turning maximum clamp into minimum clamp over many shifts.

G. FIXED: Unified load model. computeLoad()/loadToBin() now live in TCU_Data.h and are
used identically everywhere, so 1.2 bar of boost spreads across multiple adaptive cells
instead of saturating a single one.

H. IMPROVED: Limp mode. Now load-aware (won't false-trigger during high-boost launches)
and has a deliberate reset path (web "limp_reset" command, honoured only when stopped
and in P/N).

I. REFACTOR: All shift initiation centralised in beginShift(); evaluateShift() takes an
explicit shift index + direction (no more triplicated off-by-one gear arithmetic).

--------------------------------------------------------------------
CALIBRATION VALUES STILL TO BE VERIFIED BEFORE ANY ROAD TEST
--------------------------------------------------------------------
- PIN_MAP (currently 23, a guess) — confirm free and on ADC1.
- MAP sensor transfer function (MAP_KPA_AT_0V, MAP_KPA_PER_VOLT) — set to YOUR sensor.
- TPS_VOLTS_CLOSED / TPS_VOLTS_WOT — measure on the car.
- Safety thresholds — sanity-check against your engine's real limits.
- Bench-test the safety layer and limp reset with a signal generator BEFORE driving.
See HANDOFF.md for the full detail.
