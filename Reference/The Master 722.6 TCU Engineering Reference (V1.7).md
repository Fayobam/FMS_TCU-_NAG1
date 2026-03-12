The Master 722.6 TCU Engineering Reference (V1.7)
================================================
Last Updated: March 2026 (based on original V1.6 + video transcript additions)
Document ID: zU30L
Purpose: Single source of truth for all 722.6 (NAG1) firmware development, hardware notes, solenoid logic, algorithms, and code decisions.

1. Hardware & Sensor Overview
----------------------------
The 722.6 (NAG1) is a 5-speed, electronically controlled, clutch-to-clutch automatic transmission.

The Speed Sensors (Hall Effect - 6V Supply):
• N2 Sensor: Reads off the K1 clutch housing (front sun gear). Limitation: It cannot read input speed in 1st or 5th gear.
• N3 Sensor: Reads off the front carrier / rear internal gear. Used to calculate input speed during 1st, 5th, and Reverse.
• Output Speed: The OEM 722.6 does not have an internal output speed sensor. A custom external output shaft speed sensor (e.g., a 24-tooth reluctor) must be fabricated and wired to the TCU.

2. The Temp Sensor / Park-Neutral Multiplexing
---------------------------------------------
• The Thermistor: A PTC (Positive Temperature Coefficient) sensor. Hotter fluid = higher resistance = higher voltage.
• The Park/Neutral Switch: A physical push-button wired in series.
• The Software Logic: 5V = Park/Neutral. 1.2V to 3.5V = In Gear (and represents the fluid temperature).

3. The Valve Body & Solenoids
-----------------------------
A. The Routing Solenoids (Y3, Y4, Y5)
   Act as mechanical train-track switches to route fluid to the correct clutch packs.
   • Resistance: 4 Ohms.
   • The "Kick and Hold" Strategy: 83% duty cycle for the first 60 milliseconds to forcefully snap the valve open, dropping to a 25-37% hold duty cycle to prevent the 4-ohm coil from melting.
   • The 3-4 Solenoid Park/Neutral Quirk: The 3-4 Shift Solenoid (Y3/6y4) is an exception to the "only on during shifts" rule. It must be pulsed at 33-40% duty cycle whenever the transmission is idling in Park or Neutral, and during lever movement, to prepare the hydraulics for a smooth Garage Shift.

B. The Pressure Solenoids (MPC & SPC)
   Control hydraulic pressure via 1000Hz PWM with INVERTED LOGIC (0% Duty Cycle = MAX Pressure, 100% Duty Cycle = MIN Pressure).
   • SPC (Shift Pressure Control): The primary solenoid for gear transitions.
   • MPC (Modulating Pressure Control): The global line pressure solenoid.

C. The Torque Converter Clutch Solenoid (TCC)
   Controls the lockup clutch inside the torque converter. Resistance: 2.5 Ohms.
   • PWM Frequency: ~100 Hz
   • Apply Strategy (Current Firmware): Starts at 9% duty cycle then slowly ramps upward until measured slip (engine RPM – N2/N3 input speed) drops below 200 RPM.
   • Design Note: Full zero-slip lockup is intentionally avoided at low engine speeds to prevent engine vibration transfer to the driveline.
   • Unique Capability: Unlike a manual transmission, the TCC clutch can remain applied during gear shifts. This keeps engine and input shaft speeds synchronized and greatly reduces engine flare.
   • Primary Benefit: Dramatically improves fuel economy by creating a direct mechanical link between engine and transmission input shaft (no fluid slip losses).
   • Observed Effect: Gear shifts become noticeably cleaner and quicker when TCC is locked during the shift event.

D. Torque Converter Lockup Behavior (Implementation Notes)
   • The clutch engagement happens hydraulically by activating the torque converter solenoid which moves a valve in the valve body, sending main line pressure to the torque converter clutch.
   • The pressure is regulated by pulsing the solenoid at ~100 Hz, moving the valve rapidly to modulate line pressure.

4. The Shift Execution Matrix (The Overlap Strategy)
----------------------------------------------------
Table 1: Solenoid Activation by Gear Shift

Shift Request | Shift Solenoid Triggered          | State During Cruising (After Shift)
--------------|-----------------------------------|------------------------------------
1 -> 2        | Y3 (1-2 / 4-5 Shift Valve)        | All Shift Solenoids OFF
2 -> 3        | Y5 (2-3 Shift Valve)              | All Shift Solenoids OFF
3 -> 4        | Y4 (3-4 Shift Valve)              | All Shift Solenoids OFF
4 -> 5        | Y3 (1-2 / 4-5 Shift Valve)        | All Shift Solenoids OFF

Table 2: The SPC / MPC Fading Logic

Shift Phase   | Routing Solenoids (Y3/4/5)      | SPC Action (Main Fade)                  | MPC Action (Assist / Hold)
--------------|---------------------------------|-----------------------------------------|---------------------------
1. Cruising   | OFF (0V)                        | Bled (100% Duty / Min Pressure)         | Active Modulating
2. Pre-Fill   | KICKED (83% for 60ms)           | Pulsed High (Rapid fill profile)        | Slight Drop
3. Overlap    | HOLDING (25-37% Duty)           | Fading Down (Increasing pressure)       | Fading Up
4. Inertia    | HOLDING (25-37% Duty)           | Math-Driven (Until target ratio met)    | Holding Steady
5. Completion | OFF (Power removed)             | Bled (100% Duty / Min Pressure)         | Active Modulating

5. The Shifter Interface (4-Bit PRND)
-------------------------------------
The physical shifter assembly relies on four digital output pins to send a binary code to the TCU:
• Park: 0110
• Reverse: 0111
• Neutral: 1110
• Drive: 1100
• Gear 4: 1101
• Gear 3: 1001
• Gear 2: 1011
• Gear 1: 1010

6. Garage Shifts (Standstill Engagements)
-----------------------------------------
Moving from Park/Neutral into a driving gear requires filling an empty clutch pack without an offgoing clutch to release.
• Hydraulic Preparation: The 3-4 shift solenoid must be pulsed at 33-40% during Neutral/Park to buffer the valve body against sudden line pressure surges.
• Engagement Strategy: Garage shifts use independent pressure maps. The SPC must smoothly pull the N2/N3 turbine speed down to zero.

7. Gear Ratios & Torque Capacities (Big NAG vs. Small NAG)
----------------------------------------------------------
The 722.6 family uses an official naming convention: W5Axxx.
• W = Wandler (Torque Converter)
• 5 = 5 Forward Gears
• A = Automatic
• xxx = Maximum Continuous Input Torque Rating in Newton-Meters (Nm).

A. Torque Handling Capacities
While the internal valve body is largely identical, the clutch pack counts and physical planetary gearsets vary wildly based on the engine the transmission was paired with.

Factory Designation     | Max Continuous Torque          | Common Applications
------------------------|--------------------------------|--------------------
W5A280 / W5A300         | 280 - 300 Nm (207-221 lb-ft)   | Early 4-cyl naturally aspirated, Sprinter Vans
W5A330 (Strong Small NAG) | 330 Nm (243 lb-ft)           | W203 C230 Kompressor (722.695), Standard 6-cyl (M104, M112)
W5A400                  | 400 Nm (295 lb-ft)             | ML SUVs, some diesels
W5A580 (Big NAG)        | 580 Nm (428 lb-ft)             | Standard V8 / V12 (M119, M113)
W5A900 / W5A1000        | 900 - 1000 Nm (664-738 lb-ft)  | AMG V12 Biturbo (CL65, SL65)

(Note: Mercedes utilized the W5A330 behind the W203 C230 Kompressor rather than the weaker W5A280/300 because the positive-displacement supercharger generates massive low-end torque that would otherwise slip the smaller 4-cylinder clutch packs).

B. Planetary Gear Ratios
Mercedes manufactured two entirely different planetary gearsets for this transmission. The "Small NAG" has a slightly steeper 1st and 2nd gear to help smaller engines get off the line, while the "Big NAG" has taller ratios to manage V8 torque.

CRITICAL SOFTWARE WARNING: Your TCU firmware must be hardcoded with the exact gear ratios of your physical transmission. If a Small NAG ratio array is flashed to a Big NAG transmission, the TCU will detect a "slip" condition during the N2/N3 inertia phase math and trigger Limp Mode. Since the W203 C230 Kompressor utilizes the W5A330, the Small NAG ratio table must be used in the C++ firmware.

Gear | Small NAG (W5A330 - e.g. C230 Kompressor) | Big NAG (W5A580 / W5A900)
-----|-------------------------------------------|---------------------------
1st  | 3.932 (or 3.951)                          | 3.588 (or 3.59)
2nd  | 2.408 (or 2.423)                          | 2.186 (or 2.19)
3rd  | 1.486                                     | 1.405 (or 1.41)
4th  | 1.000                                     | 1.000
5th  | 0.830                                     | 0.831 (or 0.83)
Rev  | 3.100 (or 3.147)                          | 3.160

8. Adaptive Pressure Management Algorithm (SPC & MPC)
------------------------------------------------------
Previous Limitation
Early firmware used a single static pressure value per gear shift.

Current Dynamic Pressure Strategy
The TCU now calculates real-time SPC and MPC PWM values using three lookup tables:

1. Base Pressure – Selected according to current engine load/torque demand.
2. Engine Speed Multiplier – Higher input-shaft/engine RPM increases required line pressure (pump speed effect).
3. Fluid Temperature Multiplier – Compensates for viscosity change (hotter/thinner fluid flows faster).

Solenoid Behavior Reminder
Both MPC (shift firmness) and SPC (shift speed) use inverted PWM logic:
• Lower PWM duty cycle → higher hydraulic pressure to clutch packs
• Higher PWM duty cycle → lower hydraulic pressure

Shift Termination Logic Improvement
The shift sequence no longer runs on fixed timers.
• The TCU continuously calculates the current transmission ratio using N2/N3 and output speed sensors.
• Once the target gear ratio is achieved within tolerance, the shift is terminated and routing solenoids (Y3/Y4/Y5) are immediately de-energized.
• Benefit: Prevents prolonged energization of routing solenoids and reduces risk of coil overheating/burnout.

Driver Feedback Feature
Temporary CAN messages are sent during active shifts to display upshift/downshift arrows on the instrument cluster. The display returns to the current gear number once the shift completes.

Cross-Reference
This algorithm directly feeds into the Shift Execution Matrix (Section 4) and replaces the static values previously used in the Overlap Strategy tables.

End of Document (V1.7)
======================
This is the complete, up-to-date master reference. Copy the entire block above and save as:
"The Master 722.6 TCU Engineering Reference (V1.7).txt" or ".md"

All previous versions are superseded. Future updates will be version-bumped here.