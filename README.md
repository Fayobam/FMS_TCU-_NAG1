FMS TCU - 722.6 Standalone Transmission Controller

An open-source, ESP32-based standalone Transmission Control Unit (TCU) built specifically for the Mercedes-Benz 722.6 (NAG1) 5-speed automatic transmission.

Designed for high-performance applications (specifically the M111 Supercharged platform), this firmware bypasses the restrictive OEM TCU and provides complete, granular control over line pressure, shift timing, and torque converter lockup via a live Web-Socket Tuning Dashboard.


Key Features

Motorsport Adaptive Memory: 16x16 3D matrices storing thousands of shift parameters.

Ultimate-NAG52 Baselines: Pre-loaded with reverse-engineered OEM clutch volume data for safe first drives.

Dynamic TCC Slip Controller: Actively targets 50-RPM slip for NVH, or 100% lockup under boost.

Predictive Downshift Guard: Mathematically prevents "Money Shifts" that would over-rev the engine.

Limp Mode Kill-Switch: Real-time slip-detection halts shifts and maxes line pressure if a clutch fails.

Live Web Studio: Bi-directional Web Dashboard hosted entirely on the ESP32 (No internet required).

Dual-Core FreeRTOS: 1000Hz Physics Engine pinned to Core 1, Web Sockets pinned to Core 0.


Hardware Requirements

Microcontroller: ESP32 DevKit V1 (Dual-Core, 240MHz).

Sensors Needed: Throttle Position (TPS), Manifold Pressure (MAP), Engine RPM.

Custom Speed Sensor: A custom external output shaft speed sensor (e.g., 24-tooth reluctor) is required, as the 722.6 lacks an internal output speed sensor.

Power Electronics: High-side/Low-side MOSFETs capable of driving 4-ohm inductive loads (solenoids), with appropriate flyback diodes and 3.3V-to-12V logic level shifters.


Firmware Installation

Install PlatformIO for VS Code.

Clone this repository: git clone https://github.com/fayobam/FMS_TCU.git

Click Build and Upload to flash the C++ firmware to the ESP32.

Click Upload Filesystem Image in PlatformIO to upload the HTML Web Dashboard to SPIFFS.


The Tuning Dashboard

Once flashed, the ESP32 will broadcast a WiFi network named FMS_TCU (Password: shiftfast). Connect your phone or laptop, and navigate to http://192.168.4.1 to access the live Telemetry and Calibration Studio.


Acknowledgments

A massive thank you to the Ultimate-NAG52 project by rnd-ash for reverse-engineering the EGS52 parameters and shedding light on the mechanical quirks of the W5A330.

Disclaimer: This is experimental motorsport firmware. Use at your own risk. Incorrect tuning parameters can instantly destroy your transmission or engine.
