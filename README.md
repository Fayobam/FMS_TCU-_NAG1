FMS TCU - 722.6 Standalone Transmission Controller

An open-source, ESP32-based standalone Transmission Control Unit (TCU) built specifically for the Mercedes-Benz 722.6 (NAG1) 5-speed automatic transmission.


Key Features

Motorsport Adaptive Memory: 16x16 3D matrices storing thousands of shift parameters.

Ultimate-NAG52 Baselines: Pre-loaded with reverse-engineered OEM clutch volume data for safe first drives.

Dynamic TCC Slip Controller: Actively targets 50-RPM slip for NVH, or 100% lockup under boost.

Predictive Downshift Guard: Mathematically prevents "Money Shifts" that would over-rev the engine.

Limp Mode Kill-Switch: Real-time slip-detection halts shifts and maxes line pressure if a clutch fails.

Live Web Studio: Bi-directional Web Dashboard hosted entirely on the ESP32.

Dual-Core FreeRTOS: 1000Hz Physics Engine pinned to Core 1, Web Sockets pinned to Core 0.

Hardware Requirements

Microcontroller: ESP32 DevKit V1

Sensors Needed: Throttle Position (TPS), Manifold Pressure (MAP), Engine RPM.

Custom Speed Sensor: External output shaft speed sensor (e.g., 24-tooth reluctor) is required.

Disclaimer: This is experimental motorsport firmware. Use at your own risk.
