> Historical project description, preserved for context. Use ../README.md and ../docs/CONTROL_ARCHITECTURE.md for current behavior.

# FMS TCU — 722.6 (NAG1) Standalone Transmission Controller

An open-source, ESP32-based standalone Transmission Control Unit (TCU) for the Mercedes-Benz
**722.6 / W5A330 (Small NAG1)** 5-speed automatic. It bypasses the restrictive OEM EGS and gives
complete, granular control over line/shift pressure, shift timing, and torque-converter lockup —
tuned live from a self-hosted WebSocket dashboard.

Built for a high-performance build: **Mercedes 190E, M111.985 2.3 L + Eaton TVS1320 supercharger
(~1.2 bar), engine on rusEFI.** Manually commanded (paddle) shifts, with automatic safety shifts
(overrev / lug), a coast-down scheduler, and kickdown.

> **Firmware: V16.** Status: compiles clean; **bench-validation pending — not road-ready.**
> The authoritative engineering manual + live open-items tracker is
> [`src/Summary_of_knowledge_and_progress.md`](src/Summary_of_knowledge_and_progress.md);
> the ATSG-grounded design spec is in `Reference/`. Read those before flashing to a real car.

## Architecture

- **ATSG shift-class engine.** Every shift is classified once (from latched **torque**, not raw
  TPS) into one of four classes — `POWER_UP / COAST_UP / POWER_DOWN (sprag | timed) / COAST_DOWN`
  — and run through a single phase machine (PREP → FILL → TORQUE → INERTIA → LOCK → END for
  upshifts; PREP → RELEASE → CATCH → LOCK → END for downshifts). Pressure commands are quantised
  to 20 ms (ATSG p.80); exit predicates run at 1 kHz.
- **Period-measurement speed sensing (MCPWM hardware capture).** N2/N3/output/engine edges are
  timestamped at 80 MHz and averaged over a full revolution — **sub-rpm at all speeds**, far
  better than pulse-counting. Refreshes at 200 Hz internally.
- **Closed-loop SPC** in the upshift inertia phase: feedforward ramp + proportional trim toward a
  time-scheduled ratio sweep (web-tunable gain; falls back to open-loop if disabled).
- **Adaptation v2:** per-cell trims indexed `[class][shift][torque-bin]`, ATF-gated, persisted to
  NVS on Core 0 (not per shift → flash-wear safe).
- **Per-engine `EngineProfile` (NVS, web-editable):** 8×8 RPM×MAP torque surface, overrev/lug
  limits, TPS/MAP calibration, sensor PPR, fill baselines, closed-loop gain. An engine swap is a
  data change, not a recompile.
- **Dynamic TCC slip controller:** targets 50 rpm slip for NVH, forces open under boost/shift;
  rate-limited and 20 ms-quantised.
- **Safety:** predictive overrev upshift, lug downshift, money-shift guard on every downshift,
  reverse-abuse failsafe, and a ratio-classifying limp mode.
- **Dual-core FreeRTOS:** 1 kHz physics engine pinned to Core 1, WebSocket/telemetry on Core 0.

## Hardware

- **Microcontroller:** ESP32 DevKit V1 (dual-core, 240 MHz).
- **Sensors:** Throttle Position (TPS, ADC1), Manifold Pressure (MAP, ADC1), Engine RPM, plus the
  transmission's internal N2/N3 speed sensors.
- **Engine RPM feed:** speed sensing is period-based, so any clean fixed-PPR square wave works —
  a **rusEFI tach output** is recommended over the raw 60-2 crank signal (the missing-tooth gap is
  not handled). PPR is set live on the dashboard. **Level-shift any 5/12 V signal down to 3.3 V.**
- **Output speed:** a custom external output-shaft reluctor (the 722.6 has no internal output
  sensor); its PPR is configurable on the dashboard.
- **Power electronics:** high-/low-side MOSFETs for the 4-ohm inductive solenoids, with flyback
  diodes and 3.3 V ↔ 12 V level shifting.

## Firmware Installation

1. Install **PlatformIO** for VS Code.
2. Clone: `git clone https://github.com/Fayobam/FMS_TCU-_NAG1.git`
3. **Build & Upload** to flash the firmware to the ESP32. The dashboard is baked into the binary — no extra filesystem step for telemetry/tuning.
4. No filesystem upload is needed for the interface. CSS and JavaScript are bundled into the firmware; missing SPIFFS does not prevent the dashboard loading.

> Flashing a firmware that changes the `EngineProfile` struct re-seeds its NVS to defaults
> (the magic tag bumps) — re-enter any custom torque-surface / calibration values afterward.

## The Tuning Dashboard

Open [`preview.html`](preview.html) directly in your computer browser to review the
interface **without uploading firmware or connecting to the controller**. This
self-contained file shows clearly labeled simulated telemetry. Demo edits stay in
that browser session and reset on reload. Regenerate it after UI edits with
`python tools/build_preview.py`.

On hardware, automatic networking tries remembered networks and advertises
**http://tcu.local**. If none can be reached, connect to **7226-TCU** and open
**http://192.168.4.1**. Network settings support five remembered SSIDs, priority,
STA-only and AP-only modes. Passwords are not returned to the browser.

Pages: live overview, shift analysis and CSV export, calibration heatmaps,
adaptation, engine/sensor profiles, diagnostics, network settings, and guarded
bench tools. Calibration changes require engine off, stopped shafts and P/N.
Maps are staged locally; apply and NVS save are separate actions. Loading a page
never changes calibration. Bench mode must be exited explicitly.

Control starts before networking. The original sensor, shift, solenoid and safety
algorithms are retained. Read the [network/UI audit and API contracts](docs/NETWORK_UI_AUDIT.md)
for exact behavior, changed validation rules, and the required on-device bench checks.

Validation commands:

- `pio run -e esp32doit-devkit-v1` ? target firmware build.
- `python tools/test_host.py` ? control regressions, command boundary and network-state tests.
- `python tools/check_dashboard.py data/index.html` ? check the bundled scripts.
- `python tools/test_browser.py` ? optional headless Edge tests with test-only Playwright.


## Acknowledgments

Thanks to the **Ultimate-NAG52** project by rnd-ash for reverse-engineering the EGS52 parameters
and the mechanical quirks of the W5A330, and to the **ATSG 722.6** technical service information
that grounds the shift-class architecture.

---

**Disclaimer:** Experimental motorsport firmware — use at your own risk. It is **not road-validated**;
incorrect tuning parameters can instantly destroy your transmission or engine. Bench-test first.
