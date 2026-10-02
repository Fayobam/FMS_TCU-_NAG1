# FMS 722.6 TCU

ESP32 standalone controller for the Mercedes 722.6 transmission. The hardware uses
PWM solenoid drivers, N2/N3, external output speed, engine speed, TPS, MAP and ATF
inputs. It has no solenoid-current or hydraulic-pressure feedback.

**Control changes require bench validation before driving. No road validation is claimed.**

## Start here

1. [Control architecture and reading guide](docs/CONTROL_ARCHITECTURE.md): ownership,
   shift sequence, feedback, calibration and protections.
2. [Research and reliability review](docs/CONTROL_REVIEW.md): historical Ultimate-NAG52,
   dueATC comparison, changes made and outstanding hardware-dependent questions.
3. [Network/UI contracts](docs/NETWORK_UI_AUDIT.md): serving, commands and reconnect behaviour.
4. [Historical references](Reference/README.md): older plans and design material.

## Build and check

This is a PlatformIO project. `platformio.ini` pins Arduino-ESP32 3.0.4 through
pioarduino and the web libraries. The default target is an ESP32 DevKit.

```sh
pio run
python tools/test_host.py
python tools/test_browser.py
```

Host tests use g++, firmware dependencies installed by PlatformIO, and local peripheral
stubs. Browser tests use Playwright/Edge; see the test script for the local tool path.
They do not validate hydraulic pressure, pulse-capture timing or actual shift quality.

## View the interface on a computer

Open [preview.html](preview.html) directly in a browser. It uses simulated telemetry,
needs no ESP32 and sends no controller commands. Rebuild it after changing UI assets:

```sh
python tools/build_preview.py
```

Edit the source files under `data/`. The firmware build bundles them into the generated
`include/tcu_index_html.h`. Do not edit that generated header or `preview.html` by hand.
The dashboard is embedded; a filesystem upload is not needed for it to load.

## Repository map

| Folder | Purpose |
|---|---|
| `src/` | Firmware, divided into control, hardware, configuration and networking modules |
| `data/` | Browser interface source |
| `include/` | Generated embedded UI header |
| `test/` | Host regression cases and hardware/network stubs |
| `tools/` | Packaging, offline preview and test tools |
| `docs/` | Current architecture and review documents |
| `Reference/` | Historical research and design notes, not current specifications |
| `.pio/` | Ignored build products, test artifacts and downloaded research |

Local Wi-Fi credentials belong in ignored `src/WifiSecrets.h` or device network settings.
Stored calibration layouts and defaults are unchanged by the September control review.
