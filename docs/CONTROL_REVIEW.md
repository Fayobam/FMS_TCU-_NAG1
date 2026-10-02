# PWM and speed-feedback reliability review

## Hardware follow-up: malformed telemetry / reconnect cycle

Live workshop access to `/api/status` showed invalid UTF-8 bytes in `limpReason` while
`sampledMs` continued advancing. A browser observation recorded seven WebSocket opens,
six closes and stale telemetry in 16 seconds. The production JSON builder passed runtime
`const char[N]` snapshot buffers to ArduinoJson 7.4.3, whose literal adapter can borrow
their storage. Serialization occurred after the temporary snapshot was destroyed.

The builder now lives in `TelemetryJson.cpp` and uses explicit non-static `JsonString`
copies for runtime text. A host regression overwrites the source buffers before
serialization and checks the original strings, escaped content and empty values.
AsyncTCP is also moved from Core 1 to Core 0 at priority 3, keeping web callbacks
off the physics core. The physics snapshot publication and command validation stay intact.
Build and host checks pass (including the production serializer regression). The upload
attempt on COM9 failed before flashing: esptool reported normal boot mode `0x12` instead
of download mode. On-device confirmation remains pending a successful upload.

## Hardware follow-up: AsyncTCP boot watchdog (14 September)

Serial capture on COM9 repeatedly reported `task_wdt` timeout for `async_tcp (CPU 1)`
and `SW_CPU_RESET`, roughly one second after startup. The captured log is in ignored
`.pio/boot-monitor.log`. PhysicsTask was not listed as an overdue watchdog subscriber.
AsyncTCP 3.3.2 enables `CONFIG_ASYNC_TCP_USE_WDT` by default and waits up to 1000 ms
on its idle queue before feeding the shared task watchdog. The controller configures
that watchdog for 250 ms. Setting `CONFIG_ASYNC_TCP_USE_WDT=0` in `platformio.ini`
excludes networking from that watchdog without disabling the physics watchdog.
The corrected firmware requires upload and a fresh serial observation to confirm
on-device recovery; the host test suite did not cover library watchdog registration.

13 September 2026. This review changes software decisions, not hydraulic calibration.
The goal is understandable, bounded control using the installed sensors. It does not
claim that all upstream resources on the internet have been exhausted or that a host
test establishes road readiness.

## What the upstream sources actually support

The pinned pre-August-2023 Ultimate-NAG52 revision examined is
[`5a865eb82016e7e5ed73b204ab61dba8a7115cec`](https://github.com/rnd-ash/ultimate-nag52-fw/tree/5a865eb82016e7e5ed73b204ab61dba8a7115cec).
Its [gearbox phase code](https://github.com/rnd-ash/ultimate-nag52-fw/blob/5a865eb82016e7e5ed73b204ab61dba8a7115cec/src/gearbox.cpp)
uses bleed, fill, torque and overlap stages, with a rising overlap SPC demand and
observed shift progress. That supports a staged feedforward schedule as a useful reference.
It does not prove this project's PWM numbers, timings or adaptation rules.

Crucially, that revision's
[constant-current driver](https://github.com/rnd-ash/ultimate-nag52-fw/blob/5a865eb82016e7e5ed73b204ab61dba8a7115cec/src/solenoids/constant_current.cpp)
reads actual current and adjusts PWM. Its
[pressure manager](https://github.com/rnd-ash/ultimate-nag52-fw/blob/5a865eb82016e7e5ed73b204ab61dba8a7115cec/src/pressure_manager.cpp)
maps pressure demand and temperature to current targets; it also already contains
clutch-speed calculations. “Older” is not equivalent to “PWM-only, no current sensor”.
We can adopt scheduling principles without importing its electrical regulator or
pretending pressure demands are directly portable duty commands.

The upstream [changelog](https://github.com/rnd-ash/ultimate-nag52-fw/blob/main/CHANGELOG.md)
records clutch-speed/velocity algorithm changes in August 2023 and a much larger
OEM-calibration/pressure-model rewrite in June 2024. These are distinct from the
electrical current-control layer. Lack of current sensing does not invalidate
kinematic clutch-speed estimation; model correctness and observability are separate questions.

The original dueATC source at
[`1eb2e0916412caab8791fd56d0cbb7201e191a97`](https://github.com/tkontrol/dueATC/blob/1eb2e0916412caab8791fd56d0cbb7201e191a97/src/shiftControl.cpp)
is a closer match to inverse percentage commands. It selects commands at shift start,
holds a routing solenoid for a calibrated time, and then assigns the requested gear.
That is simpler, but blindly adopting timer-based gear confirmation would discard
valuable evidence available from our speed sensors. Its same command convention does
not establish matching MOSFET/flyback, supply-voltage or valve-body characteristics.

[7226ctrl](https://github.com/mkovero/7226ctrl/tree/b47da1085962cc0dfa26f43405f606547ac4c507)
was also surveyed: its documented features include pressure maps and N2/N3-based gear
evaluation, alongside many unrelated vehicle functions. It is an architectural comparison,
not an imported calibration or a fully line-by-line audited donor.

Local materials reviewed include the earlier architecture review/fix plan, simplification
and implementation backlogs, UN52 backport/video notes, dueATC calibration notes, automatic
map notes, accumulated project history, the shift-class PDF and rendered V14 handoff.
Several describe already superseded code. In particular, claims that percentage commands
are actual pressure, or that existing model seeds are inherently safe to transfer, are
not established by those documents. These are project notes, not independent validation.

## Findings and implemented changes

| Finding | Change |
|---|---|
| Clutch path ignored its opt-in switch | Fill/flare observer now requires `clSpeed` and live ratio feedback |
| Hidden clutch loop forced gain to at least 40 even with ratio loop disabled | Removed; one optional ratio P trim uses exactly `clEn`/`clKp` |
| Upshift completion accepted one reading or any ratio below target | Symmetric ±0.05 ratio band and fresh-sample dwell |
| Clutch-zero could confirm gear independently | Completion now always requires ratio evidence while moving |
| Backstop could treat a rolling shift losing output speed as stationary success | Latched initial observability; moving sensor loss results in unverified shift |
| Downshift catch could finish and then execute its timeout branch in the same call | Exit the phase handler immediately after confirmed success |
| Unobservable completion could enable learning/nudges | Only observable, trusted successful completion establishes adaptation validity |
| TCC could react to missing required feedback | Require ratio observation, engine pulse presence and valid load inputs; release otherwise |
| A combined edge sequence alone could obscure loss of a channel | Add individual interval-presence flags; require N2 and output for ratio feedback |
| Duplicate high-torque state and weaker kickdown overspeed check | One state field; central two-source downshift guard retained |
| Scheduler mixed unrelated responsibilities in ~1400 lines | Separate phase, calibration, protection and bench implementations without changing task ownership |
| Long README and historical plans disagreed with code | Short entry guide, current architecture document and historical reference index |

The UI layout is unchanged. Two profile labels now describe the fill observer and
experimental pressure model accurately; the offline preview is regenerated. API keys,
NVS structure layouts, pressure/timing table defaults, routing and coil drive polarity
are preserved. The pressure-model option remains explicit and off by default for
profile compatibility; it is not recommended as the baseline commissioning path.

## Why some complexity stays

Adjacent-only routing, source-gear verification, overspeed rejection, reverse handling,
phase timeouts, kick/hold coil drive, input filtering, load latching, and the distinction
between up/down and powered/coasting shifts have concrete jobs. Removing them simply
to reduce line count would remove safeguards or merge physically different conditions.
No new CAN torque simulation, clutch-pressure solver, PID stack or current-driver library
was added. Downshift pressure remains its existing schedule with feedback-based completion;
adding a new controller there requires observations and calibration, not just more code.

## Remaining engineering questions

These are explicit limits, not features verified by this change:

- **PWM-to-pressure relationship:** validate duty polarity, frequency, flyback behaviour,
  supply range, warm-coil behaviour, fill baseline and holding authority on this driver
  and valve body. Without current/pressure measurement, pressure is not observable directly.
- **Sensor-loss policy:** a stopped sensor and a broken wire can look identical. Recent
  pulses are necessary evidence, not electrical continuity proof. N3 can legitimately
  stop. The existing N2/N3 plausibility check is gear-dependent and delayed; it is not a
  complete diagnostic. Standstill launch and reverse selection still have inherited
  assumptions when sensors are all silent.
- **Adaptation:** existing flare/bind heuristics can confuse insufficient fill with load
  changes, braking or mechanical faults. Fill-motion timing is censored by the fill
  timer and cannot identify arbitrarily late bite. This review does not claim optimal
  learning or introduce new learned pressure semantics. ATF fallback can retain the last
  temperature, so sensor-health gating there remains incomplete.
- **Persistence and timing:** the existing adaptive dirty-mask flush can race with new
  learning, and DTC persistence also shares mutable state. UART logging in the control
  task and ESP32 flash/cache stalls still need measured worst-case timing and follow-up.
  They were not disguised as fixed by moving code between files.
- **High-load slip protection:** the inherited limp condition suppresses its ratio-slip
  check at high TPS/boost. Replacing that policy requires a defined recovery strategy;
  deleting the load gate blindly could cause false limp under converter/transient conditions.
- **Feedback calibration:** the ratio loop remains off by default. Commission its sign
  and gain on recorded/bench signals before enabling it. The ±25 trim ceiling and
  existing ramps are bounds, not proof that their hydraulic response is safe.

## Validation and commissioning

Automated checks cover legal routing, boot output state, downshift overspeed including
dead output, successful/failed shifts, stationary/bench sequencing, explicit observer
enable, disabled loop authority, bounded trim, stale samples, undershoot, individual
channel loss, TCC release and adaptation eligibility. Network/command handoff regressions
are retained. Final verification completed 14 September 2026:

- `python tools/test_host.py`: 25 control cases pass; command/handoff and network suites pass.
- `python tools/test_browser.py`: all pages, four viewport sizes, editing/validation,
  reconnect/no-replay and standalone preview checks pass.
- `python tools/check_dashboard.py data/index.html`: bundled scripts pass structural checks.
- PlatformIO ESP32 build passes: RAM 73,216 / 327,680 bytes; flash 1,247,745 / 2,097,152 bytes.
- `git diff --check`: passes. No upload, flash operation or road test was performed.

Control regression tests operate on host stubs; the new physical pulse-presence production
code is target-build checked but still requires signal-generator validation on the ESP32.

Before vehicle use, replay independent N2/N3/output/engine signals through the real capture
pins. Check all eight adjacent shifts, wrong-ratio plateaus, one-sample target spikes,
signal loss before/during/after a shift, cold/hot backstops and stationary sequencing.
Scope routing kick/hold and MPC/SPC/TCC duty throughout. Repeat while Wi-Fi reconnects,
browsers reload and NVS writes occur, measuring the longest physics-loop execution.
Then establish hydraulic command baselines under controlled conditions before trying
closed-loop gain or adaptation. No device was flashed or road-tested in this review.
