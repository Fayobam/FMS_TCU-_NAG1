# Control architecture

Current reading guide after the September 2026 reliability review. Historical
version banners and Reference plans do not override the implementation or tests.

## Read in this order

| File | Question it answers |
|---|---|
| `src/main.cpp` | What starts first, and what runs every millisecond? |
| `src/TCU_Data.h` | What are the pins, ratios, state fields and limits? |
| `src/InputManager.cpp` | How are selector, paddles and analog inputs sampled? |
| `src/SpeedReader.cpp` | How do accepted pulse intervals become shaft speeds? |
| `src/ShiftScheduler.cpp` | Who requests a shift, and who owns the outputs? |
| `src/ShiftSelector.cpp` | How does optional ATF-only mode gate gear and shift authority? |
| `src/ShiftPhases.cpp` | How does an active shift advance and finish? |
| `src/ShiftCalibration.cpp` | Where do PWM commands and learned corrections come from? |
| `src/ShiftSafety.cpp` | What overrides or rejects an ordinary shift? |
| `src/ShiftBench.cpp` | What can an explicitly enabled bench session command? |
| `src/SolenoidDriver.cpp` | How does a command become PWM on a pin? |
| `src/EngineProfile.*`, `src/TuneOverlay.*`, `src/AdaptiveMemory.*` | What is calibrated or stored? |
| `src/ControlBridge.*`, `src/WebManager.*`, `src/NetworkManager.*` | How does the browser communicate without owning control timing? |

The six Shift implementation files implement **one ShiftScheduler object**. Splitting
them introduces no tasks, queues, inheritance or extra state owners. Its header holds
the state shared between methods; all control methods run on the physics task.

## Timing and ownership

Core 1, priority 5, nominal 1 ms: consume one bounded configuration/control command;
sample inputs; update speed estimates; run scheduler; update routing-solenoid kick/hold;
poll faults; publish a rate-limited telemetry snapshot. A task watchdog monitors this loop.
The active shift retains its starting gear, load, calibration and adaptation cell.

Core 0 handles network services, JSON, browser assets and deferred NVS writes. AsyncTCP
is explicitly pinned to Core 0 at priority 3 and excluded from the physics watchdog.
TelemetryJson builds JSON from copied snapshots and owns their runtime text fields.
Core 1 performs only the bounded snapshot publication and validated command handoff;
no TCP callbacks or JSON serialization run there. Both cores still share memory and flash.
AsyncTCP
callbacks enqueue commands; they do not own a solenoid. Configuration handoff requires
stopped P/N conditions; experimental ATF-only mode uses fresh open-circuit evidence
with the engine off. See [ATF-only selector](ATF_ONLY_SELECTOR.md) for its limitations. Network disconnection is not an input to the shift state machine.
NVS writes can still stall flash/cache access: putting writes on another core is not
proof of zero control latency. Measure worst-case timing on the actual ESP32.

Speed capture interrupts record accepted pulse intervals. Filtering rejects implausibly
short intervals; averaging is bounded by a revolution and a time window. The open interval
tracks deceleration, and channel timeouts discard stale history. The new `*_signal_recent`
flags mean an unexpired measured interval exists. They are not open-circuit diagnostics.

### Sensor status on the dashboard

Telemetry exposes `n2Recent`, `n3Recent`, `outRecent` and `engRecent` individually.
No recent pulses is not automatically a fault: a shaft can be stationary, including
N3 in some valid gears. The existing gear-dependent input plausibility flag remains separate.

The ATF input shares the P/N contact on the conductor plate. The
[ATSG manual, Park/Neutral contact and fluid temperature sensor section](https://www.diysprinter.co.uk/reference/722.6_Tech_Service_Manual.pdf)
describes the series connection and availability in Reverse/forward ranges. This
firmware reports `atfCircuit` as temperature path, high/P-N-or-open, low/possible-short,
or not sampled. It does not infer an exact selector position or hydraulic valve state
from this voltage, because a broken wire can resemble an open P/N contact.

`atfMeasuredC` is null while the voltage is outside the existing validity window.
`atfSource` identifies live, held or startup-default temperature, and `atfLastValidMs`
records the last valid acquisition. The legacy `atfTemp` field retains the value used
by control. The main temperature display and charts show live measurements only;
diagnostics explicitly show the controller's held/default value. This change does not
alter the existing temperature calculation or control fallback. Wired selector decoding
remains the default. The optional ATF-only mode is described in
[ATF_ONLY_SELECTOR.md](ATF_ONLY_SELECTOR.md).

Battery/current sensing has no configured hardware input. Telemetry advertises
`batterySupported=false`, `currentSupported=false` and null measurements. The UI shows
"Sensing not configured". Actual measurement requires pin assignments, divider values
and current-sensor transfer characteristics; no pressure/PWM-to-current estimate is used.

## One shift, step by step

1. A paddle, automatic schedule or protection requests an adjacent gear.
2. `beginShift()` rejects unknown gear state, illegal routing and predicted downshift overspeed.
3. It latches the source/target and load, selects the appropriate routing solenoid and calibration.
4. Upshift: PREP → FILL → TORQUE → INERTIA → LOCK → END → CRUISING.
   Downshift: PREP → RELEASE → CATCH → LOCK → END → CRUISING.
5. Moving completion requires live ratio within ±0.05 of target, across fresh samples
   for 60 ms on upshifts and timed downshifts, 100 ms on other downshifts.
6. An unproved shift reaching its temperature-dependent backstop drops the routing
   solenoid, records a DTC and requires gear resynchronisation before another shift.

Explicit bench sequencing and shifts that begin and remain below the observation
threshold retain timer completion. They cannot establish adaptation validity. A shift
that began with observable motion cannot claim stationary success after losing sensors.
The low-speed exception is a command-based assumption, not measured gear confirmation.

## Feedback that matches this hardware

| Process | Feedback and limits |
|---|---|
| Upshift SPC | Existing feedforward PWM ramp plus optional ratio-error P trim, bounded to ±25 command points and 0–100 output |
| Fill | Existing timer; optional clutch-motion observer may end fill early when explicitly enabled |
| Shift completion | Symmetric ratio agreement and dwell; calculated clutch zero cannot override it |
| Downshift catch | Existing bounded ramp with ratio-based completion; no new uncalibrated downshift PID |
| TCC | Engine minus turbine speed, existing rate-limited slip controller; opens during shifts and with missing required feedback |
| Adaptation | Existing small bounded trims; only after observable successful shifts, valid TPS/MAP, automatic mode and permitted ATF window |
| MPC | Existing load/temperature schedule; no claim of measured hydraulic-pressure regulation |

The ratio loop requires recent N2 and output intervals, a ratio-sample update within
100 ms, trusted input speed, finite ratio, output ≥200 rpm and turbine >25 rpm.
Loss of those conditions removes the feedback trim; it does not keep integrating error.
The feedforward ramp and phase timeout remain available. N3 may legitimately stop in
some gears, so blanket “all four sensors must have pulses” logic would be incorrect.
Engine pulse presence is additionally required for TCC feedback.

`clEn` controls the ratio trim. `clKp` is its gain; zero really means zero.
`clSpeed` now controls only the optional clutch-motion fill/flare observer.
There is no hidden minimum-gain clutch-speed pressure loop. The computed clutch values
remain available for telemetry and investigation. Defaults stay off; enabling a loop
with an unvalidated gain is not a substitute for commissioning it.

## Command units and compatibility

The legacy MPC/SPC API calls its values “pressure percent”. They are **inverse PWM
command percentages**: 100 commands a de-energised pressure solenoid, 0 maximum duty.
Intermediate commands do not establish a linear pressure scale. TCC has its own normal
duty convention. Do not mix these conventions when importing tables.

`clPwr` retains the existing optional experimental torque-to-pressure approximation.
It is disabled by default and is not the recommended PWM commissioning path. Its
friction/spring seeds and linear mbar conversion are not validated valve-body calibration.
Keeping its fields preserves saved profiles; this review does not migrate or reset NVS.
No current-regulation code or additional pressure model was imported.

## Rules for small, reviewable changes

- Keep dispatch protections in the central request path; do not duplicate weaker checks.
- Keep hydraulic routing, coil polarity and kick/hold timing in explicit code and tests.
- A timeout means the controller must stop waiting; it is not proof a moving shift succeeded.
- Do not learn from bench operation, missing observations or failed shifts.
- Do not add browser, filesystem, Wi-Fi waits or allocation-heavy work to the physics task.
- Update host regression cases for changed decisions, then measure hardware timing and
  hydraulic behaviour. Compilation alone cannot validate a transmission calibration.

See [the review](CONTROL_REVIEW.md) for evidence, unresolved risks and bench checks.
