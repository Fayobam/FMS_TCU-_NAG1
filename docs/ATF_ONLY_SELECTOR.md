# ATF-only selector mode

Implemented as an **experimental, default-off** selector option. It is connected to
live input acquisition and the scheduler, with a dashboard toggle under
**Engine & sensors -> Selector source**. No TRRS, CAN selector or additional
reverse input is used while enabled. Physical paddles remain the shift requests.

## Operation

- `AtfRangeObserver` observes each actual ATF ADC acquisition. Open circuit,
  engaged-temperature circuit and unknown are distinct; none identifies an exact
  lever position. Qualification takes 100 ms; a sample gap above 20 ms restarts it.
- `ShiftSelector.cpp` requires the engaged circuit, fresh speed samples, N2 pulses,
  output speed at least 200 RPM and a forward ratio within 0.05 for 300 ms.
  Gears 2/3/4 also require recent N3 with N2/N3 agreeing within 100 RPM; gears 1/5
  require N3 below 60 RPM. These thresholds require validation on the actual hardware.
- A confirmed forward gear enters the existing scheduler with its observed gear.
  There is no assumed-second engagement, nearest-ratio fallback or garage pulse.
  Existing shift phases, pressure calibration, downshift overrev rejection and
  normal TCC targets/ramps are reused. Learning remains disabled as in manual mode. The existing D/Comfort calibration is used.
- Only physical paddle requests initiate shifts. Automatic scheduling, kickdown,
  launch selection and unsolicited overrev/lug shifts are disabled. Opposing
  paddles cancel; requests during shifts or without identification are discarded.
- At a stop, on loss of circuit/speed evidence, or after an unverified shift,
  gear authority is removed. Routing, MPC/SPC, TCC and torque-cut outputs are
  de-energized. A shift in progress is aborted without learning. Unknown gear is
  shown as a dash, never as second. Sustained forward evidence is required again.
- Once a shift starts, changing gear ratios are expected: acquisition matching is
  suspended through its phases, but ATF availability, N2/output pulses and fresh
  speed acquisition remain required. Existing phase feedback/completion guards apply.
- Limp remains latched. An explicit reset still requires stopped shafts and engine
  off; in this mode a fresh open ATF indication replaces the unavailable P/N input.

Two hazards are known and NOT yet addressed, pending bench traces:

- **Authority is asymmetric in time.** Acquiring it needs 300 ms of consistent
  evidence; losing it takes effect on the tick. Loss de-energizes routing, MPC/SPC
  and TCC, so the valve body falls to its hydraulic default — at road speed in 5th
  that is an uncommanded engagement, and nothing scales the response by speed. The
  bench item "engagement harshness with de-energized outputs" below is this.
- **Control-loop timing is safety-critical here.** Every gate above is a 20 ms
  freshness window, and worst-case loop timing is unmeasured (see
  [control review](CONTROL_REVIEW.md): NVS writes, flash/cache stalls, UART in the
  control task). With a wired selector such a stall is harmless; in this mode it
  revokes authority. Telemetry now reports `loopMaxUs`, `loopOverrunSoft` and
  `loopOverrunHard` — read `loopMaxUs` after a drive and compare it against the
  20 ms windows before trusting this mode on the road.

**This mode cannot command a standstill downshift or guarantee second gear after
stopping.** A retained higher gear is possible. Do not assume a P/N excursion resets
it. The mode also cannot prevent a mechanically selected reverse engagement while
moving. It never identifies or commands a reverse gear.

## Configuration and ownership

WebSocket command: `{"cmd":"selector.atf","on":true}` (or `false`). A real JSON
boolean is required. Both transitions require engine below 100 RPM, turbine/output
below 50 RPM, cruise phase and bench mode off. **Enabling additionally requires a
fresh, debounced open ATF circuit; disabling does not.** That asymmetry is
deliberate: enabling hands gear authority to inferred evidence, while disabling
hands it back to the TRRS and the normal resync path. Requiring a healthy ATF
reading to leave the mode would let a sensor stuck reading "engaged" latch it on,
and the setting is persisted, so a reboot would not clear it.
The operator must secure the vehicle: silence from failed speed sensors is not
independent proof of rest, and an open wire can resemble P/N.

The service task validates the command and enqueues it through `ControlBridge`.
Only the physics task applies the mode. The service task then persists the accepted
boolean to NVS namespace `tcu_selector`, key `atf_only`, and verifies it before ACK.
Existing raw profile/tune layouts are unchanged. NVS failure reports that RAM was
changed but persistence failed. Boot reads this setting before starting control;
missing settings default to the wired selector. Browser disconnects do not change it.

Changing mode clears queued paddles and range/gear authority. Restoring wired mode
requires fresh TRRS debounce; invalid wiring cannot restore the old D label. Held
paddles must be released. Bench mode cannot be entered while ATF-only is enabled.
Calibration edits remain allowed with engine off, stopped shafts and fresh open ATF.

Telemetry adds `atfOnly`, `atfRange` (0 unknown, 1 open, 2 engaged circuit) and
`forwardConfirmed`. Unknown-mode gear/target/expected ratios are JSON null.

## Evidence and physical limitations

[ATSG's manual](https://www.diysprinter.co.uk/reference/722.6_Tech_Service_Manual.pdf)
describes the temperature sensor in series with the P/N contact and limp recovery
following stopping and an ignition cycle (pages 20 and 29). This does not establish
that every P/N-to-Drive movement resets previously latched hydraulic valves.

[Historical Ultimate-NAG52 sensor code](https://github.com/rnd-ash/ultimate-nag52-fw/blob/5a865eb82016e7e5ed73b204ab61dba8a7115cec/src/sensors.cpp)
expects N2=0 in reverse and uses N3 as reverse input speed. The forward signatures
used here are conditional kinematic evidence, not a dedicated direction sensor.
A wiring fault, noise or coincident plausible sensor failure can defeat inference.
The existing turbine calculation is unchanged; reverse input speed remains unsupported.

Before road use, verify ATF voltage windows, D/R speed signatures, retained gears,
engagement harshness with de-energized outputs, mid-shift loss handling and TCC
behaviour on a secured transmission bench. No hydraulic behaviour is certified by
host tests. No firmware was uploaded as part of this implementation.

## Verification

`python tools/test_host.py` covers observer bounce/gaps/wrap, forward acquisition in
all five gears, rejected reverse-like/wrong ratios, manual-only scheduling, paddle
cancellation, mid-shift loss, stale speed/stop recovery, normal TCC availability,
TRRS restoration, held paddles and command interlocks. Existing controller, telemetry
and network regressions remain included.

`python tools/test_browser.py` checks confirmation before writing, unknown-state
rendering, reconnect without write replay, responsive layouts and offline preview.
`pio run -e esp32doit-devkit-v1` builds the embedded dashboard with the firmware.
