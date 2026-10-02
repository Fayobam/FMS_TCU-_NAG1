# Networking and interface audit

Audited the working tree on 2026-09-13 before replacing the interface. The tree
already contained uncommitted control, UI and build changes; those control changes
were retained. This document describes the code actually inspected, rather than
the older version claims in README comments.

## Existing project and timing architecture

| Area | Audited implementation |
|---|---|
| Build | PlatformIO; ESP32 Dev Module; pioarduino platform tag `51.03.04` |
| Framework | Build confirms Arduino ESP32 3.0.4; framework libs 5.1.0, IDF 5.1 family |
| Networking libraries | Framework WiFi and DNSServer; AsyncTCP 3.3.2; ESPAsyncWebServer 3.6.0; ArduinoJson 7.4.3 |
| Browser assets | `data/index.html`, optional `data/three.min.js`; gzip HTML generated into `include/tcu_index_html.h` |
| Filesystem | SPIFFS, originally `begin(true)`; only optional Three.js needed it |
| Control | `main.cpp`: Core 1 priority 5, nominal 1 ms period; watchdog 250 ms; overrun DTC above 1500 µs |
| Dashboard | Core 0 priority 1, 5 ms service period; original broadcast 100 ms |
| TCP callbacks | Core 1 priority 3, below physics; 8192-byte task stack |
| Capture | `SpeedReader`: hardware MCPWM edge timestamp interrupts, short plausibility-filtered ring writes; period averaging in task context |
| Inputs | Debounced PRND and edge-triggered paddles each tick; TPS/MAP/ATF ADC1 channels staggered round-robin |
| Outputs | Inverted pressure-% MPC/SPC; normal TCC duty; routing coil 60 ms kick then hold; optional lock and torque-cut feature gates |
| Shift engine | Class-based PREP/FILL/TORQUE/INERTIA or RELEASE/CATCH, followed by LOCK/END; pressure updates quantized at 20 ms, exit checks each control tick |
| Persistence | Preferences namespaces `engine_prof`, `tcu_tune`, `tcu_adapt2`, `tcu_dtc` |
| Logging | Serial events; persistent DTC occurrence counts; 400-sample shift trace at 2 ms, transmitted after completion; browser CSV export |

Control modules inspected: `ShiftScheduler`, `SpeedReader`, `InputManager`,
`SolenoidDriver`, `AdaptiveMemory`, `EngineProfile`, `TuneOverlay`, `DtcManager`,
`TCU_Data`, `AutoShiftMap`, and their integration/tests. The existing browser code
was inspected for commands, values, editing semantics and exports only.

## Safety behavior retained

Predictive overrev upshifts, two-source money-shift prediction, legal adjacent
routing, gear resynchronization after unverified shifts, reverse-abuse pressure
handling, P/N engagement grace, input-speed plausibility, TPS/MAP rail handling,
flare/bind confirmation, temperature-gated learning, limp behavior and guarded
recovery remain implemented by the original control modules. Their source was
not rewritten by this change. Existing calibration defaults, magic values,
sensor assignments, ratios, PWM polarity and timing were retained.

Bench behavior deserves an explicit distinction: a scheduler comment claimed
road speed exits test mode, but the implementation and existing tests deliberately
allow test mode to persist with simulated speed. That behavior was preserved.
The replacement command boundary requires engine off, stopped shafts and P/N to
**enter** bench mode; output activation also requires stopped shafts. Output
release and bench exit remain available without that activation gate. Closing a
browser does not automatically exit bench mode or release an existing latch.

## Existing web contracts

There were no REST calibration routes. All changes and configuration reads used
JSON text messages at `/ws`. Telemetry and configuration responses were broadcast
to connected clients. The replacement retains their names and data shapes.

### HTTP routes

| Route | Original behavior | Replacement |
|---|---|---|
| `GET /` | Embedded gzip dashboard | Complete replacement, embedded gzip |
| `GET /index.html` | Generic 204 | Same replacement dashboard |
| `GET /api/status` | Absent | Read-only telemetry snapshot JSON, no-store |
| `GET /three.min.js` | SPIFFS file or 404 | Retained for compatibility; new UI does not request it |
| `/generate_204`, `/gen_204`, `/fwlink/`, `/redirect` | HTTP ANY, tiny 204 | Retained |
| `/hotspot-detect.html`, `/library/test/success.html`, `/canonical.html`, `/kindle-wifi/wifiredirect.html` | HTTP ANY, tiny HTML success | Retained |
| `/connecttest.txt`, `/ncsi.txt`, `/success.txt` | HTTP ANY, small text probe | Retained |
| Other paths | 204 even for missing assets | 404; OPTIONS remains 204 |

Navigation uses URL fragments, so page changes need no server-side SPA rewrite.
The generated HTML includes all CSS and JavaScript. Source files under `data/css`
and `data/js` are build inputs; they are not additional firmware HTTP routes.

### WebSocket commands

| Command | Payload / response | Mutation and persistence |
|---|---|---|
| `get_profile` | `profile_data` | Read engine profile |
| `set_profile` | Any supported profile subset | Validate complete request; stage profile; apply at control boundary; save |
| `param.list` | `param_list`, `n`, `params[]` | Read registry: idx/id/name/group/kind/min/max/rows/cols/unit/help/v |
| `param.set` | `idx`, `row`, `col`, `val` | Integer bounds and dimensions validated before narrowing; staged live apply |
| `param.persist` | No data | Save applied tuning; engine off, stopped, P/N required |
| `param.reset` | No data | Stage firmware defaults; apply and persist |
| `get_cells` | `cell_data`: classes=4, shifts=4, tbins=4, data[192] | Read coherent adaptation snapshot |
| `set_cells` | Exactly 192 integers | Fill cycles −5..5; pressure trims −15..15; apply and schedule NVS flush |
| `get_dtcs` | `dtc_data.dtcs[]`: code/name/count/active/lastMs | Read DTC history |
| `clear_dtcs` | No data | Clear on control core; existing throttled persistence |
| `limp_reset` | No data | Request controller recovery; no bypass of scheduler gates |
| `adapt_nudge` | `dir`: −1 or +1 | Existing last-shift cell nudge via scheduler |
| `test_mode` | Boolean `on` | Existing entry/exit request with stronger entry gate |
| `test_prnd` | `v`: one of P/R/N/D/4/3/2/1 | Bench-only selector at control boundary |
| `test_paddle` | `dir`: −1 or +1 | Bench-only paddle request |
| `test_sol`, `test_io` | `id`: y3/y5/y4/mpc/spc/tcc/rp/tq; optional on and v | Existing aliases; validated 0..100 command, off maps to −1 |
| `network.get` | `network` response | New, no password fields |
| `network.set` | op mode/add/remove/up | New, validated NVS-backed network management |

`requestId` is an optional unsigned correlation value. Requests now receive a
client-specific `command_result` with `requestId`, `ok`, and `message`. Legacy
clients can ignore this additional message. Configuration data remains broadcast
for compatibility. Invalid transport frames get `type:error`; clients must send
one complete text frame, less than 3072 bytes. Four command envelopes are bounded
in RAM; overload is reported, never silently overwritten. Four WebSocket clients
are permitted. Commands from closed sessions that have not been dispatched are
discarded. A disconnect after dispatch can leave an action applied without its
acknowledgement; the UI reports unknown outcome and never replays it.

### Profile contract

`profile_data` includes `torque[64]`, immutable `rpm[8]`/`map[8]` axes, `tmax`,
`overrev`, `lug`, `engPpr`, `outPpr`, `clEn`, `clKp`, `kmhRpm`, `transVariant`,
`tcStall`, `tcCoupSr`, `clPwr`, `pFull`, `clSpeed`, `coefStat`, `coefRel`,
`coefCold`, `coefHot`, `applyFric[4]`, `relFric[4]`, `applySpring[4]`,
`relSpring[4]`, `tpsC`, `tpsW`, `map0`, `mapV`, `fillp[4]`, and `fillt[4]`.
Changing transmission variant retains its existing clutch-model reseeding semantics.
Scalar and array limits are centralized in `CommandValidation.cpp`. Invalid
values are rejected rather than clamped or converted from strings. TPS WOT must
exceed closed voltage. No defaults are changed merely by loading the interface.

Actual automatic schedules are **road km/h by TPS**, not RPM by TPS. The 8×8 engine
torque surface is RPM by MAP kPa. Cruise pressure is gear by the existing computed
load bins (12.5 units), while the inertia target uses 0–100% estimated torque load.
The replacement labels those different axes explicitly.

### Telemetry and logging

Existing telemetry keys: `prnd`, `gear`, `tgt`, `mode`, `modeName`, `engRpm`,
`turbRpm`, `outRpm`, `tps`, `map`, `mpc`, `spc`, `shiftTime`, `ratio`, `kmh`,
`flare`, `bind`, `tccPwm`, `tccTarget`, `tccActual`, `limp`, `limpReason`, `safety`,
`atfTemp`, `htMode`, `phase`, `revAbuse`, `testMode`, `tpsV`, `mapV`, `atfV`,
`din`, `tout`, `n2`, `n3`, `tEstNm`, `loadPct`, `shiftClass`, `pdType`,
`onClutch`, `offClutch`, `tInput`, `tqCut`, `dtcN`, `spdHwOk`, `inTrust`,
`tpsOk`, `mapOk`, `clients`. `clients` now counts WebSocket clients rather than
associated AP stations. Added `sampledMs`, `intervalMs`, `expectedRatio`,
`targetRatio`, `heap`, `fsOk`, `assets`, and `atfSignalOk`.

`shift_trace`: cls/pd/from/to/n plus arrays t/ph/spc/mpc/ratio/eng/turb/out/clErr/fl/
onClutch/offClutch. Ratio and control error retain ×1000 encoding; t is ms.
Capture is bounded at 400 samples; long shifts may truncate. Sending a completed
trace uses direct integer serialization and one shared WS payload, not a large
JSON DOM per client. It waits for sufficient heap and a responsive client; it
does not hold up control. Slow telemetry clients drop frames instead of building
an unbounded backlog.

There is no KISS phase, battery-voltage input, current feedback, independent
per-channel speed-valid flags, or separately commanded on/off clutch pressure.
ATF status is derived from the existing voltage acceptance window. Displayed shift
elapsed time is explicitly browser-observed; profile-based target RPM is a shaft
sync target, not the scheduler's internal inertia trajectory. The lightweight
power-path schematic retains the previous gear-element illustration semantics and
is labeled inferred; it is not a measured hydraulic state. No Three.js is loaded.

## Failure analysis and replacement boundaries

Original AP startup cycled WIFI_OFF/AP with 600 ms of explicit delays, plus a
second delayed attempt, **before** control-task creation. It supported no station
networks or reconnect management. Captive DNS remained tied to its boot AP IP.
README STA/mDNS instructions did not match this implementation. Original boot
could auto-format SPIFFS, missing URLs returned 204, and one command slot silently
dropped concurrent commands. Several configuration arrays and the bench selector
were changed by Core 0 while control read them on Core 1.

Replacement data flow:

```
Core 1: ControlBridge command application -> sensors -> scheduler -> outputs/DTC
                       ^                                 |
                 typed mailbox                   bounded snapshot publication
                       |                                 v
Core 0: WS envelope -> validation -> mailbox       JSON / WebSocket / REST
         NetworkManager + DNS + NVS + web initialization
```

In `main.cpp`, command consumption is immediately before the existing input,
speed, scheduler and output update sequence. No controller module reads Wi-Fi
state. A release/acquire mailbox has one owner on each core; JSON parsing and
NVS work never run inside it. Profile/tuning/cell changes are copied only at a
control boundary when engine off, shafts stopped, P/N and not in a shift or bench
mode. The telemetry/adaptation snapshot uses a short lock only for bounded POD
copies, never serialization. Async callbacks only enqueue command bytes. Ordinary
network loss, browser closure and backpressure cannot stop the control task.

NVS is still flash: SDK flash-cache suspension can affect timing across both
cores, even when the write originates on Core 0. Existing adaptation/DTC flush
behavior remains; new operator-initiated calibration/network saves require stopped
conditions. Actual worst-case timing needs measurement. Separate tasks do not
isolate a chip-wide panic, memory corruption or power failure; no claim of such
fault containment is made.

## Network policy

`tcu::NetworkManager` avoids the Arduino core's own global class of the same name.
It uses the framework WiFi/Preferences/ESPmDNS libraries; no new firmware package.

- Up to five SSIDs and credentials in `tcu_network`, separate from calibration.
- Optional untracked `WifiSecrets.h` seeds a network on blank settings; credentials
  are neither logged nor returned. They are stored with standard NVS semantics.
- Async scan, normally no more than once per 60 seconds; scan failure/timeout is bounded.
- Visible remembered networks tried in configured order, with 12 seconds per
  association attempt. No Wi-Fi connection wait loop.
- Automatic mode enables open fallback `7226-TCU`, `192.168.4.1`, after 20 seconds
  unavailable; immediately if no remembered networks. Failed AP start retries at
  10-second intervals. AP+STA stays available after subsequent STA connection.
- Station-only and AP-only modes are explicit user choices. The last network cannot
  be removed while station-only is selected. AP-only does not scan.
- Connected STA does not scan periodically. Loss restarts recovery and fallback timing.
- mDNS advertises `tcu.local`, retries after failure, and restarts after transitions.
- Network settings acknowledge before a deferred restart. DNS starts/stops with AP
  state. STA/AP channel changes can still interrupt a browser temporarily.

## Packaging and offline review

`data/index.html`, `data/css/app.css`, and five `data/js` modules are the editable
source. `tools/package_web.py`, called by `extra_script.py`, inlines them and emits
deterministic gzip PROGMEM bytes. Missing source assets fail the build instead of
producing an incomplete page. SPIFFS and partition offsets are retained, without
format-on-failure; a missing/unmounted filesystem does not affect the new UI.

`preview.html` is a separate self-contained **offline simulation**, created by
`python tools/build_preview.py`. Open it directly with a desktop browser. It uses
the same UI source, simulated network/telemetry and local-only command handling.
It needs no server, package install, Wi-Fi or upload to view. Reload clears demo
changes. Preview code and fixture values are never embedded in the firmware.

## Validation and bench work

Automated tools:

- `python tools/test_host.py`: existing 18 control regression tests plus malformed
  command/range/array/hysteresis validation, control mailbox gating, coherent
  snapshot behavior, and actual NetworkManager code against controlled WiFi/NVS stubs.
- Network scenarios cover no networks, unknown networks, known priority, failed
  association to next network, STA connected/lost, fallback timing, AP startup
  failure/retry, station-only/AP-only, scan throttling and millis wrap.
- `python tools/check_dashboard.py data/index.html`: bundled-script structural checks.
- `python tools/test_browser.py`: headless Edge, protocol simulator, four viewport
  sizes, page navigation, editing, malformed values, confirmation/acknowledgement,
  safe SSID rendering, reconnect/no replay, stale data, repeated reload and offline
  `file://` preview with browser networking disabled. Test-only Playwright is not
  a firmware or UI dependency.
- `pio run -e esp32doit-devkit-v1`: actual target compilation and link.

Required device bench checks (not claimed as executed):

1. Boot without reachable networks: scope control outputs before AP initialization;
   confirm control continues and AP appears at the documented address.
2. Known workshop/home/hotspot: verify DHCP, priority, mDNS and reachability from
   both AP and STA; repeat with wrong password and unknown-only scan results.
3. Remove station coverage and restore it; verify fallback remains reachable and
   control loop timing/overrun DTCs during scans, association and channel changes.
4. Four browsers, slow client, repeated HTTP reloads, trace transfers, command bursts,
   and disconnect mid-edit; record minimum heap and task stack high-water marks.
5. Boot with unformatted/missing SPIFFS; entire dashboard should still load; confirm
   unrelated missing URLs are 404 and connectivity probes remain small.
6. Verify rejected running calibration, safe-stop application, NVS persistence after
   power cycle, and unchanged scheduler gates. Perform output tests only on a secured
   bench; exit test mode explicitly before any driving.

No firmware or filesystem was uploaded by this implementation work.

Final automated results on 2026-09-13: target build passed; 18 existing control
regressions passed; command-boundary and network-state suites passed; bundled
script checks passed; headless Edge responsive, reconnect, concurrent-page,
keyboard/paste and offline-preview checks passed. Final embedded UI: 69,599 bytes
uncompressed / 21,752 bytes gzip. Target static RAM: 73,192 / 327,680 bytes;
application flash: 1,247,177 / 2,097,152 bytes. These build figures exclude runtime
heap/stack peaks, which remain part of the device bench checks above.
