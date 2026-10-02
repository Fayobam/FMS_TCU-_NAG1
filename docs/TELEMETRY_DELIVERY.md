# Telemetry delivery

The control snapshot and WebSocket cadence remain 100 ms (10 Hz), configured in
`src/TelemetryConfig.h`. The browser still marks snapshots stale after 2000 ms.
Increasing the broadcast rate is not a remedy for a two-second delivery interruption.
No shift, sensor or solenoid logic changes are part of this update.

## Delivery changes

- The service task offers due telemetry before network maintenance, command processing,
  configuration serialization and NVS writes. Large replies already on TCP cannot be
  preempted; this is scheduling priority, not a guarantee of delivery latency.
- Pending bulk reply types are serviced one per loop rather than all at once.
- The bounded JSON transmit buffer is 3072 bytes, with room for diagnostic counters,
  long escaped fault strings and numeric formatting. Oversize packets are counted,
  not transmitted partially. `/api/status` can expose the counter even if WebSocket
  telemetry cannot fit. Unavailable/slow clients still cannot block control.
- The UI clears its warning on fresh telemetry arrival instead of waiting for the
  next 500 ms display timer. A newly opened socket shows Awaiting telemetry.
- Repeated snapshots do not refresh freshness. A new snapshot already stale when
  serialized does not refresh it either. Obsolete socket callbacks are ignored.

## Network -> Telemetry delivery

| Reading | Interpretation |
|---|---|
| Last telemetry arrival | Browser time since the last valid telemetry message. |
| Snapshot age (minimum) | Snapshot age at firmware serialization plus browser time since receiving that advancing snapshot. Network transit time is not measured. Older firmware without `snapshotAgeMs` can only be checked for continued snapshot advancement. |
| Largest receive gap | Maximum gap between telemetry arrivals on a socket in this page session. Browser background throttling can also increase it. |
| WebSocket reconnects | Connections reopened after the first successful connection in this page session. |
| Invalid messages | Malformed JSON/telemetry rejected by this browser session. |
| Controller service gap (max) | Largest interval between Core 0 web-service updates since boot; includes scheduling delays and work in the previous iteration. |
| Telemetry queue skips | Cumulative per-client telemetry attempts skipped/rejected due to send capacity; a slow client need not affect other clients. |
| Oversize telemetry drops | Telemetry packets withheld because they would fill/exceed the bounded buffer. |

`Telemetry delayed` means valid telemetry arrivals have stopped long enough to go
stale. `Snapshot stale` means messages have arrived recently but the snapshot remains
old. Neither diagnoses the underlying fault on its own. Firmware counters reset at
boot and browser counters at page reload. Read values as a trend around an event.

The ESP32 refused HTTP connections during this change, so no live root cause was
confirmed. Possible bulk-frame congestion is not eliminated by scheduling changes.
Bench observation after flashing is still required; no firmware was uploaded.

## Verification

Host contract tests check snapshot-age timer wrap and bounded serialization with
escaped long strings and full-sized counters. Browser tests check repeated snapshots,
old snapshots arriving anew, invalid JSON, immediate fresh recovery, reconnects,
no command replay, responsive layout and the directly opened offline preview.
