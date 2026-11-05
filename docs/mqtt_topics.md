# MQTT topics and message formats

## Node 1 (drive)

| Topic | Direction | Payload | Notes |
|---|---|---|---|
| `duckmopper/node1/cmd` | to node 1 | `start`, `stop`, or `estop` | QoS 1, not retained. `stop` and `estop` do the same thing: wheels and scrubbers stop and stay stopped until `start`. |
| `duckmopper/node1/telemetry` | from node 1 | `{"dist_cm":42.5,"state":"DRIVING"}` | Every 500 ms. `dist_cm` is -1 when there is no echo. |
| `duckmopper/node1/status` | from node 1 | `online` or `offline` | Retained. `offline` is the last-will message. |

`state` is one of `DRIVING`, `PAUSE_BEFORE_BACKUP`, `BACKUP`, `TURNING`, `PAUSE_AFTER_TURN`, `STOPPED`.
