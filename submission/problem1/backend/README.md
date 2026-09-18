# `backend/` — snapshot store + shared bus

## `backend.h` (156 lines)

Header-only C++ library linked into the simulator (`sim/sim.cpp`).
**Not a server** — no HTTP here. Provides:

- `stateToString()` — `STATES` enum → `"OFF" … "CHARGING"` strings
- `DroneState` — immutable snapshot of one drone (`id, pos, battery,
  state, destination, base, speed`) + `last_updated` serialization timestamp
- `DroneList` — `unordered_map<int, DroneState>` with O(1)
  `addDrone / update / getDroneState`
- `writeTelemetry(filename)` — O(N) full-file JSON rewrite every sim tick.
  Serializes C++ `addr` as JSON `"address"`. No string escaping, no atomic
  rename (readers must tolerate torn writes — `api/fleet_manager.py` does).

## `telemetry.json` (gitignored, runtime only)

Rewritten in full every 2 s while `./simulator` runs. Do not hand-edit.

```json
{
  "drones": [
    {
      "id": 101,
      "position": {"lat": 31.7813, "lng": 76.9975, "alt": 30},
      "battery": 92,
      "state": "CRUISE",
      "timestamp": 1789746995829,
      "base": {"lat": 31.7813, "lng": 76.9975, "address": "BASE STATION"},
      "destination": {"lat": 31.7743, "lng": 77.0033, "address": "6442 Village Square Rd, Kamand Valley, HP"},
      "speed": 0.00025
    }
  ]
}
```

10 entries (IDs 101–110). Consumed by `api/fleet_manager.py` via absolute
path and served at `GET /telemetry` and `GET /status`.
