# `backend/` — snapshot store + dispatch inbox (shared buses)

## 🎬 Demo Video

[![Fleet Telemetry System Demo](https://img.youtube.com/vi/Uo0r4DJiyb4/0.jpg)](https://youtu.be/Uo0r4DJiyb4)

Watch the full demo: https://youtu.be/Uo0r4DJiyb4

---

## `backend.h` (155 lines)

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

Rewritten in full every 800 ms while `./simulator` runs. Do not hand-edit.

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

3 entries (IDs 101–103, `NUM_OF_DRONES = 3` in `sim/sim.cpp:25`).
Consumed by `api/fleet_manager.py` via absolute path and served at
`GET /telemetry` and `GET /status`.

## `commands.json` (runtime inbox, reverse channel)

The **API → sim** dispatch inbox. Every accepted `POST /assign-request`
appends (upserts by `drone_id`) one entry via `FleetManager._write_command`
(`api/fleet_manager.py:72-92`); the sim polls it once per tick
(`sim/sim.cpp:97-100`) and consumes the entry on liftoff
(`sim/sim.cpp:193-200`, `219-224`). Schema:

```json
[
  {"drone_id": 101, "lat": 31.7905, "lng": 77.0098, "address": "Drop zone", "package_id": "PKG-1"}
]
```

Written **atomically** (tmp file + `os.replace` on the Python side,
tmp + `rename` on the C++ side in `sim/commands.h`) so a concurrent
reader never sees a torn payload. Missing/corrupt file → empty inbox
(fail-soft, tick skipped, never a crash). Initialized to `[]`.
Read back by the API for `pending_commands` in `GET /status` and for the
inbox/backlog guards in `assign_request`.
