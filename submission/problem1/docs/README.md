# Fleet Telemetry System — Full Documentation

A full-stack drone fleet telemetry prototype: a **C++ simulator** flies 10
autonomous delivery drones around **IIT Mandi North Campus**, snapshots the
fleet to a **JSON file** every 2 seconds, a **FastAPI dispatch API** serves
that file plus delivery assignment, metrics, and alerts, and a **React +
Leaflet dashboard** renders it live on a map.

```text
sim/sim.cpp (C++, 10 drones, 2 s tick)
  → backend/telemetry.json (file snapshot, overwritten every tick)
    → api/main.py + api/fleet_manager.py (FastAPI on :8000)
      → dashboard/display/src/App.tsx (React 19 + Leaflet, polls every 2 s on :5173)
```

There is **no database and no websocket**. The JSON file is the shared bus
between the simulator and the API; dispatch state (assignments, queue,
audit log) lives only in API process memory.

---

## 1. Repository layout

```text
Fleet-Telemetry-System/
├── Makefile                     # builds the simulator binary
├── simulator                    # built binary (gitignored, produced by `make`)
├── common/
│   ├── types.h                  # shared structs + state enum
│   ├── id.h / id.cpp            # drone UID generator
├── data/
│   └── locations.h              # base station + destination pool (world model)
├── sim/
│   ├── drone.h                  # Drone entity class (data + setters only)
│   └── sim.cpp                  # physics loop + state machine + main()
├── backend/
│   ├── backend.h                # C++ snapshot store + JSON serializer
│   ├── telemetry.json           # runtime snapshot (gitignored, rewritten every 2 s)
│   └── README.md                # schema note for telemetry.json
├── api/
│   ├── main.py                  # FastAPI routes (telemetry / status / assign-request)
│   ├── fleet_manager.py         # dispatcher: assignment, metrics, alerts, pads
│   ├── __init__.py              # package marker
│   └── README.md                # stub dep note
├── tests/
│   └── test_fleet_manager.py    # unittest: assignment, rejection, metrics
├── dashboard/display/
│   ├── package.json             # Vite + React 19 + react-leaflet + leaflet
│   └── src/
│       ├── App.tsx              # entire dashboard UI + map + polling
│       ├── App.css              # dark sidebar + topbar styling
│       ├── main.tsx             # React entry point
│       └── index.css            # global styles
├── docs/
│   └── README.md                # this file
├── EXPLANATION.md               # deep deconstruction (state machine, battery model, physics)
└── INSTRUCTIONS.md              # startup instructions (Linux)
```

---

## 2. File-by-file reference

### 2.1 `Makefile` — build definition

| Line | Content |
|------|---------|
| 2–3 | `CXX = g++`, `CXXFLAGS = -std=c++17 -Wall -I.` (the `-I.` lets includes like `"common/types.h"` resolve from the repo root) |
| 6 | `SRC = sim/sim.cpp common/id.cpp` — the only two translation units |
| 9 | `TARGET = simulator` — output binary name (no `.exe` extension; on Ubuntu, Files opens `.exe` files with Archive Manager, so the binary was renamed from `sim.exe`) |
| 12–13 | `all:` compiles everything with one `g++` invocation |
| 16–17 | `clean:` removes `simulator` (plus legacy `sim.exe` if present) |

Run from the repo root: `make` → `./simulator`.

---

### 2.2 `common/types.h` — shared vocabulary (30 lines)

The contract every layer implicitly agrees on. Included by
`sim/drone.h`, `backend/backend.h`, and `data/locations.h`.

| Line | Content |
|------|---------|
| 6–16 | `enum STATES { OFF, START, TAKEOFF, CRUISE, APPROACH, DELIVERY, RETURNING, LANDED, CHARGING }` — 9 states. Note: `START` and `APPROACH` are never assigned by the simulator; only 7 states are live |
| 18–22 | `struct Position { lat, lng, alt }` — a drone's live point in space |
| 24–28 | `struct Location { lat, lng, addr }` — a named place (base or destination). Field is `addr` in C++; the JSON serializer writes it as `"address"`, which is what the dashboard reads |

---

### 2.3 `common/id.h` / `common/id.cpp` — UID generator (6 lines each)

| File | Role |
|------|------|
| `id.h:4` | Declares `int generateUID();` |
| `id.cpp:3-6` | Defines it: `static int staticId = 100; return ++staticId;` |

Every `Drone` construction calls `generateUID()`, so a fresh run with
`NUM_OF_DRONES = 10` yields IDs **101–110** in construction order.
Caveats: not thread-safe, not persistent across restarts (a second run
reuses the same IDs), and the counter keeps incrementing within one
process (a respawned drone keeps its original ID — IDs are assigned at
construction, never recycled).

---

### 2.4 `data/locations.h` — the world model (68 lines)

Defines *where* the simulation happens: IIT Mandi North Campus,
Kamand Valley (31.7812939, 76.9975020).

| Line | Content |
|------|---------|
| 10–19 | `STREET_NAMES` — 24 campus/village road names (`North Campus Main Rd`, `Kamand Valley Rd`, `Salgi Village Rd`, …) used to fabricate destination addresses |
| 21–39 | `generateDestinations(count)` — deterministic generator (`mt19937` seeded with `42`, so the pool is identical every run): uniform `lat ∈ [31.765, 31.795]`, `lng ∈ [76.985, 77.010]` (~3 km box around campus), house number `∈ [100, 9999]`, address suffix `", Kamand Valley, HP"` |
| 41–43 | `BASES` — a **single** entry: `{31.7812939, 76.9975020, "BASE STATION"}` (North Campus Main Gate). All drones spawn, return to, and recharge at this one station |
| 45 | `DESTINATIONS = generateDestinations(1000)` — the 1000-point destination pool the simulator draws from |
| 50–68 | Commented-out test fixtures (old Las Vegas bases/destinations) — dead reference, kept for testing ideas only |

To change the operating area, edit the `latDist`/`lngDist` ranges and/or
`STREET_NAMES` here and rebuild — no other file hardcodes geography
(the dashboard map center in `App.tsx` should be updated to match).

---

### 2.5 `sim/drone.h` — the Drone entity (75 lines)

Pure data holder + setters. **No behaviour, no physics** — the state
machine lives in `sim.cpp`. Fields (`:9-15`): `id`, `pos`, `battery`,
`state`, `destination`, `base`, `speed`.

| Line | Content |
|------|---------|
| 17–21 | Default constructor: ID from `generateUID()`, position origin, battery 100, state `OFF` |
| 22–29 | Main constructor `(base, destination, latOffset, lngOffset)`: spawns near the base (`base + offset`, alt 0), battery 100, state `OFF`, default speed `0.0002` deg/tick (immediately overwritten by the simulator's speed distribution) |
| 31–37 | Getters: `getId / getPosition / getBattery / getState / getDestination / getBase / getSpeed` |
| 39–48 | `setSpeed / setBase / setDestination` |
| 50–54 | `movePos(dLat, dLng, dAlt)` — blindly adds deltas; all navigation math is the caller's job |
| 56–59 | `drainBattery(amount)` — subtracts, floored at 0 |
| 63–67 | `setPosition(lat, lng, alt)` — absolute teleport (used on respawn after charging) |
| 69–72 | `chargeBattery(amount)` — adds, capped at 100 |

---

### 2.6 `sim/sim.cpp` — physics loop + state machine (195 lines)

The core of the system. One process, one thread, infinite 2-second tick.

**Configuration & RNG (`:13-32`)**

| Line | Content |
|------|---------|
| 13–14 | RNG: `random_device`-seeded `mt19937` (unlike destinations, drone assignment is **non-deterministic** per run) |
| 16–18 | Distributions: `destDist` over `DESTINATIONS`, spawn `offsetDist ±0.0003°` (~±30 m), `speedDist 0.00015–0.00035` deg/tick (~15–35 m/tick) |
| 24 | `NUM_OF_DRONES = 10` |
| 25–32 | `CRUISE_ALTITUDE = 30.0`, `DELIVERY_WAIT_TICKS = 3`, `CHARGE_PER_TICK = 4`, `LOW_BATTERY_THRESHOLD = 20`, drain every 5th tick (`-2` moving / `-1` idle) |

**Helpers (`:34-48`)** — `distance2D()` (defined but unused — CRUISE/RETURNING inline their own `sqrt`) and `maybeDrainBattery()` (drains only when `tick % 5 == 0`).

**Initialization (`:50-83`)** — creates `droneRegistry` (ground truth `vector<Drone>`), `fleet` (`DroneList` serializable mirror), and two parallel countdown arrays. Each drone gets the single `BASES.front()`, a random destination, a spawn offset, and a random speed; all 10 registrations are printed to stdout.

**State machine (`:87-187`)** — every tick, every drone:

| State | Behaviour |
|-------|-----------|
| `OFF` | Transient → `TAKEOFF` |
| `TAKEOFF` | Climb `+5` alt/tick until `alt >= 30` → `CRUISE` |
| `CRUISE` | Battery `<= 20` → `RETURNING`. Else fly one `speed` step toward destination (normalized lat/lng vector); arrival within `0.0001°` (~11 m) → `DELIVERY` with 3-tick countdown |
| `DELIVERY` | Wait 3 ticks → `RETURNING` |
| `RETURNING` | Fly toward base; on arrival descend `-5`/tick, then → `LANDED` |
| `LANDED` | Transient → `CHARGING` |
| `CHARGING` | `+4` battery/tick until 100, then respawn at base + fresh offset, pick a **new** destination (`:174`), → `TAKEOFF`. Cycle repeats forever — no terminal state |

**Output (`:186-193`)** — each drone is pushed into `fleet` via
`fleet.update(DroneState(d))`, then the whole fleet is rewritten to the
CWD-relative path `backend/telemetry.json`, `Tick N written` is printed,
and the loop sleeps 2000 ms. **Must be launched from the repo root**,
otherwise the write fails with `Failed to write telemetry`.

---

### 2.7 `backend/backend.h` — C++ snapshot store + serializer (156 lines)

Despite the directory name, this is **not a server** — it is a
header-only C++ library used by the simulator. No HTTP, no Python here.

| Line | Content |
|------|---------|
| 14–27 | `stateToString()` — maps the `STATES` enum to `"OFF" … "CHARGING"` strings for JSON |
| 29–74 | `class DroneState` — immutable-ish snapshot of one drone (`id, pos, battery, state, destination, base, speed`) plus `last_updated` (serialization time, identical for all drones within a tick — not physics time). Constructed from a `Drone` (`:41-49`) |
| 76–104 | `class DroneList` — `unordered_map<int, DroneState>` with `addDrone` / `update` / `getDroneState` (all O(1)) and `size()` |
| 105–153 | `writeTelemetry(filename)` — O(N) full-file rewrite every tick via manual `ofstream` string building. Note the field rename: C++ `addr` → JSON `"address"` (`:135,140`). No string escaping, no atomic write (tmp-file + rename) — an API read landing mid-write can see torn JSON |
| 133–142 | Per-drone JSON shape: `id, position{lat,lng,alt}, battery, state, timestamp, base{lat,lng,address}, destination{lat,lng,address}, speed` |

---

### 2.8 `backend/telemetry.json` — the shared bus (gitignored)

The runtime snapshot file. Overwritten **in full, every 2 s** by the
simulator (`sim.cpp:190`); read on every dashboard poll by the API.
Schema: `{ "drones": [ {id, position, battery, state, timestamp, base, destination, speed}, … ] }` (10 entries). See also `backend/README.md` for the minimal schema note. Never hand-edit — it is regenerated continuously while the sim runs. Timestamps update every tick.

---

### 2.9 `api/main.py` — FastAPI routes (52 lines)

HTTP layer over the `FleetManager`. Needs `fastapi` + `uvicorn`
(pydantic arrives with fastapi). Start from inside `api/` with
`uvicorn main:app --port 8000` — required for the `from fleet_manager
import FleetManager` module import (`:5`), not for file access.

| Line | Content |
|------|---------|
| 8, 10–16 | Module-level `manager = FleetManager()` (one in-memory instance per process) + CORS allowing `http://localhost:5173` and `http://127.0.0.1:5173` |
| 19–23 | `DeliveryRequest` (Pydantic): `package_id?`, `weight = 0.0`, `destination?`, `deadline_minutes = 15.0` |
| 26–28 | `GET /` → `{"message": "Fleet Telemetry System API"}` (health check) |
| 31–40 | `GET /telemetry` → `{drones, charging_pads, queued_requests, metrics, alerts}` — dashboard-shaped subset of `get_status()` |
| 43–45 | `GET /status` → full status incl. `recent_assignments` audit trail |
| 48–51 | `POST /assign-request` → `manager.assign_request(...)` → `{accepted, selected_drone, reason, audit}` |

Verify: `curl http://127.0.0.1:8000/telemetry | head -c 500`.

---

### 2.10 `api/fleet_manager.py` — dispatcher + fleet intelligence (226 lines)

The brains of the Python side. Everything except the drone snapshots is
process memory (lost on restart); drone data is re-read from disk on
every public call.

| Line | Content |
|------|---------|
| 8–9 | `BACKEND_PATH = <repo>/backend/telemetry.json` — **absolute**, resolved from `__file__`. Reads are CWD-independent |
| 13–23 | `FleetManager.__init__`: in-memory `drones / requests / audit_log / request_log` + 3 stub `charging_pads` (never synced with sim `CHARGING` states) |
| 25–41 | `_load_drones() / load_telemetry()` — re-read the JSON file; missing file or torn JSON → empty fleet (fail-soft, no 500) |
| 43–57 | `_distance_km()` (degree-hypot `×111.32`, no latitude correction) + `_estimate_travel_time_minutes()` (`max(3.0, nearest_km × 0.75)`, 8.0 when fleet empty) |
| 59–73 | `_build_alerts()` — battery `<=20` count, pads-occupied flag, first queued/assigned request |
| 75–184 | `assign_request()` gate chain: `weight ∈ (0, 2.5]` → `deadline > 0` → feasibility vs fastest-possible → per-drone eligibility (reject `CHARGING/RETURNING/LANDED`, battery `<=15`) → score `battery + (25 if IDLE else 15)`, argmax wins; nobody eligible → request queued. Returns full per-candidate `audit` |
| 186–215 | `compute_metrics()` — `on_time_delivery_rate`, `total_energy_consumption_kwh` (`Σbattery × 0.008`), `pad_utilization_rate`, `mean_delay_per_late_package`, `fleet_variance_in_battery_degradation` |
| 217–226 | `get_status()` — `{drones, charging_pads, queued_requests, metrics, alerts, recent_assignments[-10:]}` |

Caveats (see `EXPLANATION.md §6.2`): assignments mutate memory only and
are discarded by the next `_load_drones()`; pads never reflect reality;
`api/__init__.py` is just a docstring package marker.

---

### 2.11 `tests/test_fleet_manager.py` — unit tests (103 lines, unittest)

3 tests with Mandi-coordinate fixtures (3 drones, 3 pads in `setUp`):
eligible-drone assignment (expects drone 101), overweight/impossible-deadline
rejection, metrics computation. Run from repo root:
`PYTHONPATH=. python3 -m unittest discover -s tests`. Known red: the
assignment test fails against live data because `assign_request` reloads
`telemetry.json` over the fixtures (`EXPLANATION.md §6.2`).

---

### 2.12 `dashboard/display/` — React + Leaflet frontend

**`package.json`** — `react 19`, `react-leaflet 5`, `leaflet 1.9.4`, built/served by `vite 8` + `typescript 5.9`. Scripts: `dev` (port 5173), `build` (`tsc -b && vite build`), `preview`, `lint`. (`dashboard/display/README.md` is a stub noting only the Leaflet deps.)

**`src/main.tsx`** — React entry point; mounts `App` under `StrictMode`.

**`src/App.tsx`** (367 lines) — the entire dashboard:

| Line | Content |
|------|---------|
| 15–33 | TS types: `Location{lat,lng,address}`, `Drone{id, position{lat,lng,alt}, battery, state, base, destination, timestamp}` — mirrors the JSON shape from `backend.h` |
| 35–40 | `Pad{id, occupied_by, time_remaining, queue}` type for charging-pad telemetry |
| 42 | `LOW_BATTERY_THRESHOLD = 25` — note the mismatch: the sim aborts at 20, the dispatcher alerts at 20 / assigns down to 15, so the three layers disagree on "critical" |
| 46–64 | `stateColor()` — battery ≤ 25 → red; CRUISE/TAKEOFF/APPROACH/DELIVERY → blue; RETURNING → amber; CHARGING/LANDED/OFF → grey. Collapses 7 sim states into 4 visual buckets |
| 66–76 | `normalizeState()` — same thresholds → `LOW_BATTERY / ACTIVE / RETURNING / IDLE` filter buckets |
| 78–90 | `FocusDrone` — `flyTo(drone, zoom 15, 0.8 s)` when a drone is selected |
| 93–99 | State: `drones`, `chargingPads`, `queuedRequests`, `selectedDrone`, `filter`, `loading`, `error` |
| 101–126 | Polling: `fetch("http://127.0.0.1:8000/telemetry")` on mount + `setInterval(2000)` (frequency-matched to the sim tick); consumes `drones + charging_pads + queued_requests`, ignores the API's `metrics/alerts` and recomputes its own |
| 128–145 | `metrics`/`visibleDrones` via `useMemo` O(N): `total / active / returning / lowBattery / idle / avgBattery` + filter |
| 151–221 | Sidebar: brand, critical-alerts card, filter buttons (ALL/ACTIVE/RETURNING/LOW_BATTERY/IDLE), selected-drone panel (status, battery, altitude in `ft` at `:212`, destination) |
| 223–251 | Topbar: total / active / avg battery / alerts + critical banner when `lowBattery > 0` |
| 255–285 | `fleet-overview`: Charging Pads panel + Queued Requests panel. Styling gap — `fleet-overview` / `info-panel` have **no rules in `App.css`**, so these render unstyled |
| 287–297 | Map header with live count `({drones.length} drones)` + `MapContainer` centered `[31.7812939, 76.997502]` (BASE STATION) at `zoom 15` with OSM tiles |
| 305–362 | Per-drone `CircleMarker` (tooltip on hover, popup on click with id/battery/state/base/destination); selected drone gets a dashed `Polyline base → position → destination` route |

**`src/App.css`** (398 lines) — dark sidebar + light topbar, alert banner, map shell, figure title bar. **`src/index.css`** is empty (0 lines) — no global styles.

---

### 2.13 Companion docs + API/dashboard notes

| File | Role |
|------|------|
| `EXPLANATION.md` | Olympiad-style deconstruction: state-machine proof, battery-model math, dispatch-logic analysis, map-physics critique, load-bearing flaws (assignment evaporation, test isolation, threshold splits) |
| `INSTRUCTIONS.md` | Linux startup runbook: prerequisites, 3-terminal bring-up (simulator → API → dashboard), tests, ports/URLs table, troubleshooting matrix |
| `api/README.md` | Stub (lists FastAPI/Uvicorn only) — see §2.9–2.11 above for the real API reference |
| `backend/README.md` | Minimal `telemetry.json` schema note (`{"drones": []}`) |
| `api/__init__.py` | One-line package docstring marker |

---

## 3. Data flow (what actually happens at runtime)

```text
1. ./simulator (repo root)                     2 s tick
   sim.cpp state machine advances 10 drones
     → fleet.update(...) per drone (O(1))
     → fleet.writeTelemetry("backend/telemetry.json") (O(N) full rewrite)
2. uvicorn main:app (from api/)                per dashboard poll / dispatch call
   FleetManager re-reads backend/telemetry.json (absolute path)
   GET /telemetry → {drones, charging_pads, queued_requests, metrics, alerts}
   POST /assign-request → scored dispatch (memory-only, see §2.10 caveats)
3. npm run dev (from dashboard/display)        every 2 s
   App.tsx fetches :8000/telemetry → setDrones/setChargingPads/setQueuedRequests
   → Leaflet re-renders
```

Effective rate: **1 snapshot / 2 s** end to end (not per-drone streaming),
plus on-demand dispatch decisions.

---

## 4. Run order (summary)

Terminal 1 (repo root): `make && ./simulator` — leave running.
Terminal 2 (`api/`): `pip install fastapi uvicorn && uvicorn main:app --port 8000`.
Terminal 3 (`dashboard/display/`): `npm install && npm run dev` → open `http://localhost:5173`.

Details, port conflicts, and failure modes: see `INSTRUCTIONS.md`.
Internals, proofs, and load-bearing flaws: see `EXPLANATION.md`.
