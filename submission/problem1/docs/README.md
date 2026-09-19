# Fleet Telemetry System — Full Documentation

A full-stack drone fleet telemetry prototype: a **C++ simulator** parks 3
on-demand delivery drones at **IIT Mandi North Campus**, snapshots the
fleet to a **JSON file** every 800 ms, a **FastAPI dispatch API** serves
that file plus hybrid-scored delivery assignment with per-drone backlogs,
and a **React + Leaflet dashboard** renders it live on a map alongside a
**delivery-request page** that tasks drones through a reverse
**command inbox**.

```text
sim/sim.cpp (C++, 3 drones, 800 ms tick, polls backend/commands.json)
  → backend/telemetry.json (file snapshot, overwritten every tick)
    → api/main.py + api/fleet_manager.py (FastAPI on :8000)
      → dashboard/display/src/App.tsx (React 19 + Leaflet, polls every 2 s on :5173)
            + dashboard/display/src/pages/RequestPage.tsx (/request, POST /assign-request)

RequestPage.tsx --POST /assign-request--> fleet_manager.py --writes--> backend/commands.json
     --polled each tick--> sim.cpp (LANDED drone consumes entry, TAKEOFF)
```

There is **no database and no websocket**. Two JSON files are the shared
buses: `telemetry.json` (sim → API → dashboard) and `commands.json`
(API → sim takeoff inbox). Backlog/assignment state beyond the inbox
lives only in API process memory.

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
│   ├── commands.h               # command-inbox reader/writer (poll + consume)
│   └── sim.cpp                  # on-demand physics loop + state machine + main()
├── backend/
│   ├── backend.h                # C++ snapshot store + JSON serializer
│   ├── telemetry.json           # runtime snapshot (gitignored, rewritten every 800 ms)
│   ├── commands.json            # runtime dispatch inbox (API → sim, starts as [])
│   └── README.md                # schema notes for telemetry.json + commands.json
├── api/
│   ├── main.py                  # FastAPI routes (telemetry / status / assign-request)
│   ├── fleet_manager.py         # on-demand dispatcher: hybrid score, backlog, pads, inbox
│   ├── __init__.py              # package marker
│   └── README.md                # API + dispatch-model reference
├── tests/
│   └── test_fleet_manager.py    # unittest: 3 legacy + 13 on-demand dispatch tests
├── dashboard/display/
│   ├── package.json             # Vite + React 19 + react-leaflet + leaflet + react-router-dom
│   └── src/
│       ├── App.tsx              # router + dashboard map + polling (611 lines)
│       ├── pages/
│       │   └── RequestPage.tsx  # delivery-request form + click-to-pick map (244 lines)
│       ├── App.css              # sidebar/topbar/markers/legend/request-form styling (719 lines)
│       ├── main.tsx             # React entry point
│       └── index.css            # empty — no global styles
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

### 2.2 `common/types.h` — shared vocabulary (29 lines)

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
`NUM_OF_DRONES = 3` yields IDs **101–103** in construction order.
Caveats: not thread-safe, not persistent across restarts (a second run
reuses the same IDs), and the counter keeps incrementing within one
process.

---

### 2.4 `data/locations.h` — the world model (67 lines)

Defines *where* the simulation happens: IIT Mandi North Campus,
Kamand Valley (31.7812939, 76.9975020).

| Line | Content |
|------|---------|
| 10–19 | `STREET_NAMES` — 24 campus/village road names (`North Campus Main Rd`, `Kamand Valley Rd`, `Salgi Village Rd`, …) used to fabricate destination addresses |
| 21–39 | `generateDestinations(count)` — deterministic generator (`mt19937` seeded with `42`, so the pool is identical every run): uniform `lat ∈ [31.765, 31.795]`, `lng ∈ [76.985, 77.010]` (~3 km box around campus), house number `∈ [100, 9999]`, address suffix `", Kamand Valley, HP"` |
| 41–43 | `BASES` — a **single** entry: `{31.7812939, 76.9975020, "BASE STATION"}` (North Campus Main Gate). All drones spawn, return to, and recharge at this one station |
| 45 | `DESTINATIONS = generateDestinations(1000)` — the 1000-point destination pool (kept for fixtures/tests; the live sim no longer auto-assigns from it — drones wait for API commands) |

To change the operating area, edit the `latDist`/`lngDist` ranges and/or
`STREET_NAMES` here and rebuild — no other file hardcodes geography
(the dashboard map center in `App.tsx` should be updated to match).

---

### 2.5 `sim/drone.h` — the Drone entity (74 lines)

Pure data holder + setters. **No behaviour, no physics** — the state
machine lives in `sim.cpp`. Fields: `id`, `pos`, `battery`,
`state`, `destination`, `base`, `speed`.

| Line | Content |
|------|---------|
| 17–21 | Default constructor: ID from `generateUID()`, position origin, battery 100, state `OFF` |
| 22–29 | Main constructor `(base, destination, latOffset, lngOffset)`: spawns near the base (`base + offset`, alt 0), battery 100, state `OFF`, default speed `0.0002` deg/tick (immediately overwritten by the simulator's speed distribution) |
| 31–37 | Getters: `getId / getPosition / getBattery / getState / getDestination / getBase / getSpeed` |
| 39–48 | `setSpeed / setBase / setDestination` |
| 50–54 | `movePos(dLat, dLng, dAlt)` — blindly adds deltas; all navigation math is the caller's job |
| 56–59 | `drainBattery(amount)` — subtracts, floored at 0 |
| 63–67 | `setPosition(lat, lng, alt)` — absolute teleport |
| 69–72 | `chargeBattery(amount)` — adds, capped at 100 |

---

### 2.6 `sim/sim.cpp` — on-demand physics loop + state machine (231 lines)

The core of the system. One process, one thread, infinite **800 ms** tick
(`:230`), **3 drones** (`NUM_OF_DRONES = 3`, `:25`). Drones no longer
fly autonomous random missions — they park `LANDED` at base and lift
off only when the API drops a command into `backend/commands.json`.

**Configuration & RNG (`:15-37`)**

| Line | Content |
|------|---------|
| 18–19 | Distributions: spawn `offsetDist ±0.0003°` (~±30 m), `speedDist 0.00015–0.00035` deg/tick (~15–35 m/tick). (The old random-destination draw is gone — destinations arrive via the inbox) |
| 25–33 | `NUM_OF_DRONES = 3`, `CRUISE_ALTITUDE = 30.0`, `DELIVERY_WAIT_TICKS = 3`, `CHARGE_PER_TICK = 4`, `LOW_BATTERY_THRESHOLD = 20`, drain every 5th tick (`-2` moving / `-1` idle) |
| 37 | `COMMANDS_PATH = "backend/commands.json"` — on-demand dispatch inbox |
| 39–53 | `distance2D()` + `maybeDrainBattery()` (drains only when `tick % 5 == 0`) |

**Initialization (`:68-86`)** — creates `droneRegistry` (ground truth
`vector<Drone>`) and `fleet` (`DroneList` serializable mirror). Each
drone gets the single `BASES.front()` as *both* base and destination,
a spawn offset, a random speed, and state `LANDED` (awaiting request).

**Inbox poll (`:97-101`)** — once per tick, `readCommands(COMMANDS_PATH)`
is loaded into an `unordered_map<int, Command>` keyed by `drone_id`
(fail-soft: missing/corrupt file → empty inbox for this tick).

**State machine (`:103-214`)** — every tick, every drone:

| State | Behaviour |
|-------|-----------|
| `OFF` | → `LANDED` (park; the old `OFF → TAKEOFF` auto-launch is gone) |
| `TAKEOFF` | Climb `+5` alt/tick until `alt >= 30` → `CRUISE` |
| `CRUISE` | Battery `<= 20` → `RETURNING`. Else fly one step toward destination with **anti-overshoot clamp** `step = min(speed, distance)` (`:138`); arrival within `0.0001°` (~11 m) → `DELIVERY` with 3-tick countdown |
| `DELIVERY` | Wait 3 ticks → `RETURNING` |
| `RETURNING` | Fly toward base (same `min(speed, distance)` clamp, `:174`); on arrival descend (`-5`/tick, exact-to-zero `:166`), then → `LANDED` |
| `LANDED` | If `battery < 100` and no healthy-battery command waiting → `CHARGING` (**opportunistic top-up**). Else if an inbox entry addresses this drone → consume it, set destination, → `TAKEOFF` (`[Command] Drone N tasked to …` on stdout). Else (full battery, no command) → hold at base, no drain |
| `CHARGING` | `+4` battery/tick until 100, then → `LANDED` (park and wait — **no auto-generated destination, no auto-takeoff**) |

**Inbox write-back (`:219-224`)** — consumed entries are removed and the
remainder rewritten atomically via `writeCommands()`.

**Output (`:226-230`)** — each drone is pushed into `fleet` via
`fleet.update(DroneState(d))`, then the whole fleet is rewritten to the
CWD-relative path `backend/telemetry.json`, `Tick N written` is printed,
and the loop sleeps 800 ms. **Must be launched from the repo root**,
otherwise the write fails with `Failed to write telemetry`.

---

### 2.7 `sim/commands.h` — command-inbox reader/writer (162 lines)

Header-only, dependency-free mini-JSON layer for the reverse channel.
No third-party JSON library — hand-rolled `parseNumber`/`parseString`
(with `\"`/`\\` escapes), `seekKey`, and `escapeInto`.

| Line | Content |
|------|---------|
| 23–29 | `struct Command { drone_id, lat, lng, address, package_id }` |
| 32–113 | `cmd_detail` helpers: `readFile`, `parseNumber`, `parseString`, `seekKey`, `escapeInto` |
| 116–140 | `readCommands(path)` — parse all inbox entries; empty vector on missing/corrupt file |
| 142–161 | `writeCommands(path, cmds)` — atomic rewrite via `path + ".tmp"` + `rename` (writes `"[]"` when empty so readers always see valid JSON) |

---

### 2.8 `backend/backend.h` — C++ snapshot store + serializer (155 lines)

Despite the directory name, this is **not a server** — it is a
header-only C++ library used by the simulator. No HTTP, no Python here.

| Line | Content |
|------|---------|
| 14–27 | `stateToString()` — maps the `STATES` enum to `"OFF" … "CHARGING"` strings for JSON |
| 29–74 | `class DroneState` — immutable-ish snapshot of one drone (`id, pos, battery, state, destination, base, speed`) plus `last_updated` (serialization time, identical for all drones within a tick — not physics time). Constructed from a `Drone` |
| 76–104 | `class DroneList` — `unordered_map<int, DroneState>` with `addDrone` / `update` / `getDroneState` (all O(1)) and `size()` |
| 105–153 | `writeTelemetry(filename)` — O(N) full-file rewrite every tick via manual `ofstream` string building. Note the field rename: C++ `addr` → JSON `"address"`. No string escaping, no atomic write (tmp-file + rename) — an API read landing mid-write can see torn JSON |
| 133–142 | Per-drone JSON shape: `id, position{lat,lng,alt}, battery, state, timestamp, base{lat,lng,address}, destination{lat,lng,address}, speed` |

---

### 2.9 `backend/telemetry.json` + `backend/commands.json` — the two buses (gitignored / runtime)

| File | Direction | Rhythm | Schema |
|------|-----------|--------|--------|
| `telemetry.json` | sim → API → dashboard | Overwritten **in full, every 800 ms** by the simulator (`sim.cpp:226`) | `{ "drones": [ {id, position, battery, state, timestamp, base, destination, speed}, … ] }` (3 entries, IDs 101–103). See also `backend/README.md`. Never hand-edit |
| `commands.json` | API → sim | Appended on each accepted dispatch, consumed on liftoff | `[ {"drone_id": N, "lat": …, "lng": …, "address": "…", "package_id": "PKG-…"}, … ]` (starts as `[]`). Atomic tmp+rename writes on both sides; missing/corrupt → empty inbox |

---

### 2.10 `api/main.py` — FastAPI routes (52 lines)

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
| 43–45 | `GET /status` → full status incl. `pending_commands` + `recent_assignments` audit trail |
| 48–51 | `POST /assign-request` → `manager.assign_request(...)` → `{accepted, selected_drone, allotted_drone?, queue_position?, reason, audit, allotment?, cost_weights}` |

Verify: `curl http://127.0.0.1:8000/telemetry | head -c 500`.

---

### 2.11 `api/fleet_manager.py` — on-demand dispatcher + fleet intelligence (489 lines)

The brains of the Python side. Drone snapshots are re-read from disk
on every public call; backlog/assignment logs are process memory
(lost on restart); the durable tasking channel is the
`backend/commands.json` inbox.

| Line | Content |
|------|---------|
| 9–28 | `BACKEND_PATH` + `COMMANDS_PATH` (**absolute**, resolved from `__file__` — reads are CWD-independent), `AVG_MISSION_MINUTES = 10.0`, `ALLOT_WEIGHTS = {battery: 1.0, load: 10.0, distance: 5.0, slack: 1.0}` hybrid cost weights |
| 32–42 | `FleetManager.__init__`: in-memory `drones / requests / audit_log / request_log` + 3 charging pads **with map coordinates** (`lat`/`lng` offsets around the base) |
| 44–60 | `_load_drones() / load_telemetry()` — re-read the JSON file; missing file or torn JSON → empty fleet (fail-soft, no 500) |
| 62–92 | `_read_commands() / _write_command()` — inbox access; writes are upsert-by-`drone_id` and atomic (tmp + `os.replace`) |
| 94–108 | `_distance_km()` (degree-hypot `×111.32`, no latitude correction) + `_estimate_travel_time_minutes()` (`max(3.0, nearest_km × 0.75)`, 8.0 when fleet empty) |
| 110–124 | `_build_alerts()` — battery `<=20` count, pads-occupied flag, first queued/assigned request |
| 126–134 | `_drone_load()` — active + queued jobs on one drone's plate |
| 136–173 | `_score_candidate()` — hybrid cost `W_batt·battery + parked_bonus − W_load·load − W_dist·dist + W_slack·slack` with full `cost` breakdown (`load/battery/distance_km/slack_min/parked_bonus/hybrid`); legacy preset (`load = distance = slack = 0`) reproduces the old `battery + bonus` rule exactly |
| 175–191 | `_queued_for() / _inbox_ids()` — FIFO backlog per drone / ids with a takeoff command awaiting pickup |
| 193–220 | `_sync_pads()` — **pads mirror reality**: drones in `CHARGING` occupy pads in id order, overflow round-robins into `queue`s, `time_remaining` (min) from `ceil((100−batt)/4)` ticks × 2 s. Idempotent, rebuilt every `get_status` |
| 222–260 | `_drone_is_free() / _settle_and_promote()` — completion + auto-promotion engine: parked drones with no inbox entry get `assigned → completed`, then each free drone's oldest `queued` record launches with a fresh inbox command. Runs on every `assign_request`/`get_status` |
| 262–440 | `assign_request()` gate chain: `weight ∈ (0, 2.5]` → `deadline > 0` → feasibility vs fastest-possible → settle-and-promote → per-drone eligibility (`LANDED`/`IDLE` + battery > 15 + no inbox entry + no own backlog; `CHARGING/RETURNING/OFF/START/TAKEOFF` unavailable, `CRUISE/DELIVERY/APPROACH` busy) → hybrid argmax wins with per-candidate `audit`; nobody eligible → **allot** to best hybrid-cost drone's backlog (`allotted_drone`, `queue_position`) instead of leaving it ownerless |
| 442–471 | `compute_metrics()` — `on_time_delivery_rate`, `total_energy_consumption_kwh` (`Σbattery × 0.008`), `pad_utilization_rate`, `mean_delay_per_late_package`, `fleet_variance_in_battery_degradation` |
| 473–489 | `get_status()` — `{drones, charging_pads, queued_requests, pending_commands, metrics, alerts, recent_assignments[-10:]}` |

Caveats (see `EXPLANATION.md §6.2`): the cosmetic in-memory
`drone["state"]` tweak is still discarded by the next `_load_drones()` —
the durable channel is the inbox file. `api/__init__.py` is just a
docstring package marker.

---

### 2.12 `tests/test_fleet_manager.py` — unit tests (324 lines, unittest, 16 tests)

3 legacy tests with Mandi-coordinate fixtures (assignment, overweight /
impossible-deadline rejection, metrics) plus 13 `OnDemandDispatchTests`
with tmp-file inbox redirect and `_load_drones` stubbing
(the fixture-pinning fix for the known isolation quirk):

LANDED-eligible + command persisted · airborne-busy rejected ·
command upsert replaces · `get_status` exposes `pending_commands` ·
pending-inbox drone skipped · oldest-queued promotes on landing ·
legacy weight preset reproduces old behavior · least-loaded wins ·
nearest wins · slack beats raw battery · charging drone occupies pad
(with `time_remaining ≈ 0.2 min`) · empty pads stay free ·
pad overflow queues (4th charger → `queue: [4]`).

Run from repo root:
`PYTHONPATH=. python3 -m unittest discover -s tests`. Known red: one
legacy assignment test fails against live data because the un-stubbed
path still reloads `telemetry.json` over its fixtures
(`EXPLANATION.md §6.2` — currently `selected_drone 3 != 101` with the
3-drone fleet).

---

### 2.13 `dashboard/display/` — React + Leaflet frontend (map + request page)

**`package.json`** — `react 19`, `react-leaflet 5`, `leaflet 1.9.4`,
`react-router-dom 7`, built/served by `vite 8` + `typescript 5.9`.
Scripts: `dev` (port 5173), `build` (`tsc -b && vite build`), `preview`, `lint`.

**`src/main.tsx`** — React entry point; mounts `App` under `StrictMode`.

**`src/App.tsx`** (611 lines) — router + entire dashboard:

| Line | Content |
|------|---------|
| 6, 17, 603–611 | `BrowserRouter` with `/` → `Dashboard`, `/request` → `RequestPage` |
| 19–55 | TS types: `Location{lat,lng,address}`, `Drone{…}`, `Pad{id, lat?, lng?, occupied_by, time_remaining, queue}`, `FleetRequest{package_id, status, destination?, assigned_drone?, allotted_drone?, queue_position?}` — mirrors the JSON shapes from `backend.h` + `get_status()` |
| 57–58 | `BASE_LAT/BASE_LNG` constants (BASE STATION) |
| 60 | `LOW_BATTERY_THRESHOLD = 25` — note the mismatch: the sim aborts at 20, the dispatcher alerts at 20 / assigns down to 15, so the three layers disagree on "critical" |
| 64–94 | `stateColor()` / `normalizeState()` — battery ≤ 25 → red/`LOW_BATTERY`; CRUISE/TAKEOFF/APPROACH/DELIVERY → blue/`ACTIVE`; RETURNING → amber; CHARGING/LANDED/OFF → grey/`IDLE`. Collapses 7 sim states into 4 visual buckets |
| 96–140 | `droneIcon / destinationIcon / queuedIcon / padIcon` — `L.divIcon` markers: numbered circle badges, purple active-delivery triangles, amber queued-delivery triangles, green/blue pad diamonds |
| 142–155 | `FocusDrone` — `flyTo(drone, zoom 15, 0.8 s)` when a drone is selected |
| 157–… | `Dashboard`: state (`drones`, `chargingPads`, `queuedRequests`, `selectedDrone`, `filter`, `loading`, `error`) |
| … | Polling: `fetch("http://127.0.0.1:8000/telemetry")` on mount + `setInterval(2000)`; consumes `drones + charging_pads + queued_requests`, ignores the API's `metrics/alerts` and recomputes its own |
| … | `metrics`/`visibleDrones` via `useMemo` O(N): `total / active / returning / lowBattery / idle / avgBattery` + filter |
| … | `spreadDeliveries` — declutter: co-located drops spread on a small ring (~13 m); singles stay exact |
| … | Sidebar: rebranded `Drone Fleet Management`, critical-alerts card, `+ Request delivery` link to `/request`, filter buttons (ALL/ACTIVE/RETURNING/LOW_BATTERY/IDLE), selected-drone panel (status, battery, altitude in `ft`, destination) |
| … | Topbar: total / active / avg battery / alerts + critical banner when `lowBattery > 0` |
| … | `fleet-overview`: Charging Pads panel + Queued Requests panel (now showing `→ Drone N` and queue position) |
| … | Map header with live count `({drones.length} drones)` + `MapContainer` centered on BASE STATION at `zoom 15` with OSM tiles; **base-station `Rectangle`** + **pad `Marker`s** + per-drone `Marker` badges (tooltip on hover, popup on click with id/battery/state/base/destination); selected drone gets a dashed `Polyline base → position → destination` route; **active/queued delivery `Marker`s** per request |
| … | `map-legend` overlay documenting every marker |

**`src/pages/RequestPage.tsx`** (244 lines) — the `/request` view:
delivery form (`package_id`, weight validated `∈ (0, 2.5]`, lat/lng +
address label, deadline > 0) plus a click-to-pick Leaflet map
(`ClickPicker` → `lat/lng` fields) that `POST`s to
`http://127.0.0.1:8000/assign-request` and renders the decision:
accepted + selected drone vs allotted drone + queue position, reason,
and the full per-drone audit list; `← Back to Map` returns to `/`.

**`src/App.css`** (719 lines) — dark sidebar + light topbar, alert
banner, map shell, figure title bar, plus marker styles
(`drone-badge/dest-badge/queued-badge/pad-diamond`), `map-legend`,
`fleet-overview`/`info-panel` panels, and the request view
(`request-layout/request-form/picker-map/picker-hint/api-error/request-result`).
**`src/index.css`** is empty (0 lines) — no global styles.

---

### 2.14 Companion docs + API/dashboard notes

| File | Role |
|------|------|
| `EXPLANATION.md` | Olympiad-style deconstruction: on-demand state-machine proof, battery-model math, hybrid-dispatch analysis, map-physics critique, remaining flaws (telemetry torn reads, threshold splits, test isolation) |
| `INSTRUCTIONS.md` | Linux startup runbook: prerequisites, 3-terminal bring-up (simulator → API → dashboard), request flow via `/request`, tests (16), ports/URLs table, troubleshooting matrix |
| `api/README.md` | API + hybrid dispatch-model reference (weights, eligibility, backlog, pads, inbox) |
| `backend/README.md` | `telemetry.json` + `commands.json` schema notes |
| `dashboard/display/README.md` | Frontend reference: routes, markers, legend, request page |
| `api/__init__.py` | One-line package docstring marker |

---

## 3. Data flow (what actually happens at runtime)

```text
1. ./simulator (repo root)                     800 ms tick
   sim.cpp polls backend/commands.json (fail-soft), advances 3 drones
     → LANDED drone with inbox entry consumes it → TAKEOFF → CRUISE
       → DELIVERY (3 ticks) → RETURNING → LANDED → CHARGING top-up → LANDED
     → fleet.update(...) per drone (O(1))
     → fleet.writeTelemetry("backend/telemetry.json") (O(N) full rewrite)
2. uvicorn main:app (from api/)                per dashboard poll / dispatch call
   FleetManager re-reads backend/telemetry.json (absolute path)
   GET /telemetry → {drones, charging_pads, queued_requests, metrics, alerts}
   GET /status    → above + {pending_commands, recent_assignments}
   POST /assign-request → hybrid dispatch → accepted? inbox command for sim
     : assigned (flies now) vs queued/allotted (auto-promotes on landing)
   _settle_and_promote + _sync_pads run on every assign/status call
3. npm run dev (from dashboard/display)        every 2 s + on demand
   App.tsx fetches :8000/telemetry → setDrones/setChargingPads/setQueuedRequests
   → Leaflet re-renders (drones, pads, delivery triangles, legend)
   /request → RequestPage POSTs /assign-request → audit rendered;
     sim picks up the inbox entry next tick (~800 ms) and flies it
```

Effective rates: **1.25 snapshots / s** sim → file, **1 poll / 2 s**
API → dashboard, plus on-demand dispatch decisions with ~800 ms
sim pickup latency.

---

## 4. Run order (summary)

Terminal 1 (repo root): `make && ./simulator` — leave running (3 drones park `LANDED`).
Terminal 2 (`api/`): `pip install fastapi uvicorn && uvicorn main:app --port 8000`.
Terminal 3 (`dashboard/display/`): `npm install && npm run dev` → open `http://localhost:5173`
(map) and `http://localhost:5173/request` (send a delivery; watch the drone take off).

Details, port conflicts, and failure modes: see `INSTRUCTIONS.md`.
Internals, proofs, and remaining flaws: see `EXPLANATION.md`.
