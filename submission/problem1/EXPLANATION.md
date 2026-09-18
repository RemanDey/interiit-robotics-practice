# Fleet-Telemetry-System: Olympiad-Calibre Deconstruction

## 0. One-Line Theorem

This is a **3-stage, 4-language, file-coupled cyber-physical simulation with a dispatch layer**: `C++` generates physics → `JSON file` acts as shared memory → `Python/FastAPI + FleetManager` serves it and assigns delivery requests → `React/Leaflet` renders it. There is no database and no websocket.

```text
sim/sim.cpp (C++, N=10 agents, 0.5Hz loop)
  → backend/telemetry.json (overwritten snapshot, O(N) each tick)
    → api/main.py + api/fleet_manager.py (FastAPI, absolute-path read + dispatch)
      → dashboard/display/src/App.tsx (poll 2s, O(N) render on Leaflet)
```

Shared contracts: `common/types.h`, `data/locations.h`.
Verified against live run: `telemetry.json` holds 10 drones, speeds
`0.000153–0.000344` (inside the coded distribution — no longer stale).

---

## 1. Axioms: `common/`

### `common/types.h:6-16` — `enum STATES`

```cpp
OFF, START, TAKEOFF, CRUISE, APPROACH, DELIVERY, RETURNING, LANDED, CHARGING
```

**Lemma 1 (Dead states):** `START, APPROACH` are never assigned in `sim/sim.cpp:93-184`. They are dead code. Only 7 states are live.

### `common/types.h:18-28` — Geometry

```cpp
struct Position { lat,lng,alt; };
struct Location { lat,lng,addr; };
```

Note the naming fracture that propagates: C++ uses `addr`, JSON/C++ serializer uses `"address"` (`backend/backend.h:135,140`), frontend type uses `address` (`dashboard/display/src/App.tsx:18`). This is correct only by manual mapping, no schema enforcement.

### `common/id.cpp:3-6` — UID Generator

```cpp
static int staticId = 100; return ++staticId;
```

**Invariant:** Single run with `NUM_OF_DRONES=10` yields IDs `101..110` exactly — confirmed by live `backend/telemetry.json` (10 objects).

**Failure modes (contest pitfalls):**

1. Not thread-safe, not persistent — restart collides IDs.
2. Copy semantics: `Drone d(...); droneRegistry.push_back(d); fleet.addDrone(d)` — ID is copied, not re-generated. Correct, but subtle.

---

## 2. World Model: `data/locations.h`

* `BASES`: single fixed hub `BASE STATION` at IIT Mandi North Campus Main Gate (`31.7812939, 76.9975020`). All drones spawn, return, and recharge here; `sim/sim.cpp:64` takes `BASES.front()` directly (no random base choice).
* `DESTINATIONS`: `generateDestinations(1000)` with `mt19937(42)` — **deterministic**. Same 1000 addresses every run. `lat~U[31.765,31.795]`, `lng~U[76.985,77.010]` (~3.3 km × ~2.4 km box around campus), `house~U[100,9999] + 24 campus/village street names`, suffix `", Kamand Valley, HP"`.
* Seeded RNG = reproducible test fixture. Good olympiad practice.
* Lines 50-68 hold commented-out Las Vegas test fixtures — dead reference only.

---

## 3. Agent: `sim/drone.h` — `class Drone`

Pure data + setters, no behavior. Fields: `id, pos, battery(0-100 clamped), state, destination, base, speed`.

Default `speed=0.0002` deg/tick, immediately overwritten by `speedDist~U[0.00015,0.00035]` in `sim/sim.cpp:18,71`. Live telemetry confirms speeds `0.000153–0.000344` — artifact matches source.

Methods are `O(1)`: `movePos`, `drainBattery` (floor 0), `chargeBattery` (cap 100), all with clamping invariants.

---

## 4. Physics Engine: `sim/sim.cpp` — The Core

### 4.1 Initialization `sim/sim.cpp:63-79`

For each `i < 10`: take `BASES.front()`, `dest~Uniform(DESTINATIONS)`, `offset~U[-0.0003,0.0003]^2`, spawn near base. Push to two parallel structures:

* `vector<Drone> droneRegistry` — ground truth.
* `DroneList fleet` — serializable mirror.
* `deliveryTicksRemaining[10]`, `chargingTicksRemaining[10]` — parallel arrays (fragile, index-coupled; the latter is written but never read).

### 4.2 Kinematics — The Only Math

```cpp
distance = sqrt(dLat*dLat + dLng*dLng)              // sim/sim.cpp:34-38,114,144
pos += step * (target-pos)/distance                 // sim/sim.cpp:120-124,155-160
```

This is **normalized gradient descent in lat/lng degree space** with fixed step `speed` deg/tick. Termination radius `0.0001 deg ~= 11m`.

**Olympiad critique:**

1. Ignores Earth curvature. At 31.78N, 1 deg lng ~= 94.6km vs 1 deg lat ~=111km — ~15% anisotropy distorted into isotropic steps. Acceptable for the ~3km campus box (<1% absolute error), wrong for real navigation. Correct fix: haversine + meters. (Note `api/fleet_manager.py:48` already uses a degree→km scale factor `×111.32` for its own estimates — the two layers disagree on geometry.)
2. `distance2D()` is defined but never called — CRUISE/RETURNING inline their own `sqrt`. Dead helper.
3. Vertical and horizontal decoupled: `TAKEOFF: alt+=5` until `CRUISE_ALTITUDE=30`, `RETURNING`: horizontal first, then `alt-=5`. Cruise at `30` labelled `ft` in `App.tsx:212` — 30ft cruise is physically absurd (real: 200-400ft). Unit ambiguity.

### 4.3 Battery Automaton

```cpp
maybeDrainBattery(tick, isMoving): if tick%5==0: -2 else -1
CHARGE_PER_TICK=+4/tick, LOW_BATTERY_THRESHOLD=20
```

Effective rates: `0.4%/tick` moving, `0.2%/tick` idle, `+4%/tick` charging. Asymmetric by 10x — deliberate to guarantee liveness (charge faster than drain).

**Threshold disagreement across three layers:** simulator aborts at `20` (`sim.cpp:29`), dashboard flags at `25` (`App.tsx:42`), dispatcher alerts at `<=20` but declares drones ineligible at `<=15` (`fleet_manager.py:61,116`). The UI raises `LOW_BATTERY` before the sim turns around, and the dispatcher may assign a drone the dashboard calls critical (battery 16–25).

**Liveness bug visible in data:** drones can fly at 0% (no dead-stick model). Safety property `battery>0 ==> flight` is violated.

### 4.4 State Machine — Formal Proof

| State | Guard → Action |
|---|---|
| `OFF` | unconditional → `TAKEOFF` (1 tick transient) |
| `TAKEOFF` | `alt<30 → alt+=5`, else → `CRUISE` |
| `CRUISE` | `batt<=20 → RETURNING`; `dist<1e-4 → DELIVERY(3 ticks)`; else step toward `dest` |
| `DELIVERY` | countdown 3 → `RETURNING` |
| `RETURNING` | step toward `base`; if arrived: descend, then → `LANDED` |
| `LANDED` | unconditional → `CHARGING` (1 tick transient) |
| `CHARGING` | `batt<100 → +4`, else respawn at `base+offset`, new `dest`, → `TAKEOFF` |

**Theorem (Liveness):** All paths cycle `...→CHARGING→TAKEOFF→...` infinitely. No absorbing state. Verified: no `break`, `while(true)` at `sim/sim.cpp:87`, sleep 2000ms at `:193`.

### 4.5 Output `sim/sim.cpp:186-193`

```cpp
fleet.update(DroneState(d)); // O(1) hashmap replace, refreshes timestamp
fleet.writeTelemetry("backend/telemetry.json"); // O(N) full rewrite EVERY tick
```

Path is CWD-relative — must launch from repo root, else silent `Failed to write telemetry`. No atomic rename → **torn-read race** with the API reader (mitigated on the Python side: `fleet_manager.py:34-35` catches `JSONDecodeError` and serves an empty fleet rather than 500ing).

Build: `Makefile: CXX=g++ -std=c++17 -Wall -I. SRC=sim/sim.cpp common/id.cpp → simulator`.

---

## 5. Snapshot Store: `backend/backend.h` — Not a Server

`backend/backend.h` is a **C++ header-only store**: `DroneState` (snapshot + `last_updated=now()`) + `DroneList { unordered_map<int,DroneState> }` with `addDrone O(1)`, `update O(1)`, `writeTelemetry O(N)` via manual `ofstream` string concatenation.

* No HTTP, no Python dict, no uvicorn here. The Python serving layer is `api/fleet_manager.py` (§6).
* Manual JSON: no escaping of `addr`. Safe today (addresses contain no `"`), fragile tomorrow.
* `timestamp` = serialization time, identical for all 10 drones per tick — not physics time.

---

## 6. Dispatch API: `api/main.py` + `api/fleet_manager.py`

### 6.1 `api/main.py` (52 lines) — routes

| Line | Content |
|------|---------|
| 5,8 | `from fleet_manager import FleetManager` + module-level `manager = FleetManager()` (single in-memory instance: request/audit logs live only in this process) |
| 10–16 | CORS allows `http://localhost:5173` **and** `http://127.0.0.1:5173` |
| 19–23 | `DeliveryRequest` (Pydantic): `package_id?, weight=0.0, destination?, deadline_minutes=15.0` |
| 26–28 | `GET /` → `{"message": "Fleet Telemetry System API"}` |
| 31–40 | `GET /telemetry` → `{drones, charging_pads, queued_requests, metrics, alerts}` (subset of `get_status`, shaped for the dashboard) |
| 43–45 | `GET /status` → full `manager.get_status()` incl. `recent_assignments` |
| 48–51 | `POST /assign-request` → `manager.assign_request(...)` → `{accepted, selected_drone, reason, audit}` |

### 6.2 `api/fleet_manager.py` (226 lines) — the dispatcher

| Line | Content |
|------|---------|
| 8–9 | `BACKEND_PATH = <repo>/backend/telemetry.json` — **absolute**, resolved from `__file__`. Reads are CWD-independent (unlike the old shim); only the `uvicorn main:app` module import still requires launching from `api/` |
| 13–23 | `FleetManager`: in-memory `drones / requests / audit_log / request_log` + 3 stub `charging_pads` (all `occupied_by=None`, never synced with sim `CHARGING` states — pads are decorative) |
| 25–37 | `_load_drones()` — re-reads the JSON file on **every** public call; missing file or torn JSON → empty fleet (fail-soft, no 500) |
| 43–57 | `_distance_km` (degree-hypot `×111.32`, no latitude cosine correction) + `_estimate_travel_time_minutes` (`max(3.0, nearest_drone_km × 0.75)`, 8.0 fallback when fleet empty) |
| 59–73 | `_build_alerts()` — battery `<=20` count, pads-occupied flag, first queued/assigned request |
| 75–184 | `assign_request()` gate chain: `weight ∈ (0, 2.5]` → `deadline > 0` → feasibility (`deadline >= fastest_possible`) → per-drone eligibility (reject `CHARGING/RETURNING/LANDED`, battery `<=15`, weight>2.5) → score `battery + (25 if IDLE else 15)` → argmax wins; no eligible drone → request queued (`accepted=False`). Full per-candidate `audit` returned |
| 186–215 | `compute_metrics()` — `on_time_delivery_rate` (% requests with status completed/success/delivered), `total_energy_consumption_kwh` (`Σbattery × 0.008`), `pad_utilization_rate`, `mean_delay_per_late_package`, `fleet_variance_in_battery_degradation` (population variance of fleet battery) |
| 217–226 | `get_status()` — reloads file, returns all of the above + last 10 audit entries |

**Load-bearing flaws (proven, not hypothetical):**

1. **Assignments evaporate.** `assign_request` mutates `self.drones` in memory (`:173-177`), but every public method starts with `_load_drones()` (`:98,187,218`), which re-reads the file and discards those mutations. The sim also overwrites the file every 2 s. An assignment survives only until the next API call.
2. **Tests are coupled to the live file.** For the same reason, `tests/test_fleet_manager.py` fixtures are clobbered by the reload inside `assign_request` — 1 of 3 tests fails against live data (`selected_drone 110 != 101`). Run: `PYTHONPATH=. python3 -m unittest discover -s tests` from repo root.
3. **Pads are fiction.** `charging_pads` never reflects sim `CHARGING` drones; `compute_metrics` pad utilization is always 0 unless hand-edited.
4. `from fleet_manager import FleetManager` (`main.py:5`) vs `from api.fleet_manager import FleetManager` (tests) — two import styles for one module; `uvicorn api.main:app` from the root would break the former.

True data-flow rate: `1 snapshot / 2s` file→API→dashboard, plus on-demand dispatch decisions.

---

## 7. Dashboard: `dashboard/display/src/App.tsx` (367 lines)

Stack: React 19 + `react-leaflet@5` + `leaflet@1.9.4`, Vite.

* `fetch("http://127.0.0.1:8000/telemetry")` on mount + `setInterval(2000)` (`App.tsx:101-126`) — frequency-matched to sim, so aliasing/beating possible. Consumes `drones + charging_pads + queued_requests`; ignores `metrics/alerts` (recomputes its own).
* **Two-level classification** (the cleverest logic in repo):

```ts
stateColor: battery<=25 → red; CRUISE/...→blue; RETURNING→amber; else grey
normalizeState: battery<=25 → LOW_BATTERY; CRUISE/...→ACTIVE; RETURNING; else IDLE
```

This collapses 7 sim states → 4 UI buckets. `metrics = {total,active,returning,lowBattery,idle,avgBattery}` via `useMemo O(N)` (`:128-140`).

* New `fleet-overview` section (`:255-285`): **Charging Pads** panel (`Pad{id,occupied_by,time_remaining,queue}`, `:35-40,93-94,110`) and **Queued Requests** panel (`package_id/status`, `:95,111`). Styling gap: `fleet-overview` / `info-panel` classes have **no rules in `App.css`** — the panels render unstyled.
* Map: center `[31.7812939,76.997502]` (BASE STATION), zoom 15, OSM tiles (`:293-297`). `CircleMarker` per drone (10 DOM nodes — fine; 10k would need canvas/clustering). Selected drone gets `Polyline base→pos→dest` dashed + `flyTo(pos,15,0.8s)` via `FocusDrone`. Figure title shows live count `({drones.length} drones)` (`:290`).
* `Tooltip` (hover) + `Popup` (click) show `id,battery,state,base,destination`.
* Styling `App.css` (398 lines): dark sidebar + light topbar, critical alert banner if `lowBattery>0`. `index.css` is empty (0 lines); entry is `main.tsx` (`StrictMode` + `createRoot`).

Hardcodings: API URL, thresholds, map center — no env config.

---

## 8. Verdict: Strengths vs Load-Bearing Flaws

**Strengths:** deterministic world-gen, clean entity/store split, `O(1)` update + `O(N)` serialize is optimal for snapshot broadcast, UI bucketing is sound abstraction, dispatcher degrades gracefully on torn reads, dispatch decisions are fully audited.

**Must-fix for correctness:**

1. Atomic write (`write tmp + rename`) to kill torn reads (partly mitigated by fail-soft loader).
2. Unify battery thresholds across **three** layers (sim 20 vs UI 25 vs dispatcher 20-alert/15-eligible) and reconcile degree-geometry (sim isotropic steps vs dispatcher `×111.32` flat factor).
3. Persist dispatch decisions (write-back, or stop `_load_drones()` clobbering in-memory state); sync `charging_pads` with sim `CHARGING` drones or delete them; fix test isolation (inject drones instead of reloading disk — 1/3 tests currently red).
4. Model battery-death (no flight at 0%), handle `SIGTERM`, remove dead `START/APPROACH`, `distance2D`, `chargingTicksRemaining` (written never read), add missing `fleet-overview`/`info-panel` CSS, unify the `fleet_manager` import style.
5. Escape JSON strings, use haversine if going beyond demo.

Run order: `make && ./simulator` (root) → `cd api && uvicorn main:app --port 8000` → `npm run dev` (from `dashboard/display`). Tests: `PYTHONPATH=. python3 -m unittest discover -s tests` (root; expect 1 failure — see §6.2).
