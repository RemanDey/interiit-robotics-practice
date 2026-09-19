# Fleet-Telemetry-System: Olympiad-Calibre Deconstruction

## 🎬 Demo Video

[![Fleet Telemetry System Demo](https://img.youtube.com/vi/Uo0r4DJiyb4/0.jpg)](https://youtu.be/Uo0r4DJiyb4)

Watch the full demo: https://youtu.be/Uo0r4DJiyb4

---

## 0. One-Line Theorem

This is an **on-demand, bidirectional, 4-language, file-coupled cyber-physical simulation with a hybrid dispatch layer**: `C++` parks physics at base → `telemetry.json` carries snapshots forward → `Python/FastAPI + FleetManager` serves them and scores delivery requests → `commands.json` carries takeoff orders back → `C++` consumes them on liftoff → `React/Leaflet + RequestPage` renders everything and sources new demand. There is no database and no websocket.

```text
sim/sim.cpp (C++, N=3 agents, 1.25Hz loop, polls commands.json)
  → backend/telemetry.json (forward snapshot, O(N) each tick)
    → api/main.py + api/fleet_manager.py (FastAPI, absolute-path IO + hybrid dispatch)
      → dashboard/display/src/App.tsx (poll 2s, O(N) render on Leaflet)
      → dashboard/display/src/pages/RequestPage.tsx (/request → POST /assign-request)
api/fleet_manager.py --atomic inbox write--> backend/commands.json --polled/consumed--> sim/sim.cpp
```

Shared contracts: `common/types.h`, `data/locations.h`, `sim/commands.h`.
Verified against code: `NUM_OF_DRONES = 3` (`sim/sim.cpp:25`), 800 ms tick
(`sim/sim.cpp:230`), hybrid weights `ALLOT_WEIGHTS` (`api/fleet_manager.py:23-28`),
16 tests (`tests/test_fleet_manager.py`, 324 lines).

---

## 1. Axioms: `common/`

### `common/types.h:6-16` — `enum STATES`

```cpp
OFF, START, TAKEOFF, CRUISE, APPROACH, DELIVERY, RETURNING, LANDED, CHARGING
```

**Lemma 1 (Dead states):** `START, APPROACH` are never assigned in `sim/sim.cpp:103-214`. They are dead code. Only 7 states are live.

### `common/types.h:18-28` — Geometry

```cpp
struct Position { lat,lng,alt; };
struct Location { lat,lng,addr; };
```

Note the naming fracture that propagates: C++ uses `addr`, JSON/C++ serializer uses `"address"` (`backend/backend.h:135,140`), frontend type uses `address` (`dashboard/display/src/App.tsx:19-23`). This is correct only by manual mapping, no schema enforcement.

### `common/id.cpp:3-6` — UID Generator

```cpp
static int staticId = 100; return ++staticId;
```

**Invariant:** Single run with `NUM_OF_DRONES=3` yields IDs `101..103` exactly.
(The old docs said `101..110` for N=10 — rescale all such claims by the new fleet size.)

**Failure modes (contest pitfalls):**

1. Not thread-safe, not persistent — restart collides IDs.
2. Copy semantics: `Drone d(...); droneRegistry.push_back(d); fleet.addDrone(d)` — ID is copied, not re-generated. Correct, but subtle.

---

## 2. World Model: `data/locations.h`

* `BASES`: single fixed hub `BASE STATION` at IIT Mandi North Campus Main Gate (`31.7812939, 76.9975020`). All drones spawn, return, and recharge here; `sim/sim.cpp:69` takes `BASES.front()` directly (no random base choice).
* `DESTINATIONS`: `generateDestinations(1000)` with `mt19937(42)` — **deterministic**. Same 1000 addresses every run. `lat~U[31.765,31.795]`, `lng~U[76.985,77.010]` (~3.3 km × ~2.4 km box around campus), `house~U[100,9999] + 24 campus/village street names`, suffix `", Kamand Valley, HP"`.
* Seeded RNG = reproducible test fixture. Good olympiad practice.
* **New:** the live sim no longer draws from `DESTINATIONS` — destinations arrive on demand via `backend/commands.json` (user requests from `/request`). The pool survives for fixtures/tests only.

---

## 3. Agent: `sim/drone.h` — `class Drone`

Pure data + setters, no behavior. Fields: `id, pos, battery(0-100 clamped), state, destination, base, speed`.

Default `speed=0.0002` deg/tick, immediately overwritten by `speedDist~U[0.00015,0.00035]` in `sim/sim.cpp:19,77`. Same distribution as before; only the fleet size and tick rate changed.

Methods are `O(1)`: `movePos`, `drainBattery` (floor 0), `chargeBattery` (cap 100), all with clamping invariants.

---

## 4. Physics Engine: `sim/sim.cpp` — The Core (231 lines, on-demand)

### 4.1 Initialization `sim/sim.cpp:68-86`

For each `i < 3`: take `BASES.front()` as **both** base and destination, `offset~U[-0.0003,0.0003]^2`, spawn near base, state `LANDED`. Push to two parallel structures:

* `vector<Drone> droneRegistry` — ground truth.
* `DroneList fleet` — serializable mirror.
* `deliveryTicksRemaining[3]`, `chargingTicksRemaining[3]` — parallel arrays (fragile, index-coupled; the latter is written but never read).

No random destination, no auto-takeoff. Stdout confirms: `State: LANDED (awaiting delivery request)`.

### 4.2 Inbox Poll `sim/sim.cpp:97-101` + `sim/commands.h`

Once per tick, `readCommands(COMMANDS_PATH)` loads `backend/commands.json` into `unordered_map<int, Command>` keyed by `drone_id` (first entry wins on duplicates). Fail-soft: missing/corrupt file → empty inbox for this tick, never a crash. The parser is a hand-rolled, dependency-free mini-JSON layer (`sim/commands.h`, 162 lines: `parseNumber/parseString/seekKey/escapeInto`, atomic `writeCommands` via tmp + `rename` writing `"[]"` when empty).

### 4.3 Kinematics — The Only Math (now overshoot-safe)

```cpp
distance = sqrt(dLat*dLat + dLng*dLng)              // sim/sim.cpp:39-43,129,161
step = min(speed, distance); pos += step * (target-pos)/distance   // sim/sim.cpp:138,174
```

This is **normalized gradient descent in lat/lng degree space** with step `speed` deg/tick, now **clamped to the remaining distance**. The clamp fixes a real orbit bug: a full-speed step near the target used to overshoot it and circle forever outside the `0.0001 deg ~= 11m` arrival disc. Termination radius unchanged.

**Olympiad critique (unchanged):**

1. Ignores Earth curvature. At 31.78N, 1 deg lng ~= 94.6km vs 1 deg lat ~=111km — ~15% anisotropy distorted into isotropic steps. Acceptable for the ~3km campus box (<1% absolute error), wrong for real navigation. Correct fix: haversine + meters. (Note `api/fleet_manager.py:99` already uses a degree→km scale factor `×111.32` for its own estimates — the two layers disagree on geometry.)
2. `distance2D()` is now genuinely used only as documentation — CRUISE/RETURNING still inline their own `sqrt`. Near-dead helper.
3. Vertical and horizontal decoupled: `TAKEOFF: alt+=5` until `CRUISE_ALTITUDE=30`, `RETURNING`: horizontal first, then exact-to-zero descent (`sim/sim.cpp:166`: `descent = (alt>=5) ? -5 : -alt` — no alt overshoot below zero). Cruise at `30` labelled `ft` in the dashboard — 30ft cruise is physically absurd (real: 200-400ft). Unit ambiguity.
4. Tick is now **800 ms** (`sim/sim.cpp:230`), 2.5× faster than the old 2000 ms. Battery drain is still gated on `tick % 5 == 0`, so wall-clock drain is 2.5× faster too — a deliberate liveness-for-demo tradeoff, not a physics constant.

### 4.4 Battery Automaton

```cpp
maybeDrainBattery(tick, isMoving): if tick%5==0: -2 else -1
CHARGE_PER_TICK=+4/tick, LOW_BATTERY_THRESHOLD=20
```

Effective rates per tick-counter unit: `0.4%/tick` moving, `0.2%/tick` idle, `+4%/tick` charging. Asymmetric by 10x — deliberate to guarantee liveness (charge faster than drain).

**Threshold disagreement across three layers (unchanged):** simulator aborts at `20` (`sim.cpp:30`), dashboard flags at `25` (`App.tsx:60`), dispatcher alerts at `<=20` but declares drones eligible down to `>15` (`fleet_manager.py:112,320`). The UI raises `LOW_BATTERY` before the sim turns around, and the dispatcher may assign a drone the dashboard calls critical (battery 16–25).

**Liveness bug visible in data:** drones can fly at 0% (no dead-stick model). Safety property `battery>0 ==> flight` is violated.

### 4.5 State Machine — Formal Proof (on-demand revision)

| State | Guard → Action |
|---|---|
| `OFF` | → `LANDED` (park; old `OFF → TAKEOFF` auto-launch deleted) |
| `TAKEOFF` | `alt<30 → alt+=5`, else → `CRUISE` |
| `CRUISE` | `batt<=20 → RETURNING`; `dist<1e-4 → DELIVERY(3 ticks)`; else clamped step toward `dest` |
| `DELIVERY` | countdown 3 → `RETURNING` |
| `RETURNING` | clamped step toward `base`; if arrived: exact descent, then → `LANDED` |
| `LANDED` | battery-gated fork: `batt<100` without a healthy-battery command → `CHARGING` (opportunistic top-up); inbox entry for this drone → consume, set destination, → `TAKEOFF`; else hold at base with **no drain** |
| `CHARGING` | `batt<100 → +4`, else → `LANDED` (park and wait — no auto-destination, no auto-takeoff) |

**Theorem (Conditional liveness):** Without inbox entries, all paths converge to `LANDED` (parked, full battery) — the fleet is *quiescent*, not cycling. With a command for a parked, healthy drone, the path `LANDED→TAKEOFF→CRUISE→DELIVERY→RETURNING→LANDED` executes exactly once per command. Verified: `while(true)` at `sim/sim.cpp:94`, sleep 800 ms at `:230`.

The golden safety rule is enforced at pickup: a waiting command preempts charging **only if** `battery > LOW_BATTERY_THRESHOLD` (`sim/sim.cpp:186-187`).

### 4.6 Output `sim/sim.cpp:216-230`

```cpp
fleet.update(DroneState(d)); // O(1) hashmap replace, refreshes timestamp
// consumed inbox entries removed, remainder rewritten atomically (:219-224)
fleet.writeTelemetry("backend/telemetry.json"); // O(N) full rewrite EVERY tick
```

Telemetry path is CWD-relative — must launch from repo root, else silent `Failed to write telemetry`. Still no atomic rename on the telemetry side → **torn-read race** with the API reader persists (mitigated on the Python side: `fleet_manager.py:53-54` catches `JSONDecodeError` and serves an empty fleet rather than 500ing). The **inbox side is atomic on both ends** (Python tmp + `os.replace`, C++ tmp + `rename`), so dispatch orders never tear.

Build: `Makefile: CXX=g++ -std=c++17 -Wall -I. SRC=sim/sim.cpp common/id.cpp → simulator`.

---

## 5. Snapshot Store + Inbox: `backend/backend.h`, `backend/commands.json`, `sim/commands.h`

`backend/backend.h` (155 lines) is a **C++ header-only store**: `DroneState` (snapshot + `last_updated=now()`) + `DroneList { unordered_map<int,DroneState> }` with `addDrone O(1)`, `update O(1)`, `writeTelemetry O(N)` via manual `ofstream` string concatenation.

* No HTTP, no Python dict, no uvicorn here. The Python serving layer is `api/fleet_manager.py` (§6).
* Manual JSON: no escaping of `addr`. Safe today (addresses contain no `"`), fragile tomorrow.
* `timestamp` = serialization time, identical for all 3 drones per tick — not physics time.
* At 800 ms ticks the rewrite rate is 1.25 Hz; the dashboard still polls at 0.5 Hz, so roughly every other snapshot is never rendered — pure overhead for demo smoothness.

`backend/commands.json` (starts as `[]`) is the **reverse bus** (API → sim). Schema: `[{drone_id, lat, lng, address, package_id}, …]`, upsert-by-`drone_id`. `sim/commands.h` (162 lines) is its C++ endpoint: `struct Command`, `readCommands` (fail-soft parse), `writeCommands` (atomic tmp + `rename`). The Python endpoint is `FleetManager._read_commands/_write_command` (`api/fleet_manager.py:62-92`).

---

## 6. Dispatch API: `api/main.py` + `api/fleet_manager.py` (489 lines, hybrid on-demand)

### 6.1 `api/main.py` (52 lines) — routes (unchanged shape, richer payloads)

| Line | Content |
|------|---------|
| 5,8 | `from fleet_manager import FleetManager` + module-level `manager = FleetManager()` (single in-memory instance: request/audit logs live only in this process) |
| 10–16 | CORS allows `http://localhost:5173` **and** `http://127.0.0.1:5173` |
| 19–23 | `DeliveryRequest` (Pydantic): `package_id?, weight=0.0, destination?, deadline_minutes=15.0` |
| 26–28 | `GET /` → `{"message": "Fleet Telemetry System API"}` |
| 31–40 | `GET /telemetry` → `{drones, charging_pads, queued_requests, metrics, alerts}` (subset of `get_status`, shaped for the dashboard) |
| 43–45 | `GET /status` → full `manager.get_status()` incl. `pending_commands` + `recent_assignments` |
| 48–51 | `POST /assign-request` → `manager.assign_request(...)` → `{accepted, selected_drone, allotted_drone?, queue_position?, reason, audit, allotment?, cost_weights}` |

### 6.2 `api/fleet_manager.py` — the hybrid dispatcher

| Line | Content |
|------|---------|
| 9–12 | `BACKEND_PATH` + `COMMANDS_PATH` — **absolute**, resolved from `__file__`. Reads/writes are CWD-independent (unlike the old shim); only the `uvicorn main:app` module import still requires launching from `api/` |
| 16–28 | `AVG_MISSION_MINUTES = 10.0` (queue-wait estimator) + `ALLOT_WEIGHTS = {battery: 1.0, load: 10.0, distance: 5.0, slack: 1.0}` hybrid cost weights |
| 32–42 | `FleetManager`: in-memory `drones / requests / audit_log / request_log` + 3 charging pads **with map coordinates** (`lat`/`lng` offsets around the base for dashboard diamonds) |
| 44–60 | `_load_drones()` — re-reads telemetry on **every** public call; missing file or torn JSON → empty fleet (fail-soft, no 500) |
| 62–92 | `_read_commands() / _write_command()` — inbox access; upsert by `drone_id`, atomic tmp + `os.replace` |
| 94–108 | `_distance_km` (degree-hypot `×111.32`, no latitude cosine correction) + `_estimate_travel_time_minutes` (`max(3.0, nearest_drone_km × 0.75)`, 8.0 fallback when fleet empty) |
| 110–124 | `_build_alerts()` — battery `<=20` count, pads-occupied flag, first queued/assigned request |
| 126–134 | `_drone_load()` — active + queued jobs already on a drone's plate |
| 136–173 | `_score_candidate()` — hybrid cost `W_batt·battery + parked_bonus − W_load·load − W_dist·dist + W_slack·slack`, with `wait = load × 10 min`, `slack = max(0, deadline − wait − dist×0.75)`, `parked = 25 if IDLE/LANDED else 15`. Returns `(score, breakdown)`; every audit item carries `cost{load,battery,distance_km,slack_min,parked_bonus,hybrid}`. With the legacy preset (`load = distance = slack = 0`) this reduces term-for-term to the old `battery + bonus` rule (proven by `test_legacy_preset_reproduces_old_behavior`) |
| 175–191 | `_queued_for()` (FIFO backlog per drone) + `_inbox_ids()` (takeoff commands awaiting pickup) |
| 193–220 | `_sync_pads()` — **pads now mirror reality**: drones reporting `CHARGING` occupy pads in id order; overflow beyond 3 waits in round-robin `queue`s; `time_remaining` (min) from `ceil((100−batt)/4)` ticks × 2 s. Idempotent, rebuilt every `get_status`. The old "pads are fiction" flaw is fixed |
| 222–260 | `_drone_is_free() / _settle_and_promote()` — completion + auto-promotion engine: a drone is done when telemetry shows it parked (`IDLE`/`LANDED`, battery > 15) with no inbox entry → its `assigned` records flip to `completed`, then its oldest `queued` record promotes to `assigned` with a fresh inbox command. Idempotent; runs on every `assign_request`/`get_status` (side effect inside a status read is deliberate) |
| 262–440 | `assign_request()` gate chain: `weight ∈ (0, 2.5]` → `deadline > 0` → feasibility (`deadline >= fastest_possible`) → settle-and-promote → per-drone eligibility (**`LANDED`/`IDLE` now eligible** — parked means awaiting work; `CHARGING/RETURNING/OFF/START/TAKEOFF` unavailable; `CRUISE/DELIVERY/APPROACH` busy on assigned task; battery `<=15` rejected; pending-inbox and own-backlog drones skipped to prevent overwrite/queue-jumping) → hybrid argmax wins with full per-candidate `audit`; nobody eligible → **allot** to best hybrid-cost drone's backlog (`allotted_drone`, `queue_position`, `allotment` breakdown) instead of leaving it ownerless. Accepted requests write the durable inbox command (`:432`) plus a cosmetic in-memory `TAKEOFF` tweak; every decision returns `cost_weights` |
| 442–471 | `compute_metrics()` — `on_time_delivery_rate` (% requests with status completed/success/delivered), `total_energy_consumption_kwh` (`Σbattery × 0.008`), `pad_utilization_rate`, `mean_delay_per_late_package`, `fleet_variance_in_battery_degradation` (population variance of fleet battery) |
| 473–489 | `get_status()` — reloads file, settles/promotes, syncs pads, returns all of the above + `pending_commands` + last 10 audit entries |

**Load-bearing flaws: before → after:**

1. ~~Assignments evaporate~~ → **FIXED via the inbox.** The durable channel is `commands.json`, consumed on liftoff; the cosmetic in-memory `drone["state"]` tweak is still discarded by the next `_load_drones()`, but nothing load-bearing depends on it anymore.
2. **Tests are coupled to the live file — partly fixed.** The 13 new `OnDemandDispatchTests` stub `_load_drones` and redirect `COMMANDS_PATH` to tmp, so they are hermetic. The 3 legacy tests still reload disk — 1 of 16 fails against live data (`selected_drone 3 != 101` with the 3-drone fleet). Run: `PYTHONPATH=. python3 -m unittest discover -s tests` from repo root.
3. ~~Pads are fiction~~ → **FIXED via `_sync_pads()`.** Utilization is now nonzero whenever a drone charges; overflow queues are exercised by `test_pad_overflow_queues`.
4. `from fleet_manager import FleetManager` (`main.py:5`) vs `from api.fleet_manager import FleetManager` (tests) — two import styles for one module; `uvicorn api.main:app` from the root would break the former. (Unchanged.)
5. New subtle race: two `POST`s for the same parked drone in the same poll window — the second sees the first's inbox entry and correctly allots behind it (`Takeoff command already awaiting pickup`). The inbox guard converts the old overwrite bug into FIFO behavior.

True data-flow rate: `1.25 snapshots / s` sim→file, `1 poll / 2 s` file→API→dashboard, plus on-demand dispatch with ~800 ms sim pickup latency.

---

## 7. Dashboard: `dashboard/display/src/App.tsx` (611 lines) + `RequestPage.tsx` (244 lines)

Stack: React 19 + `react-leaflet@5` + `leaflet@1.9.4` + `react-router-dom@7`, Vite.

* `BrowserRouter` (`App.tsx:603-611`): `/` → `Dashboard`, `/request` → `RequestPage`. Brand renamed `FleetOps` → **`Drone Fleet Management`**.
* Dashboard polling unchanged in rhythm — `fetch("http://127.0.0.1:8000/telemetry")` on mount + `setInterval(2000)` — but the fleet is now 3 parked drones and the queue panel shows `→ Drone N` / `(pos K)` for assigned vs allotted jobs (`FleetRequest` type, `:48-55`).
* **Two-level classification** (unchanged logic, new threshold ref `:60`):
  `stateColor` / `normalizeState`: battery ≤ 25 → red/`LOW_BATTERY`; CRUISE/TAKEOFF/APPROACH/DELIVERY → blue/`ACTIVE`; RETURNING → amber; CHARGING/LANDED/OFF → grey/`IDLE`. Collapses 7 sim states → 4 UI buckets. `metrics` via `useMemo O(N)`.
* **Markers are now numbered divIcons** (`:96-140`), not `CircleMarker`s: `droneIcon` (circle badge with drone id, selected grows 26→30px), `destinationIcon` (purple triangle, active delivery), `queuedIcon` (amber triangle, queued backlog), `padIcon` (green/blue diamond). `App.css` (719 lines, +321) carries all four styles plus `map-legend` and the request-view layout — the old "unstyled `fleet-overview`" gap is closed.
* Map furniture: BASE STATION `Rectangle` (±0.00015° ≈ ±17 m) with tooltip/popup, per-pad `Marker`s at the API-supplied `lat`/`lng` with occupancy popups, per-drone `Marker` badges (tooltip on hover, popup on click), selected drone's dashed `Polyline base→pos→dest` + `flyTo(pos,15,0.8s)` via `FocusDrone`, and per-request delivery `Marker`s.
* **`spreadDeliveries` declutter** — orders sharing the exact same drop point would stack pixel-perfect; singles stay exact, groups spread on a small ring (~13 m) around the true point. Pure presentation, coordinates untouched.
* **`RequestPage.tsx`** — the demand source the old docs lacked: controlled form (`package_id`, weight validated `∈ (0, 2.5]`, lat/lng/address, deadline > 0) + click-to-pick Leaflet map (`ClickPicker` → lat/lng fields, `★` marker) → `POST http://127.0.0.1:8000/assign-request` → renders accepted/selected vs allotted/queue-position outcome plus the full per-drone audit. `← Back to Map` returns to `/`.
* `map-legend` overlay documents all 9 marker classes in situ.
* `metrics`/`visibleDrones` still `useMemo O(N)`; 3 drones + a handful of delivery markers are trivially fine (10k would need canvas/clustering — unchanged advice).

Hardcodings: API URLs, thresholds, map center — no env config. (Unchanged.)

---

## 8. Verdict: Strengths vs Load-Bearing Flaws

**Strengths:** deterministic world-gen, clean entity/store split, `O(1)` update + `O(N)` serialize is optimal for snapshot broadcast, UI bucketing is sound abstraction, dispatcher degrades gracefully on torn reads, dispatch decisions are fully audited **with per-term cost breakdowns**, assignments are now durable through the atomic inbox, pads mirror telemetry, backlogs drain automatically FIFO, and the previously untestable dispatch core now has 13 hermetic tests.

**Must-fix for correctness:**

1. Atomic telemetry write (`write tmp + rename`) to kill torn reads — the inbox side already does this on both ends; only `writeTelemetry` still tears (partly mitigated by the fail-soft loader).
2. Unify battery thresholds across **three** layers (sim 20 vs UI 25 vs dispatcher 20-alert/15-eligible) and reconcile degree-geometry (sim isotropic steps vs dispatcher `×111.32` flat factor).
3. ~~Persist dispatch decisions~~ → done via inbox; remaining: persist the backlog itself (API restart loses `requests` while the inbox file survives — a restarted server forgets *why* a pending command exists). Fix the last legacy test isolation (stub `_load_drones` like the on-demand suite does).
4. Model battery-death (no flight at 0%), handle `SIGTERM`, remove dead `START/APPROACH`, `distance2D` near-dead helper, `chargingTicksRemaining` (written never read), unify the `fleet_manager` import style.
5. Escape JSON strings in `writeTelemetry`, use haversine if going beyond demo.
6. New: throttle or delta-encode telemetry — 1.25 Hz full rewrites for a 0.5 Hz dashboard is 2.5× write overhead; consider matching the tick to the poll rate or writing on change.

Run order: `make && ./simulator` (root, 3 drones park `LANDED`) → `cd api && uvicorn main:app --port 8000` → `npm run dev` (from `dashboard/display`) → open `/request`, send a delivery, watch pickup in ~800 ms. Tests: `PYTHONPATH=. python3 -m unittest discover -s tests` (root; expect 1 legacy failure — see §6.2).
