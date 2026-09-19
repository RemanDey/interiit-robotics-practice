# Fleet Telemetry System — Startup Instructions (Linux only)

Full-stack on-demand drone fleet telemetry prototype:

```text
dashboard/display/src/pages/RequestPage.tsx (/request)
  --POST /assign-request--> api/main.py + api/fleet_manager.py (FastAPI on :8000)
    --writes--> backend/commands.json (dispatch inbox, atomic tmp+rename)
      --polled each tick--> sim/sim.cpp (C++, 3 drones, 800 ms tick, IIT Mandi North Campus)
        → backend/telemetry.json (file snapshot, overwritten every tick)
          → api (serves telemetry + pads + queue + pending_commands)
            → dashboard/display (React 19 + Leaflet + Vite on :5173, polls every 2s)
              map (/) + delivery-request page (/request)
```

Two JSON files are the shared buses: `telemetry.json` (sim → API →
dashboard) and `commands.json` (API → sim takeoff inbox). Backlog state
beyond the inbox lives only in API process memory.
There is no database and no websocket.

You need **3 terminals running at the same time, started in order**:
1. Simulator, 2. API, 3. Dashboard. Then send a delivery from
`http://localhost:5173/request` and watch a parked drone take off.

---

## 0. Prerequisites

Tested on Ubuntu/Debian Linux.

| Tool | Needed for | Check | Install (Debian/Ubuntu) |
|------|------------|-------|--------------------------|
| `g++` (C++17) | simulator | `g++ --version` | `sudo apt update && sudo apt install -y g++ make` |
| `make` | simulator build | `make --version` | same as above |
| `python3` + `pip` | FastAPI API | `python3 --version && pip3 --version` | `sudo apt install -y python3 python3-pip python3-venv` |
| `node` + `npm` | dashboard | `node -v && npm -v` | install Node 20+ from [nodejs.org](https://nodejs.org/) or via `nvm`, then `npm -v` |

Clone / enter the repo:

```bash
cd Fleet-Telemetry-System
pwd   # must show .../Fleet-Telemetry-System for Step 1
ls    # should show: api/ backend/ common/ dashboard/ data/ sim/ tests/ Makefile
```

---

## 1. Quick Start (TL;DR)

Terminal 1 (repo root):

```bash
make
./simulator
# 3 drones park LANDED at BASE STATION, polling backend/commands.json
```

Terminal 2 (API):

```bash
cd api
pip install fastapi uvicorn
uvicorn main:app --port 8000
```

Terminal 3 (dashboard):

```bash
cd dashboard/display
npm install
npm run dev
```

Open:

- API check: `http://127.0.0.1:8000/telemetry`
- Full status (pads, queue, pending commands, metrics, alerts): `http://127.0.0.1:8000/status`
- Dashboard map: `http://localhost:5173`
- **Request a delivery: `http://localhost:5173/request`** (form or click the picker map; a parked drone takes off within ~800 ms)

Details and troubleshooting below — read them if anything fails.

---

## 2. Step 1 — C++ Simulator (Terminal 1)

**Working directory matters.** The simulator reads `backend/commands.json`
and writes to the relative path `backend/telemetry.json`. You **must**
run it from the repo root. Do not double-click the binary in
Files — run it from a terminal.

```bash
# from repo root: .../Fleet-Telemetry-System
make
ls -l simulator backend/telemetry.json backend/commands.json
cat backend/commands.json   # starts as []
./simulator
```

What to expect:

- Builds with `g++ -std=c++17 -Wall -I. sim/sim.cpp common/id.cpp -o simulator` (see `Makefile`).
- Parks 3 drones (`NUM_OF_DRONES = 3`, `sim/sim.cpp:25`) in `LANDED` at the single `BASE STATION` at IIT Mandi North Campus (see `data/locations.h`). No random auto-missions: a drone lifts off only when your request addresses it.
- Runs forever in a `while(true)` loop, **one tick every 800 ms** (`sim/sim.cpp:230`).
- Every tick: polls `backend/commands.json` (`sim/commands.h`), advances the state machine (`LANDED → TAKEOFF → CRUISE → DELIVERY → RETURNING → LANDED → CHARGING → LANDED`), opportunistically tops up idle batteries to 100%, consumes picked-up commands (stdout: `[Command] Drone N tasked to …`), and overwrites `backend/telemetry.json` with 3 drones.
- Prints per-tick console output. Leave it running. `Ctrl+C` to stop.

Verify in another shell:

```bash
ls -l backend/telemetry.json
# timestamp should update roughly every second while simulator runs
watch -n 1 ls -l backend/telemetry.json
```

Rebuild from scratch:

```bash
make clean
make
```

> If you see `Failed to write telemetry`, you started `simulator` from the
> wrong directory. `cd` back to the repo root and rerun `./simulator`.
>
> If telemetry shows stale bases/drone counts, a leftover `simulator`
> process from an old build may be overwriting the file:
> `ps aux | grep simulator`, `kill <PID>`, then restart `./simulator`.
>
> If drones never take off after a request, check the inbox:
> `cat backend/commands.json` — an unconsumed entry means the sim isn't
> polling it (wrong CWD or stale binary); an empty file right after an
> accepted request means the sim already picked it up (check drone state).

---

## 3. Step 2 — FastAPI Dispatch API (Terminal 2)

`api/main.py` (52 lines) + `api/fleet_manager.py` (489 lines):

- `GET /` → `{"message": "Fleet Telemetry System API"}`
- `GET /telemetry` → `{drones, charging_pads, queued_requests, metrics, alerts}` (dashboard shape)
- `GET /status` → full status incl. `pending_commands` + `recent_assignments` audit trail
- `POST /assign-request` → on-demand dispatch: body `{package_id?, weight, destination?, deadline_minutes?}`; returns `{accepted, selected_drone, allotted_drone?, queue_position?, reason, audit, allotment?, cost_weights}`. Rules: `weight ∈ (0, 2.5]`, feasible deadline, drone parked (`LANDED`/`IDLE`) at base with battery > 15, no pending inbox entry, no own backlog; hybrid score (`battery/load/distance/slack` via `ALLOT_WEIGHTS`) wins, otherwise the request is **allotted** to a drone's backlog (`allotted_drone`, `queue_position`) and auto-launches on landing.

The manager resolves `backend/telemetry.json` and `backend/commands.json`
via **absolute paths** (`fleet_manager.py:9-12`), so file reads/writes
work regardless of CWD — but you **must** still start uvicorn from
inside `api/` because `main.py` does
`from fleet_manager import FleetManager` (a top-level module import).

### 3a. Install Python deps with pip

```bash
cd api              # .../Fleet-Telemetry-System/api
pwd                 # confirm you are in api/

# Option A — system pip (simplest):
pip3 install fastapi uvicorn
# (pydantic comes bundled with fastapi; needed for DeliveryRequest)

# Option B — venv (recommended on Ubuntu 23.04+ where PEP 668
# blocks system pip; use this if `pip install` errors with
# "externally-managed-environment"):
python3 -m venv .venv
source .venv/bin/activate
pip install fastapi uvicorn
```

There is no `requirements.txt` in this repo. No other Python deps needed.

### 3b. Run the API

```bash
# still inside .../Fleet-Telemetry-System/api, venv activated if you use one
uvicorn main:app --port 8000
```

Expected log:

```text
INFO:     Started server process
INFO:     Uvicorn running on http://127.0.0.1:8000 (Press CTRL+C to quit)
```

Verify:

```bash
curl http://127.0.0.1:8000/
# {"message":"Fleet Telemetry System API"}

curl http://127.0.0.1:8000/telemetry | head -c 500
# {"drones": [{"id": 101, ...}], "charging_pads": [...], ...}

curl -X POST http://127.0.0.1:8000/assign-request \
  -H 'Content-Type: application/json' \
  -d '{"package_id":"PKG-1","weight":0.8,"destination":{"lat":31.7905,"lng":77.0098,"address":"Drop zone"},"deadline_minutes":20}'
# {"accepted":true,"selected_drone":101,...,"cost_weights":{"battery":1.0,...}}
# ... then: cat ../backend/commands.json  → entry for drone 101 (consumed by sim in ~800 ms)

# All drones busy:
# {"accepted":false,"selected_drone":null,"allotted_drone":101,"queue_position":1,...}
```

Keep this terminal running.

> Accepted assignments are durable via `backend/commands.json` (consumed
> on liftoff); backlog/queue state beyond the inbox lives only in the API
> process memory. Every `assign/status` call also runs the
> auto-promotion engine (completes landed missions, launches the next
> queued delivery) and re-syncs pads from `CHARGING` telemetry. See
> `EXPLANATION.md §6.2`.

### 3c. Run the tests (optional)

From the **repo root** (tests import `api.fleet_manager`):

```bash
PYTHONPATH=. python3 -m unittest discover -s tests
```

16 tests: 3 legacy (assignment, rejection for overweight/impossible
deadline, metrics) + 13 on-demand (eligibility, inbox persistence/upsert
guards, auto-promotion, hybrid weights, pad sync/overflow). Note: 1
legacy test still fails because `assign_request` reloads the live
`telemetry.json` over its fixtures unless they are stubbed — run the
sim first or see `EXPLANATION.md §6.2` for the isolation note.

---

## 4. Step 3 — React Dashboard (Terminal 3)

Stack (see `dashboard/display/package.json`): Vite + React 19 +
`react-leaflet@5` + `leaflet@1.9.4` + `react-router-dom@7`.

Two routes: `/` (live map) and `/request` (delivery-request form +
click-to-pick map → `POST /assign-request`). The app hardcodes
`fetch("http://127.0.0.1:8000/telemetry")` / `.../assign-request` in
`dashboard/display/src/App.tsx` and `src/pages/RequestPage.tsx:81`,
and polls telemetry every 2s. The API's CORS rule
(`api/main.py:10-16`) allows `http://localhost:5173` and
`http://127.0.0.1:5173` (Vite defaults) — so use one of those URLs.

### 4a. Install Node deps with npm

```bash
cd dashboard/display   # .../Fleet-Telemetry-System/dashboard/display
pwd                    # confirm

npm install
```

This reads `package.json` / `package-lock.json` and installs into `node_modules/`.
Re-run it after any `git pull` that changes `package.json`.

### 4b. Run dev server

```bash
# still inside dashboard/display
npm run dev
```

Expected output:

```text
VITE ... ready in ... ms
➜  Local:   http://localhost:5173/
```

Open `http://localhost:5173` in a browser. You should see:

- Leaflet map centered on IIT Mandi North Campus `[31.7812939, 76.997502]`, zoom 15
- BASE STATION rectangle + 3 charging-pad diamonds (green free / blue occupied)
- 3 numbered drone badges, color-coded by battery/state
- Sidebar metrics (total / active / returning / low battery / avg battery)
- Charging-pads and queued-requests panels (with `→ Drone N` / queue position)
- Active (purple) and queued (amber) delivery triangles with drone # + legend overlay
- Click a drone for `Popup` with id, battery, state, base, destination + dashed route polyline

Send a delivery at `http://localhost:5173/request`:

- Fill the form (or click the picker map for lat/lng), `Request delivery`
- Accepted → purple triangle appears, drone takes off within ~800 ms
- All busy → amber triangle allotted to a drone's backlog; it launches automatically when that drone lands

Keep this terminal running. `Ctrl+C` to stop.

Optional production build:

```bash
npm run build
npm run preview   # serves dist/ locally
```

---

## 5. Ports & URLs Summary

| Service | Dir to run from | Command | URL |
|---------|----------------|---------|-----|
| Simulator | repo root | `./simulator` | writes `backend/telemetry.json` (no port), polls `backend/commands.json` |
| API | `api/` | `uvicorn main:app --port 8000` | `http://127.0.0.1:8000/telemetry`, `/status`, `POST /assign-request` |
| Dashboard map | `dashboard/display/` | `npm run dev` | `http://localhost:5173` |
| Request page | (same dev server) | — | `http://localhost:5173/request` |
| Tests | repo root | `PYTHONPATH=. python3 -m unittest discover -s tests` | n/a (16 tests) |

If a port is busy:

```bash
ss -tlnp | grep -E '8000|5173'
# kill the PID shown, or use another port:
uvicorn main:app --port 8001   # note: dashboard still fetches :8000, so prefer killing :8000
npm run dev -- --port 5174     # note: API CORS only allows :5173 origins, so prefer killing :5173
```

---

## 6. Stop / Clean

Press `Ctrl+C` in each of the 3 terminals, in reverse order (dashboard → API → sim).

```bash
# repo root
make clean      # removes simulator
rm -rf api/.venv dashboard/display/node_modules  # full clean (optional)
# reset the dispatch inbox if you want a fresh demo:
echo '[]' > backend/commands.json
```

---

## 7. Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `Failed to write telemetry` | sim started outside repo root | `cd` to repo root, rerun `./simulator` |
| `ModuleNotFoundError: fleet_manager` from uvicorn | uvicorn started outside `api/` | `cd api`, rerun `uvicorn main:app --port 8000` |
| Telemetry shows wrong bases / drone count | stale `simulator` process from an old build overwriting the file | `ps aux \| grep simulator`, `kill <PID>`, restart `./simulator` |
| Drones sit `LANDED` and never fly | on-demand fleet: no command yet (expected until you request) | send a delivery from `/request` or `POST /assign-request`; check `cat backend/commands.json` |
| Accepted request consumed but drone still parked | battery-gated preemption: drone went to `CHARGING` (batt ≤ 20) | wait for top-up to 100% — the backlog auto-promotes on the next poll |
| Dashboard empty map / `Failed to fetch` / CORS error | API not running | start Step 2 first, confirm `curl :8000/telemetry` works, use `http://localhost:5173` exactly |
| `/request` submit → `Request failed: Failed to fetch` | API down or wrong port | confirm `uvicorn` on `:8000`; the page hardcodes `http://127.0.0.1:8000/assign-request` |
| `pip install` → `externally-managed-environment` (Ubuntu) | PEP 668 | use `python3 -m venv .venv && source .venv/bin/activate` then `pip install fastapi uvicorn` |
| `npm run dev` → `vite: not found` | skipped `npm install` | `cd dashboard/display && npm install` |
| Torn JSON while sim writes | sim does non-atomic `ofstream` rewrite each tick | API fail-softs to an empty fleet and recovers next tick; just retry (commands inbox itself is atomic both sides) |
| Dashboard shows `LOW_BATTERY` but sim still flies drone | threshold mismatch: frontend 25 (`App.tsx:60`) vs sim 20 (`sim.cpp:30`) vs dispatcher 20/15 (`fleet_manager.py:112,320`) | expected demo quirk; unify thresholds to fix |
| Drones fly at `battery: 0` | sim has no dead-stick model | expected demo quirk |
| Assignment vanished after API restart | backlog state is API-process memory only (inbox file survives, queue does not) | expected demo quirk; persist to file/DB to fix |
| 1 failing test in `tests/` | legacy `assign_request` test reloads live `telemetry.json` over fixtures | run sim first for green-ish state, or stub `_load_drones` like the 13 on-demand tests do (see `EXPLANATION.md §6.2`) |

---

## 8. Known Quirks

- Sim output path is CWD-relative (`backend/telemetry.json` + `backend/commands.json`); the API reads/writes via absolute paths but its module import still requires launching from `api/`.
- Data rates: 1.25 snapshots/s sim → file, 1 poll/2 s API → dashboard, ~800 ms dispatch pickup.
- `EXPLANATION.md` has a full deconstruction (on-demand state machine, hybrid dispatch math, inbox protocol, map physics) if you want internals.
- `docs/README.md` is the file-by-file reference for every source file.
