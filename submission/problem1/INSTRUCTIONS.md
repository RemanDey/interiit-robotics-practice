# Fleet Telemetry System — Startup Instructions (Linux only)

Full-stack drone fleet telemetry prototype:

```text
sim/sim.cpp (C++, 10 drones, 2s tick, IIT Mandi North Campus)
  → backend/telemetry.json (file snapshot, overwritten every tick)
    → api/main.py + api/fleet_manager.py (FastAPI on :8000, dispatch + metrics)
      → dashboard/display (React 19 + Leaflet + Vite on :5173, polls every 2s)
```

The JSON file is the shared bus between simulator and API. The API
additionally exposes dispatch (`POST /assign-request`) and fleet
intelligence (`GET /status`) via an in-memory `FleetManager`.
There is no database and no websocket.

You need **3 terminals running at the same time, started in order**:
1. Simulator, 2. API, 3. Dashboard.

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
- Full status (pads, queue, metrics, alerts): `http://127.0.0.1:8000/status`
- Dashboard: `http://localhost:5173`

Details and troubleshooting below — read them if anything fails.

---

## 2. Step 1 — C++ Simulator (Terminal 1)

**Working directory matters.** The simulator writes to the relative path
`backend/telemetry.json`. You **must** run it from the repo root.
Do not double-click the binary in Files — run it from a terminal.

```bash
# from repo root: .../Fleet-Telemetry-System
make
ls -l simulator backend/telemetry.json
./simulator
```

What to expect:

- Builds with `g++ -std=c++17 -Wall -I. sim/sim.cpp common/id.cpp -o simulator` (see `Makefile`).
- Simulates 10 drones around the single `BASE STATION` at IIT Mandi North Campus (see `data/locations.h`).
- Runs forever in a `while(true)` loop, one tick every 2000 ms.
- Overwrites `backend/telemetry.json` every tick with 10 drones.
- Prints per-tick console output. Leave it running. `Ctrl+C` to stop.

Verify in another shell:

```bash
ls -l backend/telemetry.json
# timestamp should update every ~2s while simulator runs
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

---

## 3. Step 2 — FastAPI Dispatch API (Terminal 2)

`api/main.py` (52 lines) + `api/fleet_manager.py` (226 lines):

- `GET /` → `{"message": "Fleet Telemetry System API"}`
- `GET /telemetry` → `{drones, charging_pads, queued_requests, metrics, alerts}` (dashboard shape)
- `GET /status` → full status incl. `recent_assignments` audit trail
- `POST /assign-request` → dispatch a delivery: body `{package_id?, weight, destination?, deadline_minutes?}`; returns `{accepted, selected_drone, reason, audit}`. Rules: `weight ∈ (0, 2.5]`, feasible deadline, drone not CHARGING/RETURNING/LANDED with battery > 15; best score (`battery + IDLE bonus`) wins, otherwise the request is queued.

The manager resolves `backend/telemetry.json` via an **absolute path**
(`fleet_manager.py:8-9`), so file reads work regardless of CWD — but you
**must** still start uvicorn from inside `api/` because `main.py` does
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
# {"accepted":true,"selected_drone":101,...}
```

Keep this terminal running.

> Dispatch state (assignments, queue, audit log) lives only in the API
> process memory and is reloaded from disk on every call — assignments do
> not persist to `telemetry.json` and vanish on restart. See
> `EXPLANATION.md §6.2`.

### 3c. Run the tests (optional)

From the **repo root** (tests import `api.fleet_manager`):

```bash
PYTHONPATH=. python3 -m unittest discover -s tests
```

3 tests cover assignment, rejection (overweight/impossible deadline),
and metrics. Note: 1 test currently fails because `assign_request`
reloads the live `telemetry.json` over the fixtures — run the sim first
or see `EXPLANATION.md §6.2` for the isolation bug.

---

## 4. Step 3 — React Dashboard (Terminal 3)

Stack (see `dashboard/display/package.json`): Vite + React 19 +
`react-leaflet@5` + `leaflet@1.9.4`.

The app hardcodes `fetch("http://127.0.0.1:8000/telemetry")` in
`dashboard/display/src/App.tsx:103` and polls every 2s. The API's CORS
rule (`api/main.py:10-16`) allows `http://localhost:5173` and
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
- 10 `CircleMarker` drones, color-coded by battery/state
- Sidebar metrics (total / active / returning / low battery / avg battery)
- Charging-pads and queued-requests panels (served by the API)
- Click a drone for `Popup` with id, battery, state, base, destination + dashed route polyline

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
| Simulator | repo root | `./simulator` | writes `backend/telemetry.json` (no port) |
| API | `api/` | `uvicorn main:app --port 8000` | `http://127.0.0.1:8000/telemetry`, `/status`, `POST /assign-request` |
| Dashboard dev | `dashboard/display/` | `npm run dev` | `http://localhost:5173` |
| Tests | repo root | `PYTHONPATH=. python3 -m unittest discover -s tests` | n/a |

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
```

---

## 7. Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `Failed to write telemetry` | sim started outside repo root | `cd` to repo root, rerun `./simulator` |
| `ModuleNotFoundError: fleet_manager` from uvicorn | uvicorn started outside `api/` | `cd api`, rerun `uvicorn main:app --port 8000` |
| Telemetry shows wrong bases / drone count | stale `simulator` process from an old build overwriting the file | `ps aux \| grep simulator`, `kill <PID>`, restart `./simulator` |
| Dashboard empty map / `Failed to fetch` / CORS error | API not running | start Step 2 first, confirm `curl :8000/telemetry` works, use `http://localhost:5173` exactly |
| `pip install` → `externally-managed-environment` (Ubuntu) | PEP 668 | use `python3 -m venv .venv && source .venv/bin/activate` then `pip install fastapi uvicorn` |
| `npm run dev` → `vite: not found` | skipped `npm install` | `cd dashboard/display && npm install` |
| Torn JSON while sim writes | sim does non-atomic `ofstream` rewrite each tick | API fail-softs to an empty fleet and recovers next tick; just retry |
| Dashboard shows `LOW_BATTERY` but sim still flies drone | threshold mismatch: frontend 25 (`App.tsx:42`) vs sim 20 (`sim.cpp:29`) vs dispatcher 20/15 (`fleet_manager.py:61,116`) | expected demo quirk; unify thresholds to fix |
| Drones fly at `battery: 0` | sim has no dead-stick model | expected demo quirk |
| Assignment vanished after next poll/restart | dispatch state is API-process memory only | expected demo quirk; persist to file/DB to fix |
| 1 failing test in `tests/` | `assign_request` reloads live `telemetry.json` over fixtures | run sim first for green-ish state, or fix test isolation (see `EXPLANATION.md §6.2`) |

---

## 8. Known Quirks

- Sim output path is CWD-relative (`backend/telemetry.json`); the API reads via an absolute path but its module import still requires launching from `api/`.
- Data rate is 1 snapshot / 2s end to end.
- `EXPLANATION.md` has a full deconstruction (state machine, battery model, dispatch logic, map physics) if you want internals.
- `docs/README.md` is the file-by-file reference for every source file.
