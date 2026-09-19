# Fleet Telemetry System — Drone Fleet Management for Timed Deliveries

## 🎬 Demo Video

[![Fleet Telemetry System Demo](https://img.youtube.com/vi/Uo0r4DJiyb4/0.jpg)](https://youtu.be/Uo0r4DJiyb4)

Watch the full demo: https://youtu.be/Uo0r4DJiyb4

---

A full-stack autonomous drone delivery fleet prototype: a **C++ simulator** parks 3
on-demand delivery drones at **IIT Mandi North Campus**, snapshots the fleet to a
**JSON file** every 800 ms, a **FastAPI dispatch API** serves that file plus
hybrid-scored delivery assignment with per-drone backlogs, and a **React + Leaflet
dashboard** renders it live on a map alongside a **delivery-request page** that
tasks drones through a reverse **command inbox**.

```text
sim/sim.cpp (C++, 3 drones, 800 ms tick, polls backend/commands.json)
  → backend/telemetry.json (file snapshot, overwritten every tick)
    → api/main.py + api/fleet_manager.py (FastAPI on :8000)
      → dashboard/display/src/App.tsx (React 19 + Leaflet, polls every 2 s on :5173)
            + dashboard/display/src/pages/RequestPage.tsx (/request, POST /assign-request)

RequestPage.tsx --POST /assign-request--> fleet_manager.py --writes--> backend/commands.json
     --polled each tick--> sim.cpp (LANDED drone consumes entry, TAKEOFF)
```

There is **no database and no websocket**. Two JSON files are the shared buses:
`telemetry.json` (sim → API → dashboard) and `commands.json` (API → sim takeoff inbox).
Backlog/assignment state beyond the inbox lives only in API process memory.

See `PROBLEM_STATEMENT.md` for the original task spec,
`EXPLANATION.md` for the deep deconstruction,
`INSTRUCTIONS.md` for the startup runbook, and `docs/README.md` for the file-by-file reference.

---

## Repository layout

```text
.
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
│       ├── App.tsx              # router + dashboard map + polling
│       ├── pages/RequestPage.tsx# delivery-request form + click-to-pick map
│       ├── App.css              # sidebar/topbar/markers/legend/request-form styling
│       └── main.tsx             # React entry point
├── docs/
│   └── README.md                # full file-by-file documentation
├── EXPLANATION.md               # deep deconstruction (state machine, battery model, physics)
├── INSTRUCTIONS.md              # startup instructions (Linux)
├── PROBLEM_STATEMENT.md         # original problem spec (10-drone fleet, rules, edge cases)
└── REFERENCES.md                # reference links
```

---

## Quick Start (TL;DR)

You need **3 terminals running at the same time, started in order**:
1. Simulator, 2. API, 3. Dashboard. Then send a delivery from
`http://localhost:5173/request` and watch a parked drone take off.

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

Full details, troubleshooting, and port-conflict fixes: see `INSTRUCTIONS.md`.

---

## How it works

1. **`./simulator` (repo root, 800 ms tick):** `sim.cpp` polls `backend/commands.json`,
   advances 3 drones (`LANDED → TAKEOFF → CRUISE → DELIVERY → RETURNING → LANDED → CHARGING → LANDED`),
   and rewrites `backend/telemetry.json` every tick.
2. **`uvicorn main:app` (from `api/`):** `FleetManager` re-reads telemetry on every call.
   `GET /telemetry` serves the dashboard shape, `GET /status` adds `pending_commands` + audit,
   `POST /assign-request` runs the hybrid dispatch (`battery/load/distance/slack` weights) and writes
   an inbox command for the sim. Settle-and-promote + pad-sync run on every call.
3. **`npm run dev` (from `dashboard/display`, every 2 s):** `App.tsx` fetches `:8000/telemetry`
   and re-renders drones, pads, and delivery triangles on Leaflet.
   `/request` (`RequestPage.tsx`) POSTs new deliveries and renders the per-drone audit;
   the sim picks up the inbox entry next tick (~800 ms) and flies it.

Effective rates: **1.25 snapshots/s** sim → file, **1 poll/2 s** API → dashboard,
plus on-demand dispatch with ~800 ms sim pickup latency.

---

## Dispatch model (summary)

- **Eligibility:** `LANDED`/`IDLE` at base with battery > 15. `CHARGING/RETURNING/OFF/START/TAKEOFF`
  unavailable, `CRUISE/DELIVERY/APPROACH` busy. Drones with a pending inbox entry or own backlog are skipped.
- **Hybrid score:** `W_batt·battery + parked_bonus − W_load·load − W_dist·distance_km + W_slack·slack_min`
  with `ALLOT_WEIGHTS = {battery: 1.0, load: 10.0, distance: 5.0, slack: 1.0}`.
- **Durable assignment:** accepted requests upsert one entry into `backend/commands.json` (atomic tmp+rename).
- **Backlog allotment:** when nothing can fly now, the request is allotted to the best drone's backlog
  (`allotted_drone`, `queue_position`) and auto-launches on landing.
- **Pads:** rebuilt every `get_status` from sim `CHARGING` states (id order, overflow queues, `time_remaining`).

See `api/README.md` and `EXPLANATION.md §6` for the full model.

---

## Tests

From the repo root:

```bash
PYTHONPATH=. python3 -m unittest discover -s tests
```

16 tests: 3 legacy + 13 on-demand (eligibility, inbox persistence/upsert guards,
auto-promotion, hybrid weights, pad sync/overflow). Note: 1 legacy test fails against
live data unless fixtures are stubbed — see `EXPLANATION.md §6.2`.

---

## Docs map

| File | Role |
|------|------|
| `PROBLEM_STATEMENT.md` | Original 10-drone spec: params, golden safety rule, edge cases, extensions |
| `INSTRUCTIONS.md` | Linux startup runbook: 3-terminal bring-up, ports/URLs, troubleshooting matrix |
| `EXPLANATION.md` | Olympiad-style deconstruction: state-machine proof, battery math, dispatch analysis, flaws |
| `docs/README.md` | Full file-by-file reference for every source file |
| `api/README.md` | API + hybrid dispatch-model reference |
| `backend/README.md` | `telemetry.json` + `commands.json` schema notes |
| `dashboard/display/README.md` | Frontend reference: routes, markers, legend, request page |
| `REFERENCES.md` | Reference links |
