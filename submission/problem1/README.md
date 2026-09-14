# Robotic-PS Problem 1: Drone Fleet Management

Autonomous delivery fleet: **10 drones, 3 charging pads, 1 base** serving a continuous stream of
timed delivery requests with payload-dependent physics, degraded batteries, pad contention,
telemetry validation, link-loss handling, a web portal, and a live matplotlib 2D simulator.

Core guarantee (**golden safety rule**): a drone is never assigned a job unless
`outbound energy + return energy + 12% reserve` fits in its **degraded** usable capacity.
Every assignment writes a full decision audit (all 10 candidates compared).

## Contents

- [Features](#features)
- [Layout](#layout)
- [Physics & Decision Model](#physics--decision-model)
- [Prerequisites](#prerequisites)
- [Quickstart](#quickstart)
- [2D Simulator (`clients/fleet_2d.py`)](#2d-simulator-clientsfleet_2dpy)
- [Web Portal](#web-portal)
- [Drone Client](#drone-client)
- [HTTP API](#http-api)
- [Examples](#examples)
- [Tests](#tests)
- [Fault Injection](#fault-injection)
- [Configuration](#configuration)
- [Troubleshooting](#troubleshooting)

## Features

- 10-drone fleet (`DR-01`–`DR-10`), 3 shared pads (`PAD-1`–`PAD-3`), single base at origin.
- Payload-dependent speed `v(m) = 12·(1 − 0.35·m/2.5)·f` and energy model.
- Battery health `SoH = max(50, 100·(1 − 0.0005·cycles))`, cycle tracking, 40-min linear charge.
- Hard rejects: overweight (`> 2.5 kg`), impossible deadline (even ideal flight misses).
- Cost ranking: `deadline + energy + workload + pad-wait + low-battery` → cheapest feasible wins.
- Pad reservation with probe-then-release scoring, critical-SoC (`< 15%`) preemption + queue-jump,
  `HOLDING` vs `RECOVERING` on dock failure.
- Telemetry filter: SoC range, impossible position, implied-speed cap (28 m/s),
  SoC-jump cap (18 % / 60 s), wind `speed_factor` clamp, 20 s link-loss → `Lost`.
- Live web portal + matplotlib 2D simulator with click-to-dispatch and fault buttons.
- Request / battery / audit logs + fleet metrics (on-time %, energy, pad util, late delay, cycle variance).

## Layout

```text
submission/problem1/
├── server/src/
│   ├── models.py            # constants, Drone/Request/Pad dataclasses, dist/speed/energy
│   ├── dispatcher.py        # feasibility gates + cost ranking + assign/complete/requeue/metrics
│   ├── pad_manager.py       # reserve/release/dock/charge tick + utilization
│   ├── telemetry_filter.py  # ingest validation + mark_link_losses
│   └── server.py            # FleetSimulation (tick/move/arrive/snapshot) + HTTP API
├── server/tests/
│   ├── test_dispatcher.py
│   └── test_pad_and_telemetry.py
├── clients/
│   ├── fleet_2d.py          # live matplotlib 2D simulator (this is the visual sim)
│   ├── drone_client.py      # headless poller that also advances the sim
│   └── mock_telemetry.py    # 5-sample accept/reject demo
├── portal/index.html        # browser dashboard polling /state
├── docs/
│   ├── ALGORITHM_NOTE.md
│   └── EDGE_CASES.md
├── INSTRUCTIONS.md
├── PROBLEM_STATEMENT.md
└── README.md
```

## Physics & Decision Model

| Quantity | Formula | Code |
|---|---|---|
| Distance | `dist(a, b)` Euclidean | `models.py: distance_m` |
| Speed | `max(3, 12·(1 − 0.35·m/2.5)·f)` | `cruise_speed_mps` |
| Leg energy | `(ℓ/v)·(520/1500)·(0.52 + 0.48·m/2.5)` | `energy_for_leg_wh` |
| Health | `max(50, 100·(1 − 0.0005·cycles))` | `Drone.soh_pct` |
| Usable / avail | `U = 520·SoH/100`, `A = U·SoC/100` | `usable_capacity_wh` |

Eligibility per drone (`dispatcher._score_drone`): status in `{Idle, Returning, Holding}`,
`SoC ≥ 15%`, `E_out + E_back + 12%·U ≤ A`, `now + ETA ≤ deadline`.
Cost = `80·ETA/urgency + E/12 + max(0,c−c̄)·25 + flights·0.6 + pad_wait/30 + (10 if SoC<30)`.
Full derivation: `docs/ALGORITHM_NOTE.md`; edge cases: `docs/EDGE_CASES.md`.

## Prerequisites

- Python 3.9+.
- Zero dependencies for server / tests / portal / headless client.
- Only the 2D simulator needs: `pip install matplotlib`.

## Quickstart

From `submission/problem1/`:

```bash
# 1. server (seeds REQ-001..003, ticks 5×30 s, serves :8080)
python3 server/src/server.py

# 2. web portal — open portal/index.html in a browser (auto-refresh 2 s)

# 3. 2D simulator (separate terminal, needs matplotlib)
pip install matplotlib
python3 clients/fleet_2d.py

# 4. headless drone client (optional, also advances sim 5 s per loop)
python3 clients/drone_client.py DR-01
```

## 2D Simulator (`clients/fleet_2d.py`)

Direct-linked visualizer: drives the real `FleetSimulation` via `tick()` + `snapshot()`
(no HTTP, no forked physics). Seeded demo loads automatically when the request table is empty.

Map (left, ±3000 m, equal aspect):

- Base = black square at origin; pads = labeled boxes (`red` = occupied).
- Drones = `^` with 30-step trail (`red` SoC<20, `blue` In-Flight, `green` idle, gray `x` Lost).
- Assigned job = blue dashed line drone→destination + orange `★`; queued = orange `○`.

Side panel (right): sim time, pad queue, on-time %, total energy, pad util, mean late delay,
per-drone `status/SoC/job`, last 6 requests.

Controls:

| Control | Action |
|---|---|
| Click map | `submit(REQ-VIS-N, (x, y, 50), payload_slider, deadline_slider)` |
| Play/Pause, `+30s`, Reset | run control (`Reset` rebuilds sim + trails) |
| `dt s` slider (1–120) | seconds advanced per animation frame (~300 ms) + `t` key |
| `payload kg` / `deadline min` sliders | values used by next click-dispatch |
| `wind factor` slider (0.4–1.2) | sets `speed_factor` on all drones live |
| `drone 1–10` slider | selects victim for fault buttons |
| Link loss / Drain 20% / Wind .8 | freeze heartbeat (→ Lost), `SoC −= 20`, force `f = 0.8` |
| Keys `space` / `t` / `r` | pause, single tick, reset |

Headless check (no display): `MPLBACKEND=Agg python3 clients/fleet_2d.py` renders via `savefig`.

## Web Portal

`portal/index.html` polls `GET /state` every 2 s: canvas map (x/8 scale, same color rule),
tables for drones (`ID/Status/SoC/SoH/Job`), pads, requests, plus `Advance 30s` (`POST /tick`).

## Drone Client

`clients/drone_client.py DR-01` loops: `GET /state` → update local pos/SoC/state
(`Executing` if In-Flight/Returning, `ReturnHome` if Lost) → `POST /tick {dt_s: 5}` → print.
On exception: `stale += 1`, `HOLDING` (< 3) else `RETURN_HOME`. Run up to 10 copies (`DR-01`–`DR-10`).

## HTTP API

Base `http://127.0.0.1:8080`. All JSON; `inf` serializes as `null`.

| Method | Path | Body → Response |
|---|---|---|
| `GET` | `/state` | → `{time_s, drones[], requests[], pads[], queue[], metrics{}}` |
| `GET` | `/audit` | → `{audits[≤20]}` each with `selected_drone`, 10 `candidates`, `decision_reason` |
| `POST` | `/request` | `{request_id, destination[x,y,z], payload_kg, deadline_s} → {audit}` (`201`) |
| `POST` | `/tick` | `{dt_s} → snapshot` (charges, moves, completes, docks, requeues) |

Drone entry: `{id, status, position[3], soc_pct, soh_pct, payload_kg, active_request}`.
Request entry: `{id, status, assigned_drone, deadline_s, eta_s, reason}`.

## Examples

```bash
curl -X POST http://127.0.0.1:8080/request \
  -H 'Content-Type: application/json' \
  -d '{"request_id":"REQ-004","destination":[1500,600,50],"payload_kg":1.2,"deadline_s":1500}'

curl -X POST http://127.0.0.1:8080/tick \
  -H 'Content-Type: application/json' -d '{"dt_s":30}'

curl http://127.0.0.1:8080/state | python3 -m json.tool
curl http://127.0.0.1:8080/audit  | python3 -m json.tool
```

Overweight (`3.0 kg`) → `Rejected`; unreachable (`[5000,0,20]`, 30 s) →
`Rejected: impossible deadline`; all-busy → `Queued` and retried earliest-deadline-first each tick.

## Tests

```bash
python3 -m unittest discover -s server/tests
# 8 tests: overweight/impossible reject, audit breadth, degraded-capacity,
# saturated-fleet queue, pad-queue priority, telemetry reject, link-loss
```

## Fault Injection

```bash
python3 clients/mock_telemetry.py
# 0s accept / 2s accept / 4s reject (teleport) / 6s reject (SoC jump) / 8s accept
```

In the 2D sim select a drone then press **Link loss** (→ `Lost` after 20 s sim time),
**Drain 20%** (forces pad contention / preemption path), **Wind .8** (grows all ETAs).
In code: set `drone.speed_factor`, drop `soc_pct`, or backdate `last_heartbeat_s`.

## Configuration

Tune in `server/src/models.py`: `MAX_PAYLOAD_KG`, `BASE_SPEED_MPS`, `PAYLOAD_SPEED_ALPHA`,
`NOMINAL_BATTERY_WH` (520), `FULL_PAYLOAD_ENDURANCE_S` (1500), `CHARGE_TIME_S` (2400),
`DEGRADATION_PER_FULL_CYCLE` (0.0005), `RESERVE_SOC` (12), `CRITICAL_SOC` (15), `LOW_SOC` (30),
`FLEET_SIZE` (10), `PAD_COUNT` (3). Filter caps in `telemetry_filter.py`
(`max_speed_mps=28`, `max_soc_jump_pct=18`, `timeout_s=20`); map limits `LIM=3000`,
trail `TRAIL=30` in `clients/fleet_2d.py`.

## Troubleshooting

- `port 8080 in use`: stop the old server; only one `FleetSimulation` owns the port.
- Portal blank/CORS: serve `portal/` over HTTP or allow `file://` fetch; check server is up.
- `fleet_2d.py` no window (SSH/CI): use `MPLBACKEND=Agg` headless render path.
- Clicks do nothing: click inside axes; check payload/deadline sliders (rejects still log to audit).
- All requests `Queued`: fleet saturated or wind/SOC too strict — drain faults or `+30s` to free drones.
- `Lost` drones never recover in sim: by design `mark_link_losses` is one-way; reset sim or re-seed.
