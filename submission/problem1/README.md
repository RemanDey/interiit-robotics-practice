# Robotic-PS Problem 1: Drone Fleet Management

This is a standard-library Python implementation of the Problem 1 drone fleet manager. It includes a central dispatcher, charging pad manager, telemetry anomaly filter, simulated drone client, portal, unit tests, and algorithm documentation.

## Layout

```text
Robotic-PS/
├── server/
│   ├── src/
│   │   ├── dispatcher.py
│   │   ├── pad_manager.py
│   │   ├── telemetry_filter.py
│   │   ├── models.py
│   │   └── server.py
│   └── tests/
├── clients/
│   ├── drone_client.py
│   └── mock_telemetry.py
├── portal/
│   └── index.html
├── docs/
│   ├── ALGORITHM_NOTE.md
│   └── EDGE_CASES.md
├── INSTRUCTIONS.md
└── README.md
```

## Run

From this directory:

```bash
python3 server/src/server.py
```

Open `portal/index.html` in a browser. The portal polls `http://127.0.0.1:8080/state` and renders drone positions, request statuses, pad occupancy, and fleet metrics.

Submit a request:

```bash
curl -X POST http://127.0.0.1:8080/request \
  -H 'Content-Type: application/json' \
  -d '{"request_id":"REQ-NEW","destination":[900,1200,50],"payload_kg":1.4,"deadline_s":1200}'
```

Advance simulation:

```bash
curl -X POST http://127.0.0.1:8080/tick \
  -H 'Content-Type: application/json' \
  -d '{"dt_s":30}'
```

Inspect decision audits:

```bash
curl http://127.0.0.1:8080/audit
```

## Test

```bash
python3 -m unittest discover -s server/tests
```

## Implemented Requirements

- 10-drone fleet and 3 shared charging pads.
- Payload-dependent speed and energy model.
- Degraded battery capacity and cycle-count tracking.
- Hard rejection of overweight and impossible-deadline requests.
- Safety reserve before every assignment.
- Decision audit with per-candidate feasibility and cost breakdown.
- Pad reservation, emergency queue priority, and opportunistic charging.
- Telemetry filtering for bad SoC, impossible coordinates, and speed outliers.
- Link-loss marking for airborne drones.
- Simple live web portal.
