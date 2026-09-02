# Instructions

## Prerequisites

- Python 3.9 or newer.
- No external Python packages are required.

## Start The Central Server

```bash
cd submission/problem1/Robotic-PS
python3 server/src/server.py
```

The server starts at `http://127.0.0.1:8080` with three seeded demo requests.

## Open The Portal

Open:

```text
submission/problem1/Robotic-PS/portal/index.html
```

The dashboard refreshes automatically and also has an `Advance 30s` button.

## Run A Drone Client

In another terminal:

```bash
cd submission/problem1/Robotic-PS
python3 clients/drone_client.py DR-01
```

Start more clients by changing the ID from `DR-01` to `DR-10`.

## Submit Delivery Requests

```bash
curl -X POST http://127.0.0.1:8080/request \
  -H 'Content-Type: application/json' \
  -d '{"request_id":"REQ-004","destination":[1500,600,50],"payload_kg":1.2,"deadline_s":1500}'
```

Fields:

- `request_id`: unique package identifier.
- `destination`: `[x, y, z]` in meters.
- `payload_kg`: package mass.
- `deadline_s`: seconds from current server simulation time.

## Inspect System State

```bash
curl http://127.0.0.1:8080/state
curl http://127.0.0.1:8080/audit
```

## Run Tests

```bash
cd submission/problem1/Robotic-PS
python3 -m unittest discover -s server/tests
```

## Manual Fault Injection

```bash
cd submission/problem1/Robotic-PS
python3 clients/mock_telemetry.py
```

This prints accepted and rejected telemetry samples for abrupt battery jumps and invalid positions.
