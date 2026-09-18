# `api/` — FastAPI dispatch API

## Dependencies

```bash
pip install fastapi uvicorn   # pydantic arrives bundled with fastapi
```

## Files

| File | Role |
|------|------|
| `main.py` (52 lines) | Routes: `GET /` (health), `GET /telemetry` (dashboard shape), `GET /status` (full status + audit), `POST /assign-request` (dispatch). CORS allows `http://localhost:5173` and `http://127.0.0.1:5173`. Holds one module-level `FleetManager` |
| `fleet_manager.py` (226 lines) | Dispatcher: absolute-path telemetry reads, 3 stub charging pads, scored delivery assignment (`weight ∈ (0, 2.5]`, feasible deadline, eligibility + `battery + IDLE-bonus` scoring), fleet metrics, alerts, in-memory request/audit logs |
| `__init__.py` | Package marker |

## Run

```bash
cd api
uvicorn main:app --port 8000
```

Must start from inside `api/` (for the `from fleet_manager import …`
module import — file reads themselves are absolute-path). Verify with
`curl http://127.0.0.1:8000/telemetry` and
`curl http://127.0.0.1:8000/status`.

Dispatch example:

```bash
curl -X POST http://127.0.0.1:8000/assign-request \
  -H 'Content-Type: application/json' \
  -d '{"package_id":"PKG-1","weight":0.8,"destination":{"lat":31.7905,"lng":77.0098,"address":"Drop zone"},"deadline_minutes":20}'
```

Caveats: dispatch state is process memory only (lost on restart, and
assignments are discarded on the next telemetry reload — see
`EXPLANATION.md §6.2`). Tests live in `tests/` and run from the repo
root: `PYTHONPATH=. python3 -m unittest discover -s tests`.
