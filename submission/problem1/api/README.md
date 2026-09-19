# `api/` — FastAPI dispatch API (on-demand fleet)

## 🎬 Demo Video

[![Fleet Telemetry System Demo](https://img.youtube.com/vi/Uo0r4DJiyb4/0.jpg)](https://youtu.be/Uo0r4DJiyb4)

Watch the full demo: https://youtu.be/Uo0r4DJiyb4

---

## Dependencies

```bash
pip install fastapi uvicorn   # pydantic arrives bundled with fastapi
```

## Files

| File | Role |
|------|------|
| `main.py` (52 lines) | Routes: `GET /` (health), `GET /telemetry` (dashboard shape), `GET /status` (full status + audit + `pending_commands`), `POST /assign-request` (dispatch). CORS allows `http://localhost:5173` and `http://127.0.0.1:5173`. Holds one module-level `FleetManager` |
| `fleet_manager.py` (489 lines) | On-demand dispatcher: absolute-path telemetry reads, `_sync_pads()` mirroring sim `CHARGING` states, hybrid-scored delivery assignment (`battery/load/distance/slack` via `ALLOT_WEIGHTS`), per-drone backlog allotment + `_settle_and_promote()` auto-promotion, atomic `backend/commands.json` inbox writes, fleet metrics, alerts, in-memory request/audit logs |
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
# {"accepted":true,"selected_drone":101,...,"cost_weights":{...}}

# All drones busy → backlogged instead of ownerless:
# {"accepted":false,"selected_drone":null,"allotted_drone":101,"queue_position":1,...}
```

## Dispatch model (what `fleet_manager.py` does)

- **Eligibility** (`assign_request`, `:303-347`): `LANDED`/`IDLE` at base
  with battery > 15 is eligible (parked = awaiting work).
  `CHARGING/RETURNING/OFF/START/TAKEOFF` → `"Drone unavailable"`;
  `CRUISE/DELIVERY/APPROACH` → `"Drone busy on assigned task"`.
  Drones with a pending inbox entry or their own queued backlog are
  skipped (no overwrite, no queue-jumping).
- **Hybrid score** (`_score_candidate`, `:136-173`):
  `W_batt·battery + parked_bonus − W_load·load − W_dist·distance_km + W_slack·slack_min`,
  with `ALLOT_WEIGHTS = {battery: 1.0, load: 10.0, distance: 5.0, slack: 1.0}`
  and `AVG_MISSION_MINUTES = 10.0` for queue-wait estimates.
  Every audit item carries the full `cost` breakdown.
  The legacy preset (`load/distance/slack = 0`) reduces exactly to the old
  `battery + IDLE-bonus` rule.
- **Durable assignment**: accepted requests write one upsert-by-`drone_id`
  entry to `backend/commands.json` atomically (`_write_command`, `:72-92`);
  the sim polls and consumes it on liftoff. The in-memory
  `drone["state"] = "TAKEOFF"` tweak is cosmetic only.
- **Backlog allotment** (`:349-403`): when nothing can fly now, the
  request is allotted to the best hybrid-cost drone's backlog
  (`allotted_drone`, `queue_position`) instead of being left ownerless.
- **Auto-promotion** (`_settle_and_promote`, `:232-260`): every
  `assign_request`/`get_status` call completes `assigned` records for
  parked drones with no inbox entry and launches each free drone's
  oldest `queued` record with a fresh inbox command (FIFO, idempotent).
- **Pads** (`_sync_pads`, `:193-220`): rebuilt from scratch every
  `get_status` — drones in `CHARGING` occupy pads in id order,
  overflow waits in round-robin `queue`s, `time_remaining` (min) from
  the sim charge model (`ceil((100−batt)/4)` ticks × 2 s). Pads carry
  map coordinates (`lat`/`lng`) for dashboard markers.

Caveats: dispatch state is process memory only (lost on restart).
Tests live in `tests/` and run from the repo root:
`PYTHONPATH=. python3 -m unittest discover -s tests`
(16 tests: 3 legacy + 13 on-demand; 1 legacy test still fails because
`assign_request` reloads the live `telemetry.json` over its fixtures —
see `EXPLANATION.md §6.2`).
