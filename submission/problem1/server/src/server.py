"""Runnable central server simulation and lightweight JSON API."""

from __future__ import annotations

import json
import math
import random
from dataclasses import asdict, is_dataclass
from enum import Enum
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from time import time
from typing import Dict, Tuple

from dispatcher import Dispatcher
from models import (
    BASE_POSITION,
    DeliveryRequest,
    Drone,
    DroneStatus,
    RequestStatus,
    build_default_fleet,
    build_default_pads,
    cruise_speed_mps,
    distance_m,
    energy_for_leg_wh,
)
from pad_manager import PadManager
from telemetry_filter import TelemetryFilter, TelemetrySample


class FleetSimulation:
    def __init__(self) -> None:
        self.now_s = 0.0
        self.fleet = build_default_fleet()
        self.pads = build_default_pads()
        self.pad_manager = PadManager(self.pads)
        self.dispatcher = Dispatcher(self.fleet, self.pad_manager)
        self.telemetry = TelemetryFilter()
        for drone in self.fleet.values():
            drone.last_heartbeat_s = self.now_s

    def submit(self, request_id: str, destination: Tuple[float, float, float], payload_kg: float, deadline_s: float):
        request = DeliveryRequest(request_id, destination, payload_kg, self.now_s, self.now_s + deadline_s)
        return self.dispatcher.submit_request(request, self.now_s)

    def tick(self, dt_s: float = 1.0) -> None:
        self.now_s += dt_s
        self.pad_manager.tick_charging(self.fleet, self.now_s, dt_s)
        self.telemetry.mark_link_losses(self.fleet, self.now_s)
        for drone in self.fleet.values():
            if drone.status not in {DroneStatus.IN_FLIGHT, DroneStatus.RETURNING}:
                continue
            self._advance_drone(drone, dt_s)
        self.dispatcher.requeue_unassigned(self.now_s)

    def _advance_drone(self, drone: Drone, dt_s: float) -> None:
        target = drone.target
        remaining_m = distance_m(drone.position, target)
        speed = cruise_speed_mps(drone.payload_kg, drone.speed_factor)
        step_m = min(remaining_m, speed * dt_s)
        if remaining_m <= 1e-6:
            self._arrive(drone)
            return
        ratio = step_m / remaining_m
        next_position = tuple(drone.position[i] + (target[i] - drone.position[i]) * ratio for i in range(3))
        energy_wh = energy_for_leg_wh(step_m, drone.payload_kg)
        drone.total_distance_m += step_m
        drone.total_energy_wh += energy_wh
        drone.soc_pct = max(0.0, drone.soc_pct - 100.0 * energy_wh / drone.usable_capacity_wh)
        drone.cycle_count += energy_wh / drone.usable_capacity_wh
        drone.position = next_position
        drone.last_heartbeat_s = self.now_s
        if step_m >= remaining_m - 1e-6:
            self._arrive(drone)

    def _arrive(self, drone: Drone) -> None:
        if drone.status == DroneStatus.IN_FLIGHT and drone.active_request:
            self.dispatcher.complete_request(drone, self.now_s)
            return
        if drone.status == DroneStatus.RETURNING:
            drone.position = BASE_POSITION
            self.pad_manager.dock_drone(drone, self.now_s)

    def ingest_telemetry(self, sample: TelemetrySample):
        return self.telemetry.ingest(self.fleet[sample.drone_id], sample)

    def snapshot(self) -> Dict[str, object]:
        return {
            "time_s": round(self.now_s, 2),
            "drones": [
                {
                    "id": drone.drone_id,
                    "status": drone.status.value,
                    "position": [round(v, 2) for v in drone.position],
                    "soc_pct": round(drone.soc_pct, 2),
                    "soh_pct": round(drone.soh_pct, 2),
                    "payload_kg": drone.payload_kg,
                    "active_request": drone.active_request,
                }
                for drone in self.fleet.values()
            ],
            "requests": [
                {
                    "id": req.request_id,
                    "status": req.status.value,
                    "assigned_drone": req.assigned_drone,
                    "deadline_s": req.deadline_at,
                    "eta_s": req.eta_s,
                    "reason": req.reason,
                }
                for req in self.dispatcher.requests.values()
            ],
            "pads": [
                {
                    "id": pad.pad_id,
                    "occupied_by": pad.occupied_by,
                    "reserved_by": pad.reserved_by,
                    "available_at": round(pad.available_at, 2),
                }
                for pad in self.pads.values()
            ],
            "queue": list(self.pad_manager.queue),
            "metrics": self.dispatcher.metrics(max(1.0, self.now_s)).__dict__,
        }


SIM = FleetSimulation()


def json_ready(value):
    if is_dataclass(value):
        return json_ready(asdict(value))
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, dict):
        return {key: json_ready(item) for key, item in value.items()}
    if isinstance(value, list):
        return [json_ready(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


class FleetRequestHandler(BaseHTTPRequestHandler):
    def _json(self, status: int, payload: Dict[str, object]) -> None:
        body = json.dumps(payload, indent=2).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        if self.path == "/state":
            self._json(200, SIM.snapshot())
        elif self.path == "/audit":
            self._json(200, {"audits": json_ready(SIM.dispatcher.audit_log[-20:])})
        else:
            self._json(404, {"error": "use /state or /audit"})

    def do_POST(self) -> None:
        length = int(self.headers.get("Content-Length", "0"))
        payload = json.loads(self.rfile.read(length) or b"{}")
        if self.path == "/request":
            audit = SIM.submit(
                payload.get("request_id", f"REQ-{int(time())}"),
                tuple(payload.get("destination", [random.randint(200, 2500), random.randint(200, 2500), 40])),
                float(payload.get("payload_kg", 1.0)),
                float(payload.get("deadline_s", 1800.0)),
            )
            self._json(201, {"audit": json_ready(audit)})
        elif self.path == "/tick":
            SIM.tick(float(payload.get("dt_s", 30.0)))
            self._json(200, SIM.snapshot())
        else:
            self._json(404, {"error": "unknown endpoint"})


def seed_demo_requests(sim: FleetSimulation) -> None:
    sim.submit("REQ-001", (1200.0, 250.0, 60.0), 1.1, 12 * 60)
    sim.submit("REQ-002", (800.0, 1600.0, 45.0), 2.2, 20 * 60)
    sim.submit("REQ-003", (2600.0, -200.0, 50.0), 0.6, 18 * 60)


def main() -> None:
    seed_demo_requests(SIM)
    for _ in range(5):
        SIM.tick(30.0)
    server = ThreadingHTTPServer(("127.0.0.1", 8080), FleetRequestHandler)
    print("Fleet server running on http://127.0.0.1:8080")
    print("Portal can be opened from portal/index.html")
    server.serve_forever()


if __name__ == "__main__":
    main()
