"""Fleet simulation + tiny JSON API: /state, /audit, /request, /tick."""

import json
import math
import random
from dataclasses import asdict, is_dataclass
from enum import Enum
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from time import time

from dispatcher import Dispatcher
from models import (
    BASE_POSITION, DeliveryRequest, DroneStatus,
    build_default_fleet, build_default_pads,
    cruise_speed_mps, distance_m, energy_for_leg_wh,
)
from pad_manager import PadManager
from telemetry_filter import TelemetryFilter


class FleetSimulation:
    def __init__(self):
        self.now_s = 0.0
        self.fleet = build_default_fleet()
        self.pads = build_default_pads()
        self.pad_manager = PadManager(self.pads)
        self.dispatcher = Dispatcher(self.fleet, self.pad_manager)
        self.telemetry = TelemetryFilter()
        for drone in self.fleet.values():
            drone.last_heartbeat_s = self.now_s

    def submit(self, request_id, destination, payload_kg, deadline_s):
        req = DeliveryRequest(request_id, tuple(destination), payload_kg, self.now_s, self.now_s + deadline_s)
        return self.dispatcher.submit_request(req, self.now_s)

    def tick(self, dt_s=1.0):
        self.now_s += dt_s
        self.pad_manager.tick_charging(self.fleet, self.now_s, dt_s)
        self.telemetry.mark_link_losses(self.fleet, self.now_s)
        for drone in self.fleet.values():
            if drone.status in {DroneStatus.IN_FLIGHT, DroneStatus.RETURNING}:
                self._move(drone, dt_s)
        self.dispatcher.requeue_unassigned(self.now_s)

    def _move(self, drone, dt_s):
        remaining = distance_m(drone.position, drone.target)
        if remaining <= 1e-6:
            return self._arrive(drone)
        speed = cruise_speed_mps(drone.payload_kg, drone.speed_factor)
        step = min(remaining, speed * dt_s)
        k = step / remaining
        drone.position = tuple(drone.position[i] + (drone.target[i] - drone.position[i]) * k for i in range(3))
        energy = energy_for_leg_wh(step, drone.payload_kg)
        drone.total_distance_m += step
        drone.total_energy_wh += energy
        drone.soc_pct = max(0.0, drone.soc_pct - 100.0 * energy / drone.usable_capacity_wh)
        drone.cycle_count += energy / drone.usable_capacity_wh
        drone.last_heartbeat_s = self.now_s
        if step >= remaining - 1e-6:
            self._arrive(drone)

    def _arrive(self, drone):
        if drone.status == DroneStatus.IN_FLIGHT:
            self.dispatcher.complete_request(drone, self.now_s)
        elif drone.status == DroneStatus.RETURNING:
            drone.position = BASE_POSITION
            self.pad_manager.dock_drone(drone, self.now_s)

    def snapshot(self):
        return {
            "time_s": round(self.now_s, 2),
            "drones": [{"id": d.drone_id, "status": d.status.value, "position": [round(v, 2) for v in d.position],
                        "soc_pct": round(d.soc_pct, 2), "soh_pct": round(d.soh_pct, 2),
                        "payload_kg": d.payload_kg, "active_request": d.active_request} for d in self.fleet.values()],
            "requests": [{"id": r.request_id, "status": r.status.value, "assigned_drone": r.assigned_drone,
                          "deadline_s": r.deadline_at, "eta_s": r.eta_s, "reason": r.reason}
                         for r in self.dispatcher.requests.values()],
            "pads": [{"id": p.pad_id, "occupied_by": p.occupied_by, "reserved_by": p.reserved_by,
                      "available_at": round(p.available_at, 2)} for p in self.pads.values()],
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
        return {k: json_ready(v) for k, v in value.items()}
    if isinstance(value, list):
        return [json_ready(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, obj):
        body = json.dumps(obj, indent=2).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _body(self):
        return json.loads(self.rfile.read(int(self.headers.get("Content-Length", "0"))) or b"{}")

    def do_GET(self):
        if self.path == "/state":
            self._send(200, SIM.snapshot())
        elif self.path == "/audit":
            self._send(200, {"audits": json_ready(SIM.dispatcher.audit_log[-20:])})
        else:
            self._send(404, {"error": "use /state or /audit"})

    def do_POST(self):
        if self.path == "/request":
            p = self._body()
            audit = SIM.submit(p.get("request_id", f"REQ-{int(time())}"),
                               p.get("destination", [random.randint(200, 2500), random.randint(200, 2500), 40]),
                               float(p.get("payload_kg", 1.0)), float(p.get("deadline_s", 1800.0)))
            self._send(201, {"audit": json_ready(audit)})
        elif self.path == "/tick":
            SIM.tick(float(self._body().get("dt_s", 30.0)))
            self._send(200, SIM.snapshot())
        else:
            self._send(404, {"error": "unknown endpoint"})

    def log_message(self, *args):
        pass


def seed(sim):
    sim.submit("REQ-001", (1200.0, 250.0, 60.0), 1.1, 12 * 60)
    sim.submit("REQ-002", (800.0, 1600.0, 45.0), 2.2, 20 * 60)
    sim.submit("REQ-003", (2600.0, -200.0, 50.0), 0.6, 18 * 60)


def main():
    seed(SIM)
    for _ in range(5):
        SIM.tick(30.0)
    print("Fleet server running on http://127.0.0.1:8080")
    ThreadingHTTPServer(("127.0.0.1", 8080), Handler).serve_forever()


if __name__ == "__main__":
    main()
