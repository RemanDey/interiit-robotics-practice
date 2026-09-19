from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Dict, List

BASE_DIR = Path(__file__).resolve().parent.parent
BACKEND_PATH = BASE_DIR / "backend" / "telemetry.json"


class FleetManager:
    def __init__(self) -> None:
        self.drones: List[Dict[str, Any]] = []
        self.requests: List[Dict[str, Any]] = []
        self.charging_pads: List[Dict[str, Any]] = [
            {"id": 1, "lat": 31.7812939, "lng": 76.997502, "occupied_by": None, "time_remaining": 0, "queue": []},
            {"id": 2, "lat": 31.7813939, "lng": 76.997602, "occupied_by": None, "time_remaining": 0, "queue": []},
            {"id": 3, "lat": 31.7811939, "lng": 76.997402, "occupied_by": None, "time_remaining": 0, "queue": []},
        ]
        self.audit_log: List[Dict[str, Any]] = []
        self.request_log: List[Dict[str, Any]] = []
        self._load_drones()

    def _load_drones(self) -> List[Dict[str, Any]]:
        if not BACKEND_PATH.exists():
            self.drones = []
            return self.drones

        try:
            data = json.loads(BACKEND_PATH.read_text(encoding="utf-8"))
            raw_drones = data.get("drones", []) if isinstance(data, dict) else []
            self.drones = raw_drones
        except (json.JSONDecodeError, OSError):
            self.drones = []

        return self.drones

    def load_telemetry(self) -> Dict[str, List[Dict[str, Any]]]:
        self._load_drones()
        return {"drones": self.drones}

    def _distance_km(self, a: Dict[str, Any], b: Dict[str, Any]) -> float:
        lat1 = float(a.get("lat", 0.0) or 0.0)
        lng1 = float(a.get("lng", 0.0) or 0.0)
        lat2 = float(b.get("lat", 0.0) or 0.0)
        lng2 = float(b.get("lng", 0.0) or 0.0)
        return math.hypot(lat2 - lat1, lng2 - lng1) * 111.32

    def _estimate_travel_time_minutes(self, destination: Dict[str, Any]) -> float:
        if not self.drones:
            return 8.0

        distances = [self._distance_km(dr.get("position", {}), destination) for dr in self.drones]
        if not distances:
            return 8.0
        return max(3.0, min(distances) * 0.75)

    def _build_alerts(self) -> List[str]:
        alerts: List[str] = []
        low_battery_count = sum(1 for d in self.drones if float(d.get("battery", 0)) <= 20)
        if low_battery_count:
            alerts.append(f"{low_battery_count} drones are below the low-battery threshold")

        if any(pad.get("occupied_by") is not None for pad in self.charging_pads):
            alerts.append("Charging pads are currently occupied")

        for request in self.requests:
            if request.get("status") in {"queued", "assigned"}:
                alerts.append(f"Request {request.get('package_id')} requires attention")
                break

        return alerts

    def assign_request(self, request: Dict[str, Any]) -> Dict[str, Any]:
        request = dict(request or {})
        package_id = request.get("package_id") or request.get("id") or f"PKG-{len(self.requests) + 1}"
        weight = float(request.get("weight", 0.0) or 0.0)
        destination = request.get("destination") or {"lat": 31.79, "lng": 77.01, "address": "Default"}
        deadline_minutes = float(request.get("deadline_minutes", request.get("deadline", 0.0)) or 0.0)

        if weight <= 0 or weight > 2.5:
            return {
                "accepted": False,
                "selected_drone": None,
                "reason": "Overweight request or invalid payload",
                "audit": [],
            }

        if deadline_minutes <= 0:
            return {
                "accepted": False,
                "selected_drone": None,
                "reason": "Missing or invalid deadline",
                "audit": [],
            }

        self._load_drones()
        fastest_possible = self._estimate_travel_time_minutes(destination)
        if deadline_minutes < fastest_possible:
            return {
                "accepted": False,
                "selected_drone": None,
                "reason": f"Impossible deadline: {deadline_minutes} min < {fastest_possible:.2f} min fastest possible",
                "audit": [],
            }

        audit: List[Dict[str, Any]] = []
        eligible: List[Dict[str, Any]] = []
        for drone in self.drones:
            state = str(drone.get("state", "IDLE")).upper()
            battery = float(drone.get("battery", 0) or 0)
            if state in {"CHARGING", "RETURNING", "LANDED"}:
                audit.append({"drone_id": drone.get("id"), "eligible": False, "score": 0.0, "reason": "Drone unavailable"})
                continue
            if battery <= 15:
                audit.append({"drone_id": drone.get("id"), "eligible": False, "score": 0.0, "reason": "Battery below safe reserve"})
                continue
            if weight > 2.5:
                audit.append({"drone_id": drone.get("id"), "eligible": False, "score": 0.0, "reason": "Payload exceeds payload limit"})
                continue

            score = battery + (25 if state == "IDLE" else 15)
            item = {
                "drone_id": drone.get("id"),
                "eligible": True,
                "score": score,
                "battery": battery,
                "state": state,
                "distance_km": self._distance_km(drone.get("position", {}), destination),
            }
            eligible.append(item)
            audit.append(item)

        if not eligible:
            queued_request = dict(request)
            queued_request["package_id"] = package_id
            queued_request["status"] = "queued"
            queued_request["deadline_minutes"] = deadline_minutes
            self.requests.append(queued_request)
            self.request_log.append({
                "package_id": package_id,
                "weight": weight,
                "assigned_drone": None,
                "status": "queued",
                "deadline_minutes": deadline_minutes,
            })
            return {
                "accepted": False,
                "selected_drone": None,
                "reason": "No drone was eligible; request queued",
                "audit": audit,
            }

        selected = max(eligible, key=lambda item: item["score"])
        selected_drone = int(selected["drone_id"])
        record = {
            "package_id": package_id,
            "weight": weight,
            "destination": destination,
            "deadline_minutes": deadline_minutes,
            "assigned_drone": selected_drone,
            "status": "assigned",
        }
        self.requests.append(record)
        self.request_log.append(record)
        self.audit_log.append({
            "package_id": package_id,
            "selected_drone": selected_drone,
            "candidates": audit,
        })

        for drone in self.drones:
            if int(drone.get("id")) == selected_drone:
                drone["state"] = "CRUISE"
                drone["destination"] = destination
                break

        return {
            "accepted": True,
            "selected_drone": selected_drone,
            "reason": "Assigned to best-eligible drone",
            "audit": audit,
        }

    def compute_metrics(self) -> Dict[str, float]:
        self._load_drones()
        total_requests = len(self.requests)
        completed = sum(1 for request in self.requests if str(request.get("status", "")).lower() in {"completed", "success", "delivered"})
        on_time_rate = (completed / total_requests * 100.0) if total_requests else 0.0

        occupied_pads = sum(1 for pad in self.charging_pads if pad.get("occupied_by") is not None)
        pad_utilization = (occupied_pads / max(len(self.charging_pads), 1)) * 100.0

        late_delays = [
            float(request.get("delay_minutes", 0.0) or 0.0)
            for request in self.requests
            if float(request.get("delay_minutes", 0.0) or 0.0) > 0
        ]
        mean_delay = (sum(late_delays) / len(late_delays)) if late_delays else 0.0

        battery_values = [float(drone.get("battery", 0.0) or 0.0) for drone in self.drones]
        if battery_values:
            fleet_mean = sum(battery_values) / len(battery_values)
            fleet_variance = sum((battery - fleet_mean) ** 2 for battery in battery_values) / len(battery_values)
        else:
            fleet_variance = 0.0

        return {
            "on_time_delivery_rate": round(on_time_rate, 2),
            "total_energy_consumption_kwh": round(sum(float(drone.get("battery", 0.0) or 0.0) * 0.008 for drone in self.drones), 2),
            "pad_utilization_rate": round(pad_utilization, 2),
            "mean_delay_per_late_package": round(mean_delay, 2),
            "fleet_variance_in_battery_degradation": round(fleet_variance, 2),
        }

    def get_status(self) -> Dict[str, Any]:
        self._load_drones()
        return {
            "drones": self.drones,
            "charging_pads": self.charging_pads,
            "queued_requests": [r for r in self.requests if str(r.get("status", "")).lower() in {"queued", "assigned"}],
            "metrics": self.compute_metrics(),
            "alerts": self._build_alerts(),
            "recent_assignments": self.audit_log[-10:],
        }
