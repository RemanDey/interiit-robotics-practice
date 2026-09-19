from __future__ import annotations

import json
import math
import os
from pathlib import Path
from typing import Any, Dict, List

BASE_DIR = Path(__file__).resolve().parent.parent
BACKEND_PATH = BASE_DIR / "backend" / "telemetry.json"
# On-demand dispatch inbox polled by the sim (sim/commands.h) each tick.
COMMANDS_PATH = BASE_DIR / "backend" / "commands.json"

# Observed round-trip (cruise + delivery wait + return + descent) is ~5-8
# min; used to estimate queue-wait when scoring deadline slack.
AVG_MISSION_MINUTES = 10.0

# Hybrid dispatch cost weights. Terms live on different scales (battery
# 0-100, load 0-N jobs, distance ~0-2 km, slack minutes) — magnitudes below
# balance them deliberately. Legacy preset reproducing the pre-hybrid
# behavior exactly: {"battery": 1.0, "load": 0.0, "distance": 0.0, "slack": 0.0}
# (plus the parked bonus, which is kept as-is).
ALLOT_WEIGHTS = {
    "battery": 1.0,   # per SoC point
    "load": 10.0,     # per queued+active job (one waiting job ~= 10 batt points)
    "distance": 5.0,  # per km to destination
    "slack": 1.0,     # per minute of deadline slack
}


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

    def _read_commands(self) -> List[Dict[str, Any]]:
        """Pending sim inbox entries. [] when the file is missing/corrupt."""
        try:
            data = json.loads(COMMANDS_PATH.read_text(encoding="utf-8"))
            if isinstance(data, list):
                return [c for c in data if isinstance(c, dict)]
        except (json.JSONDecodeError, OSError):
            pass
        return []

    def _write_command(
        self, drone_id: int, destination: Dict[str, Any], package_id: str
    ) -> None:
        """Address a takeoff command to one drone (upsert by drone_id).

        Written atomically (tmp file + os.replace) so the sim, which polls
        this file every tick, never reads a torn payload.
        """
        commands = self._read_commands()
        entry = {
            "drone_id": int(drone_id),
            "lat": float(destination.get("lat", 0.0) or 0.0),
            "lng": float(destination.get("lng", 0.0) or 0.0),
            "address": str(destination.get("address", "") or ""),
            "package_id": str(package_id),
        }
        commands = [c for c in commands if int(c.get("drone_id", -1)) != int(drone_id)]
        commands.append(entry)
        tmp = COMMANDS_PATH.with_suffix(".tmp")
        tmp.write_text(json.dumps(commands), encoding="utf-8")
        os.replace(tmp, COMMANDS_PATH)

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

    def _drone_load(self, drone_id: int) -> int:
        """Jobs already on a drone's plate (active + queued)."""
        count = 0
        for request in self.requests:
            if str(request.get("status", "")).lower() not in {"assigned", "queued"}:
                continue
            if request.get("assigned_drone") == drone_id or request.get("allotted_drone") == drone_id:
                count += 1
        return count

    def _score_candidate(
        self,
        drone: Dict[str, Any],
        destination: Dict[str, Any],
        deadline_minutes: float,
    ) -> tuple[float, Dict[str, Any]]:
        """Hybrid dispatch cost. Returns (hybrid_score, breakdown).

        hybrid = W_batt*battery + parked_bonus
                 + W_load*(-load) - W_dist*distance_km + W_slack*slack_min
        With ALLOT_WEIGHTS = legacy preset this reduces term-for-term to the
        pre-hybrid score (battery + parked bonus).
        """
        drone_id = int(drone.get("id"))
        state = str(drone.get("state", "IDLE")).upper()
        battery = float(drone.get("battery", 0) or 0)
        load = self._drone_load(drone_id)
        dist = self._distance_km(drone.get("position", {}), destination)
        wait = load * AVG_MISSION_MINUTES
        slack = max(0.0, deadline_minutes - wait - dist * 0.75)
        parked = 25 if state in ("IDLE", "LANDED") else 15
        w = ALLOT_WEIGHTS
        hybrid = (
            w["battery"] * battery
            + parked
            + w["load"] * (-load)
            - w["distance"] * dist
            + w["slack"] * slack
        )
        breakdown = {
            "load": load,
            "battery": battery,
            "distance_km": round(dist, 3),
            "slack_min": round(slack, 2),
            "parked_bonus": parked,
            "hybrid": round(hybrid, 2),
        }
        return hybrid, breakdown

    def _queued_for(self, drone_id: int) -> List[Dict[str, Any]]:
        """Queued (allotted, not yet flying) records for one drone, in FIFO order."""
        return [
            r for r in self.requests
            if str(r.get("status", "")).lower() == "queued"
            and r.get("allotted_drone") == drone_id
        ]

    def _inbox_ids(self) -> set:
        """Drone ids with a takeoff command awaiting sim pickup."""
        ids = set()
        for cmd in self._read_commands():
            try:
                ids.add(int(cmd.get("drone_id", -1)))
            except (TypeError, ValueError):
                continue
        return ids

    def _sync_pads(self) -> None:
        """Mirror pad occupancy from telemetry.

        Drones reporting CHARGING occupy pads in id order; any beyond the
        pad count wait in the shortest queue. time_remaining is estimated
        from the sim's charge model (CHARGE_PER_TICK = 4 per 2 s tick in
        sim/sim.cpp). Idempotent — rebuilt from scratch every poll.
        """
        charging = [
            d for d in self.drones
            if str(d.get("state", "")).upper() == "CHARGING"
        ]
        charging.sort(key=lambda d: int(d.get("id", 0)))
        for pad in self.charging_pads:
            pad["occupied_by"] = None
            pad["time_remaining"] = 0
            pad["queue"] = []
        for i, drone in enumerate(charging):
            batt = float(drone.get("battery", 0) or 0)
            ticks_left = max(0, math.ceil((100.0 - batt) / 4.0))
            mins = round(ticks_left * 2 / 60.0, 1)
            if i < len(self.charging_pads):
                self.charging_pads[i]["occupied_by"] = int(drone.get("id"))
                self.charging_pads[i]["time_remaining"] = mins
            else:
                self.charging_pads[i % len(self.charging_pads)]["queue"].append(
                    int(drone.get("id"))
                )

    def _drone_is_free(self, drone: Dict[str, Any], inbox_ids: set) -> bool:
        """Parked at base, healthy battery, nothing awaiting liftoff."""
        state = str(drone.get("state", "IDLE")).upper()
        battery = float(drone.get("battery", 0) or 0)
        try:
            did = int(drone.get("id"))
        except (TypeError, ValueError):
            return False
        return state in ("IDLE", "LANDED") and battery > 15 and did not in inbox_ids

    def _settle_and_promote(self) -> None:
        """Close out finished missions and launch the next queued delivery.

        A drone is done when telemetry shows it parked with no inbox entry
        (its last command was consumed on liftoff and it has since landed):
        its 'assigned' records flip to 'completed'. Its oldest 'queued'
        record is then promoted to 'assigned' with a fresh inbox command.
        Idempotent — safe to run on every assign/status poll.
        """
        inbox_ids = self._inbox_ids()
        for drone in self.drones:
            try:
                did = int(drone.get("id"))
            except (TypeError, ValueError):
                continue
            if not self._drone_is_free(drone, inbox_ids):
                continue
            for record in self.requests:
                if (str(record.get("status", "")).lower() == "assigned"
                        and record.get("assigned_drone") == did):
                    record["status"] = "completed"
            pending = self._queued_for(did)
            if pending:
                nxt = pending[0]
                dest = nxt.get("destination") or {"lat": 31.79, "lng": 77.01, "address": "Default"}
                self._write_command(did, dest, str(nxt.get("package_id", f"PKG-{did}")))
                nxt["status"] = "assigned"
                nxt["assigned_drone"] = did
                inbox_ids.add(did)

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

        # Settle finished missions first: LANDED drones with no pending inbox
        # entry get their 'assigned' records completed and their oldest
        # 'queued' record promoted to a fresh takeoff command.
        self._settle_and_promote()
        inbox_ids = self._inbox_ids()

        audit: List[Dict[str, Any]] = []
        eligible: List[Dict[str, Any]] = []
        for drone in self.drones:
            state = str(drone.get("state", "IDLE")).upper()
            battery = float(drone.get("battery", 0) or 0)
            try:
                did = int(drone.get("id"))
            except (TypeError, ValueError):
                audit.append({"drone_id": drone.get("id"), "eligible": False, "score": 0.0, "reason": "Unknown drone id"})
                continue
            # On-demand fleet: LANDED/IDLE at base means "parked, awaiting a
            # request" and IS eligible. Airborne states mean the drone is
            # already flying YOUR assigned task, so it cannot take another.
            if state in {"CHARGING", "RETURNING", "OFF", "START", "TAKEOFF"}:
                audit.append({"drone_id": drone.get("id"), "eligible": False, "score": 0.0, "reason": "Drone unavailable"})
                continue
            if state in {"CRUISE", "DELIVERY", "APPROACH"}:
                audit.append({"drone_id": drone.get("id"), "eligible": False, "score": 0.0, "reason": "Drone busy on assigned task"})
                continue
            if battery <= 15:
                audit.append({"drone_id": drone.get("id"), "eligible": False, "score": 0.0, "reason": "Battery below safe reserve"})
                continue
            if weight > 2.5:
                audit.append({"drone_id": drone.get("id"), "eligible": False, "score": 0.0, "reason": "Payload exceeds payload limit"})
                continue
            if did in inbox_ids:
                # A takeoff command is already awaiting sim pickup for this
                # drone — overwriting it would lose that delivery. Queue behind.
                audit.append({"drone_id": drone.get("id"), "eligible": False, "score": 0.0, "reason": "Takeoff command already awaiting pickup"})
                continue
            if self._queued_for(did):
                # Fairness: a drone with backlog doesn't jump its own queue.
                audit.append({"drone_id": drone.get("id"), "eligible": False, "score": 0.0, "reason": "Drone has queued backlog"})
                continue

            hybrid, breakdown = self._score_candidate(drone, destination, deadline_minutes)
            item = {
                "drone_id": drone.get("id"),
                "eligible": True,
                "score": hybrid,
                "battery": battery,
                "state": state,
                "distance_km": self._distance_km(drone.get("position", {}), destination),
                "cost": breakdown,
            }
            eligible.append(item)
            audit.append(item)

        if not eligible:
            # Nothing can fly right now: allot the delivery to a drone's
            # backlog (hybrid cost over all battery-healthy drones, busy or
            # charging included — they drain it on recovery) instead of
            # leaving it ownerless. It auto-promotes on landing, oldest first.
            allotment: List[Dict[str, Any]] = []
            for drone in self.drones:
                battery = float(drone.get("battery", 0) or 0)
                if battery <= 15:
                    continue
                hybrid, breakdown = self._score_candidate(drone, destination, deadline_minutes)
                allotment.append({
                    "drone_id": drone.get("id"),
                    "score": hybrid,
                    "cost": breakdown,
                })
            if not allotment:
                return {
                    "accepted": False,
                    "selected_drone": None,
                    "allotted_drone": None,
                    "reason": "Fleet has no battery-healthy drone; request held",
                    "audit": audit,
                    "cost_weights": dict(ALLOT_WEIGHTS),
                }
            winner = max(allotment, key=lambda item: item["score"])
            allotted = int(winner["drone_id"])
            position = len(self._queued_for(allotted)) + 1
            queued_request = dict(request)
            queued_request["package_id"] = package_id
            queued_request["weight"] = weight
            queued_request["destination"] = destination
            queued_request["status"] = "queued"
            queued_request["deadline_minutes"] = deadline_minutes
            queued_request["allotted_drone"] = allotted
            queued_request["queue_position"] = position
            self.requests.append(queued_request)
            self.request_log.append({
                "package_id": package_id,
                "weight": weight,
                "assigned_drone": None,
                "allotted_drone": allotted,
                "status": "queued",
                "deadline_minutes": deadline_minutes,
            })
            return {
                "accepted": False,
                "selected_drone": None,
                "allotted_drone": allotted,
                "queue_position": position,
                "reason": f"All drones busy; allotted to Drone {allotted} (position {position})",
                "audit": audit,
                "allotment": allotment,
                "cost_weights": dict(ALLOT_WEIGHTS),
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
                # Cosmetic local update (reloaded from disk on the next call);
                # the durable channel is the sim inbox written below.
                drone["state"] = "TAKEOFF"
                drone["destination"] = destination
                break

        # Durable command for the sim: polled each tick, consumed on liftoff.
        self._write_command(selected_drone, destination, package_id)

        return {
            "accepted": True,
            "selected_drone": selected_drone,
            "reason": "Assigned to best-eligible drone; takeoff command queued for pickup",
            "audit": audit,
            "cost_weights": dict(ALLOT_WEIGHTS),
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
        # Drive the backlog: each poll settles finished missions and launches
        # the next queued delivery per free drone. Side effect inside a
        # status read is deliberate — this is the auto-promotion engine.
        self._settle_and_promote()
        # Mirror reality: pads reflect drones actually in CHARGING state.
        self._sync_pads()
        return {
            "drones": self.drones,
            "charging_pads": self.charging_pads,
            "queued_requests": [r for r in self.requests if str(r.get("status", "")).lower() in {"queued", "assigned"}],
            "pending_commands": self._read_commands(),
            "metrics": self.compute_metrics(),
            "alerts": self._build_alerts(),
            "recent_assignments": self.audit_log[-10:],
        }
