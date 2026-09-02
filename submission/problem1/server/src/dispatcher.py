"""Decision engine for deadline-aware drone dispatch."""

from __future__ import annotations

from math import inf
from statistics import variance
from typing import Dict, Iterable, List, Optional, Tuple

from models import (
    BASE_POSITION,
    CRITICAL_SOC,
    LOW_SOC,
    MAX_PAYLOAD_KG,
    RESERVE_SOC,
    DecisionAudit,
    DecisionCandidate,
    DeliveryRequest,
    Drone,
    DroneStatus,
    FleetMetrics,
    RequestStatus,
    cruise_speed_mps,
    distance_m,
    energy_for_leg_wh,
)
from pad_manager import PadManager


class Dispatcher:
    def __init__(self, fleet: Dict[str, Drone], pad_manager: PadManager):
        self.fleet = fleet
        self.pad_manager = pad_manager
        self.requests: Dict[str, DeliveryRequest] = {}
        self.audit_log: List[DecisionAudit] = []
        self.request_log: List[Dict[str, object]] = []

    def submit_request(self, request: DeliveryRequest, now_s: float) -> DecisionAudit:
        self.requests[request.request_id] = request
        if request.payload_kg > MAX_PAYLOAD_KG:
            request.status = RequestStatus.REJECTED
            request.reason = f"payload {request.payload_kg:.2f}kg exceeds {MAX_PAYLOAD_KG:.2f}kg limit"
            audit = DecisionAudit(request.request_id, None, now_s, [], request.reason)
            self.audit_log.append(audit)
            return audit

        fastest_time = self._fastest_possible_delivery_s(request)
        if now_s + fastest_time > request.deadline_at:
            request.status = RequestStatus.REJECTED
            request.reason = f"impossible deadline; fastest delivery is {fastest_time:.1f}s"
            audit = DecisionAudit(request.request_id, None, now_s, [], request.reason)
            self.audit_log.append(audit)
            return audit

        audit = self.evaluate_request(request, now_s)
        self.audit_log.append(audit)
        if audit.selected_drone:
            self.assign(audit.selected_drone, request, now_s, audit)
        else:
            request.status = RequestStatus.QUEUED
            request.reason = audit.decision_reason
        return audit

    def evaluate_request(self, request: DeliveryRequest, now_s: float) -> DecisionAudit:
        candidates = [self._score_drone(drone, request, now_s) for drone in self.fleet.values()]
        eligible = [candidate for candidate in candidates if candidate.eligible]
        if not eligible:
            reason = "no eligible drone can satisfy payload, deadline, battery reserve, and pad return constraints"
            return DecisionAudit(request.request_id, None, now_s, candidates, reason)
        selected = min(eligible, key=lambda candidate: candidate.cost)
        reason = (
            f"selected {selected.drone_id}: cost={selected.cost:.2f}, "
            f"ETA={selected.eta_s:.1f}s, margin={selected.deadline_margin_s:.1f}s, "
            f"energy={selected.energy_wh:.1f}Wh"
        )
        return DecisionAudit(request.request_id, selected.drone_id, now_s, candidates, reason)

    def _score_drone(self, drone: Drone, request: DeliveryRequest, now_s: float) -> DecisionCandidate:
        if drone.status not in {DroneStatus.IDLE, DroneStatus.RETURNING, DroneStatus.HOLDING}:
            return self._ineligible(drone.drone_id, "drone unavailable due to status " + drone.status.value)
        if drone.soc_pct < CRITICAL_SOC:
            return self._ineligible(drone.drone_id, f"SoC {drone.soc_pct:.1f}% is below critical threshold")
        if request.payload_kg > MAX_PAYLOAD_KG:
            return self._ineligible(drone.drone_id, "payload above physical limit")

        launch_point = drone.position if drone.status != DroneStatus.IDLE else BASE_POSITION
        to_destination_m = distance_m(launch_point, request.destination)
        return_m = distance_m(request.destination, BASE_POSITION)
        loaded_energy = energy_for_leg_wh(to_destination_m, request.payload_kg)
        return_energy = energy_for_leg_wh(return_m, 0.0)
        total_energy = loaded_energy + return_energy
        reserve_wh = drone.usable_capacity_wh * RESERVE_SOC / 100.0
        if total_energy + reserve_wh > drone.available_energy_wh:
            reason = (
                f"insufficient degraded battery: need {total_energy + reserve_wh:.1f}Wh "
                f"with reserve, have {drone.available_energy_wh:.1f}Wh"
            )
            return self._ineligible(drone.drone_id, reason, energy_wh=total_energy)

        eta_delivery_s = to_destination_m / cruise_speed_mps(request.payload_kg, drone.speed_factor)
        return_s = return_m / cruise_speed_mps(0.0, drone.speed_factor)
        _, pad_wait_s, pad_reason = self.pad_manager.reserve_pad(drone, now_s, eta_delivery_s + return_s)
        self.pad_manager.release_reservation(drone)
        if drone.drone_id in self.pad_manager.queue:
            self.pad_manager.queue.remove(drone.drone_id)
        arrival_s = now_s + eta_delivery_s
        margin_s = request.deadline_at - arrival_s
        if margin_s < 0.0:
            return self._ineligible(drone.drone_id, f"deadline miss by {-margin_s:.1f}s", eta_s=eta_delivery_s, energy_wh=total_energy)

        mean_cycles = sum(item.cycle_count for item in self.fleet.values()) / max(1, len(self.fleet))
        workload_penalty = max(0.0, drone.cycle_count - mean_cycles) * 25.0 + drone.assigned_flights * 0.6
        urgency = max(1.0, request.deadline_at - now_s)
        deadline_cost = eta_delivery_s / urgency * 80.0
        energy_cost = total_energy / 12.0
        pad_cost = pad_wait_s / 30.0
        low_soc_cost = 10.0 if drone.soc_pct < LOW_SOC else 0.0
        cost = deadline_cost + energy_cost + workload_penalty + pad_cost + low_soc_cost
        return DecisionCandidate(
            drone.drone_id,
            True,
            eta_delivery_s,
            total_energy,
            margin_s,
            workload_penalty,
            pad_wait_s,
            cost,
            pad_reason,
        )

    def assign(self, drone_id: str, request: DeliveryRequest, now_s: float, audit: DecisionAudit) -> None:
        drone = self.fleet[drone_id]
        selected = next(candidate for candidate in audit.candidates if candidate.drone_id == drone_id)
        drone.status = DroneStatus.IN_FLIGHT
        drone.active_request = request.request_id
        drone.payload_kg = request.payload_kg
        drone.target = request.destination
        drone.route = [request.destination, BASE_POSITION]
        drone.assigned_flights += 1
        request.assigned_drone = drone_id
        request.eta_s = now_s + selected.eta_s
        request.status = RequestStatus.ASSIGNED
        request.reason = audit.decision_reason
        self.request_log.append(
            {
                "timestamp_s": now_s,
                "package_id": request.request_id,
                "weight_kg": request.payload_kg,
                "destination": request.destination,
                "assigned_drone": drone_id,
                "expected_eta_s": request.eta_s,
                "status": request.status.value,
            }
        )

    def complete_request(self, drone: Drone, now_s: float) -> None:
        if not drone.active_request:
            return
        request = self.requests[drone.active_request]
        request.actual_arrival_s = now_s
        request.status = RequestStatus.COMPLETED if now_s <= request.deadline_at else RequestStatus.LATE
        request.reason = "delivered on time" if request.status == RequestStatus.COMPLETED else "deadline failure"
        drone.active_request = None
        drone.payload_kg = 0.0
        drone.status = DroneStatus.RETURNING
        drone.target = BASE_POSITION

    def requeue_unassigned(self, now_s: float) -> List[DecisionAudit]:
        audits = []
        queued = sorted(
            (request for request in self.requests.values() if request.status == RequestStatus.QUEUED),
            key=lambda item: item.deadline_at,
        )
        for request in queued:
            audits.append(self.submit_request(request, now_s))
        return audits

    def metrics(self, horizon_s: float) -> FleetMetrics:
        completed = [r for r in self.requests.values() if r.status in {RequestStatus.COMPLETED, RequestStatus.LATE}]
        on_time = [r for r in completed if r.status == RequestStatus.COMPLETED]
        late_delays = [max(0.0, (r.actual_arrival_s or 0.0) - r.deadline_at) for r in completed if r.status == RequestStatus.LATE]
        cycles = [drone.cycle_count for drone in self.fleet.values()]
        return FleetMetrics(
            on_time_rate=100.0 * len(on_time) / len(completed) if completed else 0.0,
            total_energy_wh=sum(drone.total_energy_wh for drone in self.fleet.values()),
            pad_utilization_pct=self.pad_manager.utilization_pct(horizon_s),
            mean_late_delay_s=sum(late_delays) / len(late_delays) if late_delays else 0.0,
            cycle_variance=variance(cycles) if len(cycles) > 1 else 0.0,
        )

    def _fastest_possible_delivery_s(self, request: DeliveryRequest) -> float:
        return distance_m(BASE_POSITION, request.destination) / cruise_speed_mps(request.payload_kg, 1.2)

    @staticmethod
    def _ineligible(drone_id: str, reason: str, eta_s: float = inf, energy_wh: float = inf) -> DecisionCandidate:
        return DecisionCandidate(drone_id, False, eta_s, energy_wh, -inf, inf, inf, inf, reason)
