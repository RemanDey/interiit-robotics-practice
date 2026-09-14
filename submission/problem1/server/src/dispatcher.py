"""Pick the cheapest feasible drone for each request."""

from math import inf
from typing import Dict, List

from models import (
    BASE_POSITION, CRITICAL_SOC, LOW_SOC, MAX_PAYLOAD_KG, RESERVE_SOC,
    DecisionAudit, DecisionCandidate, DeliveryRequest, Drone,
    DroneStatus, FleetMetrics, RequestStatus,
    cruise_speed_mps, distance_m, energy_for_leg_wh,
)
from pad_manager import PadManager

FREE = {DroneStatus.IDLE, DroneStatus.RETURNING, DroneStatus.HOLDING}


class Dispatcher:
    def __init__(self, fleet: Dict[str, Drone], pad_manager: PadManager):
        self.fleet = fleet
        self.pad_manager = pad_manager
        self.requests: Dict[str, DeliveryRequest] = {}
        self.audit_log: List[DecisionAudit] = []
        self.request_log: List[dict] = []

    # --- entry point ---
    def submit_request(self, request: DeliveryRequest, now_s: float) -> DecisionAudit:
        self.requests[request.request_id] = request
        if request.payload_kg > MAX_PAYLOAD_KG:
            return self._reject(request, now_s, f"payload {request.payload_kg:.2f}kg exceeds {MAX_PAYLOAD_KG:.2f}kg limit")
        fastest = distance_m(BASE_POSITION, request.destination) / cruise_speed_mps(request.payload_kg, 1.2)
        if now_s + fastest > request.deadline_at:
            return self._reject(request, now_s, f"impossible deadline; fastest delivery is {fastest:.1f}s")

        audit = self.evaluate_request(request, now_s)
        self.audit_log.append(audit)
        if audit.selected_drone:
            self.assign(audit.selected_drone, request, now_s, audit)
        else:
            request.status = RequestStatus.QUEUED
            request.reason = audit.decision_reason
        return audit

    def _reject(self, request, now_s, reason) -> DecisionAudit:
        request.status = RequestStatus.REJECTED
        request.reason = reason
        audit = DecisionAudit(request.request_id, None, now_s, [], reason)
        self.audit_log.append(audit)
        return audit

    def evaluate_request(self, request: DeliveryRequest, now_s: float) -> DecisionAudit:
        cands = [self._score_drone(d, request, now_s) for d in self.fleet.values()]
        ok = [c for c in cands if c.eligible]
        if not ok:
            return DecisionAudit(request.request_id, None, now_s, cands,
                                 "no eligible drone can satisfy payload, deadline, battery reserve, and pad return constraints")
        best = min(ok, key=lambda c: c.cost)
        reason = (f"selected {best.drone_id}: cost={best.cost:.2f}, "
                  f"ETA={best.eta_s:.1f}s, margin={best.deadline_margin_s:.1f}s, energy={best.energy_wh:.1f}Wh")
        return DecisionAudit(request.request_id, best.drone_id, now_s, cands, reason)

    # --- feasibility + cost ---
    def _score_drone(self, drone: Drone, request: DeliveryRequest, now_s: float) -> DecisionCandidate:
        if drone.status not in FREE:
            return self._ineligible(drone.drone_id, "drone unavailable due to status " + drone.status.value)
        if drone.soc_pct < CRITICAL_SOC:
            return self._ineligible(drone.drone_id, f"SoC {drone.soc_pct:.1f}% is below critical threshold")

        start = drone.position if drone.status != DroneStatus.IDLE else BASE_POSITION
        out_m = distance_m(start, request.destination)
        back_m = distance_m(request.destination, BASE_POSITION)
        need_wh = energy_for_leg_wh(out_m, request.payload_kg) + energy_for_leg_wh(back_m, 0.0)
        reserve_wh = drone.usable_capacity_wh * RESERVE_SOC / 100.0
        if need_wh + reserve_wh > drone.available_energy_wh:
            return self._ineligible(drone.drone_id,
                f"insufficient degraded battery: need {need_wh + reserve_wh:.1f}Wh with reserve, have {drone.available_energy_wh:.1f}Wh",
                energy_wh=need_wh)

        eta = out_m / cruise_speed_mps(request.payload_kg, drone.speed_factor)
        back = back_m / cruise_speed_mps(0.0, drone.speed_factor)
        _, pad_wait, pad_reason = self.pad_manager.reserve_pad(drone, now_s, eta + back)
        self.pad_manager.release_reservation(drone)  # probe only; real booking happens on dock
        self.pad_manager.queue = [q for q in self.pad_manager.queue if q != drone.drone_id]

        margin = request.deadline_at - (now_s + eta)
        if margin < 0:
            return self._ineligible(drone.drone_id, f"deadline miss by {-margin:.1f}s", eta_s=eta, energy_wh=need_wh)

        mean_cycles = sum(d.cycle_count for d in self.fleet.values()) / max(1, len(self.fleet))
        workload = max(0.0, drone.cycle_count - mean_cycles) * 25.0 + drone.assigned_flights * 0.6
        urgency = max(1.0, request.deadline_at - now_s)
        cost = (eta / urgency * 80.0 + need_wh / 12.0 + workload
                + pad_wait / 30.0 + (10.0 if drone.soc_pct < LOW_SOC else 0.0))
        return DecisionCandidate(drone.drone_id, True, eta, need_wh, margin, workload, pad_wait, cost, pad_reason)

    # --- state changes ---
    def assign(self, drone_id: str, request: DeliveryRequest, now_s: float, audit: DecisionAudit) -> None:
        drone = self.fleet[drone_id]
        eta = next(c for c in audit.candidates if c.drone_id == drone_id).eta_s
        drone.status = DroneStatus.IN_FLIGHT
        drone.active_request = request.request_id
        drone.payload_kg = request.payload_kg
        drone.target = request.destination
        drone.route = [request.destination, BASE_POSITION]
        drone.assigned_flights += 1
        request.assigned_drone = drone_id
        request.eta_s = now_s + eta
        request.status = RequestStatus.ASSIGNED
        request.reason = audit.decision_reason
        self.request_log.append({"timestamp_s": now_s, "package_id": request.request_id,
            "weight_kg": request.payload_kg, "destination": request.destination,
            "assigned_drone": drone_id, "expected_eta_s": request.eta_s, "status": request.status.value})

    def complete_request(self, drone: Drone, now_s: float) -> None:
        if not drone.active_request:
            return
        req = self.requests[drone.active_request]
        req.actual_arrival_s = now_s
        req.status = RequestStatus.COMPLETED if now_s <= req.deadline_at else RequestStatus.LATE
        req.reason = "delivered on time" if req.status == RequestStatus.COMPLETED else "deadline failure"
        drone.active_request = None
        drone.payload_kg = 0.0
        drone.status = DroneStatus.RETURNING
        drone.target = BASE_POSITION

    def requeue_unassigned(self, now_s: float) -> List[DecisionAudit]:
        queued = sorted((r for r in self.requests.values() if r.status == RequestStatus.QUEUED),
                        key=lambda r: r.deadline_at)
        return [self.submit_request(r, now_s) for r in queued]

    def metrics(self, horizon_s: float) -> FleetMetrics:
        done = [r for r in self.requests.values() if r.status in {RequestStatus.COMPLETED, RequestStatus.LATE}]
        on_time = [r for r in done if r.status == RequestStatus.COMPLETED]
        delays = [(r.actual_arrival_s or 0.0) - r.deadline_at for r in done if r.status == RequestStatus.LATE]
        cycles = [d.cycle_count for d in self.fleet.values()]
        mean = sum(cycles) / len(cycles) if cycles else 0.0
        var = sum((c - mean) ** 2 for c in cycles) / max(1, len(cycles) - 1) if len(cycles) > 1 else 0.0
        return FleetMetrics(
            on_time_rate=100.0 * len(on_time) / len(done) if done else 0.0,
            total_energy_wh=sum(d.total_energy_wh for d in self.fleet.values()),
            pad_utilization_pct=self.pad_manager.utilization_pct(horizon_s),
            mean_late_delay_s=sum(delays) / len(delays) if delays else 0.0,
            cycle_variance=var,
        )

    @staticmethod
    def _ineligible(drone_id: str, reason: str, eta_s: float = inf, energy_wh: float = inf) -> DecisionCandidate:
        return DecisionCandidate(drone_id, False, eta_s, energy_wh, -inf, inf, inf, inf, reason)
