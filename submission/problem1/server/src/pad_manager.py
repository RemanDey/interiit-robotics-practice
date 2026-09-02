"""Charging pad reservation and contention management."""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

from models import CHARGE_TIME_S, CRITICAL_SOC, LOW_SOC, Drone, DroneStatus, Pad


class PadManager:
    """Owns pad occupancy, future reservations, and emergency preemption."""

    def __init__(self, pads: Dict[str, Pad]):
        self.pads = pads
        self.queue: List[str] = []
        self.total_occupied_s = 0.0
        self.last_update_s = 0.0

    def update_occupancy_clock(self, now_s: float) -> None:
        elapsed = max(0.0, now_s - self.last_update_s)
        occupied = sum(1 for pad in self.pads.values() if pad.occupied_by)
        self.total_occupied_s += elapsed * occupied
        self.last_update_s = now_s

    def reserve_pad(self, drone: Drone, now_s: float, eta_to_base_s: float) -> Tuple[Optional[str], float, str]:
        """Reserve earliest pad and return pad id, wait time, and explanation."""
        self.update_occupancy_clock(now_s)
        arrival_s = now_s + eta_to_base_s
        free_pads = [pad for pad in self.pads.values() if not pad.occupied_by and not pad.reserved_by]
        if free_pads:
            pad = sorted(free_pads, key=lambda p: p.pad_id)[0]
            pad.reserved_by = drone.drone_id
            pad.available_at = arrival_s + CHARGE_TIME_S * max(0.0, 100.0 - drone.soc_pct) / 100.0
            drone.pad_reservation = pad.pad_id
            return pad.pad_id, 0.0, f"{pad.pad_id} reserved; no pad wait expected"

        ranked = sorted(self.pads.values(), key=lambda p: p.available_at)
        soonest = ranked[0]
        wait_s = max(0.0, soonest.available_at - arrival_s)
        if drone.soc_pct < CRITICAL_SOC:
            evicted = self._preempt_low_priority_reservation(drone.drone_id)
            if evicted:
                return evicted, 0.0, f"critical SoC preempted reservation on {evicted}"

        if drone.drone_id not in self.queue:
            if drone.soc_pct < CRITICAL_SOC:
                self.queue.insert(0, drone.drone_id)
            else:
                self.queue.append(drone.drone_id)
        return None, wait_s, f"all pads busy; queued with estimated wait {wait_s:.1f}s"

    def _preempt_low_priority_reservation(self, incoming_drone_id: str) -> Optional[str]:
        for pad in self.pads.values():
            if pad.reserved_by and pad.reserved_by != incoming_drone_id and not pad.occupied_by:
                pad.reserved_by = incoming_drone_id
                return pad.pad_id
        return None

    def release_reservation(self, drone: Drone) -> None:
        if not drone.pad_reservation:
            return
        pad = self.pads.get(drone.pad_reservation)
        if pad and pad.reserved_by == drone.drone_id:
            pad.reserved_by = None
        drone.pad_reservation = None

    def dock_drone(self, drone: Drone, now_s: float) -> Optional[str]:
        self.update_occupancy_clock(now_s)
        pad = None
        if drone.pad_reservation:
            candidate = self.pads.get(drone.pad_reservation)
            if candidate and not candidate.occupied_by:
                pad = candidate
        if not pad:
            free = [item for item in self.pads.values() if not item.occupied_by]
            if not free:
                drone.status = DroneStatus.HOLDING if drone.soc_pct >= CRITICAL_SOC else DroneStatus.RECOVERING
                if drone.drone_id not in self.queue:
                    self.queue.insert(0 if drone.soc_pct < CRITICAL_SOC else len(self.queue), drone.drone_id)
                return None
            pad = sorted(free, key=lambda p: p.pad_id)[0]

        pad.occupied_by = drone.drone_id
        pad.reserved_by = None
        pad.available_at = now_s + CHARGE_TIME_S * max(0.0, 100.0 - drone.soc_pct) / 100.0
        drone.pad_reservation = pad.pad_id
        drone.status = DroneStatus.CHARGING
        if drone.drone_id in self.queue:
            self.queue.remove(drone.drone_id)
        return pad.pad_id

    def tick_charging(self, fleet: Dict[str, Drone], now_s: float, dt_s: float) -> None:
        self.update_occupancy_clock(now_s)
        charge_rate_pct_s = 100.0 / CHARGE_TIME_S
        for pad in self.pads.values():
            if not pad.occupied_by:
                continue
            drone = fleet[pad.occupied_by]
            drone.soc_pct = min(100.0, drone.soc_pct + charge_rate_pct_s * dt_s)
            if drone.soc_pct >= 99.95:
                drone.soc_pct = 100.0
                drone.status = DroneStatus.IDLE
                drone.pad_reservation = None
                pad.occupied_by = None
                pad.available_at = now_s

        for drone_id in list(self.queue):
            drone = fleet[drone_id]
            if any(not pad.occupied_by for pad in self.pads.values()):
                self.dock_drone(drone, now_s)

    def utilization_pct(self, horizon_s: float) -> float:
        if horizon_s <= 0.0 or not self.pads:
            return 0.0
        return 100.0 * self.total_occupied_s / (horizon_s * len(self.pads))
