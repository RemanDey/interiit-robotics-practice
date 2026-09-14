"""3 pads, 10 drones: reserve a free pad or join the queue."""

from typing import Dict, List, Optional, Tuple

from models import CHARGE_TIME_S, CRITICAL_SOC, Drone, DroneStatus, Pad


class PadManager:
    def __init__(self, pads: Dict[str, Pad]):
        self.pads = pads
        self.queue: List[str] = []
        self.total_occupied_s = 0.0
        self.last_update_s = 0.0

    def _tick_clock(self, now_s: float) -> None:
        self.total_occupied_s += max(0.0, now_s - self.last_update_s) * sum(1 for p in self.pads.values() if p.occupied_by)
        self.last_update_s = now_s

    def _charge_time(self, soc_pct: float) -> float:
        return CHARGE_TIME_S * max(0.0, 100.0 - soc_pct) / 100.0

    def reserve_pad(self, drone: Drone, now_s: float, eta_to_base_s: float) -> Tuple[Optional[str], float, str]:
        self._tick_clock(now_s)
        arrival = now_s + eta_to_base_s
        for pad in sorted(self.pads.values(), key=lambda p: p.pad_id):
            if not pad.occupied_by and not pad.reserved_by:
                pad.reserved_by = drone.drone_id
                pad.available_at = arrival + self._charge_time(drone.soc_pct)
                drone.pad_reservation = pad.pad_id
                return pad.pad_id, 0.0, f"{pad.pad_id} reserved; no pad wait expected"
        # all busy: critical batteries steal a reservation, everyone else queues
        if drone.soc_pct < CRITICAL_SOC:
            for pad in self.pads.values():
                if pad.reserved_by and not pad.occupied_by:
                    pad.reserved_by = drone.drone_id
                    return pad.pad_id, 0.0, f"critical SoC preempted reservation on {pad.pad_id}"
            if drone.drone_id not in self.queue:
                self.queue.insert(0, drone.drone_id)
        elif drone.drone_id not in self.queue:
            self.queue.append(drone.drone_id)
        wait = max(0.0, min(p.available_at for p in self.pads.values()) - arrival)
        return None, wait, f"all pads busy; queued with estimated wait {wait:.1f}s"

    def release_reservation(self, drone: Drone) -> None:
        pad = self.pads.get(drone.pad_reservation or "")
        if pad and pad.reserved_by == drone.drone_id:
            pad.reserved_by = None
        drone.pad_reservation = None

    def dock_drone(self, drone: Drone, now_s: float) -> Optional[str]:
        self._tick_clock(now_s)
        pad = self.pads.get(drone.pad_reservation or "")
        if not pad or pad.occupied_by:
            free = [p for p in self.pads.values() if not p.occupied_by]
            if not free:
                drone.status = DroneStatus.RECOVERING if drone.soc_pct < CRITICAL_SOC else DroneStatus.HOLDING
                if drone.drone_id not in self.queue:
                    self.queue.insert(0 if drone.soc_pct < CRITICAL_SOC else len(self.queue), drone.drone_id)
                return None
            pad = sorted(free, key=lambda p: p.pad_id)[0]
        pad.occupied_by = drone.drone_id
        pad.reserved_by = None
        pad.available_at = now_s + self._charge_time(drone.soc_pct)
        drone.pad_reservation = pad.pad_id
        drone.status = DroneStatus.CHARGING
        if drone.drone_id in self.queue:
            self.queue.remove(drone.drone_id)
        return pad.pad_id

    def tick_charging(self, fleet: Dict[str, Drone], now_s: float, dt_s: float) -> None:
        self._tick_clock(now_s)
        rate = 100.0 / CHARGE_TIME_S
        for pad in self.pads.values():
            if not pad.occupied_by:
                continue
            drone = fleet[pad.occupied_by]
            drone.soc_pct = min(100.0, drone.soc_pct + rate * dt_s)
            if drone.soc_pct >= 99.95:
                drone.soc_pct, drone.status, drone.pad_reservation = 100.0, DroneStatus.IDLE, None
                pad.occupied_by, pad.available_at = None, now_s
        for drone_id in list(self.queue):
            if any(not p.occupied_by for p in self.pads.values()):
                self.dock_drone(fleet[drone_id], now_s)

    def utilization_pct(self, horizon_s: float) -> float:
        if horizon_s <= 0 or not self.pads:
            return 0.0
        return 100.0 * self.total_occupied_s / (horizon_s * len(self.pads))
