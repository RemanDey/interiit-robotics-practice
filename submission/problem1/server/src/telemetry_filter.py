"""Reject bad telemetry: out-of-range SoC, teleporting positions, SoC jumps."""

from dataclasses import dataclass
from typing import Dict, Tuple

from models import Drone, DroneStatus, distance_m

MAX_POS_M = 25000.0


@dataclass
class TelemetrySample:
    drone_id: str
    timestamp_s: float
    position: Tuple[float, float, float]
    soc_pct: float
    speed_factor: float = 1.0


@dataclass
class TelemetryResult:
    accepted: bool
    reason: str


class TelemetryFilter:
    def __init__(self, max_speed_mps=28.0, max_soc_jump_pct=18.0, timeout_s=20.0):
        self.max_speed_mps = max_speed_mps
        self.max_soc_jump_pct = max_soc_jump_pct
        self.timeout_s = timeout_s
        self.last_good: Dict[str, TelemetrySample] = {}

    def ingest(self, drone: Drone, sample: TelemetrySample) -> TelemetryResult:
        if not 0.0 <= sample.soc_pct <= 100.0:
            return TelemetryResult(False, "rejected telemetry: SoC outside 0..100")
        x, y, z = sample.position
        if max(abs(x), abs(y)) > MAX_POS_M or z < -1.0:
            return TelemetryResult(False, "rejected telemetry: impossible position")
        prev = self.last_good.get(sample.drone_id)
        if prev:
            dt = max(1e-6, sample.timestamp_s - prev.timestamp_s)
            if distance_m(prev.position, sample.position) / dt > self.max_speed_mps:
                return TelemetryResult(False, "rejected telemetry: implied speed too high")
            if abs(sample.soc_pct - prev.soc_pct) > self.max_soc_jump_pct and dt < 60.0:
                return TelemetryResult(False, "rejected telemetry: abrupt SoC jump")
        drone.position, drone.soc_pct = sample.position, sample.soc_pct
        drone.speed_factor = max(0.4, min(1.2, sample.speed_factor))
        drone.last_heartbeat_s = sample.timestamp_s
        self.last_good[sample.drone_id] = sample
        return TelemetryResult(True, "accepted")

    def mark_link_losses(self, fleet: Dict[str, Drone], now_s: float) -> None:
        for drone in fleet.values():
            if (drone.status in {DroneStatus.IN_FLIGHT, DroneStatus.RETURNING, DroneStatus.HOLDING}
                    and now_s - drone.last_heartbeat_s > self.timeout_s):
                drone.status = DroneStatus.LOST
