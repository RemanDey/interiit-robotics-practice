"""Telemetry validation, outlier rejection, and link-loss tracking."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple

from models import Drone, DroneStatus, distance_m


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
    def __init__(self, max_speed_mps: float = 28.0, max_soc_jump_pct: float = 18.0, timeout_s: float = 20.0):
        self.max_speed_mps = max_speed_mps
        self.max_soc_jump_pct = max_soc_jump_pct
        self.timeout_s = timeout_s
        self.last_good: Dict[str, TelemetrySample] = {}

    def ingest(self, drone: Drone, sample: TelemetrySample) -> TelemetryResult:
        if not (0.0 <= sample.soc_pct <= 100.0):
            return TelemetryResult(False, "rejected telemetry: SoC outside 0..100")
        if any(abs(axis) > 25000.0 for axis in sample.position) or sample.position[2] < -1.0:
            return TelemetryResult(False, "rejected telemetry: impossible position")

        previous = self.last_good.get(sample.drone_id)
        if previous:
            dt_s = max(1e-6, sample.timestamp_s - previous.timestamp_s)
            speed = distance_m(previous.position, sample.position) / dt_s
            if speed > self.max_speed_mps:
                return TelemetryResult(False, f"rejected telemetry: implied speed {speed:.1f} m/s")
            if abs(sample.soc_pct - previous.soc_pct) > self.max_soc_jump_pct and dt_s < 60.0:
                return TelemetryResult(False, "rejected telemetry: abrupt SoC jump")

        drone.position = sample.position
        drone.soc_pct = sample.soc_pct
        drone.speed_factor = max(0.4, min(1.2, sample.speed_factor))
        drone.last_heartbeat_s = sample.timestamp_s
        self.last_good[sample.drone_id] = sample
        return TelemetryResult(True, "accepted")

    def mark_link_losses(self, fleet: Dict[str, Drone], now_s: float) -> None:
        for drone in fleet.values():
            if drone.status != DroneStatus.LOST and now_s - drone.last_heartbeat_s > self.timeout_s:
                if drone.status in {DroneStatus.IN_FLIGHT, DroneStatus.RETURNING, DroneStatus.HOLDING}:
                    drone.status = DroneStatus.LOST
