"""Shared models and constants for the drone fleet simulator."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from math import hypot
from typing import Dict, List, Optional, Tuple


BASE_POSITION = (0.0, 0.0, 0.0)
FLEET_SIZE = 10
PAD_COUNT = 3
MAX_PAYLOAD_KG = 2.5
BASE_SPEED_MPS = 12.0
PAYLOAD_SPEED_ALPHA = 0.35
FULL_PAYLOAD_ENDURANCE_S = 25 * 60
NOMINAL_BATTERY_WH = 520.0
CHARGE_TIME_S = 40 * 60
DEGRADATION_PER_FULL_CYCLE = 0.0005
RESERVE_SOC = 12.0
CRITICAL_SOC = 15.0
LOW_SOC = 30.0


class DroneStatus(str, Enum):
    IDLE = "Idle"
    IN_FLIGHT = "In-Flight"
    CHARGING = "Charging"
    HOLDING = "Holding"
    RETURNING = "Returning"
    RECOVERING = "Recovering"
    LOST = "Lost"


class RequestStatus(str, Enum):
    QUEUED = "Queued"
    ASSIGNED = "Assigned"
    COMPLETED = "Completed"
    LATE = "Late"
    REJECTED = "Rejected"
    FAILED = "Failed"


@dataclass
class DeliveryRequest:
    request_id: str
    destination: Tuple[float, float, float]
    payload_kg: float
    created_at: float
    deadline_at: float
    status: RequestStatus = RequestStatus.QUEUED
    assigned_drone: Optional[str] = None
    reason: str = ""
    eta_s: Optional[float] = None
    actual_arrival_s: Optional[float] = None


@dataclass
class Drone:
    drone_id: str
    position: Tuple[float, float, float] = BASE_POSITION
    soc_pct: float = 100.0
    cycle_count: float = 0.0
    status: DroneStatus = DroneStatus.IDLE
    payload_kg: float = 0.0
    active_request: Optional[str] = None
    pad_reservation: Optional[str] = None
    total_distance_m: float = 0.0
    total_energy_wh: float = 0.0
    assigned_flights: int = 0
    last_heartbeat_s: float = 0.0
    speed_factor: float = 1.0
    target: Tuple[float, float, float] = BASE_POSITION
    route: List[Tuple[float, float, float]] = field(default_factory=list)

    @property
    def soh_pct(self) -> float:
        return max(50.0, 100.0 * (1.0 - self.cycle_count * DEGRADATION_PER_FULL_CYCLE))

    @property
    def usable_capacity_wh(self) -> float:
        return NOMINAL_BATTERY_WH * self.soh_pct / 100.0

    @property
    def available_energy_wh(self) -> float:
        return self.usable_capacity_wh * self.soc_pct / 100.0


@dataclass
class Pad:
    pad_id: str
    occupied_by: Optional[str] = None
    reserved_by: Optional[str] = None
    available_at: float = 0.0


@dataclass
class DecisionCandidate:
    drone_id: str
    eligible: bool
    eta_s: float
    energy_wh: float
    deadline_margin_s: float
    workload_penalty: float
    pad_wait_s: float
    cost: float
    reason: str


@dataclass
class DecisionAudit:
    request_id: str
    selected_drone: Optional[str]
    timestamp_s: float
    candidates: List[DecisionCandidate]
    decision_reason: str


@dataclass
class FleetMetrics:
    on_time_rate: float
    total_energy_wh: float
    pad_utilization_pct: float
    mean_late_delay_s: float
    cycle_variance: float


def distance_m(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    return hypot(hypot(a[0] - b[0], a[1] - b[1]), a[2] - b[2])


def cruise_speed_mps(payload_kg: float, speed_factor: float = 1.0) -> float:
    payload_ratio = min(max(payload_kg / MAX_PAYLOAD_KG, 0.0), 1.0)
    return max(3.0, BASE_SPEED_MPS * (1.0 - PAYLOAD_SPEED_ALPHA * payload_ratio) * speed_factor)


def energy_for_leg_wh(distance: float, payload_kg: float) -> float:
    full_payload_power_wh_per_s = NOMINAL_BATTERY_WH / FULL_PAYLOAD_ENDURANCE_S
    payload_factor = 0.52 + 0.48 * min(max(payload_kg / MAX_PAYLOAD_KG, 0.0), 1.0)
    speed = cruise_speed_mps(payload_kg)
    return (distance / speed) * full_payload_power_wh_per_s * payload_factor


def build_default_fleet() -> Dict[str, Drone]:
    return {f"DR-{index:02d}": Drone(drone_id=f"DR-{index:02d}") for index in range(1, FLEET_SIZE + 1)}


def build_default_pads() -> Dict[str, Pad]:
    return {f"PAD-{index}": Pad(pad_id=f"PAD-{index}") for index in range(1, PAD_COUNT + 1)}
