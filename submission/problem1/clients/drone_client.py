"""Simulated drone airframe client with local estimator and return-to-home failsafe."""

from __future__ import annotations

import json
import sys
import time
import urllib.request
from dataclasses import dataclass
from enum import Enum
from typing import Tuple


class LocalState(str, Enum):
    IDLE = "Idle"
    EXECUTING = "Executing"
    HOLDING = "Holding"
    RETURN_HOME = "ReturnHome"


@dataclass
class LocalEstimator:
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    soc_pct: float = 100.0
    stale_ticks: int = 0

    def update(self, position, soc_pct: float) -> None:
        self.position = tuple(position)
        self.soc_pct = soc_pct
        self.stale_ticks = 0


class DroneClient:
    def __init__(self, drone_id: str, server_url: str = "http://127.0.0.1:8080") -> None:
        self.drone_id = drone_id
        self.server_url = server_url.rstrip("/")
        self.estimator = LocalEstimator()
        self.state = LocalState.IDLE

    def fetch_state(self) -> None:
        with urllib.request.urlopen(f"{self.server_url}/state", timeout=4) as response:
            payload = json.loads(response.read())
        drone = next(item for item in payload["drones"] if item["id"] == self.drone_id)
        self.estimator.update(drone["position"], drone["soc_pct"])
        if drone["status"] in {"In-Flight", "Returning"}:
            self.state = LocalState.EXECUTING
        elif drone["status"] == "Lost":
            self.state = LocalState.RETURN_HOME
        else:
            self.state = LocalState.IDLE

    def tick_server(self, dt_s: float = 5.0) -> None:
        body = json.dumps({"dt_s": dt_s}).encode()
        request = urllib.request.Request(
            f"{self.server_url}/tick",
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(request, timeout=4):
            pass

    def run(self) -> None:
        while True:
            try:
                self.fetch_state()
                self.tick_server(5.0)
                print(f"{self.drone_id}: {self.state.value} pos={self.estimator.position} soc={self.estimator.soc_pct:.1f}%")
            except Exception as exc:
                self.estimator.stale_ticks += 1
                self.state = LocalState.HOLDING if self.estimator.stale_ticks < 3 else LocalState.RETURN_HOME
                print(f"{self.drone_id}: link degraded, local state={self.state.value}: {exc}")
            time.sleep(1.0)


if __name__ == "__main__":
    drone_id = sys.argv[1] if len(sys.argv) > 1 else "DR-01"
    DroneClient(drone_id).run()
