"""Print which telemetry samples the filter accepts/rejects."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "server" / "src"))

from models import build_default_fleet
from telemetry_filter import TelemetryFilter, TelemetrySample

SAMPLES = [
    (0.0, (0.0, 0.0, 0.0), 50.0),      # ok
    (2.0, (20.0, 0.0, 0.0), 49.0),     # ok
    (4.0, (10000.0, 0.0, 0.0), 48.0),  # too fast -> reject
    (6.0, (30.0, 0.0, 0.0), 5.0),      # SoC jump -> reject
    (8.0, (40.0, 0.0, 0.0), 47.0),     # ok
]

if __name__ == "__main__":
    drone = build_default_fleet()["DR-01"]
    f = TelemetryFilter()
    for t, pos, soc in SAMPLES:
        r = f.ingest(drone, TelemetrySample("DR-01", t, pos, soc))
        print(f"{t:>4.0f}s accepted={str(r.accepted):<5} {r.reason}")
