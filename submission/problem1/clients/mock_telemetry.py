"""Generate telemetry anomaly scenarios for manual testing."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "server" / "src"))

from models import build_default_fleet
from telemetry_filter import TelemetryFilter, TelemetrySample


def main() -> None:
    fleet = build_default_fleet()
    drone = fleet["DR-01"]
    telemetry_filter = TelemetryFilter()
    samples = [
        TelemetrySample("DR-01", 0.0, (0.0, 0.0, 0.0), 50.0),
        TelemetrySample("DR-01", 2.0, (20.0, 0.0, 0.0), 49.0),
        TelemetrySample("DR-01", 4.0, (10000.0, 0.0, 0.0), 48.0),
        TelemetrySample("DR-01", 6.0, (30.0, 0.0, 0.0), 5.0),
        TelemetrySample("DR-01", 8.0, (40.0, 0.0, 0.0), 47.0),
    ]
    for sample in samples:
        result = telemetry_filter.ingest(drone, sample)
        print(f"{sample.timestamp_s:>4.0f}s accepted={result.accepted:<5} {result.reason}")


if __name__ == "__main__":
    main()
