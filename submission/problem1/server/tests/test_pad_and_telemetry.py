import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(ROOT))

from models import DroneStatus, build_default_fleet, build_default_pads
from pad_manager import PadManager
from telemetry_filter import TelemetryFilter, TelemetrySample


class PadAndTelemetryTests(unittest.TestCase):
    def test_critical_drone_gets_front_of_pad_queue(self):
        fleet = build_default_fleet()
        pads = build_default_pads()
        manager = PadManager(pads)
        for index, pad in enumerate(pads.values(), start=1):
            pad.occupied_by = f"DR-0{index}"
            pad.available_at = 1000.0
        fleet["DR-04"].soc_pct = 10.0
        manager.reserve_pad(fleet["DR-04"], 0.0, 60.0)
        self.assertEqual(manager.queue[0], "DR-04")

    def test_telemetry_rejects_bad_soc_jump_and_bad_position(self):
        fleet = build_default_fleet()
        drone = fleet["DR-01"]
        telemetry = TelemetryFilter()
        self.assertTrue(telemetry.ingest(drone, TelemetrySample("DR-01", 0.0, (0, 0, 0), 50.0)).accepted)
        self.assertFalse(telemetry.ingest(drone, TelemetrySample("DR-01", 2.0, (10, 0, 0), 5.0)).accepted)
        self.assertFalse(telemetry.ingest(drone, TelemetrySample("DR-01", 4.0, (99999, 0, 0), 49.0)).accepted)

    def test_link_loss_marks_airborne_drone_lost(self):
        fleet = build_default_fleet()
        fleet["DR-01"].status = DroneStatus.IN_FLIGHT
        telemetry = TelemetryFilter(timeout_s=5.0)
        telemetry.mark_link_losses(fleet, 10.0)
        self.assertEqual(fleet["DR-01"].status, DroneStatus.LOST)


if __name__ == "__main__":
    unittest.main()
