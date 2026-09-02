import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(ROOT))

from dispatcher import Dispatcher
from models import DeliveryRequest, DroneStatus, RequestStatus, build_default_fleet, build_default_pads
from pad_manager import PadManager


def make_dispatcher():
    fleet = build_default_fleet()
    pads = build_default_pads()
    return Dispatcher(fleet, PadManager(pads)), fleet


class DispatcherTests(unittest.TestCase):
    def test_rejects_overweight_request(self):
        dispatcher, _ = make_dispatcher()
        audit = dispatcher.submit_request(DeliveryRequest("OW", (100, 0, 20), 3.0, 0, 1000), 0)
        self.assertIsNone(audit.selected_drone)
        self.assertEqual(dispatcher.requests["OW"].status, RequestStatus.REJECTED)

    def test_rejects_impossible_deadline(self):
        dispatcher, _ = make_dispatcher()
        audit = dispatcher.submit_request(DeliveryRequest("FAST", (5000, 0, 20), 1.0, 0, 30), 0)
        self.assertIsNone(audit.selected_drone)
        self.assertIn("impossible deadline", dispatcher.requests["FAST"].reason)

    def test_assigns_with_audit_breakdown(self):
        dispatcher, _ = make_dispatcher()
        audit = dispatcher.submit_request(DeliveryRequest("OK", (300, 0, 30), 1.0, 0, 900), 0)
        self.assertIsNotNone(audit.selected_drone)
        self.assertEqual(len(audit.candidates), 10)
        self.assertTrue(any(candidate.eligible for candidate in audit.candidates))
        self.assertEqual(dispatcher.requests["OK"].status, RequestStatus.ASSIGNED)

    def test_respects_degraded_usable_capacity(self):
        dispatcher, fleet = make_dispatcher()
        fleet["DR-01"].cycle_count = 450.0
        fleet["DR-01"].soc_pct = 18.0
        audit = dispatcher.submit_request(DeliveryRequest("ENERGY", (1400, 0, 50), 2.0, 0, 1800), 0)
        candidate = next(item for item in audit.candidates if item.drone_id == "DR-01")
        self.assertFalse(candidate.eligible)
        self.assertTrue("degraded battery" in candidate.reason or "critical" in candidate.reason)

    def test_ignores_unavailable_drones(self):
        dispatcher, fleet = make_dispatcher()
        for drone in fleet.values():
            drone.status = DroneStatus.IN_FLIGHT
        audit = dispatcher.submit_request(DeliveryRequest("SAT", (200, 0, 20), 0.5, 0, 900), 0)
        self.assertIsNone(audit.selected_drone)
        self.assertEqual(dispatcher.requests["SAT"].status, RequestStatus.QUEUED)


if __name__ == "__main__":
    unittest.main()
