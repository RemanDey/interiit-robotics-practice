import unittest

from api.fleet_manager import FleetManager


class FleetManagerTests(unittest.TestCase):
    def setUp(self):
        self.manager = FleetManager()
        self.manager.drones = [
            {
                "id": 101,
                "battery": 92,
                "position": {"lat": 31.782, "lng": 76.998, "alt": 0.0},
                "base": {"lat": 31.7813, "lng": 76.9975, "address": "Base"},
                "destination": {"lat": 31.79, "lng": 77.01, "address": "A"},
                "state": "IDLE",
                "speed": 0.00025,
                "payload": 0.0,
            },
            {
                "id": 102,
                "battery": 35,
                "position": {"lat": 31.782, "lng": 76.998, "alt": 0.0},
                "base": {"lat": 31.7813, "lng": 76.9975, "address": "Base"},
                "destination": {"lat": 31.79, "lng": 77.01, "address": "A"},
                "state": "IDLE",
                "speed": 0.00025,
                "payload": 0.0,
            },
            {
                "id": 103,
                "battery": 78,
                "position": {"lat": 31.782, "lng": 76.998, "alt": 0.0},
                "base": {"lat": 31.7813, "lng": 76.9975, "address": "Base"},
                "destination": {"lat": 31.79, "lng": 77.01, "address": "A"},
                "state": "CHARGING",
                "speed": 0.00025,
                "payload": 0.0,
            },
        ]
        self.manager.charging_pads = [
            {"id": 1, "occupied_by": None, "time_remaining": 0, "queue": []},
            {"id": 2, "occupied_by": None, "time_remaining": 0, "queue": []},
            {"id": 3, "occupied_by": None, "time_remaining": 0, "queue": []},
        ]

    def test_assigns_eligible_drone_and_records_decision(self):
        request = {
            "package_id": "PKG-1",
            "weight": 0.8,
            "destination": {"lat": 31.7905, "lng": 77.0098, "address": "Drop zone"},
            "deadline_minutes": 20,
        }

        decision = self.manager.assign_request(request)
        self.assertIn("selected_drone", decision)
        self.assertEqual(decision["selected_drone"], 101)
        self.assertIn("audit", decision)
        self.assertTrue(any(item["eligible"] for item in decision["audit"]))

    def test_rejects_overweight_and_impossible_deadlines(self):
        overweight = {
            "package_id": "PKG-2",
            "weight": 3.0,
            "destination": {"lat": 31.7905, "lng": 77.0098, "address": "Nowhere"},
            "deadline_minutes": 10,
        }
        self.assertFalse(self.manager.assign_request(overweight)["accepted"])

        impossible = {
            "package_id": "PKG-3",
            "weight": 0.5,
            "destination": {"lat": 31.7905, "lng": 77.0098, "address": "Nowhere"},
            "deadline_minutes": 1,
        }
        self.assertFalse(self.manager.assign_request(impossible)["accepted"])

    def test_metrics_are_computed_for_requests_and_pads(self):
        self.manager.requests = [
            {
                "package_id": "PKG-1",
                "weight": 0.8,
                "destination": {"lat": 31.7905, "lng": 77.0098, "address": "Drop zone"},
                "deadline_minutes": 20,
                "status": "queued",
            },
            {
                "package_id": "PKG-2",
                "weight": 0.5,
                "destination": {"lat": 31.7850, "lng": 77.0020, "address": "Other"},
                "deadline_minutes": 8,
                "status": "completed",
            },
        ]
        self.manager.charging_pads[0]["occupied_by"] = 103
        metrics = self.manager.compute_metrics()
        self.assertIn("on_time_delivery_rate", metrics)
        self.assertIn("pad_utilization_rate", metrics)
        self.assertIn("fleet_variance_in_battery_degradation", metrics)


if __name__ == "__main__":
    unittest.main()
