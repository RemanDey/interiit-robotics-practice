import json
import tempfile
import unittest
from pathlib import Path

import api.fleet_manager as fleet_module
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


def make_drone(drone_id, state, battery=90):
    return {
        "id": drone_id,
        "battery": battery,
        "position": {"lat": 31.7813, "lng": 76.9975, "alt": 0.0},
        "base": {"lat": 31.7813, "lng": 76.9975, "address": "Base"},
        "destination": {"lat": 31.7813, "lng": 76.9975, "address": "Base"},
        "state": state,
        "speed": 0.00025,
        "payload": 0.0,
    }


class OnDemandDispatchTests(unittest.TestCase):
    """On-demand fleet: LANDED at base is eligible; airborne is busy.

    Fixtures are pinned by stubbing _load_drones (otherwise assign_request
    reloads the live telemetry.json over them — the known isolation quirk).
    The command inbox is redirected to a tmp file.
    """

    def setUp(self):
        self.manager = FleetManager()
        # Pin fixtures: don't let disk reloads clobber them.
        self.manager._load_drones = lambda: self.manager.drones
        self._tmpdir = tempfile.TemporaryDirectory()
        self._orig_commands = fleet_module.COMMANDS_PATH
        fleet_module.COMMANDS_PATH = Path(self._tmpdir.name) / "commands.json"
        self._orig_weights = dict(fleet_module.ALLOT_WEIGHTS)

    def tearDown(self):
        fleet_module.COMMANDS_PATH = self._orig_commands
        fleet_module.ALLOT_WEIGHTS.clear()
        fleet_module.ALLOT_WEIGHTS.update(self._orig_weights)
        self._tmpdir.cleanup()

    def _request(self, package_id="PKG-1", weight=0.8):
        return {
            "package_id": package_id,
            "weight": weight,
            "destination": {"lat": 31.7905, "lng": 77.0098, "address": "Drop zone"},
            "deadline_minutes": 20,
        }

    def _read_inbox(self):
        path = Path(self._tmpdir.name) / "commands.json"
        if not path.exists():
            return []
        return json.loads(path.read_text(encoding="utf-8"))

    def test_landed_drone_is_eligible_and_command_persisted(self):
        self.manager.drones = [make_drone(1, "LANDED"), make_drone(2, "CRUISE")]
        decision = self.manager.assign_request(self._request())
        self.assertTrue(decision["accepted"])
        self.assertEqual(decision["selected_drone"], 1)

        inbox = self._read_inbox()
        self.assertEqual(len(inbox), 1)
        self.assertEqual(inbox[0]["drone_id"], 1)
        self.assertAlmostEqual(inbox[0]["lat"], 31.7905)
        self.assertAlmostEqual(inbox[0]["lng"], 77.0098)
        self.assertEqual(inbox[0]["package_id"], "PKG-1")

    def test_busy_airborne_drone_is_not_eligible(self):
        self.manager.drones = [make_drone(1, "CRUISE")]
        decision = self.manager.assign_request(self._request())
        self.assertFalse(decision["accepted"])
        self.assertIsNone(decision["selected_drone"])
        self.assertEqual(self._read_inbox(), [])

    def test_command_upsert_replaces_previous_entry(self):
        # A tasked drone reports TAKEOFF, so a second request correctly
        # queues instead of double-booking it. Upsert itself is exercised
        # directly: re-addressing the same drone replaces, not duplicates.
        self.manager.drones = [make_drone(1, "LANDED")]
        self.manager.assign_request(self._request(package_id="PKG-1"))
        self.manager._write_command(
            1,
            {"lat": 31.79, "lng": 77.0, "address": "Second drop"},
            "PKG-2",
        )
        inbox = self._read_inbox()
        self.assertEqual(len(inbox), 1)
        self.assertEqual(inbox[0]["package_id"], "PKG-2")

    def test_status_exposes_pending_commands(self):
        self.manager.drones = [make_drone(1, "LANDED")]
        self.manager.assign_request(self._request())
        status = self.manager.get_status()
        self.assertIn("pending_commands", status)
        self.assertEqual(len(status["pending_commands"]), 1)
        self.assertEqual(status["pending_commands"][0]["drone_id"], 1)

    def test_assign_skips_drone_with_pending_inbox_entry(self):
        # Drone 1 is LANDED but its takeoff command hasn't been picked up:
        # overwriting it would lose that delivery, so the new request must
        # go to drone 2 and the old command must survive.
        self.manager.drones = [make_drone(1, "LANDED"), make_drone(2, "LANDED")]
        self.manager._write_command(
            1, {"lat": 31.79, "lng": 77.0, "address": "First drop"}, "PKG-OLD")
        decision = self.manager.assign_request(self._request(package_id="PKG-NEW"))
        self.assertTrue(decision["accepted"])
        self.assertEqual(decision["selected_drone"], 2)
        inbox = {c["package_id"]: c["drone_id"] for c in self._read_inbox()}
        self.assertEqual(inbox, {"PKG-OLD": 1, "PKG-NEW": 2})

    def test_promote_oldest_on_landing(self):
        # Drone 1 lands with one active + two queued: the active flips to
        # completed, the oldest queued launches, the other stays queued.
        self.manager.drones = [make_drone(1, "LANDED")]
        self.manager.requests = [
            {"package_id": "A", "assigned_drone": 1, "status": "assigned",
             "destination": {"lat": 31.79, "lng": 77.0, "address": "A-drop"}},
            {"package_id": "Q1", "status": "queued", "allotted_drone": 1,
             "queue_position": 1,
             "destination": {"lat": 31.791, "lng": 77.001, "address": "Q1-drop"}},
            {"package_id": "Q2", "status": "queued", "allotted_drone": 1,
             "queue_position": 2,
             "destination": {"lat": 31.792, "lng": 77.002, "address": "Q2-drop"}},
        ]
        status = self.manager.get_status()
        by_id = {r["package_id"]: r["status"] for r in self.manager.requests}
        self.assertEqual(by_id, {"A": "completed", "Q1": "assigned", "Q2": "queued"})
        self.assertEqual(status["pending_commands"][0]["package_id"], "Q1")

    def test_legacy_preset_reproduces_old_behavior(self):
        # Zero load/distance/slack weights => hybrid == battery + parked
        # bonus, i.e. the pre-hybrid rule: fullest battery wins.
        fleet_module.ALLOT_WEIGHTS.update(
            {"battery": 1.0, "load": 0.0, "distance": 0.0, "slack": 0.0}
        )
        d1 = make_drone(1, "LANDED", battery=80)
        d2 = make_drone(2, "LANDED", battery=95)
        self.manager.drones = [d1, d2]
        decision = self.manager.assign_request(self._request())
        self.assertEqual(decision["selected_drone"], 2)
        costs = {a["drone_id"]: a["cost"] for a in decision["audit"] if a["eligible"]}
        self.assertAlmostEqual(costs[2]["hybrid"], 95 + 25)
        self.assertAlmostEqual(costs[1]["hybrid"], 80 + 25)

    def test_least_loaded_wins_on_equal_battery(self):
        # Both airborne => nothing flies now => allotment path. Load term
        # steers the job to the emptier drone's backlog.
        d1 = make_drone(1, "CRUISE", battery=90)
        d2 = make_drone(2, "CRUISE", battery=90)
        self.manager.drones = [d1, d2]
        self.manager.requests = [
            {"package_id": "OLD-1", "assigned_drone": 1, "status": "assigned"},
            {"package_id": "OLD-2", "assigned_drone": 1, "status": "queued",
             "allotted_drone": 1},
        ]
        decision = self.manager.assign_request(self._request())
        self.assertFalse(decision["accepted"])
        self.assertEqual(decision["allotted_drone"], 2)
        self.assertEqual(decision["queue_position"], 1)
        self.assertEqual(self._read_inbox(), [])
        costs = {a["drone_id"]: a["cost"] for a in decision["allotment"]}
        self.assertEqual(costs[1]["load"], 2)
        self.assertEqual(costs[2]["load"], 0)

    def test_nearer_drone_wins_on_equal_battery_and_load(self):
        near = make_drone(1, "LANDED", battery=90)
        near["position"] = {"lat": 31.7900, "lng": 77.0090, "alt": 30.0}
        far = make_drone(2, "LANDED", battery=90)
        far["position"] = {"lat": 31.7813, "lng": 76.9975, "alt": 0.0}
        self.manager.drones = [near, far]
        decision = self.manager.assign_request(self._request())
        self.assertEqual(decision["selected_drone"], 1)
        costs = {a["drone_id"]: a["cost"] for a in decision["audit"] if a["eligible"]}
        self.assertLess(costs[1]["distance_km"], costs[2]["distance_km"])

    def test_tight_deadline_prefers_free_drone_over_full_busy_one(self):
        # Both airborne => allotment path. Drone 1: fuller battery but a
        # queued job ahead (~10 min wait) => ~1 min slack. Drone 2: weaker
        # battery, no wait => ~11 min slack => slack points win.
        busy = make_drone(1, "CRUISE", battery=95)
        free = make_drone(2, "CRUISE", battery=80)
        self.manager.drones = [busy, free]
        self.manager.requests = [
            {"package_id": "OLD-1", "assigned_drone": 1, "status": "assigned"},
            {"package_id": "OLD-2", "assigned_drone": 1, "status": "queued",
             "allotted_drone": 1},
        ]
        request = self._request()
        request["deadline_minutes"] = 12
        decision = self.manager.assign_request(request)
        self.assertFalse(decision["accepted"])
        self.assertEqual(decision["allotted_drone"], 2)

    def test_charging_drone_occupies_pad(self):
        self.manager.drones = [
            make_drone(1, "CHARGING", battery=80),
            make_drone(2, "LANDED", battery=100),
        ]
        status = self.manager.get_status()
        pad1 = status["charging_pads"][0]
        self.assertEqual(pad1["occupied_by"], 1)
        # (100-80)/4 = 5 ticks x 2 s = ~0.2 min remaining.
        self.assertAlmostEqual(pad1["time_remaining"], 0.2)
        self.assertIsNone(status["charging_pads"][1]["occupied_by"])
        self.assertGreater(status["metrics"]["pad_utilization_rate"], 0)

    def test_no_charging_drones_leaves_pads_free(self):
        self.manager.drones = [make_drone(1, "LANDED", battery=100)]
        status = self.manager.get_status()
        self.assertTrue(
            all(p["occupied_by"] is None for p in status["charging_pads"])
        )

    def test_pad_overflow_queues(self):
        self.manager.drones = [make_drone(i, "CHARGING", battery=50) for i in (1, 2, 3, 4)]
        status = self.manager.get_status()
        occupied = [p["occupied_by"] for p in status["charging_pads"]]
        self.assertEqual(occupied, [1, 2, 3])
        self.assertEqual(status["charging_pads"][0]["queue"], [4])


if __name__ == "__main__":
    unittest.main()
