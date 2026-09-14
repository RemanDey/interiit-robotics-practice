"""Poll the server and advance the sim. Fails safe to hold/return-home on link loss."""

import json
import sys
import time
import urllib.request

IDLE, EXECUTING, HOLDING, RETURN_HOME = "Idle", "Executing", "Holding", "ReturnHome"


class DroneClient:
    def __init__(self, drone_id, server_url="http://127.0.0.1:8080"):
        self.drone_id = drone_id
        self.server_url = server_url.rstrip("/")
        self.pos, self.soc, self.state, self.stale = (0, 0, 0), 100.0, IDLE, 0

    def fetch_state(self):
        with urllib.request.urlopen(f"{self.server_url}/state", timeout=4) as r:
            drone = next(d for d in json.loads(r.read())["drones"] if d["id"] == self.drone_id)
        self.pos, self.soc, self.stale = tuple(drone["position"]), drone["soc_pct"], 0
        self.state = EXECUTING if drone["status"] in {"In-Flight", "Returning"} else RETURN_HOME if drone["status"] == "Lost" else IDLE

    def tick_server(self, dt_s=5.0):
        req = urllib.request.Request(f"{self.server_url}/tick", data=json.dumps({"dt_s": dt_s}).encode(),
                                     headers={"Content-Type": "application/json"}, method="POST")
        with urllib.request.urlopen(req, timeout=4):
            pass

    def run(self):
        while True:
            try:
                self.fetch_state()
                self.tick_server(5.0)
                print(f"{self.drone_id}: {self.state} pos={self.pos} soc={self.soc:.1f}%")
            except Exception as exc:
                self.stale += 1
                self.state = HOLDING if self.stale < 3 else RETURN_HOME
                print(f"{self.drone_id}: link degraded, local state={self.state}: {exc}")
            time.sleep(1.0)


if __name__ == "__main__":
    DroneClient(sys.argv[1] if len(sys.argv) > 1 else "DR-01").run()
