"""Live 2D visualizer for the drone fleet (matplotlib, server-linked).

Run:  pip install matplotlib && python3 clients/fleet_2d.py
Click the map to dispatch a delivery to that (x, y).
"""

import sys
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "server" / "src"))

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, Slider

from server import FleetSimulation, seed

TRAIL = 30
LIM = 3000


def color(d):
    if d["status"] == "Lost":
        return "gray"
    if d["soc_pct"] < 20:
        return "red"
    if d["status"] == "In-Flight":
        return "blue"
    return "green"


class FleetViz:
    def __init__(self, sim=None):
        self.sim = sim or FleetSimulation()
        if not self.sim.dispatcher.requests:
            seed(self.sim)
        self.playing = True
        self.counter = 0
        self.trails = {d: deque(maxlen=TRAIL) for d in self.sim.fleet}
        self.sel = "DR-01"

        self.fig = plt.figure(figsize=(12, 7))
        self.fig.canvas.manager.set_window_title("Drone Fleet 2D Simulator")
        self.ax = self.fig.add_axes([0.05, 0.18, 0.58, 0.77])
        self.info = self.fig.add_axes([0.66, 0.38, 0.32, 0.57])
        self.info.axis("off")

        self._widgets()
        self.fig.canvas.mpl_connect("button_press_event", self._click)
        self.fig.canvas.mpl_connect("key_press_event", self._key)

    # --- widgets ---
    def _widgets(self):
        def btn(pos, label, fn):
            b = Button(self.fig.add_axes(pos), label)
            b.on_clicked(fn)
            return b

        btn([0.05, 0.06, 0.09, 0.05], "Play/Pause", lambda e: setattr(self, "playing", not self.playing))
        btn([0.15, 0.06, 0.09, 0.05], "+30s", lambda e: self.sim.tick(30.0))
        btn([0.25, 0.06, 0.09, 0.05], "Reset", lambda e: self._reset())
        btn([0.66, 0.30, 0.10, 0.05], "Link loss", lambda e: self._fault("link"))
        btn([0.77, 0.30, 0.10, 0.05], "Drain 20%", lambda e: self._fault("drain"))
        btn([0.88, 0.30, 0.09, 0.05], "Wind .8", lambda e: self._fault("wind"))

        def slider(pos, label, lo, hi, val):
            s = Slider(self.fig.add_axes(pos), label, lo, hi, valinit=val)
            return s

        self.s_dt = slider([0.40, 0.06, 0.20, 0.04], "dt s", 1, 120, 10)
        self.s_payload = slider([0.66, 0.20, 0.31, 0.03], "payload kg", 0.2, 2.5, 1.0)
        self.s_deadline = slider([0.66, 0.15, 0.31, 0.03], "deadline min", 2, 45, 15)
        self.s_wind = slider([0.66, 0.10, 0.31, 0.03], "wind factor", 0.4, 1.2, 1.0)
        self.s_drone = slider([0.66, 0.05, 0.31, 0.03], "drone 1-10", 1, 10, 1, )
        self.s_drone.on_changed(lambda v: setattr(self, "sel", f"DR-{int(v):02d}"))
        self.s_wind.on_changed(lambda v: self._apply_wind())

    def _reset(self):
        self.__init__()  # fresh sim + clear trails

    def _apply_wind(self):
        for d in self.sim.fleet.values():
            d.speed_factor = round(float(self.s_wind.val), 2)

    def _fault(self, kind):
        d = self.sim.fleet[self.sel]
        if kind == "link":
            d.last_heartbeat_s = self.sim.now_s - 1000.0  # Lost on next tick
        elif kind == "drain":
            d.soc_pct = max(0.0, d.soc_pct - 20.0)
        elif kind == "wind":
            for x in self.sim.fleet.values():
                x.speed_factor = 0.8
            try:
                self.s_wind.set_val(0.8)
            except Exception:
                pass

    # --- events ---
    def _click(self, ev):
        if ev.inaxes is not self.ax or ev.xdata is None:
            return
        self.counter += 1
        self.sim.submit(f"REQ-VIS-{self.counter:03d}", (float(ev.xdata), float(ev.ydata), 50.0),
                        round(float(self.s_payload.val), 2), float(self.s_deadline.val) * 60.0)

    def _key(self, ev):
        if ev.key == " ":
            self.playing = not self.playing
        elif ev.key == "t":
            self.sim.tick(float(self.s_dt.val))
        elif ev.key == "r":
            self._reset()

    # --- draw ---
    def draw(self, snap):
        self.ax.clear()
        self.ax.set_aspect("equal")
        self.ax.set_xlim(-LIM, LIM)
        self.ax.set_ylim(-LIM, LIM)
        self.ax.set_title(f"t={snap['time_s']}s  sel={self.sel} (click map = dispatch, space=pause)")
        self.ax.set_xlabel("x (m)")
        self.ax.set_ylabel("y (m)")
        self.ax.plot(0, 0, "ks", ms=10, label="Base")
        for i, p in enumerate(snap["pads"]):
            self.ax.text(120 + i * 220, -120, f"{p['id']}\n{p['occupied_by'] or '-'}",
                         fontsize=7, ha="center",
                         bbox=dict(facecolor="red" if p["occupied_by"] else "white", alpha=0.7))

        for i, d in enumerate(snap["drones"]):
            x, y = d["position"][0], d["position"][1]
            self.trails[d["id"]].append((x, y))
            tx, ty = zip(*self.trails[d["id"]])
            self.ax.plot(tx, ty, color=color(d), lw=0.8, alpha=0.6)
            marker = "x" if d["status"] == "Lost" else "^"
            self.ax.plot(x, y, marker, color=color(d), ms=9)
            self.ax.text(x + 60, y + 60 + (i % 5) * 90, f"{d['id']} {d['soc_pct']:.0f}%", fontsize=6)
            if d["active_request"]:
                req = next((r for r in snap["requests"] if r["id"] == d["active_request"]), None)
                dest = self.sim.dispatcher.requests.get(d["active_request"])
                if req and dest:
                    dx, dy = dest.destination[0], dest.destination[1]
                    self.ax.plot([x, dx], [y, dy], "b--", lw=0.7, alpha=0.6)
                    self.ax.plot(dx, dy, "*", color="orange", ms=10)

        for r in snap["requests"]:
            if r["status"] == "Queued":
                dest = self.sim.dispatcher.requests.get(r["id"])
                if dest:
                    self.ax.plot(dest.destination[0], dest.destination[1], "o",
                                 color="orange", ms=8, alpha=0.7)

        m = snap["metrics"]
        self.info.clear()
        self.info.axis("off")
        lines = [
            f"time {snap['time_s']}s  queue(pads)={snap['queue']}",
            f"on-time {m['on_time_rate']:.1f}%  energy {m['total_energy_wh']:.0f}Wh",
            f"pad util {m['pad_utilization_pct']:.1f}%  late {m['mean_late_delay_s']:.0f}s",
            "",
            *[f"{d['id']} {d['status'][:4]} {d['soc_pct']:5.1f}% job={d['active_request'] or '-'}" for d in snap["drones"]],
            "",
            *[f"{r['id']} {r['status'][:4]} {r['assigned_drone'] or '-'}" for r in snap["requests"][-6:]],
        ]
        self.info.text(0, 1, "\n".join(lines), va="top", family="monospace", fontsize=8)

    def update(self, _frame):
        if self.playing:
            self.sim.tick(float(self.s_dt.val))
        self.draw(self.sim.snapshot())

    def run(self):
        FuncAnimation(self.fig, self.update, interval=300, cache_frame_data=False)
        plt.show()


def main():
    FleetViz().run()


if __name__ == "__main__":
    main()
