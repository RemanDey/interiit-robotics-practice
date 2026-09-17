"""Pygame 2D visualizer for the drone fleet (pygame + pygame_gui).

Parity with clients/fleet_2d.py (same layout, same sim, same colors/rules).

Run:  pip install pygame pygame_gui && python3 clients/fleet_pygame.py
Click the map to dispatch a delivery to that (x, y).
Headless smoke:  SDL_VIDEODRIVER=dummy python3 clients/fleet_pygame.py --smoke
"""

import sys
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "server" / "src"))

from server import FleetSimulation, seed  # noqa: E402

TRAIL = 30
LIM = 3000
WIN_W, WIN_H = 1200, 700
MAP_RECT_TUPLE = (20, 20, 730, 560)  # left map panel (~ axes [0.05,0.18,0.58,0.77])

COLORS = {
    "bg": (238, 243, 248),
    "white": (255, 255, 255),
    "black": (23, 32, 51),
    "red": (193, 18, 31),
    "blue": (31, 111, 235),
    "green": (24, 134, 75),
    "gray": (130, 130, 130),
    "orange": (255, 140, 0),
    "grid": (203, 213, 225),
}


def drone_color(status, soc_pct):
    if status == "Lost":
        return COLORS["gray"]
    if soc_pct < 20:
        return COLORS["red"]
    if status == "In-Flight":
        return COLORS["blue"]
    return COLORS["green"]


class FleetPygame:
    def __init__(self, headless=False):
        import pygame
        import pygame_gui

        self.pygame = pygame
        self.pygame_gui = pygame_gui
        pygame.init()
        pygame.font.init()
        self.headless = headless
        self.screen = pygame.display.set_mode((WIN_W, WIN_H))
        pygame.display.set_caption("Drone Fleet 2D Simulator (pygame)")
        self.clock = pygame.time.Clock()
        self.manager = pygame_gui.UIManager((WIN_W, WIN_H))

        from pygame_gui.elements import UIButton, UIHorizontalSlider, UILabel, UITextBox

        self._el = {}  # name -> element
        self._labels = {}
        mh = self.manager
        # --- left map rect ---
        self.map_rect = pygame.Rect(*MAP_RECT_TUPLE)

        # --- transport buttons (bottom-left, under map) ---
        by = 610
        self._el["play"] = UIButton(pygame.Rect(20, by, 110, 35), "Pause", mh)
        self._el["step"] = UIButton(pygame.Rect(140, by, 80, 35), "+30s", mh)
        self._el["reset"] = UIButton(pygame.Rect(230, by, 80, 35), "Reset", mh)
        self._labels["dt"] = UILabel(pygame.Rect(330, by - 18, 300, 20), "dt s (tick per 0.3s)", mh)
        self._el["dt"] = UIHorizontalSlider(pygame.Rect(330, by + 5, 250, 25), 10.0, (1.0, 120.0), mh)
        self._labels["dtval"] = UILabel(pygame.Rect(585, by + 5, 90, 25), "10s", mh)

        # --- right info box ---
        self.info_box = UITextBox("", pygame.Rect(770, 20, 410, 360), mh)

        # --- fault buttons ---
        fy = 390
        self._el["link"] = UIButton(pygame.Rect(770, fy, 130, 35), "Link loss", mh)
        self._el["drain"] = UIButton(pygame.Rect(908, fy, 130, 35), "Drain 20%", mh)
        self._el["windbtn"] = UIButton(pygame.Rect(1046, fy, 134, 35), "Wind .8", mh)

        # --- right sliders: payload / deadline / wind / drone ---
        rows = [("payload", "payload kg", 1.0, (0.2, 2.5), 450),
                ("deadline", "deadline min", 15.0, (2.0, 45.0), 510),
                ("wind", "wind factor", 1.0, (0.4, 1.2), 570),
                ("drone", "drone 1-10", 1.0, (1.0, 10.0), 630)]
        for name, text, start, rng, y in rows:
            self._labels[name] = UILabel(pygame.Rect(770, y - 20, 410, 20), text, mh)
            self._el[name] = UIHorizontalSlider(pygame.Rect(770, y, 300, 25), start, rng, mh)
            self._labels[name + "val"] = UILabel(pygame.Rect(1075, y, 105, 25), str(start), mh)

        # --- sim state (same as FleetViz) ---
        self.sim = FleetSimulation()
        if not self.sim.dispatcher.requests:
            seed(self.sim)
        self.playing = True
        self.counter = 0
        self.trails = {d: deque(maxlen=TRAIL) for d in self.sim.fleet}
        self.sel = "DR-01"
        self.font = pygame.font.SysFont("monospace", 13)
        self.bigfont = pygame.font.SysFont("monospace", 15, bold=True)
        self.accum_ms = 0.0
        self._refresh_info()

    # --- coordinate helpers ---
    def world_to_screen(self, x, y):
        r = self.map_rect
        return (r.centerx + x / LIM * (r.w / 2), r.centery - y / LIM * (r.h / 2))

    def screen_to_world(self, mx, my):
        r = self.map_rect
        return ((mx - r.centerx) / (r.w / 2) * LIM, (r.centery - my) / (r.h / 2) * LIM)

    # --- sim ops (mirror fleet_2d.py) ---
    def _reset(self):
        self.sim = FleetSimulation()
        if not self.sim.dispatcher.requests:
            seed(self.sim)
        self.trails = {d: deque(maxlen=TRAIL) for d in self.sim.fleet}
        self.accum_ms = 0.0
        self._refresh_info()

    def _apply_wind(self, val):
        for d in self.sim.fleet.values():
            d.speed_factor = round(float(val), 2)

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
                self._el["wind"].set_current_value(0.8)
                self._labels["windval"].set_text("0.8")
            except Exception:
                pass

    def _dispatch_click(self, mx, my):
        wx, wy = self.screen_to_world(mx, my)
        wx = max(-LIM, min(LIM, wx))
        wy = max(-LIM, min(LIM, wy))
        self.counter += 1
        payload = round(float(self._el["payload"].get_current_value()), 2)
        deadline = float(self._el["deadline"].get_current_value()) * 60.0
        self.sim.submit(f"REQ-VIS-{self.counter:03d}", (float(wx), float(wy), 50.0),
                        payload, deadline)
        self._refresh_info()

    def _refresh_info(self):
        snap = self.sim.snapshot()
        m = snap["metrics"]
        lines = [
            f"time {snap['time_s']}s&nbsp;&nbsp;sel={self.sel}",
            f"queue(pads)={snap['queue']}",
            f"on-time {m['on_time_rate']:.1f}%&nbsp;&nbsp;energy {m['total_energy_wh']:.0f}Wh",
            f"pad util {m['pad_utilization_pct']:.1f}%&nbsp;&nbsp;late {m['mean_late_delay_s']:.0f}s",
            "",
        ]
        for d in snap["drones"]:
            lines.append(f"{d['id']} {d['status'][:4]} {d['soc_pct']:5.1f}% job={d['active_request'] or '-'}")
        lines.append("")
        for r in snap["requests"][-6:]:
            lines.append(f"{r['id']} {r['status'][:4]} {r['assigned_drone'] or '-'}")
        self.info_box.html_text = "<br>".join(lines)
        self.info_box.rebuild()

    # --- drawing ---
    def _draw_dashed(self, surf, a, b, color, width=1, dash=8):
        import math
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        if length < 1e-6:
            return
        n = max(1, int(length / dash))
        for i in range(0, n, 2):
            t0, t1 = i / n, min(1.0, (i + 1) / n)
            p0 = (a[0] + dx * t0, a[1] + dy * t0)
            p1 = (a[0] + dx * t1, a[1] + dy * t1)
            self.pygame.draw.line(surf, color, p0, p1, width)

    def draw(self):
        pg = self.pygame
        surf = self.screen
        surf.fill((245, 247, 251))
        r = self.map_rect
        pg.draw.rect(surf, COLORS["white"], r)
        # grid
        for gx in (-1500, 0, 1500):
            x, _ = self.world_to_screen(gx, 0)
            pg.draw.line(surf, COLORS["grid"], (x, r.top), (x, r.bottom), 1)
        for gy in (-1500, 0, 1500):
            _, y = self.world_to_screen(0, gy)
            pg.draw.line(surf, COLORS["grid"], (r.left, y), (r.right, y), 1)
        pg.draw.rect(surf, COLORS["black"], r, 2)
        title = self.bigfont.render(
            f"t={self.sim.now_s:.0f}s  sel={self.sel} (click map = dispatch, space=pause)", True, COLORS["black"])
        surf.blit(title, (r.left + 6, r.top - 20 if r.top - 20 > 0 else r.top + 4))
        snap = self.sim.snapshot()

        # base
        bx, by = self.world_to_screen(0, 0)
        pg.draw.rect(surf, COLORS["black"], pg.Rect(bx - 6, by - 6, 12, 12))
        surf.blit(self.font.render("Base", True, COLORS["black"]), (bx + 10, by - 8))

        # pads (offset row like fleet_2d text at 120+i*220, -120)
        for i, p in enumerate(snap["pads"]):
            px, py = self.world_to_screen(120 + i * 900, -2600)
            box = pg.Rect(px - 45, py - 16, 90, 34)
            pg.draw.rect(surf, COLORS["red"] if p["occupied_by"] else COLORS["white"], box)
            pg.draw.rect(surf, COLORS["black"], box, 1)
            surf.blit(self.font.render(f"{p['id']}", True, COLORS["black"]), (box.x + 5, box.y + 2))
            surf.blit(self.font.render(f"{p['occupied_by'] or '-'}", True, COLORS["black"]), (box.x + 5, box.y + 17))

        # trails + drones + routes
        for i, d in enumerate(snap["drones"]):
            x, y = d["position"][0], d["position"][1]
            self.trails[d["id"]].append((x, y))
            col = drone_color(d["status"], d["soc_pct"])
            if len(self.trails[d["id"]]) > 1:
                pts = [self.world_to_screen(tx, ty) for tx, ty in self.trails[d["id"]]]
                try:
                    pg.draw.lines(surf, col, False, pts, 1)
                except Exception:
                    pass
            sx, sy = self.world_to_screen(x, y)
            if d["status"] == "Lost":
                pg.draw.line(surf, col, (sx - 6, sy - 6), (sx + 6, sy + 6), 2)
                pg.draw.line(surf, col, (sx - 6, sy + 6), (sx + 6, sy - 6), 2)
            else:
                pg.draw.polygon(surf, col, [(sx, sy - 8), (sx - 6, sy + 6), (sx + 6, sy + 6)])
            surf.blit(self.font.render(f"{d['id']} {d['soc_pct']:.0f}%", True, COLORS["black"]),
                      (sx + 10, sy + 8 + (i % 5) * 2))
            if d["active_request"]:
                dest = self.sim.dispatcher.requests.get(d["active_request"])
                if dest:
                    dx, dy = self.world_to_screen(dest.destination[0], dest.destination[1])
                    self._draw_dashed(surf, (sx, sy), (dx, dy), COLORS["blue"])
                    pg.draw.circle(surf, COLORS["orange"], (int(dx), int(dy)), 6)
        for req in snap["requests"]:
            if req["status"] == "Queued":
                dest = self.sim.dispatcher.requests.get(req["id"])
                if dest:
                    qx, qy = self.world_to_screen(dest.destination[0], dest.destination[1])
                    pg.draw.circle(surf, COLORS["orange"], (int(qx), int(qy)), 6, 2)

    # --- main loop ---
    def run(self, max_frames=None):
        pg = self.pygame
        frames = 0
        running = True
        while running:
            dt_ms = self.clock.tick(60)
            time_delta = dt_ms / 1000.0
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    running = False
                elif event.type == pg.KEYDOWN:
                    if event.key == pg.K_SPACE:
                        self.playing = not self.playing
                        self._el["play"].set_text("Play" if not self.playing else "Pause")
                    elif event.key == pg.K_t:
                        self.sim.tick(float(self._el["dt"].get_current_value()))
                        self._refresh_info()
                    elif event.key == pg.K_r:
                        self._reset()
                elif event.type == pg.MOUSEBUTTONDOWN and event.button == 1:
                    if self.map_rect.collidepoint(event.pos):
                        self._dispatch_click(*event.pos)
                if hasattr(self.pygame_gui, "UI_BUTTON_PRESSED") and event.type == self.pygame_gui.UI_BUTTON_PRESSED:
                    el = event.ui_element
                    if el == self._el["play"]:
                        self.playing = not self.playing
                        el.set_text("Play" if not self.playing else "Pause")
                    elif el == self._el["step"]:
                        self.sim.tick(30.0)
                        self._refresh_info()
                    elif el == self._el["reset"]:
                        self._reset()
                    elif el == self._el["link"]:
                        self._fault("link")
                    elif el == self._el["drain"]:
                        self._fault("drain")
                    elif el == self._el["windbtn"]:
                        self._fault("wind")
                if hasattr(self.pygame_gui, "UI_HORIZONTAL_SLIDER_MOVED") and \
                        event.type == self.pygame_gui.UI_HORIZONTAL_SLIDER_MOVED:
                    el = event.ui_element
                    if el == self._el["dt"]:
                        self._labels["dtval"].set_text(f"{float(event.value):.0f}s")
                    elif el == self._el["payload"]:
                        self._labels["payloadval"].set_text(f"{float(event.value):.2f}")
                    elif el == self._el["deadline"]:
                        self._labels["deadlineval"].set_text(f"{float(event.value):.0f}")
                    elif el == self._el["wind"]:
                        self._labels["windval"].set_text(f"{float(event.value):.2f}")
                        self._apply_wind(event.value)
                    elif el == self._el["drone"]:
                        self.sel = f"DR-{int(event.value):02d}"
                        self._labels["droneval"].set_text(self.sel)
                        self._refresh_info()
                self.manager.process_events(event)

            # fixed-step sim advance (~ every 300ms) — explicit, never GC'd
            if self.playing:
                self.accum_ms += dt_ms
                if self.accum_ms >= 300:
                    self.accum_ms = 0.0
                    self.sim.tick(float(self._el["dt"].get_current_value()))
                    self._refresh_info()

            self.manager.update(time_delta)
            self.draw()
            self.manager.draw_ui(self.screen)
            pg.display.update()
            frames += 1
            if max_frames is not None and frames >= max_frames:
                running = False
        if self.headless:
            return self.sim.now_s


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--smoke", action="store_true", help="headless smoke test, no window loop")
    args = ap.parse_args()
    if args.smoke:
        import os
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
        viz = FleetPygame(headless=True)
        t0 = viz.sim.now_s
        for _ in range(10):
            viz.sim.tick(10.0)
        viz._refresh_info()
        viz.draw()
        viz.manager.update(0.016)
        viz.manager.draw_ui(viz.screen)
        viz.pygame.display.flip()
        print(f"smoke ok: t {t0:.0f} -> {viz.sim.now_s:.0f}s, "
              f"drones={len(viz.sim.fleet)}, requests={len(viz.sim.dispatcher.requests)}")
        return
    FleetPygame().run()


if __name__ == "__main__":
    main()
