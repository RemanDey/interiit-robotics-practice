# Solutions Overview

This file summarizes the submissions in `submission/` (note: singular, not `submissions/`).

## Layout

```
submission/
  problem1/  # Drone Fleet Management for Timed Deliveries — implemented
  problem2/  # Semantic Mapping of an Indoor Environment — implemented
  problem3/  # Empty — no submission yet
  problem7/  # USB serial persistent naming via udev — notes only
```

Root context files: `software.pdf`, `electrical_ps.pdf`, `mech.pdf` (problem statements),
`target_foot_tip_path.csv`, `demo_dynamic.webm`, `tasks/task1/`.

## problem1 — Drone Fleet Management for Timed Deliveries

* Source spec: `submission/problem1/PROBLEM_STATEMENT.md` (10 drones, 3 charging pads, payload ≤ 2.5 kg, deadlines 10–45 min, golden safety rule + explainability).
* What was built: 3-stage file-coupled prototype:
  `sim/sim.cpp` (C++, 10 agents, 2 s tick) → `backend/telemetry.json` → `api/main.py` + `api/fleet_manager.py` (FastAPI dispatch on `:8000`) → `dashboard/display/src/App.tsx` (React 19 + Leaflet, polls `:8000/telemetry` every 2 s).
* Key logic: `assign_request` gates on weight / deadline feasibility, eligibility (not CHARGING/RETURNING/LANDED, battery > 15), score `battery + IDLE bonus`; full per-candidate audit trail; metrics (on-time rate, kWh, pad utilization, delay, battery variance).
* Run: see `submission/problem1/INSTRUCTIONS.md` — `make && ./simulator` (repo root) → `cd api && uvicorn main:app --port 8000` → `cd dashboard/display && npm install && npm run dev`.
* Deep dive / known gaps: `submission/problem1/EXPLANATION.md` and `submission/problem1/docs/`, `REFERENCES.md` (assignments evaporate on reload, pads unsynced, threshold mismatches, 1/3 tests red).

## problem2 — Semantic Mapping of an Indoor Environment (Perception, Hard)

* Source spec: `submission/problem2/PROBLEM_STATEMENT.md` (no pre-built maps, pixel masks, 3D floor point, global frame, ≥10–15 FPS, <8 GB RAM, TensorRT/ONNX).
* What was built: ROS 2 Humble pipeline for TurtleBot 4 + OAK-D in Ignition Gazebo:
  RGB + depth (time-synced) → YOLO-seg + ByteTrack (`persist=True`) → mask-centroid + 5×5 median depth → `PinholeCameraModel` → TF2 `map` frame → `map.json` + `/perception_map` + `/perception/debug_image`.
* Main files: `src/perception/perception/semantic_segmentation.py` (`perception_segmentor`), `object_navigator.py` (Nav2 `NavigateToPose` + RViz markers), `map.json` outputs, TF dump `frames_2026-09-16_21.55.56.gv/.pdf`, weights `yolov8n-seg.pt` / `yolo11s-seg.pt`.
* Run/build: `submission/problem2/README.md` (`colcon build --packages-select perception`, sim + SLAM/Nav2 launch, `ros2 run perception perception_segmentor`); custom ADE20K + COCO training note in `KAGGLE_TRAINING.md`.
* Known gaps (per README): no launch file / latency-RAM logging / TensorRT path, ByteTrack instead of custom Kalman/Euclidean dedup, stored `z` is centroid height not floor-projected, raw YOLO labels unfiltered.

## problem3 — Empty

* `submission/problem3/` exists but contains no files. No solution submitted.

## problem7 — Stable USB Serial Naming

* Single note: `submission/problem7/APPROACH.md`.
* Approach: invariant is the physical USB port location; use `udevadm info -a -n /dev/ttyUSB0` to get `KERNELS` path, write `/etc/udev/rules.d/99-robot-serial.rules` e.g. `SUBSYSTEM=="tty", KERNELS=="1-2.1:1.0", SYMLINK+="robot/port_1_2", MODE="0666"`, then `udevadm control --reload-rules && udevadm trigger`, verify with `ls -l /dev/robot/`.
