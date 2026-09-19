# PS02: Semantic Mapping of an Indoor Environment

## Demo Video

[![Demo Video](https://img.youtube.com/vi/yKyAMs5Lchg/hqdefault.jpg)](https://youtu.be/yKyAMs5Lchg)

Watch the demo: https://youtu.be/yKyAMs5Lchg

## Overview

ROS 2 (Humble) semantic-mapping pipeline for a TurtleBot 4 with OAK-D RGB-D camera in Ignition Gazebo. A perception node runs YOLO instance segmentation with persistent tracking (ByteTrack), lifts each mask centroid to 3D with aligned depth + camera intrinsics, transforms it into the `map` frame via TF2, and maintains a persistent `map.json` keyed by track ID. A companion navigator node reads `map.json`, sends the best-confidence match to Nav2, and publishes RViz label markers.

## Architecture

```
RGB + Depth (time-synced) → YOLO-seg track (ByteTrack, persist=True)
  → mask centroid (moments) → median depth in 5×5 window
  → PinholeCameraModel.projectPixelTo3dRay → TF2 → map frame
  → tracked_objects dict → /perception_map (String JSON)
                         → /perception/debug_image (Image)
                         → map.json (cwd of node)
```

Dedup/tracking is done by the Ultralytics tracker (`bytetrack.yaml`, `persist=True`), not by a custom Kalman filter. There is no min-Z floor projection — the stored point is the centroid ray at median depth, transformed to `map`.

## Packages / Files

| Path | What it is |
|------|------------|
| `src/perception/perception/semantic_segmentation.py` | Active node `Object3DMapperNode`, entry point `perception_segmentor` |
| `src/perception/perception/semantic_segmentation_v2.py` | Flattened variant of the same node (not registered in `setup.py`) |
| `src/perception/perception/object_navigator.py` | `ObjectNavigator`: `map.json` lookup → Nav2 `NavigateToPose` + `TEXT_VIEW_FACING` markers on `/object_labels` @ 1 Hz (not registered in `setup.py`, run directly) |
| `src/perception/perception/test_markers.py` | Minimal SPHERE marker publisher on `/object_labels` for RViz testing |
| `src/perception/perception/map.json` | Latest mapping run output (dict keyed by track ID) |
| `map.json` (repo root) | Older/smaller mapping run output, same schema |
| `frames_2026-09-16_21.55.56.gv` / `.pdf` | `view_frames` TF-tree dump (2026-09-16): `map → odom → base_link → … → oakd_*_optical_frame` |
| `yolov8n-seg.pt`, `yolo11s-seg.pt` (root) + `src/perception/perception/{yolov8n,yolo11n,yolo11s}-seg.pt` | YOLO-seg weights (root copies duplicate the in-package ones) |
| `result-2026-KD-0-20260902.json` | Host environment inventory (distro + `apt` package list), **not** a mapping result despite the name |
| `src/experimentation/` | Offline prototyping: webcam/image YOLO-seg tests, YOLO-World/YOLOE scripts, custom ADE20K training (`my_training/`), local weights |
| `src/turtlebot4_simulator/` | Upstream TurtleBot 4 Ignition Gazebo sim (worlds in `turtlebot4_ignition_bringup/worlds/`: `depot`, `maze`, `warehouse`, `my_world`) |
| `src/turtlebot4/` | Upstream TurtleBot 4 drivers/msgs/description/navigation |
| `src/m-explore-ros2/` | Third-party `m-explore` ROS 2 port (autonomous exploration / map merge) |
| `build/` `install/` `log/` | Local `colcon` artifacts — do not commit |

## Output Format

Actual schema is a dict keyed by track ID with `position` as an `{x, y, z}` object (not a list):

```json
{
  "20": {
    "id": 20,
    "label": "bench",
    "position": { "x": -4.27, "y": -1.69, "z": 0.52 },
    "confidence": 0.5561
  }
}
```

The node rewrites `map.json` (relative to the node's working directory) on every callback while any object is tracked, and also publishes the same dict as a JSON string on `/perception_map`.

## Requirements

- ROS 2 Humble, Ubuntu 22.04, Python 3.10+
- ROS deps: `tf2_ros`, `tf2_geometry_msgs`, `message_filters`, `cv_bridge`, `image_geometry`, `sensor_msgs`, `visualization_msgs`, `nav2_msgs` (navigator only)
- Python deps: `numpy`, `ultralytics`, `opencv-python` (`pip install numpy ultralytics opencv-python`)

## Building

```bash
cd /path/to/workspace   # this folder
colcon build --packages-select perception
source install/setup.bash
```

Only `perception_segmentor` is exposed as a console script. `semantic_segmentation_v2`, `object_navigator`, and `test_markers` have `main()` but no entry point — run them with `ros2 run perception <module>` after adding entry points, or directly with `python3 <file>` inside a sourced environment.

## Running

```bash
# 1. Sim + SLAM/Nav2 (provides map -> odom and oakd topics)
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true nav2:=true rviz:=true model:=lite

# 2. Perception node (active implementation)
ros2 run perception perception_segmentor --ros-args \
  -p model_name:=yolov8n-seg.pt \
  -p rgb_topic:=/oakd/rgb/preview/image_raw \
  -p depth_topic:=/oakd/rgb/preview/depth \
  -p camera_info_topic:=/oakd/rgb/preview/camera_info \
  -p map_frame:=map

# 3. Monitor output
ros2 topic echo /perception_map
ros2 topic echo /perception/debug_image   # or view in RViz
cat map.json                              # written to the node's cwd

# 4. Navigate to a mapped object (reads ./map.json, prompts for label)
python3 src/perception/perception/object_navigator.py
```

## Configuration Parameters (`semantic_segmentation.py`)

| Parameter | Default | Notes |
|-----------|---------|-------|
| `model_name` | `yolov8-seg.pt` | N.B. differs from `_v2` default (`yolov8n-seg.pt`); resolved relative to node cwd unless absolute |
| `rgb_topic` | `/oakd/rgb/preview/image_raw` | Synced via `ApproximateTimeSynchronizer` (queue 10, slop 0.1) |
| `depth_topic` | `/oakd/rgb/preview/depth` | `passthrough`; uint16 assumed mm, rejected outside 0.1–10.0 m |
| `camera_info_topic` | `/oakd/rgb/preview/camera_info` | Latched once via `PinholeCameraModel` |
| `map_frame` | `map` | TF lookup timeout 0.1 s; failures warn/skip |
| `json_output_topic` | `/perception_map` | `std_msgs/String` with the full dict |
| `debug_image_topic` | `/perception/debug_image` | `results.plot()` + centroid dot + `ID + Map:(x,y,z)` text |

Depth sampling: median of valid (>0) pixels in a 5×5 window around the mask centroid; masks resized with `INTER_NEAREST` when needed.

## TF Frame Chain (from `frames_*.gv`)

```
map → odom → base_link → {oakd_camera_bracket, rplidar_link, wheels, …}
oakd_camera_bracket → oakd_link → oakd_rgb_camera_frame → oakd_rgb_camera_optical_frame
```

## Known Gaps vs PROBLEM_STATEMENT

- No launch file, no `object_mapping` package, no per-stage latency/RAM logging, no TensorRT/ONNX path in the ROS code (RAM timing only exists in `src/experimentation/experimentation.py`).
- No custom Kalman filter or Euclidean dedup node — identity persistence comes from ByteTrack IDs.
- Stored `z` is the transformed centroid height, not a floor-projected meeting point.
- Labels/IDs in `map.json` are raw YOLO-seg + ByteTrack output (note mislabels in sample runs, e.g. `airplane`, `traffic light` indoors); no confidence filtering or label allowlist.

## License

Apache-2.0
