# PS02: Semantic Mapping of an Indoor Environment

## Overview

This repository implements a complete semantic mapping pipeline for indoor environments using a TurtleBot 4 with OAK-D RGB-D camera in Gazebo simulation. The system performs real-time instance segmentation, 3D localization, and persistent semantic mapping with Kalman filter-based spatial deduplication.

## Architecture

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐     ┌────────────────┐
│  RGB-D      │────▶│  YOLOv8n-   │────▶│  Mask +     │────▶│  TF2 Transform │
│  Camera     │     │  seg        │     │  Depth      │     │  to World      │
└─────────────┘     └─────────────┘     └─────────────┘     └────────────────┘
                                                              │
                                                              ▼
┌─────────────┐     ┌─────────────┐     ┌─────────────┐     ┌──────────────┐
│  Semantic   │◀────│  Kalman     │◀────│  Floor      │◀────│  Min-Z       │
│  Map JSON   │     │  Trackers   │     │  Projection │     │  Projection  │
└─────────────┘     └─────────────┘     └─────────────┘     └──────────────┘
```

## Key Features

- **Pixel-level segmentation**: YOLOv8n-seg with dynamic masks (not bounding boxes)
- **3D Floor Coordinates**: Min-Z point in mask projected to floor plane (z=0)
- **World Frame Mapping**: TF2 transformation from camera optical frame → map frame
- **Spatial Deduplication**: Kalman filter (constant velocity) for persistent object tracking
- **Benchmarking**: Per-stage latency, FPS, and peak RAM monitoring
- **Output Format**: Compliant JSON schema as specified

## Output Format

```json
[
  {
    "id": 1,
    "label": "sofa",
    "position": [1.2, 3.4, 0.0],
    "confidence": 0.89
  }
]
```

## Requirements

- ROS 2 Humble
- Ubuntu 22.04
- Python 3.10+
- NVIDIA GPU (for TensorRT optimization, optional)

### Python Dependencies
```bash
pip install numpy scipy ultralytics opencv-python psutil
```

### ROS 2 Dependencies
```bash
sudo apt install ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
    ros-humble-message-filters ros-humble-cv-bridge
```

## Building

```bash
cd /path/to/workspace
colcon build --packages-select object_mapping perception
source install/setup.bash
```

## Running

### 1. Start Gazebo Simulation (small_house world)
```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true nav2:=true rviz:=true model:=lite
```

### 2. Launch Semantic Mapping
```bash
ros2 launch object_mapping semantic_mapping.launch.py
```

### 3. Monitor Output
```bash
# View semantic map (updated in real-time)
watch -n 1 cat semantic_map.json

# View ROS topic
ros2 topic echo /perception
```

### 4. Optional: Run Standalone Perception Node
```bash
ros2 run perception perception_segmentor
```

## Configuration Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `model_name` | `yolov8n-seg.pt` | YOLO model file |
| `image_topic` | `/oakd/rgb/preview/image_raw` | RGB image topic |
| `depth_topic` | `/oakd/rgb/preview/depth` | Aligned depth topic |
| `camera_info_topic` | `/oakd/rgb/preview/camera_info` | Camera intrinsics |
| `json_output_path` | `semantic_map.json` | Output JSON file |
| `target_frame` | `map` | TF target frame (map/odom) |
| `dedup_threshold` | `0.5` | Deduplication distance (m) |
| `max_missed` | `30` | Frames before removing tracker |
| `benchmark_enabled` | `true` | Enable performance logging |

## TF Frame Chain

```
map → odom → base_link → oakd_rgb_camera_frame → oakd_rgb_camera_optical_frame
```

The node transforms from `oakd_rgb_camera_optical_frame` (Z-forward, X-right, Y-down) to the specified `target_frame`.

## Benchmarking

The node logs per-stage latency every 100 frames:
- `inference`: YOLOv8 forward pass
- `projection`: Mask depth sampling + 3D deprojection
- `tf_lookup`: TF2 buffer lookup
- `deduplication`: Kalman filter predict/update
- `total`: End-to-end pipeline

Also tracks peak RAM usage via `psutil`.

## TensorRT Optimization (Optional)

For deployment on Jetson Nano / edge devices:

```bash
# Export to ONNX
yolo export model=yolov8n-seg.pt format=onnx opset=12

# Convert to TensorRT FP16
trtexec --onnx=yolov8n-seg.onnx --saveEngine=yolov8n-seg_fp16.engine --fp16

# For INT8 (requires calibration dataset)
trtexec --onnx=yolov8n-seg.onnx --saveEngine=yolov8n-seg_int8.engine --int8 --calib=calibration.cache
```

Then modify inference to use TensorRT Python API or ONNX Runtime with TensorRT EP.

## Performance Targets

| Metric | Target | Notes |
|--------|--------|-------|
| FPS | ≥ 10-15 | On Jetson Nano / RPi |
| Peak RAM | < 8 GB | Including model weights |
| Coordinate Drift | < 0.2m | After multiple loops |

## File Structure

```
src/
├── object_mapping/
│   ├── object_mapping/
│   │   ├── __init__.py
│   │   └── object_mapping_node.py    # Main mapping node
│   ├── launch/
│   │   └── semantic_mapping.launch.py
│   ├── setup.py
│   └── package.xml
├── perception/
│   ├── perception/
│   │   ├── __init__.py
│   │   └── semantic_segmentation.py  # Standalone seg node
│   ├── setup.py
│   └── package.xml
├── semantic_mapping_python/
│   ├── main.py                        # Unified entry point
│   └── experimentation.py             # Dev/testing script
└── turtlebot4_simulator/              # Gazebo simulation
```

## License

Apache-2.0