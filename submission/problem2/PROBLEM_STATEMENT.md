# PS02: Semantic Mapping of an Indoor Environment

**Category:** Perception (Hard)  
**Target Hardware:** Jetson Nano / Raspberry Pi (< 8 GB Peak RAM limit)  
**Simulation Environment:** Gazebo `small_house` world (TurtleBot 3 Waffle or TurtleBot 4)  
**Repository Target:** `Team-Deimos-IIT-Mandi/Robotic-PS`

---

## 1. Core Objectives & Non-Negotiable Rules
- **No Pre-built Maps:** The robot starts with no prior map data or pre-loaded object coordinates.
- **Pixel-Level Segmentation:** Must use dynamic pixel masks (not 2D bounding boxes) to prevent background depth corruption.
- **3D Floor Coordinate:** Convert spatial mask points into a single 3D point projected onto the floor meeting point ($z_{\text{min}}$ projected to floor surface).
- **Fixed Reference Frame:** Map object coordinates into a unified global world frame ($X, Y, Z$) to avoid camera-frame drift.
- **Edge Budget:** Target FPS $\ge$ 10–15, Peak RAM $< 8.0\text{ GB}$. Must run deployable models (TensorRT INT8/FP16 or ONNX runtime).

---

## 2. Technical Pipeline Breakdown
1. **Sensor Stream:** ROS 2 RGB-D alignment topics (`/camera/color/image_raw` and `/camera/aligned_depth_to_color/image_raw`).
2. **Segmentation Engine:** Lightweight vision front-end (e.g., YOLOv8n-seg engine running on TensorRT).
3. **3D Point Projection:**
   $$X_c = \frac{(u - c_x) \cdot Z}{f_x}, \quad Y_c = \frac{(v - c_y) \cdot Z}{f_y}$$
   Transform camera coordinates to world coordinates using dynamic TF trees ($T_{\text{world}}^{\text{camera}}$).
4. **Spatial Deduplication:** Store detections in a persistent database. Use Euclidean distance thresholding or Kalman filtering to update existing objects when observed multiple times.

---

## 3. Benchmark Metrics to Record
- **Peak Memory Usage:** RAM consumption (MB/GB) logged throughout full map runs.
- **End-to-End Latency:** Per-stage timing breakdown (Inference ms, Point Cloud projection ms, Spatial deduplication ms).
- **Pose & Coordinate Drift:** Absolute error of stored base-point coordinates after multiple loops around the house.

---

## 4. Required Deliverables
1. **GitHub Repository:** Complete vision and mapping nodes with full setup README.
2. **Semantic Map Output:** Formatted JSON or YAML object output file:
   ```json
   {
     "id": 1,
     "label": "sofa",
     "position": [1.2, 3.4, 0.0],
     "confidence": 0.89
   }