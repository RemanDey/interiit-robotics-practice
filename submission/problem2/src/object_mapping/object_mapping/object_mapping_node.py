#!/usr/bin/env python3

import json
import os
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from ultralytics import YOLO
import message_filters
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped
from scipy.linalg import block_diag


def imgmsg_to_cv2_direct(msg: Image) -> np.ndarray:
    """Directly converts a ROS 2 Image message to an OpenCV BGR numpy array."""
    dtype = np.uint8
    if msg.encoding in ['bgr8', 'rgb8']:
        channels = 3
    elif msg.encoding == 'mono8':
        channels = 1
    else:
        raise ValueError(f'Unsupported encoding for direct conversion: {msg.encoding}')

    frame = np.frombuffer(msg.data, dtype=dtype).reshape((msg.height, msg.width, channels))

    if msg.encoding == 'rgb8':
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    return frame


def depthmsg_to_cv2_direct(msg: Image) -> np.ndarray:
    """Directly converts a ROS 2 Depth Image message to a float32 depth map (in meters)."""
    if msg.encoding == '16UC1':
        depth = np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width))
        return depth.astype(np.float32) / 1000.0
    elif msg.encoding == '32FC1':
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
        return depth
    else:
        raise ValueError(f'Unsupported depth encoding: {msg.encoding}')


def cv2_to_imgmsg_direct(cv_image: np.ndarray, header, encoding: str = 'bgr8') -> Image:
    """Directly converts an OpenCV image array to a ROS 2 Image message."""
    msg = Image()
    msg.header = header
    msg.height, msg.width = cv_image.shape[:2]

    if encoding == 'bgr8':
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = cv_image.tobytes()
    else:
        raise ValueError(f'Unsupported encoding for direct conversion: {encoding}')

    return msg


class KalmanTracker:
    """Constant velocity Kalman filter for 2D world position tracking."""

    def __init__(self, initial_pos, track_id, label, confidence):
        self.x = np.array([initial_pos[0], initial_pos[1], 0.0, 0.0])
        self.P = np.eye(4) * 100.0
        self.F = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]])
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        self.Q = block_diag(np.eye(2) * 0.01, np.eye(2) * 0.1)
        self.R = np.eye(2) * 0.05
        self.track_id = track_id
        self.label = label
        self.confidence = confidence
        self.last_update_time = time.time()
        self.hits = 1
        self.missed = 0

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x[:2].copy()

    def update(self, measurement, confidence):
        z = np.array(measurement)
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P
        self.confidence = 0.9 * self.confidence + 0.1 * confidence
        self.hits += 1
        self.missed = 0
        self.last_update_time = time.time()

    def get_position(self):
        return [float(self.x[0]), float(self.x[1]), 0.0]

    def increment_missed(self):
        self.missed += 1


class Timer:
    """Simple context manager for timing code blocks."""

    def __init__(self, name, logger, enabled=True):
        self.name = name
        self.logger = logger
        self.enabled = enabled
        self.start = 0.0
        self.elapsed = 0.0

    def __enter__(self):
        if self.enabled:
            self.start = time.perf_counter()
        return self

    def __exit__(self, *args):
        if self.enabled:
            self.elapsed = (time.perf_counter() - self.start) * 1000.0


class PerceptionSegmentorNode(Node):

    def __init__(self):
        super().__init__('perception_segmentor_node')

        self.declare_parameter('model_name', 'yolov8n-seg.pt')
        self.declare_parameter('image_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter('depth_topic', '/oakd/rgb/preview/depth')
        self.declare_parameter('camera_info_topic', '/oakd/rgb/preview/camera_info')
        self.declare_parameter('perception_topic', '/perception')
        self.declare_parameter('mask_topic', '/perception_mask')
        self.declare_parameter('classification_topic', '/perception_classification')
        self.declare_parameter('json_output_path', 'semantic_map.json')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('dedup_threshold', 0.5)
        self.declare_parameter('max_missed', 30)
        self.declare_parameter('benchmark_enabled', True)

        model_name = self.get_parameter('model_name').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        perception_topic = self.get_parameter('perception_topic').get_parameter_value().string_value
        mask_topic = self.get_parameter('mask_topic').get_parameter_value().string_value
        classification_topic = self.get_parameter('classification_topic').get_parameter_value().string_value
        self.json_output_path = self.get_parameter('json_output_path').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.dedup_threshold = self.get_parameter('dedup_threshold').get_parameter_value().double_value
        self.max_missed = self.get_parameter('max_missed').get_parameter_value().integer_value
        self.benchmark_enabled = self.get_parameter('benchmark_enabled').get_parameter_value().bool_value

        self.get_logger().info(f'Loading YOLO model: {model_name}...')
        self.model = YOLO(model_name)

        self.intrinsics = None
        self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 10)

        self.rgb_sub = message_filters.Subscriber(self, Image, image_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=10, slop=0.05
        )
        self.ts.registerCallback(self.synchronized_callback)

        self.json_publisher_ = self.create_publisher(String, perception_topic, 10)
        self.mask_publisher_ = self.create_publisher(Image, mask_topic, 10)
        self.classification_publisher_ = self.create_publisher(Image, classification_topic, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.trackers = {}
        self.next_persistent_id = 1

        self.frame_count = 0
        self.latencies = {
            'inference': [],
            'projection': [],
            'tf_lookup': [],
            'deduplication': [],
            'total': []
        }
        self.memory_samples = []

        self.get_logger().info(
            f'Perception node initialized.\n'
            f' - RGB Input: {image_topic}\n'
            f' - Depth Input: {depth_topic}\n'
            f' - Camera Info: {camera_info_topic}\n'
            f' - Target Frame: {self.target_frame}\n'
            f' - Output JSON File: {self.json_output_path}'
        )

    def camera_info_callback(self, msg: CameraInfo):
        if self.intrinsics is None:
            self.intrinsics = {
                'fx': msg.k[0],
                'fy': msg.k[4],
                'cx': msg.k[2],
                'cy': msg.k[5]
            }
            self.get_logger().info('Camera intrinsics successfully received.')

    def synchronized_callback(self, rgb_msg: Image, depth_msg: Image):
        total_timer = Timer('total', self.get_logger(), self.benchmark_enabled)
        total_timer.__enter__()

        if self.intrinsics is None:
            self.get_logger().warn(2.0, 'Waiting for CameraInfo intrinsics...')
            return

        try:
            frame = imgmsg_to_cv2_direct(rgb_msg)
            depth_map = depthmsg_to_cv2_direct(depth_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to convert ROS images: {e}')
            return

        with Timer('inference', self.get_logger(), self.benchmark_enabled) as t:
            results = self.model.track(
                frame, persist=True, tracker='bytetrack.yaml', verbose=False
            )[0]
        if self.benchmark_enabled:
            self.latencies['inference'].append(t.elapsed)

        annotated_frame = results.plot()
        annotated_frame_bgr = np.ascontiguousarray(annotated_frame, dtype=np.uint8)

        try:
            bbox_msg = cv2_to_imgmsg_direct(annotated_frame_bgr, rgb_msg.header, encoding='bgr8')
            self.classification_publisher_.publish(bbox_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish classification image: {e}')

        mask_overlay = np.zeros_like(frame)

        if results.masks is not None:
            combined_mask = results.masks.data.any(dim=0).cpu().numpy().astype(np.uint8)
            if combined_mask.shape[:2] != frame.shape[:2]:
                combined_mask = cv2.resize(
                    combined_mask,
                    (frame.shape[1], frame.shape[0]),
                    interpolation=cv2.INTER_NEAREST,
                )
            mask_overlay = cv2.bitwise_and(frame, frame, mask=combined_mask)

        try:
            mask_msg = cv2_to_imgmsg_direct(mask_overlay, rgb_msg.header, encoding='bgr8')
            self.mask_publisher_.publish(mask_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish mask image: {e}')

        new_detections = []

        if (
            results.boxes is not None
            and results.boxes.id is not None
            and results.masks is not None
        ):
            track_ids = results.boxes.id.int().cpu().tolist()
            class_ids = results.boxes.cls.int().cpu().tolist()
            confidences = results.boxes.conf.float().cpu().tolist()
            masks_tensor = results.masks.data.cpu().numpy()

            for idx, (track_id, cls_id, conf) in enumerate(
                zip(track_ids, class_ids, confidences)
            ):
                obj_mask = masks_tensor[idx].astype(np.uint8)
                if obj_mask.shape[:2] != depth_map.shape[:2]:
                    obj_mask = cv2.resize(
                        obj_mask,
                        (depth_map.shape[1], depth_map.shape[0]),
                        interpolation=cv2.INTER_NEAREST,
                    )

                mask_bool = obj_mask > 0
                if not np.any(mask_bool):
                    continue

                mask_depths = depth_map[mask_bool]
                valid_depths = mask_depths[(mask_depths > 0) & (~np.isnan(mask_depths))]

                if len(valid_depths) == 0:
                    continue

                with Timer('projection', self.get_logger(), self.benchmark_enabled) as t:
                    min_z = float(np.min(valid_depths))
                    min_z_indices = np.where((depth_map == min_z) & mask_bool)
                    if len(min_z_indices[0]) > 0:
                        u_min = float(min_z_indices[1][0])
                        v_min = float(min_z_indices[0][0])
                    else:
                        v_indices, u_indices = np.where(mask_bool)
                        u_min = float(np.mean(u_indices))
                        v_min = float(np.mean(v_indices))
                        min_z = float(np.median(valid_depths))

                    x_cam = (u_min - self.intrinsics['cx']) * min_z / self.intrinsics['fx']
                    y_cam = (v_min - self.intrinsics['cy']) * min_z / self.intrinsics['fy']

                    point_camera = PointStamped()
                    point_camera.header = rgb_msg.header
                    point_camera.header.frame_id = 'oakd_rgb_camera_optical_frame'
                    point_camera.point.x = x_cam
                    point_camera.point.y = y_cam
                    point_camera.point.z = min_z

                    try:
                        with Timer('tf_lookup', self.get_logger(), self.benchmark_enabled) as tf_timer:
                            transform = self.tf_buffer.lookup_transform(
                                self.target_frame,
                                point_camera.header.frame_id,
                                rclpy.time.Time(),
                                timeout=rclpy.duration.Duration(seconds=0.1)
                            )
                        if self.benchmark_enabled:
                            self.latencies['tf_lookup'].append(tf_timer.elapsed)

                        point_world = do_transform_point(point_camera, transform)
                        floor_point = [point_world.point.x, point_world.point.y, 0.0]

                    except tf2_ros.TransformException as e:
                        self.get_logger().warn(f'TF lookup failed: {e}')
                        continue
                if self.benchmark_enabled:
                    self.latencies['projection'].append(t.elapsed)

                class_name = self.model.names[cls_id]

                with Timer('deduplication', self.get_logger(), self.benchmark_enabled) as t:
                    best_match_id = None
                    best_dist = self.dedup_threshold

                    for pid, tracker in self.trackers.items():
                        pred_pos = tracker.predict()
                        dist = np.linalg.norm(np.array(floor_point[:2]) - pred_pos)
                        if dist < best_dist:
                            best_dist = dist
                            best_match_id = pid

                    if best_match_id is not None:
                        self.trackers[best_match_id].update(floor_point[:2], conf)
                        persistent_id = best_match_id
                    else:
                        persistent_id = self.next_persistent_id
                        self.next_persistent_id += 1
                        self.trackers[persistent_id] = KalmanTracker(
                            floor_point, persistent_id, class_name, conf
                        )
                if self.benchmark_enabled:
                    self.latencies['deduplication'].append(t.elapsed)

                new_detections.append({
                    'persistent_id': persistent_id,
                    'class': class_name,
                    'confidence': round(conf, 4),
                    'floor_position': floor_point
                })

        stale = [pid for pid, t in self.trackers.items() if t.missed > self.max_missed]
        for pid in stale:
            del self.trackers[pid]

        for tracker in self.trackers.values():
            if tracker.missed == 0:
                tracker.increment_missed()

        if new_detections or self.frame_count % 30 == 0:
            self.write_semantic_map()

        self.frame_count += 1
        total_timer.__exit__()
        if self.benchmark_enabled:
            self.latencies['total'].append(total_timer.elapsed)

        if self.benchmark_enabled and self.frame_count % 100 == 0:
            self.log_benchmarks()

    def write_semantic_map(self):
        output_data = []
        for pid, tracker in self.trackers.items():
            pos = tracker.get_position()
            output_data.append({
                "id": pid,
                "label": tracker.label,
                "position": pos,
                "confidence": round(tracker.confidence, 4)
            })

        try:
            with open(self.json_output_path, 'w') as f:
                json.dump(output_data, f, indent=2)
        except Exception as e:
            self.get_logger().error(f'Failed to write semantic map: {e}')

        json_msg = String()
        json_msg.data = json.dumps(output_data)
        self.json_publisher_.publish(json_msg)

        if self.frame_count % 30 == 0 or new_detections:
            self.get_logger().info(f'Semantic map updated: {len(output_data)} objects')

    def log_benchmarks(self):
        import psutil
        process = psutil.Process(os.getpid())
        mem_mb = process.memory_info().rss / 1024 / 1024
        self.memory_samples.append(mem_mb)

        self.get_logger().info(f'--- Benchmark (frame {self.frame_count}) ---')
        for stage, times in self.latencies.items():
            if times:
                recent = times[-100:]
                avg = np.mean(recent)
                p95 = np.percentile(recent, 95)
                self.get_logger().info(f'  {stage}: avg={avg:.1f}ms p95={p95:.1f}ms')
        if self.memory_samples:
            peak = max(self.memory_samples)
            self.get_logger().info(f'  RAM: current={mem_mb:.0f}MB peak={peak:.0f}MB')


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionSegmentorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()