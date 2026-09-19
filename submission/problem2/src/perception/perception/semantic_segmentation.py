#!/usr/bin/env python3

import json
import traceback
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import message_filters
from ultralytics import YOLO
from image_geometry import PinholeCameraModel
from cv_bridge import CvBridge

try:
    from perception.profiler import Profiler
except ImportError:  # direct `python3 <file>` run without install
    from profiler import Profiler  # type: ignore
'''
{
  "id": 1,
  "label": "sofa",
  "position": [1.2, 3.4, 0.0],
  "confidence": 0.89
}
'''


class Object3DMapperNode(Node):
    DEPTH_WINDOW_SIZE = 5
    DEPTH_MIN_M = 0.1
    DEPTH_MAX_M = 10.0
    TF_TIMEOUT_SEC = 0.1
    CENTROID_RADIUS = 5
    TEXT_FONT = cv2.FONT_HERSHEY_SIMPLEX
    TEXT_SCALE = 0.45
    TEXT_THICKNESS_OUTLINE = 2
    TEXT_THICKNESS = 1
    TEXT_OFFSET = (-40, -10)
    WHITE = (255, 255, 255)
    GREEN = (0, 255, 0)
    RED = (0, 0, 255)

    def __init__(self):
        super().__init__('object_3d_mapper_node')
        self._bridge = CvBridge()
        self._profiler = Profiler()  # attached in _init_profiling(); all bench logic lives there
        self._load_parameters()
        self._init_model()
        self._init_camera()
        self._init_tf()
        self._init_subscribers()
        self._init_publishers()
        self._init_profiling()
        self.get_logger().info("3D Object Mapper Node Initialized.")

    def _load_parameters(self) -> None:
        self.declare_parameter('model_name', 'yolov8-seg.pt')
        self.declare_parameter('rgb_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter('depth_topic', '/oakd/rgb/preview/depth')
        self.declare_parameter('camera_info_topic', '/oakd/rgb/preview/camera_info')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('json_output_topic', '/perception_map')
        self.declare_parameter('debug_image_topic', '/perception/debug_image')
        self.declare_parameter('enable_profiling', True)
        self.declare_parameter('diagnostics_topic', '/diagnostics')
        self.declare_parameter('csv_path', 'profiling.csv')
        self.declare_parameter('summary_path', 'profiling_summary.json')
        self.declare_parameter('stats_hz', 1.0)

        self._model_name = self.get_parameter('model_name').value
        self._rgb_topic = self.get_parameter('rgb_topic').value
        self._depth_topic = self.get_parameter('depth_topic').value
        self._camera_info_topic = self.get_parameter('camera_info_topic').value
        self._map_frame = self.get_parameter('map_frame').value
        self._json_output_topic = self.get_parameter('json_output_topic').value
        self._debug_image_topic = self.get_parameter('debug_image_topic').value
        self._profiling_enabled = bool(self.get_parameter('enable_profiling').value)
        self._diagnostics_topic = self.get_parameter('diagnostics_topic').value
        self._csv_path = self.get_parameter('csv_path').value
        self._summary_path = self.get_parameter('summary_path').value
        self._stats_hz = float(self.get_parameter('stats_hz').value)

    def _init_model(self) -> None:
        self.model = YOLO(self._model_name)
        self.tracked_objects: dict = {}
        try:
            overhead = self._profiler.mark_model_loaded()
            self.get_logger().info(f"Model loaded. RAM overhead: {overhead:.1f} MB")
        except Exception:
            pass

    def _init_camera(self) -> None:
        self.camera_model = PinholeCameraModel()
        self.has_camera_info = False

    def _init_tf(self) -> None:
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _init_subscribers(self) -> None:
        self.info_sub = self.create_subscription(
            CameraInfo, self._camera_info_topic, self.camera_info_callback, 10
        )
        self.rgb_sub = message_filters.Subscriber(self, Image, self._rgb_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, self._depth_topic)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1
        )
        self.ts.registerCallback(self.perception_callback)

    def _init_publishers(self) -> None:
        self.json_pub = self.create_publisher(String, self._json_output_topic, 10)
        self.debug_pub = self.create_publisher(Image, self._debug_image_topic, 10)

    def _init_profiling(self) -> None:
        # Everything (DiagnosticArray @ stats_hz + CSV + summary JSON) is owned
        # by profiler.py. Launching this node is enough to get all outputs.
        self._profiler.attach(
            node=self,
            enabled=self._profiling_enabled,
            diagnostics_topic=self._diagnostics_topic,
            csv_path=self._csv_path,
            summary_path=self._summary_path,
            stats_hz=self._stats_hz,
            model_name=self._model_name,
        )

    def camera_info_callback(self, info_msg: CameraInfo) -> None:
        if not self.has_camera_info:
            self.camera_model.fromCameraInfo(info_msg)
            self.has_camera_info = True
            self.get_logger().info("Camera Intrinsics Loaded.")

    def _convert_images(self, rgb_msg: Image, depth_msg: Image) -> tuple[np.ndarray, np.ndarray] | None:
        try:
            rgb_image = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_image = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            return rgb_image, depth_image
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return None

    def _get_mask_centroid(self, mask: np.ndarray, image_shape: tuple) -> tuple[int, int] | None:
        mask_uint8 = (mask * 255).astype(np.uint8)
        if mask_uint8.shape != image_shape[:2]:
            mask_uint8 = cv2.resize(
                mask_uint8, (image_shape[1], image_shape[0]), interpolation=cv2.INTER_NEAREST
            )
        moments = cv2.moments(mask_uint8)
        if moments['m00'] == 0:
            return None
        u = int(moments['m10'] / moments['m00'])
        v = int(moments['m01'] / moments['m00'])
        return u, v

    def _get_median_depth(self, depth_image: np.ndarray, u: int, v: int) -> float | None:
        half = self.DEPTH_WINDOW_SIZE // 2
        depth_crop = depth_image[
            max(0, v - half):v + half + 1,
            max(0, u - half):u + half + 1
        ]
        valid_depths = depth_crop[depth_crop > 0]
        if len(valid_depths) == 0:
            return None
        depth_m = float(np.median(valid_depths))
        if depth_image.dtype == np.uint16:
            depth_m /= 1000.0
        if depth_m <= self.DEPTH_MIN_M or depth_m > self.DEPTH_MAX_M:
            return None
        return depth_m

    def _project_to_3d(self, u: int, v: int, depth_m: float) -> tuple[float, float, float]:
        ray = self.camera_model.projectPixelTo3dRay((u, v))
        return ray[0] * depth_m, ray[1] * depth_m, ray[2] * depth_m

    def _transform_to_map(self, x_c: float, y_c: float, z_c: float, header) -> PointStamped | None:
        pt_cam = PointStamped()
        pt_cam.header = header
        pt_cam.point.x = x_c
        pt_cam.point.y = y_c
        pt_cam.point.z = z_c
        try:
            transform = self.tf_buffer.lookup_transform(
                self._map_frame,
                header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.TF_TIMEOUT_SEC)
            )
            return do_transform_point(pt_cam, transform)
        except Exception as e:
            self.get_logger().warn(f"TF Lookup failed ({header.frame_id} -> {self._map_frame}): {e}")
            return None

    def _annotate_debug_frame(self, debug_frame: np.ndarray, u: int, v: int, track_id: int,
                               x_map: float, y_map: float, z_map: float) -> None:
        cv2.circle(debug_frame, (u, v), self.CENTROID_RADIUS, self.RED, -1)
        text = f"ID:{track_id} Map:({x_map},{y_map},{z_map})m"
        cv2.putText(debug_frame, text, (u + self.TEXT_OFFSET[0], v + self.TEXT_OFFSET[1]),
                    self.TEXT_FONT, self.TEXT_SCALE, self.WHITE, self.TEXT_THICKNESS_OUTLINE)
        cv2.putText(debug_frame, text, (u + self.TEXT_OFFSET[0], v + self.TEXT_OFFSET[1]),
                    self.TEXT_FONT, self.TEXT_SCALE, self.GREEN, self.TEXT_THICKNESS)

    def _publish_debug_image(self, debug_frame: np.ndarray, header) -> None:
        try:
            debug_msg = self._bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
            debug_msg.header = header
            self.debug_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish debug image: {repr(e)}")
            self.get_logger().error(traceback.format_exc())

    def _update_tracked_object(self, track_id: int, cls_id: int, conf: float,
                                x_map: float, y_map: float, z_map: float, timestamp: float) -> None:
        class_name = self.model.names[cls_id]
        self.tracked_objects[track_id] = {
            'id': track_id,
            'label': class_name,
            'position': {
                'x': round(x_map, 2),
                'y': round(y_map, 2),
                'z': round(z_map, 2),
            },
            'confidence': round(conf, 4),
        }

    def _save_map_json(self) -> None:
        if self.tracked_objects:
            with open('map.json', 'w+') as f:
                json.dump(self.tracked_objects, f, indent=2)

    def perception_callback(self, rgb_msg: Image, depth_msg: Image) -> None:
        prof = self._profiler if self._profiling_enabled else None
        t_e2e = Profiler.now_ns() if prof else 0
        fps_instant = prof.tick_frame() if prof else 0.0
        stage = {k: 0.0 for k in (
            'convert_ms', 'infer_track_ms', 'mask_centroid_ms',
            'depth_sample_ms', 'project_ms', 'tf_lookup_ms',
            'map_update_ms', 'debug_plot_publish_ms',
        )}
        n_tracks = 0

        if not self.has_camera_info:
            self.get_logger().warn("Waiting for CameraInfo...", throttle_duration_sec=2.0)
            return

        t0 = Profiler.now_ns() if prof else 0
        conversion = self._convert_images(rgb_msg, depth_msg)
        if prof:
            stage['convert_ms'] = Profiler.elapsed_ms(t0, Profiler.now_ns())
        if conversion is None:
            return
        rgb_image, depth_image = conversion

        t0 = Profiler.now_ns() if prof else 0
        results = self.model.track(rgb_image, persist=True, tracker='bytetrack.yaml', verbose=False)[0]
        if prof:
            stage['infer_track_ms'] = Profiler.elapsed_ms(t0, Profiler.now_ns())

        t_dbg = Profiler.now_ns() if prof else 0
        debug_frame = results.plot() if results is not None else rgb_image.copy()

        if (results.boxes is not None and results.boxes.id is not None
                and results.masks is not None):
            track_ids = results.boxes.id.int().cpu().tolist()
            class_ids = results.boxes.cls.int().cpu().tolist()
            confidences = results.boxes.conf.float().cpu().tolist()
            masks = results.masks.data.cpu().numpy()

            timestamp = rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec * 1e-9

            for track_id, cls_id, conf, mask in zip(track_ids, class_ids, confidences, masks):
                t1 = Profiler.now_ns() if prof else 0
                centroid = self._get_mask_centroid(mask, rgb_image.shape)
                if prof:
                    stage['mask_centroid_ms'] += Profiler.elapsed_ms(t1, Profiler.now_ns())
                if centroid is None:
                    continue
                u, v = centroid

                t1 = Profiler.now_ns() if prof else 0
                depth_m = self._get_median_depth(depth_image, u, v)
                if prof:
                    stage['depth_sample_ms'] += Profiler.elapsed_ms(t1, Profiler.now_ns())
                if depth_m is None:
                    if prof:
                        prof.depth_reject_count += 1
                    continue

                t1 = Profiler.now_ns() if prof else 0
                x_c, y_c, z_c = self._project_to_3d(u, v, depth_m)
                if prof:
                    stage['project_ms'] += Profiler.elapsed_ms(t1, Profiler.now_ns())

                t1 = Profiler.now_ns() if prof else 0
                pt_map = self._transform_to_map(x_c, y_c, z_c, rgb_msg.header)
                if prof:
                    stage['tf_lookup_ms'] += Profiler.elapsed_ms(t1, Profiler.now_ns())
                if pt_map is None:
                    if prof:
                        prof.tf_fail_count += 1
                    continue

                x_map, y_map, z_map = pt_map.point.x, pt_map.point.y, pt_map.point.z

                t1 = Profiler.now_ns() if prof else 0
                self._update_tracked_object(track_id, cls_id, conf, x_map, y_map, z_map, timestamp)
                if prof:
                    stage['map_update_ms'] += Profiler.elapsed_ms(t1, Profiler.now_ns())
                self._annotate_debug_frame(debug_frame, u, v, track_id, x_map, y_map, z_map)
                n_tracks += 1

        self._publish_debug_image(debug_frame, rgb_msg.header)
        # Publish map JSON string (existing behaviour) + file.
        try:
            msg = String()
            msg.data = json.dumps(self.tracked_objects)
            self.json_pub.publish(msg)
        except Exception:
            pass
        self._save_map_json()
        if prof:
            stage['debug_plot_publish_ms'] = Profiler.elapsed_ms(t_dbg, Profiler.now_ns())
            total_ms = Profiler.elapsed_ms(t_e2e, Profiler.now_ns())
            for k, v in stage.items():
                prof.record(k, v)
            prof.record('total_e2e_ms', total_ms)
            stamp_ros = rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec * 1e-9
            prof.write_csv_row(stamp_ros, fps_instant, stage, total_ms, n_tracks)

    def _publish_diagnostics(self) -> None:
        # Thin delegate: all DiagnosticArray logic lives in profiler.py.
        self._profiler.publish_diagnostics(live_tracks=len(self.tracked_objects))

    def write_summary(self) -> dict:
        # Thin delegate: summary JSON logic lives in profiler.py.
        return self._profiler.write_summary(
            model=str(self._model_name), live_tracks=len(self.tracked_objects)
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Object3DMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._profiler.shutdown(
                model=str(node._model_name), live_tracks=len(node.tracked_objects)
            )
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()