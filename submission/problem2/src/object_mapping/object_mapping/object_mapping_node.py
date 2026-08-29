#!/usr/bin/env python3

import json
import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from ultralytics import YOLO
import message_filters


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
        # Depth in millimeters -> convert to meters
        depth = np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width))
        return depth.astype(np.float32) / 1000.0
    elif msg.encoding == '32FC1':
        # Depth already in meters
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


class PerceptionSegmentorNode(Node):

    def __init__(self):
        super().__init__('perception_segmentor_node')

        # Parameters
        self.declare_parameter('model_name', 'yolov8n-seg.pt')
        self.declare_parameter('image_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter('depth_topic', '/oakd/stereo/image_raw')
        self.declare_parameter('camera_info_topic', '/oakd/rgb/preview/camera_info')
        self.declare_parameter('perception_topic', '/perception')
        self.declare_parameter('mask_topic', '/perception_mask')
        self.declare_parameter('classification_topic', '/perception_classification')
        self.declare_parameter('json_output_path', 'detected_objects_3d.json')

        model_name = self.get_parameter('model_name').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        perception_topic = self.get_parameter('perception_topic').get_parameter_value().string_value
        mask_topic = self.get_parameter('mask_topic').get_parameter_value().string_value
        classification_topic = self.get_parameter('classification_topic').get_parameter_value().string_value
        self.json_output_path = self.get_parameter('json_output_path').get_parameter_value().string_value

        # Load YOLO model
        self.get_logger().info(f'Loading YOLO model: {model_name}...')
        self.model = YOLO(model_name)

        # Track seen object IDs across all frames
        self.seen_track_ids = set()

        # Camera Intrinsics Matrix K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.intrinsics = None
        self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 10)

        # Synchronized RGB and Depth Subscriptions
        self.rgb_sub = message_filters.Subscriber(self, Image, image_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=10, slop=0.05
        )
        self.ts.registerCallback(self.synchronized_callback)

        # ROS 2 Publishers
        self.json_publisher_ = self.create_publisher(String, perception_topic, 10)
        self.mask_publisher_ = self.create_publisher(Image, mask_topic, 10)
        self.classification_publisher_ = self.create_publisher(Image, classification_topic, 10)

        self.get_logger().info(
            f'Perception node initialized.\n'
            f' - RGB Input: {image_topic}\n'
            f' - Depth Input: {depth_topic}\n'
            f' - Camera Info: {camera_info_topic}\n'
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
        if self.intrinsics is None:
            self.get_logger().warn_throttle(2.0, 'Waiting for CameraInfo intrinsics...')
            return

        try:
            frame = imgmsg_to_cv2_direct(rgb_msg)
            depth_map = depthmsg_to_cv2_direct(depth_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to convert ROS images: {e}')
            return

        # Run ByteTrack tracking with YOLO segmentor
        results = self.model.track(
            frame, persist=True, tracker='bytetrack.yaml', verbose=False
        )[0]

        # -------------------------------------------------------------
        # 1. Process & Publish Annotated Bounding Box Image
        # -------------------------------------------------------------
        annotated_frame = results.plot()
        annotated_frame_bgr = np.ascontiguousarray(annotated_frame, dtype=np.uint8)

        try:
            bbox_msg = cv2_to_imgmsg_direct(annotated_frame_bgr, rgb_msg.header, encoding='bgr8')
            self.classification_publisher_.publish(bbox_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish classification image: {e}')

        # -------------------------------------------------------------
        # 2. Process & Publish Mask Image
        # -------------------------------------------------------------
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

        # -------------------------------------------------------------
        # 3. Process Instance Depth & Calculate 3D Coordinates
        # -------------------------------------------------------------
        new_objects = []

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
                # Extract object-specific binary mask
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

                # Sample valid depth values within the object mask
                mask_depths = depth_map[mask_bool]
                valid_depths = mask_depths[(mask_depths > 0) & (~np.isnan(mask_depths))]

                if len(valid_depths) == 0:
                    continue

                # Median depth (robust against noisy edge values)
                z_dist = float(np.median(valid_depths))

                # Calculate 2D centroid of mask
                v_indices, u_indices = np.where(mask_bool)
                u_center = float(np.mean(u_indices))
                v_center = float(np.mean(v_indices))

                # Deproject 2D centroid to 3D camera frame coordinates (X, Y, Z)
                x_coord = (u_center - self.intrinsics['cx']) * z_dist / self.intrinsics['fx']
                y_coord = (v_center - self.intrinsics['cy']) * z_dist / self.intrinsics['fy']

                class_name = self.model.names[cls_id]

                object_entry = {
                    'track_id': track_id,
                    'class': class_name,
                    'confidence': round(conf, 4),
                    'distance_m': round(z_dist, 3),
                    'centroid_2d': [round(u_center, 1), round(v_center, 1)],
                    'coordinates_3d': {
                        'x': round(x_coord, 3),
                        'y': round(y_coord, 3),
                        'z': round(z_dist, 3)
                    },
                    'timestamp': rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec * 1e-9
                }

                # Publish each unique detection once (or continuously if preferred)
                if track_id not in self.seen_track_ids:
                    self.seen_track_ids.add(track_id)
                    object_entry['total_unique_seen'] = len(self.seen_track_ids)
                    new_objects.append(object_entry)

        if new_objects:
            # Publish JSON string to ROS 2 topic
            json_msg = String()
            json_msg.data = json.dumps(new_objects)
            self.json_publisher_.publish(json_msg)

            # Write detections to local file disk
            self.append_to_json_file(new_objects)

            self.get_logger().info(
                f'Published & Saved {len(new_objects)} new object(s) with 3D coordinates.'
            )

    def append_to_json_file(self, data_list):
        """Appends spatial detection records to a local JSON file."""
        file_path = self.json_output_path
        existing_data = []

        if os.path.exists(file_path):
            try:
                with open(file_path, 'r') as f:
                    existing_data = json.load(f)
            except (json.JSONDecodeError, FileNotFoundError):
                existing_data = []

        existing_data.extend(data_list)

        with open(file_path, 'w') as f:
            json.dump(existing_data, f, indent=4)


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