#!/usr/bin/env python3

import json
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import message_filters
from ultralytics import YOLO
from image_geometry import PinholeCameraModel
from cv_bridge import CvBridge

bridge = CvBridge()

def imgmsg_to_cv2_direct(msg: Image) -> np.ndarray:
    """Converts ROS 2 Image to OpenCV image using cv_bridge."""
    try:
        frame = bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )
        return frame
    except Exception as e:
        raise ValueError(f"Image conversion failed: {msg.encoding} - {e}")


class Object3DMapperNode(Node):

    def __init__(self):
        super().__init__('object_3d_mapper_node')

        # Parameters
        self.declare_parameter('model_name', 'yolov8n-seg.pt')
        self.declare_parameter('rgb_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter('depth_topic', '/oakd/rgb/preview/depth')
        self.declare_parameter('camera_info_topic', '/oakd/rgb/preview/camera_info')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('json_output_topic', '/perception_map')
        self.declare_parameter('debug_image_topic', '/perception/debug_image')

        model_name = self.get_parameter('model_name').value
        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        json_output_topic = self.get_parameter('json_output_topic').value
        debug_image_topic = self.get_parameter('debug_image_topic').value

        # YOLO Model & Tracking
        self.model = YOLO(model_name)
        self.tracked_objects = {}  # Store tracked objects by track_id

        # Camera Geometry & TF Buffer
        self.camera_model = PinholeCameraModel()
        self.has_camera_info = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Camera Info Subscriber (Once)
        self.info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, 10
        )

        # Synchronized RGB & Depth Subscribers
        self.rgb_sub = message_filters.Subscriber(self, Image, rgb_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1
        )
        self.ts.registerCallback(self.perception_callback)

        # Publishers
        self.json_pub = self.create_publisher(String, json_output_topic, 10)
        self.debug_pub = self.create_publisher(Image, debug_image_topic, 10)

        self.get_logger().info("3D Object Mapper Node Initialized.")

    def camera_info_callback(self, info_msg: CameraInfo):
        if not self.has_camera_info:
            self.camera_model.fromCameraInfo(info_msg)
            self.has_camera_info = True
            self.get_logger().info("Camera Intrinsics Loaded.")

    def perception_callback(self, rgb_msg: Image, depth_msg: Image):
        if not self.has_camera_info:
            self.get_logger().warn("Waiting for CameraInfo...", throttle_duration_sec=2.0)
            return

        # Convert Image Messages
        try:
            rgb_image = imgmsg_to_cv2_direct(rgb_msg)
            depth_image = imgmsg_to_cv2_direct(depth_msg)
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        # Run ByteTrack Segmentation
        results = self.model.track(
            rgb_image, persist=True, tracker='bytetrack.yaml', verbose=False
        )[0]

        # Base debug frame with segmentation overlays and bounding boxes
        debug_frame = results.plot() if results is not None else rgb_image.copy()

        if results.boxes is not None and results.boxes.id is not None and results.masks is not None:
            track_ids = results.boxes.id.int().cpu().tolist()
            class_ids = results.boxes.cls.int().cpu().tolist()
            confidences = results.boxes.conf.float().cpu().tolist()
            masks = results.masks.data.cpu().numpy()
            
            for track_id, cls_id, conf, mask in zip(track_ids, class_ids, confidences, masks):
                # Compute centroid of mask (u, v)
                mask_uint8 = (mask * 255).astype(np.uint8)
                if mask_uint8.shape != (rgb_image.shape[0], rgb_image.shape[1]):
                    mask_uint8 = cv2.resize(
                        mask_uint8, (rgb_image.shape[1], rgb_image.shape[0]), interpolation=cv2.INTER_NEAREST
                    )

                moments = cv2.moments(mask_uint8)
                if moments['m00'] == 0:
                    continue

                u = int(moments['m10'] / moments['m00'])
                v = int(moments['m01'] / moments['m00'])

                # Sample Median Depth over a 5x5 window around centroid (in meters)
                depth_crop = depth_image[max(0, v - 2):v + 3, max(0, u - 2):u + 3]
                valid_depths = depth_crop[depth_crop > 0]

                if len(valid_depths) == 0:
                    continue

                # Handle 16-bit depth (mm) vs float depth (m)
                depth_m = float(np.median(valid_depths))
                if depth_image.dtype == np.uint16:
                    depth_m /= 1000.0

                if depth_m <= 0.1 or depth_m > 10.0:  # Ignore out-of-range depths
                    continue

                # 3D Ray projection in Camera Optical Frame
                ray = self.camera_model.projectPixelTo3dRay((u, v))
                x_c = ray[0] * depth_m
                y_c = ray[1] * depth_m
                z_c = ray[2] * depth_m

                # Create PointStamped for TF Transform
                pt_cam = PointStamped()
                pt_cam.header = rgb_msg.header
                pt_cam.point.x = x_c
                pt_cam.point.y = y_c
                pt_cam.point.z = z_c

                # Transform Point to Map Frame
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.map_frame,
                        rgb_msg.header.frame_id,
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.1)
                    )
                    pt_map = do_transform_point(pt_cam, transform)
                except Exception as e:
                    self.get_logger().warn(f"TF Lookup failed ({rgb_msg.header.frame_id} -> {self.map_frame}): {e}")
                    continue

                # Update tracked object database
                class_name = self.model.names[cls_id]
                x_map, y_map, z_map = round(pt_map.point.x, 2), round(pt_map.point.y, 2), round(pt_map.point.z, 2)
                
                self.tracked_objects[track_id] = {
                    'id': track_id,
                    'class': class_name,
                    'confidence': round(conf, 4),
                    'position_map': {
                        'x': x_map,
                        'y': y_map,
                        'z': z_map,
                    },
                    'frame_id': self.map_frame,
                    'timestamp': rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec * 1e-9
                }

                # Annotate Debug Frame with Centroid Point & Map Coordinates
                cv2.circle(debug_frame, (u, v), 5, (0, 0, 255), -1)
                text = f"ID:{track_id} Map:({x_map},{y_map},{z_map})m"
                cv2.putText(
                    debug_frame, text, (u - 40, v - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 2
                )
                cv2.putText(
                    debug_frame, text, (u - 40, v - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1
                )

        # Publish debug image
        try:
            debug_msg = bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
            debug_msg.header = rgb_msg.header
            self.debug_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish debug image: {e}")

        # Publish state as JSON
        if self.tracked_objects:
            json_msg = String()
            json_msg.data = json.dumps(list(self.tracked_objects.values()), indent=2)
            self.json_pub.publish(json_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Object3DMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()