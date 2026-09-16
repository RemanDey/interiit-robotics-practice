#!/usr/bin/env python3

import json
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


class Object3DMapperNode(Node):

    def __init__(self):
        super().__init__('object_3d_mapper_node')
        self.model_name = self.declare_parameter(
            'model_name', 'yolov8n-seg.pt'
        ).value

        self.rgb_topic = self.declare_parameter(
            'rgb_topic', '/oakd/rgb/preview/image_raw'
        ).value

        self.depth_topic = self.declare_parameter(
            'depth_topic', '/oakd/rgb/preview/depth'
        ).value

        self.camera_info_topic = self.declare_parameter(
            'camera_info_topic', '/oakd/rgb/preview/camera_info'
        ).value

        self.map_frame = self.declare_parameter(
            'map_frame', 'map'
        ).value

        self.json_topic = self.declare_parameter(
            'json_output_topic', '/perception_map'
        ).value

        self.debug_topic = self.declare_parameter(
            'debug_image_topic', '/perception/debug_image'
        ).value


        self.bridge = CvBridge()
        self.model = YOLO(self.model_name)

        self.camera = PinholeCameraModel()
        self.camera_ready = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self
        )

        self.objects = {}

        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )

        rgb = message_filters.Subscriber(
            self, Image, self.rgb_topic
        )

        depth = message_filters.Subscriber(
            self, Image, self.depth_topic
        )

        sync = message_filters.ApproximateTimeSynchronizer(
            [rgb, depth],
            queue_size=10,
            slop=0.1
        )
        sync.registerCallback(self.perception_callback)
        self.json_pub = self.create_publisher(
            String,
            self.json_topic,
            10
        )

        self.debug_pub = self.create_publisher(
            Image,
            self.debug_topic,
            10
        )
        self.get_logger().info(
            "3D Object Mapper Node Started"
        )
    def camera_info_callback(self, msg):
        if not self.camera_ready:
            self.camera.fromCameraInfo(msg)
            self.camera_ready = True
            self.get_logger().info(
                "Camera intrinsics loaded"
            )
    def perception_callback(self, rgb_msg, depth_msg):
        if not self.camera_ready:
            return
        try:
            rgb = self.bridge.imgmsg_to_cv2(
                rgb_msg,
                desired_encoding='bgr8'
            )

            depth = self.bridge.imgmsg_to_cv2(
                depth_msg,
                desired_encoding='passthrough'
            )

        except Exception as e:
            self.get_logger().error(
                f"Image conversion failed: {e}"
            )
            return

        result = self.model.track(
            rgb,
            persist=True,
            tracker='bytetrack.yaml',
            verbose=False
        )[0]

        debug = result.plot()
        if (
            result.boxes is None
            or result.boxes.id is None
            or result.masks is None
        ):
            self.publish_debug(debug, rgb_msg.header)
            return
        ids = result.boxes.id.int().cpu().tolist()
        classes = result.boxes.cls.int().cpu().tolist()
        confidences = result.boxes.conf.cpu().tolist()
        masks = result.masks.data.cpu().numpy()
        for track_id, cls_id, confidence, mask in zip(ids, classes, confidences, masks):
            mask = cv2.resize(
                mask.astype(np.uint8),
                (rgb.shape[1], rgb.shape[0]),
                interpolation=cv2.INTER_NEAREST
            )
            M = cv2.moments(mask)
            if M['m00'] == 0:
                continue
            u = int(M['m10'] / M['m00'])
            v = int(M['m01'] / M['m00'])
            window = depth[
                max(0, v - 2):v + 3,
                max(0, u - 2):u + 3
            ]
            valid = window[window > 0]
            if len(valid) == 0:
                continue
            depth_m = float(np.median(valid))
            if depth.dtype == np.uint16:
                depth_m /= 1000.0

            if depth_m <= 0.1 or depth_m > 10.0:
                continue
            ray = self.camera.projectPixelTo3dRay(
                (u, v)
            )

            x = ray[0] * depth_m
            y = ray[1] * depth_m
            z = ray[2] * depth_m


            point = PointStamped()
            point.header = rgb_msg.header
            point.point.x = x
            point.point.y = y
            point.point.z = z

            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    rgb_msg.header.frame_id,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.1)
                )

                point = do_transform_point(
                    point,
                    transform
                )

            except Exception:
                continue


            name = self.model.names[cls_id]

            self.objects[track_id] = {
                'id': track_id,
                'label': name,
                'position': {
                    'x': round(point.point.x, 2),
                    'y': round(point.point.y, 2),
                    'z': round(point.point.z, 2)
                },
                'confidence': round(
                    float(confidence), 4
                )
            }
            cv2.circle(
                debug,
                (u, v),
                5,
                (0, 0, 255),
                -1
            )

            text = (
                f"ID:{track_id} "
                f"Map:({point.point.x:.2f},"
                f"{point.point.y:.2f},"
                f"{point.point.z:.2f})m"
            )

            cv2.putText(
                debug,
                text,
                (u - 40, v - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 0, 0),
                2
            )

            cv2.putText(
                debug,
                text,
                (u - 40, v - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 255, 0),
                1
            )

        self.publish_debug(
            debug,
            rgb_msg.header
        )
        msg = String()
        msg.data = json.dumps(
            self.objects,
            indent=2
        )

        self.json_pub.publish(msg)

        with open('map.json', 'w') as f:
            json.dump(
                self.objects,
                f,
                indent=2
            )


    def publish_debug(self, image, header):
        msg = self.bridge.cv2_to_imgmsg(
            image,
            encoding='bgr8'
        )
        msg.header = header
        self.debug_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = Object3DMapperNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

