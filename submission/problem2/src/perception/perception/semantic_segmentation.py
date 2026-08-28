#!/usr/bin/env python3

import json
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO


def imgmsg_to_cv2_direct(msg: Image) -> np.ndarray:
    """Directly converts a ROS 2 Image message to an OpenCV BGR numpy array

    bypassing cv_bridge to prevent NumPy 2.x ABI issues.
    """
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


def cv2_to_imgmsg_direct(cv_image: np.ndarray, header, encoding: str = 'bgr8') -> Image:
    """Directly converts an OpenCV image array to a ROS 2 Image message

    bypassing cv_bridge to prevent NumPy 2.x ABI issues (KeyError: 16).
    """
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
        self.declare_parameter('perception_topic', '/perception')
        self.declare_parameter('mask_topic', '/perception_mask')
        self.declare_parameter('classification_topic', '/perception_classification')

        model_name = (
            self.get_parameter('model_name').get_parameter_value().string_value
        )
        image_topic = (
            self.get_parameter('image_topic').get_parameter_value().string_value
        )
        perception_topic = (
            self.get_parameter('perception_topic').get_parameter_value().string_value
        )
        mask_topic = (
            self.get_parameter('mask_topic').get_parameter_value().string_value
        )
        classification_topic = (
            self.get_parameter('classification_topic').get_parameter_value().string_value
        )

        # Load YOLO model
        self.get_logger().info(f'Loading YOLO model: {model_name}...')
        self.model = YOLO(model_name)

        # Track seen object IDs across all frames
        self.seen_track_ids = set()

        # ROS 2 Subscriptions & Publishers
        self.subscription = self.create_subscription(
            Image, image_topic, self.image_callback, 10
        )
        self.json_publisher_ = self.create_publisher(
            String, perception_topic, 10
        )
        self.mask_publisher_ = self.create_publisher(Image, mask_topic, 10)
        self.classification_publisher_ = self.create_publisher(
            Image, classification_topic, 10
        )

        self.get_logger().info(
            f'Perception node initialized.\n'
            f' - Input: {image_topic}\n'
            f' - JSON Output: {perception_topic}\n'
            f' - Mask Output: {mask_topic}\n'
            f' - Bounding Box Output: {classification_topic}'
        )

    def image_callback(self, msg: Image):
        try:
            # Convert ROS Image message to OpenCV BGR array directly
            frame = imgmsg_to_cv2_direct(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to convert ROS image: {e}')
            return

        # Run ByteTrack algorithm using YOLO segmentor model
        results = self.model.track(
            frame, persist=True, tracker='bytetrack.yaml', verbose=False
        )[0]

        # -------------------------------------------------------------
        # 1. Process & Publish Bounded Box Image (/perception_classification)
        # -------------------------------------------------------------
        annotated_frame = results.plot()
        annotated_frame_bgr = np.ascontiguousarray(annotated_frame, dtype=np.uint8)

        try:
            bbox_msg = cv2_to_imgmsg_direct(annotated_frame_bgr, msg.header, encoding='bgr8')
            self.classification_publisher_.publish(bbox_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish classification image: {e}')

        # -------------------------------------------------------------
        # 2. Process & Publish Mask Image Topic (/perception_mask)
        # -------------------------------------------------------------
        mask_overlay = np.zeros_like(frame)

        if results.masks is not None:
            combined_mask = (
                results.masks.data.any(dim=0).cpu().numpy().astype(np.uint8)
            )

            if combined_mask.shape[:2] != frame.shape[:2]:
                combined_mask = cv2.resize(
                    combined_mask,
                    (frame.shape[1], frame.shape[0]),
                    interpolation=cv2.INTER_NEAREST,
                )

            mask_overlay = cv2.bitwise_and(frame, frame, mask=combined_mask)

        try:
            mask_msg = cv2_to_imgmsg_direct(mask_overlay, msg.header, encoding='bgr8')
            self.mask_publisher_.publish(mask_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish mask image: {e}')

        # -------------------------------------------------------------
        # 3. Process & Publish New Objects JSON Topic (/perception)
        # -------------------------------------------------------------
        new_objects = []

        if results.boxes is not None and results.boxes.id is not None:
            track_ids = results.boxes.id.int().cpu().tolist()
            class_ids = results.boxes.cls.int().cpu().tolist()
            confidences = results.boxes.conf.float().cpu().tolist()

            for track_id, cls_id, conf in zip(
                track_ids, class_ids, confidences
            ):
                if track_id not in self.seen_track_ids:
                    self.seen_track_ids.add(track_id)

                    class_name = self.model.names[cls_id]

                    new_objects.append({
                        'id': track_id,
                        'class': class_name,
                        'confidence': round(conf, 4),
                        'total_unique_seen': len(self.seen_track_ids),
                    })

        if new_objects:
            json_msg = String()
            json_msg.data = json.dumps(new_objects)
            self.json_publisher_.publish(json_msg)
            self.get_logger().info(
                f'Published {len(new_objects)} new object(s) to perception topic'
            )


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