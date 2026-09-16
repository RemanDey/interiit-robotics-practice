#!/usr/bin/env python3

import json
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker


class ObjectNavigator(Node):

    def __init__(self):
        super().__init__('object_navigator')

        self.json_file = 'map.json'

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.marker_pub = self.create_publisher(
            Marker,
            '/object_labels',
            10
        )

        self.marker_timer = self.create_timer(
            1.0,
            self.publish_object_labels
        )

    def find_object(self, label):

        try:
            with open(self.json_file, 'r') as f:
                data = json.load(f)

        except FileNotFoundError:
            self.get_logger().error(
                f'JSON file not found: {self.json_file}'
            )
            return None

        except json.JSONDecodeError as e:
            self.get_logger().error(
                f'Invalid JSON: {e}'
            )
            return None

        matches = []

        for object_id, obj in data.items():

            if obj.get('label', '').lower() == label.lower():

                matches.append({
                    'id': object_id,
                    'position': obj['position'],
                    'confidence': obj.get('confidence', 0.0)
                })

        if not matches:
            self.get_logger().error(
                f'No "{label}" found in JSON.'
            )
            return None

        # Highest-confidence detection
        best = max(
            matches,
            key=lambda obj: obj['confidence']
        )

        self.get_logger().info(
            f'Found {label} '
            f'(ID: {best["id"]}, '
            f'confidence: {best["confidence"]:.3f})'
        )

        return best

    def navigate_to_object(self, label):

        obj = self.find_object(label)

        if obj is None:
            return

        x = obj['position']['x']
        y = obj['position']['y']

        self.get_logger().info(
            'Waiting for Nav2...'
        )

        if not self.nav_client.wait_for_server(
                timeout_sec=10.0):

            self.get_logger().error(
                'Nav2 is not available.'
            )
            return

        goal = NavigateToPose.Goal()

        goal.pose = PoseStamped()

        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0

        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = 0.0
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f'Navigating to {label}: '
            f'x={x:.2f}, y={y:.2f}'
        )

        future = self.nav_client.send_goal_async(goal)

        future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:

            self.get_logger().error(
                'Nav2 rejected the goal.'
            )
            return

        self.get_logger().info(
            'Goal accepted by Nav2.'
        )

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.result_callback
        )

    def result_callback(self, future):

        result = future.result()

        if result.status == 4:
            self.get_logger().info(
                'Successfully reached the object!'
            )
        else:
            self.get_logger().warn(
                f'Navigation ended with status '
                f'{result.status}'
            )

    def publish_object_labels(self):

        try:
            with open(self.json_file, 'r') as f:
                data = json.load(f)

        except Exception as e:
            self.get_logger().error(
                f'Could not read JSON: {e}'
            )
            return

        for object_id, obj in data.items():

            position = obj['position']

            marker = Marker()

            marker.header.frame_id = 'map'
            marker.header.stamp = (
                self.get_clock().now().to_msg()
            )

            marker.ns = 'objects'
            marker.id = int(object_id)

            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD

            marker.pose.position.x = position['x']
            marker.pose.position.y = position['y']

            # Slightly above the 2D map plane
            marker.pose.position.z = 0.2

            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.scale.z = 0.55

            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            confidence = obj.get('confidence', 0.0)

            marker.text = (
                f"{obj['label']} "
                f"[{confidence:.2f}]"
            )

            self.marker_pub.publish(marker)


def get_user_input(node):

    label = input(
        '\nEnter object to navigate to: '
    ).strip()

    if not label:
        node.get_logger().error(
            'No object entered.'
        )
        return

    node.navigate_to_object(label)


def main(args=None):

    rclpy.init(args=args)

    node = ObjectNavigator()

    # Run input() separately so ROS callbacks continue running
    input_thread = threading.Thread(
        target=get_user_input,
        args=(node,),
        daemon=True
    )

    input_thread.start()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()