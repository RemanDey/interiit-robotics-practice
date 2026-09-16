#!/usr/bin/env python3

import json
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class ObjectNavigator(Node):

    def __init__(self):
        super().__init__('object_navigator')

        self.declare_parameter(
            'json_file',
            '/home/deimos/objects.json'
        )

        self.json_file = self.get_parameter(
            'json_file'
        ).get_parameter_value().string_value

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.get_logger().info(
            f'Using object database: {self.json_file}'
        )

    def load_objects(self):
        try:
            with open(self.json_file, 'r') as f:
                return json.load(f)

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

    def find_object(self, label):
        data = self.load_objects()

        if data is None:
            return None

        matches = []

        for object_id, obj in data.items():

            if obj.get('label', '').lower() == label.lower():

                matches.append({
                    'id': object_id,
                    'label': obj['label'],
                    'position': obj['position_map'],
                    'confidence': obj.get('confidence', 0.0)
                })

        if not matches:
            self.get_logger().error(
                f'No object found with label "{label}"'
            )
            return None

        # Select the highest-confidence detection
        best = max(
            matches,
            key=lambda obj: obj['confidence']
        )

        self.get_logger().info(
            f'Found {label}: '
            f'id={best["id"]}, '
            f'confidence={best["confidence"]:.3f}, '
            f'position={best["position"]}'
        )

        return best

    def send_goal(self, label):

        obj = self.find_object(label)

        if obj is None:
            return

        x = obj['position']['x']
        y = obj['position']['y']
        z = obj['position'].get('z', 0.0)

        # Wait for Nav2
        self.get_logger().info(
            'Waiting for Nav2...'
        )

        if not self.nav_client.wait_for_server(
                timeout_sec=10.0):

            self.get_logger().error(
                'Nav2 action server not available!'
            )
            return

        # Create Nav2 goal
        goal_msg = NavigateToPose.Goal()

        goal_msg.pose = PoseStamped()

        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # Nav2 operates in 2D, so orientation is initially
        # set to zero yaw.
        goal_msg.pose.pose.position.z = 0.0

        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f'Sending Nav2 goal for "{label}"'
        )

        self.get_logger().info(
            f'Goal: x={x:.2f}, y={y:.2f}'
        )

        send_future = self.nav_client.send_goal_async(
            goal_msg
        )

        send_future.add_done_callback(
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
            'Nav2 goal accepted.'
        )

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.result_callback
        )

    def result_callback(self, future):

        result = future.result()

        status = result.status

        if status == 4:
            self.get_logger().info(
                'Robot reached the object!'
            )
        else:
            self.get_logger().warn(
                f'Navigation finished with status: {status}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = ObjectNavigator()

    # Get object label from command line
    if len(sys.argv) < 2:

        node.get_logger().error(
            'Usage: ros2 run <package> object_navigator chair'
        )

        node.destroy_node()
        rclpy.shutdown()
        return

    label = sys.argv[1]

    node.send_goal(label)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()