import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker


class MarkerTest(Node):

    def __init__(self):
        super().__init__('marker_test')

        self.pub = self.create_publisher(
            Marker,
            '/object_labels',
            10
        )

        self.timer = self.create_timer(
            1.0,
            self.publish_marker
        )

    def publish_marker(self):

        marker = Marker()

        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'test'
        marker.id = 1

        # SPHERE instead of text
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        marker.pose.orientation.w = 1.0

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime.sec = 0

        self.pub.publish(marker)


def main():

    rclpy.init()

    node = MarkerTest()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()