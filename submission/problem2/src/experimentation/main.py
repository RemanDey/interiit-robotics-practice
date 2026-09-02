#!/usr/bin/env python3
"""Unified entry point for semantic mapping pipeline."""

import rclpy
from object_mapping.object_mapping_node import PerceptionSegmentorNode


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