#!/usr/bin/env python3
"""Publish an empty, fresh obstacle cloud for motor-disabled host tests."""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


class SyntheticTrackerPublisher(Node):
    """Test-only fresh sensor source; it contains no physical observations."""

    def __init__(self):
        super().__init__("rs_control_navigation_synthetic_tracker")
        self.publisher = self.create_publisher(
            PointCloud2,
            "/simulation/mecanum/tracker_points",
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(0.05, self.publish_empty_cloud)

    def publish_empty_cloud(self):
        """Publish a valid zero-point cloud in the canonical map frame."""
        message = PointCloud2()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.height = 1
        message.width = 0
        message.fields = []
        for name, offset in (("x", 0), ("y", 4), ("z", 8)):
            field = PointField()
            field.name = name
            field.offset = offset
            field.datatype = PointField.FLOAT32
            field.count = 1
            message.fields.append(field)
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = 0
        message.is_dense = True
        self.publisher.publish(message)


def main():
    """Run until the host-test orchestrator terminates the process."""
    rclpy.init()
    node = SyntheticTrackerPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
