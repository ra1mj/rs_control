#!/usr/bin/env python3
"""Publish a fresh, non-obstructing sensor cloud for motor-disabled host tests."""

import struct

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster


class SyntheticTrackerPublisher(Node):
    """Test-only lidar source matching the deployed Nav2 sensor-frame contract."""

    def __init__(self):
        super().__init__("rs_control_navigation_synthetic_tracker")
        self.publisher = self.create_publisher(
            PointCloud2,
            "/sensors/lidar/points",
            qos_profile_sensor_data,
        )
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_sensor_transform()
        self.timer = self.create_timer(0.05, self.publish_sensor_cloud)

    def publish_sensor_transform(self):
        """Provide the static base-to-lidar link required by the costmap filter."""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "base_link"
        transform.child_frame_id = "lidar_link"
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)

    def publish_sensor_cloud(self):
        """Publish one range-filtered point so Nav2 records fresh sensor input."""
        message = PointCloud2()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "lidar_link"
        message.height = 1
        message.width = 1
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
        message.row_step = 12
        # The configured local/global obstacle ranges are at most 8 m.  Keeping
        # this point at 20 m proves the sensor path without adding an obstacle.
        message.data = struct.pack("<fff", 20.0, 0.0, 0.0)
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
