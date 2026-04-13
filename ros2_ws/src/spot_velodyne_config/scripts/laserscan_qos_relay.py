#!/usr/bin/env python3
"""Relay LaserScan from best-effort sensor QoS to reliable slam_toolbox QoS."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class LaserScanQosRelay(Node):
    def __init__(self):
        super().__init__("laserscan_qos_relay")
        self.input_topic = self.declare_parameter("input_topic", "/slam_toolbox/scan_raw").value
        self.output_topic = self.declare_parameter("output_topic", "/slam_toolbox/scan").value
        self.restamp = self.declare_parameter("restamp", True).value
        self.output_frame = self.declare_parameter("output_frame", "slam_toolbox_velodyne").value

        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.durability = DurabilityPolicy.VOLATILE

        reliable_qos = QoSProfile(depth=10)
        reliable_qos.reliability = ReliabilityPolicy.RELIABLE
        reliable_qos.durability = DurabilityPolicy.VOLATILE

        self.publisher = self.create_publisher(LaserScan, self.output_topic, reliable_qos)
        self.subscription = self.create_subscription(LaserScan, self.input_topic, self.callback, sensor_qos)
        self.get_logger().info(
            f"Relaying LaserScan {self.input_topic} -> {self.output_topic} with reliable QoS; "
            f"restamp={self.restamp}; output_frame={self.output_frame}"
        )

    def callback(self, msg):
        if self.restamp:
            msg.header.stamp = self.get_clock().now().to_msg()
        if self.output_frame:
            msg.header.frame_id = self.output_frame
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = LaserScanQosRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
