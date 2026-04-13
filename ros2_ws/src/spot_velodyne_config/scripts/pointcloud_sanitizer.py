#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


class PointCloudSanitizer(Node):
    def __init__(self) -> None:
        super().__init__("mola_pointcloud_sanitizer")
        self.declare_parameter("input_topic", "/velodyne/points")
        self.declare_parameter("output_topic", "/velodyne/points_mola")

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        self._publisher = self.create_publisher(PointCloud2, output_topic, 10)
        self._subscription = self.create_subscription(PointCloud2, input_topic, self._callback, 10)

    def _callback(self, msg: PointCloud2) -> None:
        point_count = msg.width * msg.height
        if point_count == 0 or not msg.data:
            return

        source_dtype = np.dtype(
            {
                "names": ["x", "y", "z", "intensity"],
                "formats": ["<f4", "<f4", "<f4", "<f4"],
                "offsets": [0, 4, 8, 12],
                "itemsize": msg.point_step,
            }
        )
        points = np.frombuffer(msg.data, dtype=source_dtype, count=point_count)

        clean = np.empty(
            point_count,
            dtype=[
                ("x", "<f4"),
                ("y", "<f4"),
                ("z", "<f4"),
                ("intensity", "<f4"),
            ],
        )
        clean["x"] = points["x"]
        clean["y"] = points["y"]
        clean["z"] = points["z"]
        clean["intensity"] = points["intensity"]

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = point_count
        out.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        out.is_bigendian = False
        out.point_step = 16
        out.row_step = out.point_step * out.width
        out.is_dense = msg.is_dense
        out.data = clean.tobytes()
        self._publisher.publish(out)


def main() -> None:
    rclpy.init()
    node = PointCloudSanitizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
