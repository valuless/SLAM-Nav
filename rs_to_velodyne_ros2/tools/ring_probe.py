#!/usr/bin/env python3
"""Report ring values from one PointCloud2 frame."""

import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


DTYPE_FORMAT = {
    PointField.INT8: ("b", 1),
    PointField.UINT8: ("B", 1),
    PointField.INT16: ("h", 2),
    PointField.UINT16: ("H", 2),
    PointField.INT32: ("i", 4),
    PointField.UINT32: ("I", 4),
    PointField.FLOAT32: ("f", 4),
    PointField.FLOAT64: ("d", 8),
}


class RingProbe(Node):
    def __init__(self):
        super().__init__("ring_probe")
        self.subscription = self.create_subscription(
            PointCloud2, "/rslidar_points", self.on_cloud,
            qos_profile_sensor_data)

    def finish(self, message):
        self.get_logger().info(message)
        rclpy.shutdown()

    def on_cloud(self, msg):
        field = next((item for item in msg.fields if item.name == "ring"), None)
        if field is None:
            self.finish("no ring field")
            return
        if field.count != 1:
            self.finish(f"ring count must be 1, got {field.count}")
            return

        format_info = DTYPE_FORMAT.get(field.datatype)
        if format_info is None:
            self.finish(f"unsupported ring datatype {field.datatype}")
            return
        format_char, field_size = format_info
        if msg.point_step == 0 or field.offset + field_size > msg.point_step:
            self.finish("ring field exceeds point_step")
            return
        minimum_row_step = msg.width * msg.point_step
        if msg.row_step < minimum_row_step:
            self.finish("row_step is smaller than width * point_step")
            return
        required_size = msg.row_step * msg.height
        if len(msg.data) < required_size:
            self.finish(
                f"data is truncated: have {len(msg.data)}, need {required_size}")
            return

        unpack = struct.Struct((">" if msg.is_bigendian else "<") + format_char)
        values = set()
        invalid = 0
        for row in range(msg.height):
            row_offset = row * msg.row_step
            for col in range(msg.width):
                offset = row_offset + col * msg.point_step + field.offset
                value = unpack.unpack_from(msg.data, offset)[0]
                if isinstance(value, float):
                    if not math.isfinite(value) or value < 0 or value > 65535:
                        invalid += 1
                        continue
                    integer = int(value)
                    if integer != value:
                        invalid += 1
                        continue
                    value = integer
                elif value < 0 or value > 65535:
                    invalid += 1
                    continue
                values.add(value)

        if not values:
            self.finish(f"no valid ring values; invalid={invalid}")
            return
        self.finish(
            f"ring datatype={field.datatype} is_bigendian={msg.is_bigendian}; "
            f"min={min(values)} max={max(values)} unique={len(values)} "
            f"invalid={invalid}")


def main():
    rclpy.init()
    node = RingProbe()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
