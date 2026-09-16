#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage

class TimeCorrector(Node):
    def __init__(self):
        super().__init__('time_corrector')
        # 订阅原始数据
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 100)
        self.tf_sub = self.create_subscription(TFMessage, '/tf', self.tf_cb, 100)
        self.cloud_sub = self.create_subscription(PointCloud2, '/cloud_registered', self.cloud_cb, 100)

        # 发布修正时间戳后的数据
        self.odom_pub = self.create_publisher(Odometry, '/odom_fixed', 100)
        self.tf_pub = self.create_publisher(TFMessage, '/tf_fixed', 100)
        self.cloud_pub = self.create_publisher(PointCloud2, '/cloud_registered_fixed', 100)

        # 防止回写/tf时产生无限循环：记录最近发布的消息序号
        self._last_tf_seq = None
        self._last_odom_seq = None
        self._last_cloud_seq = None

        self.get_logger().info('Time Corrector Started - Publishing /odom_fixed, /tf_fixed, /cloud_registered_fixed')

    def odom_cb(self, msg):
        # 去重：如果消息内容未变（来自我们自己的回写），跳过
        if msg.header.frame_id == 'odom_fixed':
            return
        msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_pub.publish(msg)

    def tf_cb(self, msg):
        # 用变换内容做去重key，避免将自身发布的消息再次处理
        content_key = tuple(
            (t.header.frame_id, t.child_frame_id,
             round(t.transform.translation.x, 6),
             round(t.transform.translation.y, 6),
             round(t.transform.translation.z, 6),
             round(t.transform.rotation.x, 6),
             round(t.transform.rotation.y, 6),
             round(t.transform.rotation.z, 6),
             round(t.transform.rotation.w, 6))
            for t in msg.transforms
        )
        if content_key == self._last_tf_seq:
            return
        self._last_tf_seq = content_key

        for t in msg.transforms:
            t.header.stamp = self.get_clock().now().to_msg()
        self.tf_pub.publish(msg)

    def cloud_cb(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.cloud_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TimeCorrector()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
