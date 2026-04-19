#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import argparse
import tf_transformations
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped

class PublishInitialPose(Node):
    def __init__(self, x, y, z, yaw, pitch, roll):
        super().__init__('publish_initial_pose')
        self.x, self.y, self.z = x, y, z
        self.yaw, self.pitch, self.roll = yaw, pitch, roll
        self.max_attempts = 10       # 最多尝试 10 次
        self.attempt = 0
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        # 每 2 秒发布一次, 直到 global_localization 初始化成功或达到最大次数
        self.timer_ = self.create_timer(2.0, self._publish_callback)

    def _publish_callback(self):
        self.attempt += 1
        self.publish_pose(self.x, self.y, self.z, self.yaw, self.pitch, self.roll)
        if self.attempt >= self.max_attempts:
            self.get_logger().warn(f'已发布 {self.max_attempts} 次初始位姿, 停止重试')
            raise SystemExit

    def publish_pose(self, x, y, z, yaw, pitch, roll):
        quat = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.pose.pose.position = Point(x=float(x), y=float(y), z=float(z))
        pose_msg.pose.pose.orientation = Quaternion(x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3]))
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        self.publisher_.publish(pose_msg)
        self.get_logger().info(f'[{self.attempt}/{self.max_attempts}] Initial Pose published: x={x}, y={y}, z={z}, yaw={yaw}')

def main(args=None):
    rclpy.init(args=args)

    # Support both command-line args and ROS params
    import sys
    # Filter out ROS args (starting with - or containing :=)
    cli_args = [a for a in sys.argv[1:] if not a.startswith('-') and ':=' not in a]

    if len(cli_args) >= 6:
        x, y, z = float(cli_args[0]), float(cli_args[1]), float(cli_args[2])
        yaw, pitch, roll = float(cli_args[3]), float(cli_args[4]), float(cli_args[5])
    else:
        x, y, z, yaw, pitch, roll = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

    node = PublishInitialPose(x, y, z, yaw, pitch, roll)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
