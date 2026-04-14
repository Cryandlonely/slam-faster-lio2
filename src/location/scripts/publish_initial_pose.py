#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import argparse
import tf_transformations
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped

class PublishInitialPose(Node):
    def __init__(self, x, y, z, yaw, pitch, roll):
        super().__init__('publish_initial_pose')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        # Wait a moment for publisher to be ready
        self.create_timer(1.0, lambda: self._publish_and_shutdown(x, y, z, yaw, pitch, roll))

    def _publish_and_shutdown(self, x, y, z, yaw, pitch, roll):
        self.publish_pose(x, y, z, yaw, pitch, roll)
        raise SystemExit

    def publish_pose(self, x, y, z, yaw, pitch, roll):
        # Roll, pitch, yaw 순서에 따라 쿼터니언 계산 (ROS1과 동일한 tf_transformations 사용)
        quat = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.pose.pose.position = Point(x=float(x), y=float(y), z=float(z))
        pose_msg.pose.pose.orientation = Quaternion(x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3]))
        # 현재 노드의 clock을 이용하여 헤더 타임스탬프 설정
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        self.publisher_.publish(pose_msg)
        self.get_logger().info(f'Initial Pose published: x={x}, y={y}, z={z}, yaw={yaw}, pitch={pitch}, roll={roll}')

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
