#!/usr/bin/env python3

import struct
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def load_pcd_binary(filepath):
    """Parse binary PCD file, return Nx3 float32 numpy array (xyz only)."""
    with open(filepath, 'rb') as f:
        num_points = 0
        num_fields = 0
        field_sizes = []
        while True:
            line = f.readline().decode('ascii', errors='ignore').strip()
            if line.startswith('FIELDS'):
                num_fields = len(line.split()) - 1
            elif line.startswith('SIZE'):
                field_sizes = [int(s) for s in line.split()[1:]]
            elif line.startswith('POINTS'):
                num_points = int(line.split()[1])
            elif line.startswith('DATA'):
                break
        if num_points == 0 or not field_sizes:
            return np.zeros((0, 3), dtype=np.float32)
        point_size = sum(field_sizes)
        raw = f.read(num_points * point_size)
    # Parse all points, extract first 3 floats (x, y, z)
    all_data = np.frombuffer(raw, dtype=np.uint8).reshape(num_points, point_size)
    xyz = np.zeros((num_points, 3), dtype=np.float32)
    for i in range(3):
        offset = sum(field_sizes[:i])
        xyz[:, i] = np.frombuffer(
            all_data[:, offset:offset + 4].tobytes(), dtype=np.float32
        )
    return xyz


class MapPublisherNode(Node):
    def __init__(self):
        super().__init__('map_publisher')
        self.declare_parameter('map_file_path', '')
        self.declare_parameter('interval', 5)
        path = self.get_parameter('map_file_path').value
        interval = self.get_parameter('interval').value

        self.points = None
        self.cloud_msg = None
        self.get_logger().info(f'map_file_path = "{path}"')
        if path:
            try:
                self.points = load_pcd_binary(path)
                self.get_logger().info(f'Loaded {len(self.points)} points from: {path}')
                if len(self.points) > 0:
                    self._build_cloud_msg()
            except Exception as e:
                self.get_logger().error(f'Failed to load PCD: {e}')
        else:
            self.get_logger().warn('No map_file_path provided; map not loaded')

        self.pub_map = self.create_publisher(
            PointCloud2, '/global_map',
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            )
        )
        self.create_timer(interval, self.publish_map)
        self.get_logger().info(f'Map Publisher Node Initialized (interval={interval}s)')

    def _build_cloud_msg(self):
        """Pre-build the PointCloud2 message once to avoid repeated conversion."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        point_step = 12
        row_step = point_step * len(self.points)
        self.cloud_msg = PointCloud2()
        self.cloud_msg.header = header
        self.cloud_msg.height = 1
        self.cloud_msg.width = len(self.points)
        self.cloud_msg.fields = fields
        self.cloud_msg.is_bigendian = False
        self.cloud_msg.point_step = point_step
        self.cloud_msg.row_step = row_step
        self.cloud_msg.data = self.points.astype(np.float32).tobytes()
        self.cloud_msg.is_dense = True

    def publish_map(self):
        if self.cloud_msg is None:
            self.get_logger().warn('No map loaded; skipping publish')
            return
        self.cloud_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_map.publish(self.cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()        