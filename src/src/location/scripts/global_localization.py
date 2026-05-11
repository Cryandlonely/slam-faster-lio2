#!/usr/bin/env python3
# coding=utf-8

import copy
import threading
import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
import tf_transformations

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header


class GlobalLocalizationNode(Node):
    def __init__(self):
        super().__init__('location')

        # ─── Parameters ───────────────────────────────────────────
        self.declare_parameter('map_voxel_size', 0.1)
        self.declare_parameter('scan_voxel_size', 0.1)
        self.declare_parameter('freq_localization', 0.5)      # Hz
        self.declare_parameter('localization_th', 0.4)
        self.declare_parameter('fov', 2 * np.pi)
        self.declare_parameter('fov_far', 100.0)
        self.declare_parameter('enable_global_search', True)   # 是否启用全局搜索定位
        self.declare_parameter('search_step', 3.0)             # 搜索网格步长 (m)

        self.map_voxel_size    = self.get_parameter('map_voxel_size').value
        self.scan_voxel_size   = self.get_parameter('scan_voxel_size').value
        self.freq_localization = self.get_parameter('freq_localization').value
        self.localization_th   = self.get_parameter('localization_th').value
        self.FOV               = self.get_parameter('fov').value
        self.FOV_FAR           = self.get_parameter('fov_far').value
        self.enable_global_search = self.get_parameter('enable_global_search').value
        self.search_step          = self.get_parameter('search_step').value

        # ─── State Variables ─────────────────────────────────────
        self.global_map    = None
        self.initialized   = False
        self.T_map_to_odom = np.eye(4)
        self.cur_odom      = None
        self.cur_scan      = None

        # ─── Publishers ──────────────────────────────────────────
        self.pub_pc_in_map   = self.create_publisher(PointCloud2, '/cur_scan_in_map', 1)
        self.pub_submap      = self.create_publisher(PointCloud2, '/submap', 1)
        self.pub_map_to_odom = self.create_publisher(Odometry,     '/map_to_odom', 1)

        # ─── Subscriptions ───────────────────────────────────────
        self.create_subscription(PointCloud2,                  '/cloud_registered', self.cb_save_cur_scan, 1)
        self.create_subscription(Odometry,                    '/Odometry',         self.cb_save_cur_odom,  1)
        self._map_sub  = self.create_subscription(PointCloud2, '/global_map',               self.cb_init_map,      1)
        self._init_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.cb_init_pose,
            1
        )

        self.get_logger().info('GlobalLocalizationNode initialized.')

    def pc2_to_array(self, pc_msg: PointCloud2) -> np.ndarray:
        """PointCloud2 → (N×3) NumPy array"""
        pts = []
        for x, y, z in pc2.read_points(pc_msg, field_names=('x','y','z'), skip_nans=True):
            pts.append((x, y, z))
        return np.array(pts, dtype=np.float32)

    def cb_init_map(self, msg: PointCloud2):
        pts = self.pc2_to_array(msg)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        self.global_map = self.voxel_down_sample(pcd, self.map_voxel_size)
        self.get_logger().info('Global map received and downsampled.')
        self.destroy_subscription(self._map_sub)

    def cb_init_pose(self, msg: PoseWithCovarianceStamped):
        if self.global_map is None:
            self.get_logger().warn('等待全局地图加载...')
            return
        if self.initialized:
            return
        if self.cur_scan is None:
            self.get_logger().warn('等待第一帧点云...')
            return
        if self.cur_odom is None:
            self.get_logger().warn('等待SLAM里程计数据...')
            return

        initial = self.pose_to_mat(msg)

        # 全局搜索: 在整个地图上寻找最佳匹配位置
        if self.enable_global_search:
            self.get_logger().info('='*50)
            self.get_logger().info('启动全局位置搜索 (任意点定位)...')
            T_best, fitness = self.perform_global_search()
            if fitness > self.localization_th:
                initial = T_best   # 用搜索结果替代初始猜测
                self.get_logger().info(f'全局搜索成功! fitness={fitness:.3f}')
            else:
                self.get_logger().warn(
                    f'全局搜索未过阈值 (fitness={fitness:.3f} < {self.localization_th}), '
                    f'回退到初始位姿猜测')

        success = self.global_localization(initial)
        if success:
            self.initialized = True
            period = 1.0 / self.freq_localization
            self.create_timer(period, self.timer_callback)
            xyz = tf_transformations.translation_from_matrix(self.T_map_to_odom)
            yaw = tf_transformations.euler_from_matrix(self.T_map_to_odom)[2]
            self.get_logger().info(
                f'定位初始化成功! 位置=({xyz[0]:.2f}, {xyz[1]:.2f}), '
                f'朝向={np.degrees(yaw):.1f}°')
            self.get_logger().info('='*50)

    def cb_save_cur_odom(self, msg: Odometry):
        self.cur_odom = msg

    def cb_save_cur_scan(self, msg: PointCloud2):
        msg.header.frame_id = 'camera_init'
        msg.header.stamp    = self.get_clock().now().to_msg()
        self.pub_pc_in_map.publish(msg)

        pts = self.pc2_to_array(msg)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        self.cur_scan = pcd

    def timer_callback(self):
        self.global_localization(self.T_map_to_odom)

    # ─── 全局搜索定位 ────────────────────────────────────────

    @staticmethod
    def _make_tf(x, y, yaw):
        """构造 2D 位姿对应的 4×4 齐次变换矩阵"""
        c, s = np.cos(yaw), np.sin(yaw)
        T = np.eye(4)
        T[0, 0] = c;  T[0, 1] = -s;  T[0, 3] = x
        T[1, 0] = s;  T[1, 1] = c;   T[1, 3] = y
        return T

    def perform_global_search(self):
        """多假设全局位置搜索

        步骤:
          1. 在地图上均匀撒候选位置 (按 search_step 步长)
          2. 每个位置尝试 4 个朝向 (0°, 90°, 180°, 270°)
          3. 粗 ICP (scale=5) 快速打分
          4. 在最佳位置附近精搜 (±1m, ±30°) 用精 ICP (scale=1)
          5. 返回最佳变换和匹配得分
        """
        import time
        t0 = time.time()

        scan = copy.deepcopy(self.cur_scan)
        map_pts = np.asarray(self.global_map.points)
        map_xy = map_pts[:, :2]

        # 地图范围
        min_xy = map_xy.min(axis=0)
        max_xy = map_xy.max(axis=0)
        step = self.search_step

        self.get_logger().info(
            f'地图范围: x=[{min_xy[0]:.1f}, {max_xy[0]:.1f}], '
            f'y=[{min_xy[1]:.1f}, {max_xy[1]:.1f}], 搜索步长={step}m')

        # 粗搜索: 4 个朝向
        coarse_angles = [0, np.pi / 2, np.pi, 3 * np.pi / 2]

        xs = np.arange(min_xy[0], max_xy[0] + step, step)
        ys = np.arange(min_xy[1], max_xy[1] + step, step)

        # 预过滤: 只测试有足够地图点的网格, 跳过空旷区域
        valid_positions = []
        for x in xs:
            for y in ys:
                mask = ((np.abs(map_xy[:, 0] - x) < step) &
                        (np.abs(map_xy[:, 1] - y) < step))
                if np.sum(mask) > 50:
                    valid_positions.append((x, y))

        n_total = len(valid_positions) * len(coarse_angles)
        self.get_logger().info(
            f'粗搜索: {len(valid_positions)} 个有效位置 × '
            f'{len(coarse_angles)} 朝向 = {n_total} 候选')

        best_fitness = 0.0
        best_rmse = float('inf')
        best_T = np.eye(4)
        count = 0

        for x, y in valid_positions:
            for yaw in coarse_angles:
                T_init = self._make_tf(x, y, yaw)
                try:
                    submap = self.crop_global_map_in_FOV(
                        scan, T_init, self.cur_odom)
                    if len(submap.points) < 50:
                        continue
                    T_try, fitness, rmse = self.registration_at_scale(
                        scan, submap, initial=T_init, scale=2)
                    # 优先选 fitness 高的; fitness 接近时选 RMSE 低的
                    if (fitness > best_fitness + 0.05) or \
                       (fitness > best_fitness - 0.05 and rmse < best_rmse):
                        best_fitness = fitness
                        best_rmse = rmse
                        best_T = T_try
                        self.get_logger().info(
                            f'  更优匹配: ({x:.1f}, {y:.1f}, '
                            f'{np.degrees(yaw):.0f}°) '
                            f'fitness={fitness:.3f}, rmse={rmse:.4f}')
                except Exception:
                    pass
                count += 1
                if count % 50 == 0:
                    self.get_logger().info(
                        f'  粗搜索进度: {count}/{n_total}')

        self.get_logger().info(
            f'粗搜索完成: 最佳 fitness={best_fitness:.3f}, '
            f'耗时 {time.time()-t0:.1f}s')

        if best_fitness < 0.3:
            self.get_logger().warn('粗搜索未找到可靠匹配')
            return np.eye(4), 0.0

        # 精搜索: 在最佳位置附近 ±1m, ±30° 细化
        coarse_xyz = tf_transformations.translation_from_matrix(best_T)
        coarse_yaw = tf_transformations.euler_from_matrix(best_T)[2]

        fine_offsets = [-1.0, 0.0, 1.0]
        fine_angles = np.radians([-30, -15, 0, 15, 30])

        for dx in fine_offsets:
            for dy in fine_offsets:
                for da in fine_angles:
                    T_init = self._make_tf(
                        coarse_xyz[0] + dx,
                        coarse_xyz[1] + dy,
                        coarse_yaw + da)
                    try:
                        submap = self.crop_global_map_in_FOV(
                            scan, T_init, self.cur_odom)
                        if len(submap.points) < 50:
                            continue
                        T_try, fitness, rmse = self.registration_at_scale(
                            scan, submap, initial=T_init, scale=1)
                        if (fitness > best_fitness + 0.05) or \
                           (fitness > best_fitness - 0.05 and rmse < best_rmse):
                            best_fitness = fitness
                            best_rmse = rmse
                            best_T = T_try
                    except Exception:
                        pass

        final_xyz = tf_transformations.translation_from_matrix(best_T)
        final_yaw = tf_transformations.euler_from_matrix(best_T)[2]
        self.get_logger().info(
            f'精搜索完成: ({final_xyz[0]:.2f}, {final_xyz[1]:.2f}), '
            f'朝向={np.degrees(final_yaw):.1f}°, '
            f'fitness={best_fitness:.3f}, '
            f'总耗时 {time.time()-t0:.1f}s')

        return best_T, best_fitness

    # ─── ICP 定位 ─────────────────────────────────────────────

    def global_localization(self, pose_est):
        self.get_logger().info('Performing ICP refinement...')
        scan_copy = copy.deepcopy(self.cur_scan)

        submap = self.crop_global_map_in_FOV(scan_copy, pose_est, self.cur_odom)

        # 多级 ICP 逐步精化: 5 → 2 → 1
        T, _, _       = self.registration_at_scale(scan_copy, submap, initial=pose_est, scale=5)
        T, _, _       = self.registration_at_scale(scan_copy, submap, initial=T,         scale=2)
        T, fitness, rmse = self.registration_at_scale(scan_copy, submap, initial=T,     scale=1)
        self.get_logger().info(f'ICP fitness: {fitness:.3f}, rmse: {rmse:.4f}')

        if fitness > self.localization_th:
            self.T_map_to_odom = T
            odom = Odometry()
            xyz  = tf_transformations.translation_from_matrix(T)
            quat = tf_transformations.quaternion_from_matrix(T)
            # 올바른 Odometry 메시지 필드 설정
            odom.pose.pose.position    = Point(x=xyz[0], y=xyz[1], z=xyz[2])
            odom.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            odom.header.stamp          = self.cur_odom.header.stamp
            odom.header.frame_id       = 'map'
            self.pub_map_to_odom.publish(odom)
            return True

        self.get_logger().warn('Global localization failed (fitness below threshold).')
        return False

    def crop_global_map_in_FOV(self, scan, pose_est, odom):
        T_scan     = self.pose_to_mat(odom)
        T_map2scan = np.linalg.inv(pose_est @ T_scan)

        pts = np.asarray(self.global_map.points)
        hom = np.hstack([pts, np.ones((pts.shape[0],1))])
        pts_scan = (T_map2scan @ hom.T).T

        if self.FOV >= 2*np.pi:
            mask = (pts_scan[:,0] < self.FOV_FAR)
        else:
            ang  = np.arctan2(pts_scan[:,1], pts_scan[:,0])
            mask = (pts_scan[:,0]>0)&(pts_scan[:,0]<self.FOV_FAR)&(np.abs(ang)<self.FOV/2)

        subpts = pts[mask]
        submap = o3d.geometry.PointCloud()
        submap.points = o3d.utility.Vector3dVector(subpts)

        header = Header()
        header.stamp    = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        cloud = pc2.create_cloud_xyz32(header, subpts[::10].tolist())
        self.pub_submap.publish(cloud)

        return submap

    def registration_at_scale(self, scan, submap, initial, scale):
        def down(p): return p.voxel_down_sample(self.scan_voxel_size * scale)
        max_iter = 30 if scale >= 2 else 50
        reg = o3d.pipelines.registration.registration_icp(
            down(scan), down(submap),
            max_correspondence_distance=1.0*scale,
            init=initial,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter)
        )
        return reg.transformation, reg.fitness, reg.inlier_rmse

    @staticmethod
    def pose_to_mat(pose_stamped):
        t = pose_stamped.pose.pose.position
        q = pose_stamped.pose.pose.orientation
        return tf_transformations.translation_matrix([t.x,t.y,t.z]) \
             @ tf_transformations.quaternion_matrix([q.x,q.y,q.z,q.w])

    @staticmethod
    def voxel_down_sample(pcd, vs):
        try:
            return pcd.voxel_down_sample(vs)
        except:
            return o3d.geometry.voxel_down_sample(pcd, vs)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalLocalizationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
