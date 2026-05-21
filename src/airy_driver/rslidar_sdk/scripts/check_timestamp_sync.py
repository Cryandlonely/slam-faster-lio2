#!/usr/bin/env python3
"""
check_timestamp_sync.py
========================
检测 Airy 激光雷达 (XYZIRT) 点云与 IMU 的时间戳对齐情况。

实测点云格式 (PointXYZIRT，rslidar_sdk):
  x         float32  offset  0
  y         float32  offset  4
  z         float32  offset  8
  intensity float32  offset 12
  ring      uint16   offset 16
  timestamp float64  offset 18
  point_step = 26 字节

时间戳说明:
  header.stamp 和每点 timestamp 均为 ROS 单调时钟 (开机秒数)，
  不是 UNIX 绝对时间戳。两者来自同一时钟源，正常应极度接近。

检测项:
  1. 点云帧率与帧间抖动 (理论 10 Hz)
  2. IMU 频率与帧间抖动 (理论 ~400 Hz)
  3. 帧内首点 timestamp 与 header.stamp 之差 (期望 < 1 ms)
  4. 帧内 timestamp 时间跨度 (末点 - 首点，期望 ~100 ms)
  5. 激光 ↔ IMU 时差：对每帧激光找最近 IMU 戳，差值期望 < 5 ms
  6. 时间戳回跳 / 跳变检测

用法:
  source /opt/ros/humble/setup.bash
  source <workspace>/install/setup.bash
  python3 check_timestamp_sync.py [--duration 10] [--lidar /rslidar_points] [--imu /rslidar_imu_data]
"""

import argparse
import bisect
import math
import struct
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu


# ---------------------------------------------------------------------------
# 辅助：从 PointCloud2 (XYZIRT) 解出首点和末点的 timestamp
# ---------------------------------------------------------------------------
_DATATYPE_FLOAT32 = 7
_DATATYPE_FLOAT64 = 8


def _find_field(cloud_msg: PointCloud2, name: str):
    for f in cloud_msg.fields:
        if f.name == name:
            return f
    return None


def _extract_first_last_ts(cloud_msg: PointCloud2):
    """
    只取首点和末点的 timestamp，避免遍历几万个点。
    返回 (first_ts, last_ts, field_name) 或 (None, None, None)。
    """
    field = _find_field(cloud_msg, "timestamp") or _find_field(cloud_msg, "time")
    if field is None:
        return None, None, None

    if field.datatype == _DATATYPE_FLOAT64:
        fmt = "<d"
    elif field.datatype == _DATATYPE_FLOAT32:
        fmt = "<f"
    else:
        return None, None, field.name

    n = cloud_msg.width * cloud_msg.height
    if n == 0:
        return None, None, field.name

    ps = cloud_msg.point_step
    data = cloud_msg.data
    fo = field.offset

    first_ts = struct.unpack_from(fmt, data, fo)[0]
    last_ts  = struct.unpack_from(fmt, data, (n - 1) * ps + fo)[0]
    return first_ts, last_ts, field.name


# ---------------------------------------------------------------------------
# 统计工具
# ---------------------------------------------------------------------------
def _stats(values):
    if not values:
        return None
    n = len(values)
    mean = sum(values) / n
    var = sum((v - mean) ** 2 for v in values) / n
    return {"n": n, "mean": mean, "std": math.sqrt(var),
            "min": min(values), "max": max(values)}


def _print_stats(label, values, scale=1000.0, unit="ms"):
    s = _stats(values)
    if s is None:
        print(f"  {label}: 无数据")
        return
    print(f"  {label} (n={s['n']}): "
          f"mean={s['mean']*scale:.3f}{unit}  "
          f"std={s['std']*scale:.3f}{unit}  "
          f"min={s['min']*scale:.3f}{unit}  "
          f"max={s['max']*scale:.3f}{unit}")


# ---------------------------------------------------------------------------
# 主节点
# ---------------------------------------------------------------------------
class TimestampChecker(Node):
    def __init__(self, lidar_topic: str, imu_topic: str, duration: float):
        super().__init__("timestamp_checker")

        self._duration = duration
        self._start_time: float | None = None
        self._done = threading.Event()

        # 激光
        self._lidar_count = 0
        self._lidar_intervals: list[float] = []   # header.stamp 帧间差
        self._lidar_hdr_stamps: list[float] = []  # 所有帧 header.stamp
        self._ts_vs_hdr: list[float] = []         # 首点 timestamp - header.stamp
        self._intra_span: list[float] = []        # 末点ts - 首点ts（帧内跨度）
        self._last_lidar_stamp: float | None = None
        self._ts_field_name: str | None = None

        # IMU
        self._imu_count = 0
        self._imu_stamps: list[float] = []        # 所有 IMU header.stamp（用于最近邻搜索）
        self._imu_intervals: list[float] = []
        self._last_imu_stamp: float | None = None

        # 激光 ↔ IMU 最近邻时差（每帧激光找最近 IMU stamp）
        self._lidar_imu_nearest: list[float] = []

        self._lock = threading.Lock()

        self._lidar_sub = self.create_subscription(
            PointCloud2, lidar_topic, self._lidar_cb, 10)
        self._imu_sub = self.create_subscription(
            Imu, imu_topic, self._imu_cb, 200)

        self._timer = self.create_timer(1.0, self._progress_cb)
        self.get_logger().info(
            f"开始采集 {duration:.0f} 秒 | 激光: {lidar_topic} | IMU: {imu_topic}")

    # ------------------------------------------------------------------
    def _lidar_cb(self, msg: PointCloud2):
        hdr_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        first_ts, last_ts, fname = _extract_first_last_ts(msg)

        with self._lock:
            if self._start_time is None:
                self._start_time = time.monotonic()
            if fname and self._ts_field_name is None:
                self._ts_field_name = fname

            self._lidar_count += 1
            self._lidar_hdr_stamps.append(hdr_t)

            # 帧间抖动
            if self._last_lidar_stamp is not None:
                dt = hdr_t - self._last_lidar_stamp
                self._lidar_intervals.append(dt)
                if dt < 0:
                    self.get_logger().warn(
                        f"[激光] 时间戳回跳! dt={dt*1000:.2f}ms  t={hdr_t:.3f}s")
                elif dt > 0.25:
                    self.get_logger().warn(
                        f"[激光] 帧间跳变! dt={dt*1000:.2f}ms  t={hdr_t:.3f}s")
            self._last_lidar_stamp = hdr_t

            # 帧内分析
            if first_ts is not None and last_ts is not None:
                self._ts_vs_hdr.append(first_ts - hdr_t)
                self._intra_span.append(last_ts - first_ts)

            # 激光 ↔ IMU 最近邻时差
            imu_stamps = self._imu_stamps
            if imu_stamps:
                idx = bisect.bisect_left(imu_stamps, hdr_t)
                candidates = []
                if idx < len(imu_stamps):
                    candidates.append(imu_stamps[idx] - hdr_t)
                if idx > 0:
                    candidates.append(imu_stamps[idx - 1] - hdr_t)
                self._lidar_imu_nearest.append(min(candidates, key=abs))

    def _imu_cb(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            self._imu_count += 1
            # 维持有序列表（IMU本应单调，bisect保险）
            idx = bisect.bisect_left(self._imu_stamps, t)
            self._imu_stamps.insert(idx, t)

            if self._last_imu_stamp is not None:
                dt = t - self._last_imu_stamp
                self._imu_intervals.append(dt)
                if dt < 0:
                    self.get_logger().warn(
                        f"[IMU] 时间戳回跳! dt={dt*1000:.2f}ms  t={t:.3f}s")
                elif dt > 0.015:  # 超过 1.5 个 IMU 周期 (~2.5ms)
                    self.get_logger().warn(
                        f"[IMU] 帧间跳变! dt={dt*1000:.2f}ms  t={t:.3f}s")
            self._last_imu_stamp = t

    def _progress_cb(self):
        elapsed = (time.monotonic() - self._start_time) if self._start_time else 0.0
        with self._lock:
            ln, im = self._lidar_count, self._imu_count
        print(f"\r已采集 {elapsed:.0f}s | 激光帧: {ln}  IMU帧: {im}", end="", flush=True)
        if elapsed >= self._duration:
            print()
            self._done.set()

    # ------------------------------------------------------------------
    def wait_and_report(self):
        self._done.wait()
        self._print_report()

    def _print_report(self):
        with self._lock:
            lidar_n          = self._lidar_count
            imu_n            = self._imu_count
            lidar_intervals  = list(self._lidar_intervals)
            imu_intervals    = list(self._imu_intervals)
            ts_vs_hdr        = list(self._ts_vs_hdr)
            intra_span       = list(self._intra_span)
            nearest_diffs    = list(self._lidar_imu_nearest)
            last_lidar       = self._last_lidar_stamp
            last_imu         = self._last_imu_stamp
            ts_field         = self._ts_field_name

        dur = self._duration
        sep = "=" * 62

        print(f"\n{sep}")
        print("              时间戳对齐检测报告")
        print(sep)

        # [1] 帧率
        print(f"\n[1] 帧率")
        if lidar_n:
            rate = lidar_n / dur
            flag = "✔" if 9.0 <= rate <= 11.0 else "✘"
            print(f"  激光帧数: {lidar_n}  平均帧率: {rate:.2f} Hz  (期望 10 Hz)  {flag}")
        else:
            print("  激光: 未收到任何帧 — 请检查话题名称和驱动是否启动")
        if imu_n:
            rate_i = imu_n / dur
            flag_i = "✔" if rate_i > 200 else "✘"
            print(f"  IMU 帧数:  {imu_n}  平均频率: {rate_i:.1f} Hz  {flag_i}")
        else:
            print("  IMU:  未收到任何帧 — 请检查话题名称和驱动是否启动")

        # [2] 帧间抖动
        print(f"\n[2] 帧间时差抖动")
        _print_stats("激光帧间 dt", lidar_intervals)
        _print_stats("IMU  帧间 dt", imu_intervals)
        if imu_intervals:
            mean_imu_dt = sum(imu_intervals) / len(imu_intervals)
            print(f"  IMU 平均频率(反算): {1.0/mean_imu_dt:.1f} Hz")

        # [3] 帧内首点 timestamp vs header.stamp
        print(f"\n[3] 帧内首点 timestamp 与 header.stamp 之差")
        if ts_field:
            print(f"  timestamp 字段名: '{ts_field}' (float64, ROS单调时钟)")
            _print_stats("首点ts - header.stamp", ts_vs_hdr)
            if ts_vs_hdr:
                mean_diff = abs(sum(ts_vs_hdr) / len(ts_vs_hdr)) * 1000
                if mean_diff < 1.0:
                    print(f"  ✔ 两者差值 {mean_diff:.3f} ms，点云 timestamp 与 header 时钟一致")
                else:
                    print(f"  ✘ 差值 {mean_diff:.3f} ms 偏大，timestamp 与 header 时钟源可能不同")
        else:
            print("  未找到 timestamp/time 字段，跳过")

        # [4] 帧内时间跨度
        print(f"\n[4] 帧内点时间跨度 (末点ts - 首点ts)")
        if intra_span:
            _print_stats("帧内跨度", intra_span)
            mean_span = sum(intra_span) / len(intra_span)
            if 0.085 <= mean_span <= 0.115:
                print(f"  ✔ 帧内跨度 {mean_span*1000:.2f} ms，与 10 Hz 旋转周期吻合")
            else:
                print(f"  ✘ 帧内跨度 {mean_span*1000:.2f} ms，期望约 100 ms，请检查帧率设置")
        else:
            print("  无帧内时间戳数据，跳过")

        # [5] 激光 ↔ IMU 最近邻时差
        print(f"\n[5] 激光 ↔ IMU 最近邻 header.stamp 时差")
        print(f"  (对每帧激光 header.stamp 在 IMU stamp列表中做最近邻搜索)")
        if nearest_diffs:
            _print_stats("最近邻时差", nearest_diffs)
            mean_nd = abs(sum(nearest_diffs) / len(nearest_diffs)) * 1000
            if mean_nd < 3.0:
                print(f"  ✔ 均值 {mean_nd:.2f} ms < 3 ms，激光与 IMU 时间戳对齐良好")
            elif mean_nd < 10.0:
                print(f"  △ 均值 {mean_nd:.2f} ms，有轻微偏差，可设 time_offset_lidar_to_imu 补偿")
                bias = sum(nearest_diffs) / len(nearest_diffs)
                print(f"     建议: time_offset_lidar_to_imu: {-bias:.5f}")
            else:
                print(f"  ✘ 均值 {mean_nd:.2f} ms 过大，激光与 IMU 时钟源可能不一致")
                bias = sum(nearest_diffs) / len(nearest_diffs)
                print(f"     建议: time_offset_lidar_to_imu: {-bias:.5f}")
        else:
            print("  激光或 IMU 无数据，跳过")

        # [6] 时间戳类型判断
        print(f"\n[6] 时间戳类型")
        for name, stamp in [("激光 header.stamp", last_lidar), ("IMU  header.stamp", last_imu)]:
            if stamp is None:
                print(f"  {name}: 无数据")
            elif stamp > 1.5e9:
                print(f"  {name}: {stamp:.3f} s  → UNIX 绝对时间戳 ✔")
            elif stamp > 0:
                print(f"  {name}: {stamp:.3f} s  → ROS 单调时钟 (开机秒数，非 UNIX 时间)")
                print(f"    注意: FAST-LIO 要求激光与 IMU 使用同一时钟源，ROS时钟是正常的")
            else:
                print(f"  {name}: {stamp:.3f} s  → 异常值")

        # [7] 回跳/跳变汇总
        bad_l = sum(1 for d in lidar_intervals if d < 0 or d > 0.25)
        bad_i = sum(1 for d in imu_intervals   if d < 0 or d > 0.015)
        print(f"\n[7] 异常帧间统计")
        print(f"  激光异常: {bad_l} 次  |  IMU 异常: {bad_i} 次")
        if bad_l == 0 and bad_i == 0:
            print("  ✔ 无回跳或跳变")
        else:
            print("  ✘ 存在异常，详见上方 WARN 日志")

        print(f"\n{sep}\n")


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Airy 激光雷达时间戳对齐检测")
    parser.add_argument("--duration", type=float, default=10.0,
                        help="采集时长（秒，默认 10）")
    parser.add_argument("--lidar",    type=str, default="/rslidar_points",
                        help="点云话题名称")
    parser.add_argument("--imu",      type=str, default="/rslidar_imu_data",
                        help="IMU 话题名称")
    args = parser.parse_args()

    rclpy.init(args=sys.argv)
    checker = TimestampChecker(args.lidar, args.imu, args.duration)
    spin_thread = threading.Thread(target=rclpy.spin, args=(checker,), daemon=True)
    spin_thread.start()

    try:
        checker.wait_and_report()
    except KeyboardInterrupt:
        print("\n用户中断，输出当前结果...")
        checker._print_report()
    finally:
        checker.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
