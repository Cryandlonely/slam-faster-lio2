#!/usr/bin/env python3
"""
VRU620/621PRC-232 串口数据接收测试脚本
直接解析串口原始帧，无需 ROS2 运行。

协议格式（手册 5.2 节）：
  [0x7A][0x7B][字节数][功能字节][数据体...][校验字节][0xBB]

功能字节：
  0xB0 — 原始 ADC（加速度 + 角速度，各 2 字节 WORD）
  0xB1 — 解算姿态角（yaw/pitch/roll/tilt 各 3 字节 + 解算速率 2 字节）

用法：
  python3 vru_test.py                        # 默认 /dev/ttyS0, 115200
  python3 vru_test.py -p /dev/ttyUSB0 -b 115200
  python3 vru_test.py --no-imu               # 只显示姿态角
  python3 vru_test.py --no-attitude          # 只显示 IMU 原始数据
"""

import argparse
import sys
import time
import struct
from collections import deque

try:
    import serial
except ImportError:
    print("[错误] 缺少 pyserial，请先安装: pip install pyserial")
    sys.exit(1)


# ─────────────────────────── 协议解析 ───────────────────────────

def decode_angle_3byte(data: bytes, offset: int) -> float:
    """从 3 字节有符号数解码角度值（首字节最高位为符号位，分辨率 0.0001°）"""
    b0, b1, b2 = data[offset], data[offset + 1], data[offset + 2]
    negative = bool(b0 & 0x80)
    raw = ((b0 & 0x7F) << 16) | (b1 << 8) | b2
    val = raw * 0.0001
    return -val if negative else val


def decode_word_signed(data: bytes, offset: int, resolution: float) -> float:
    """从 2 字节有符号数解码 ADC 值（首字节最高位为符号位）"""
    b0, b1 = data[offset], data[offset + 1]
    negative = bool(b0 & 0x80)
    raw = ((b0 & 0x7F) << 8) | b1
    val = raw * resolution
    return -val if negative else val


def calc_checksum(data: bytes, start: int, end: int) -> int:
    """异或校验"""
    result = 0
    for b in data[start:end]:
        result ^= b
    return result


def parse_attitude(data: bytes):
    """
    解析功能字节 0xB1 姿态角数据
    返回 dict: yaw/pitch/roll/tilt (度), has_yaw
    """
    has_yaw = len(data) >= 14
    offset = 0

    yaw = 0.0
    if has_yaw:
        yaw = decode_angle_3byte(data, offset)
        offset += 3

    if len(data) < offset + 11:
        return None

    pitch = decode_angle_3byte(data, offset);  offset += 3
    roll  = decode_angle_3byte(data, offset);  offset += 3

    # 倾斜角（弧度 × 0.0001，再 × 57.3 → 度）
    b0, b1, b2 = data[offset], data[offset + 1], data[offset + 2]
    tilt_neg = bool(b0 & 0x80)
    tilt_raw = ((b0 & 0x7F) << 16) | (b1 << 8) | b2
    tilt_deg = (tilt_raw * 0.0001 * 57.3) * (-1 if tilt_neg else 1)

    return {
        "has_yaw": has_yaw,
        "yaw":   yaw,
        "pitch": pitch,
        "roll":  roll,
        "tilt":  tilt_deg,
    }


def parse_adc(data: bytes, accel_res: float, gyro_res: float):
    """
    解析功能字节 0xB0 原始 ADC 数据
    返回 dict: ax/ay/az (m/s²), gx/gy/gz (rad/s)
    """
    if len(data) < 12:
        return None

    G = 9.80665
    D2R = 3.141592653589793 / 180.0

    ax = decode_word_signed(data, 0, accel_res) * G
    ay = decode_word_signed(data, 2, accel_res) * G
    az = decode_word_signed(data, 4, accel_res) * G
    gx = decode_word_signed(data, 6, gyro_res)  * D2R
    gy = decode_word_signed(data, 8, gyro_res)  * D2R
    gz = decode_word_signed(data, 10, gyro_res) * D2R

    return {"ax": ax, "ay": ay, "az": az, "gx": gx, "gy": gy, "gz": gz}


# ─────────────────────────── 主循环 ───────────────────────────

def run(port: str, baud: int, accel_range: int, gyro_range: int,
        show_attitude: bool, show_imu: bool):

    # 量程表
    accel_res_table = [1.0/16384, 1.0/8192, 1.0/4096, 1.0/2048]
    gyro_res_table  = [125.0/16384, 125.0/8192, 125.0/4096, 125.0/2048]
    accel_res = accel_res_table[accel_range]
    gyro_res  = gyro_res_table[gyro_range]

    print(f"[配置] 串口={port}  波特率={baud}")
    print(f"[配置] 加速度量程=±{[2,4,8,16][accel_range]}g  "
          f"角速度量程=±{[250,500,1000,2000][gyro_range]}dps")
    print(f"[配置] 显示姿态角={'是' if show_attitude else '否'}  "
          f"显示IMU原始={'是' if show_imu else '否'}")
    print("-" * 60)

    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        print(f"[OK] 串口已打开: {port}")
    except serial.SerialException as e:
        print(f"[错误] 无法打开串口 {port}: {e}")
        print("  常见原因: 设备未插入 / 权限不足 (sudo usermod -aG dialout $USER)")
        sys.exit(1)

    buf = bytearray()
    stats = {"attitude": 0, "adc": 0, "crc_err": 0, "sync_err": 0, "total_bytes": 0}
    t_start = time.time()
    t_last_print = t_start

    print("[等待数据...] 按 Ctrl-C 退出\n")

    try:
        while True:
            raw = ser.read(256)
            if not raw:
                # 超时无数据
                elapsed = time.time() - t_start
                if elapsed > 3.0 and stats["total_bytes"] == 0:
                    print(f"\r[警告] {elapsed:.0f}s 内未收到任何数据，"
                          "请检查串口连接和设备上电", end="", flush=True)
                continue

            buf.extend(raw)
            stats["total_bytes"] += len(raw)

            # 解析帧
            while len(buf) >= 5:
                # 对齐帧头
                if buf[0] != 0x7A or buf[1] != 0x7B:
                    buf.pop(0)
                    stats["sync_err"] += 1
                    continue

                byte_count = buf[2]
                frame_len  = 2 + byte_count

                if len(buf) < frame_len:
                    break  # 等待更多数据

                # 校验结束字节
                if buf[frame_len - 1] != 0xBB:
                    buf.pop(0)
                    stats["sync_err"] += 1
                    continue

                # 校验 XOR
                crc_calc = calc_checksum(buf, 2, frame_len - 2)
                if crc_calc != buf[frame_len - 2]:
                    stats["crc_err"] += 1
                    buf.pop(0)
                    continue

                func = buf[3]
                data = bytes(buf[4: frame_len - 2])
                buf = buf[frame_len:]

                now_str = time.strftime("%H:%M:%S")

                if func == 0xB1 and show_attitude:
                    result = parse_attitude(data)
                    if result:
                        stats["attitude"] += 1
                        yaw_str = f"yaw={result['yaw']:+8.4f}°  " if result["has_yaw"] else ""
                        print(f"[{now_str}] [姿态] "
                              f"{yaw_str}"
                              f"pitch={result['pitch']:+8.4f}°  "
                              f"roll={result['roll']:+8.4f}°  "
                              f"tilt={result['tilt']:+8.4f}°")

                elif func == 0xB0 and show_imu:
                    result = parse_adc(data, accel_res, gyro_res)
                    if result:
                        stats["adc"] += 1
                        print(f"[{now_str}] [IMU]  "
                              f"ax={result['ax']:+7.4f} ay={result['ay']:+7.4f} az={result['az']:+7.4f} m/s²  "
                              f"gx={result['gx']:+7.4f} gy={result['gy']:+7.4f} gz={result['gz']:+7.4f} rad/s")

                elif func not in (0xB0, 0xB1):
                    print(f"[{now_str}] [未知功能字节] 0x{func:02X}  数据长度={len(data)}")

            # 每 5 秒打印一次统计
            now_t = time.time()
            if now_t - t_last_print >= 5.0:
                elapsed = now_t - t_start
                rate_att = stats["attitude"] / elapsed
                rate_adc = stats["adc"] / elapsed
                print(f"\n--- 统计 ({elapsed:.0f}s) ---  "
                      f"姿态帧={stats['attitude']}({rate_att:.1f}Hz)  "
                      f"ADC帧={stats['adc']}({rate_adc:.1f}Hz)  "
                      f"CRC错误={stats['crc_err']}  同步丢弃={stats['sync_err']}  "
                      f"总字节={stats['total_bytes']}\n")
                t_last_print = now_t

            # 防止缓冲区无限增长
            if len(buf) > 2048:
                buf = buf[-1024:]

    except KeyboardInterrupt:
        elapsed = time.time() - t_start
        print(f"\n\n[退出] 运行 {elapsed:.1f}s")
        print(f"  姿态帧(0xB1): {stats['attitude']}  ADC帧(0xB0): {stats['adc']}")
        print(f"  CRC错误: {stats['crc_err']}  同步丢弃: {stats['sync_err']}")
        print(f"  总接收字节: {stats['total_bytes']}")
    finally:
        ser.close()


# ─────────────────────────── 入口 ───────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="VRU620/621 串口数据接收测试",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument("-p", "--port",   default="/dev/ttyS0",
                        help="串口设备 (默认: /dev/ttyS0)")
    parser.add_argument("-b", "--baud",   default=115200, type=int,
                        help="波特率 (默认: 115200)")
    parser.add_argument("--accel-range",  default=0, type=int, choices=[0, 1, 2, 3],
                        help="加速度量程 0=±2g 1=±4g 2=±8g 3=±16g (默认: 0)")
    parser.add_argument("--gyro-range",   default=0, type=int, choices=[0, 1, 2, 3],
                        help="角速度量程 0=±250dps 1=±500dps 2=±1000dps 3=±2000dps (默认: 0)")
    parser.add_argument("--no-attitude",  action="store_true",
                        help="不显示姿态角(0xB1)数据")
    parser.add_argument("--no-imu",       action="store_true",
                        help="不显示原始IMU(0xB0)数据")

    args = parser.parse_args()

    run(
        port=args.port,
        baud=args.baud,
        accel_range=args.accel_range,
        gyro_range=args.gyro_range,
        show_attitude=not args.no_attitude,
        show_imu=not args.no_imu,
    )
