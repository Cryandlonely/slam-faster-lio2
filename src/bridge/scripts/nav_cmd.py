#!/usr/bin/env python3
"""
导航控制脚本 — 通过 TCP 向 BridgeNode 发送导航指令
用法: python3 nav_cmd.py [--host HOST] [--port PORT]
"""

import argparse
import json
import select
import socket
import sys
import time

HOST = "localhost"
PORT = 9090


# ==================== 网络工具 ====================

def tcp_send(data: dict, timeout: float = 3.0) -> str:
    msg = json.dumps(data, ensure_ascii=False) + "\n"
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(timeout)
        s.connect((HOST, PORT))
        s.sendall(msg.encode("utf-8"))
        buf = b""
        try:
            while True:
                chunk = s.recv(4096)
                if not chunk:
                    break
                buf += chunk
                if b"\n" in buf:
                    break
        except socket.timeout:
            pass
    return buf.decode("utf-8", errors="replace").strip()


def send_and_print(data: dict):
    try:
        resp = tcp_send(data)
        try:
            print(f"  <- {json.dumps(json.loads(resp), ensure_ascii=False)}")
        except json.JSONDecodeError:
            print(f"  <- {resp}")
    except (ConnectionRefusedError, socket.timeout) as e:
        print(f"  ✗ 连接失败: {e}")


def send_nav_and_wait(data: dict, timeout: float = 300.0) -> str:
    """发送导航指令，保持连接等待 nav_result，返回 'REACHED'|'ERROR'|'TIMEOUT'|'CONNECT_ERROR'"""
    msg = json.dumps(data, ensure_ascii=False) + "\n"
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((HOST, PORT))
            s.sendall(msg.encode("utf-8"))
            buf = b""
            deadline = time.time() + timeout
            while time.time() < deadline:
                s.settimeout(min(deadline - time.time(), 0.5))
                try:
                    chunk = s.recv(4096)
                    if not chunk:
                        print("  ✗ 服务器断开连接")
                        return "ERROR"
                    buf += chunk
                except socket.timeout:
                    pass

                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        j = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    if j.get("type") == "nav_result":
                        return j.get("result", "UNKNOWN")
                    elif j.get("type") == "pose":
                        state = j.get("state", "")
                        pos = f"lat={j['lat']:.6f} lon={j['lon']:.6f}" if "lat" in j \
                              else f"x={j.get('x',0.0):6.2f} y={j.get('y',0.0):6.2f}"
                        print(f"\r  [{state}] {pos}  yaw={j.get('yaw',0.0):.2f}  ",
                              end="", flush=True)
                    else:
                        print(f"  <- {json.dumps(j, ensure_ascii=False)}")
        return "TIMEOUT"
    except (ConnectionRefusedError, OSError) as e:
        print(f"  ✗ 连接失败: {e}")
        return "CONNECT_ERROR"


# ==================== 输入工具 ====================

def read_gps_point(prompt: str):
    while True:
        raw = input(prompt).strip()
        if not raw:
            return None
        parts = raw.replace(",", " ").split()
        if len(parts) < 2:
            print("  格式错误，请输入 lat lon")
            continue
        try:
            return {"lat": float(parts[0]), "lon": float(parts[1])}
        except ValueError:
            print("  数值格式错误")


def read_target_vel(default: float = 1.5) -> float:
    raw = input(f"目标速度 m/s [回车默认 {default}]: ").strip()
    try:
        return float(raw) if raw else default
    except ValueError:
        return default


# ==================== 命令处理 ====================

def do_goal():
    wp = read_gps_point("目标点 (lat lon, 回车取消): ")
    if wp is None:
        print("已取消")
        return
    print(f"  -> 导航到: lat={wp['lat']}, lon={wp['lon']}")
    result = send_nav_and_wait({"cmd": "nav_goal", "coord_mode": "gps", **wp})
    print()
    print("  ★ 导航完成!" if result == "REACHED" else
          "  ✗ 导航异常!" if result == "ERROR" else "  ✗ 等待超时")


def do_waypoints():
    waypoints = []
    print("逐个输入航点 (直接回车结束):")
    idx = 1
    while True:
        wp = read_gps_point(f"  航点{idx} (lat lon): ")
        if wp is None:
            break
        waypoints.append(wp)
        idx += 1

    if not waypoints:
        print("未输入航点，已取消")
        return

    vel = read_target_vel()
    summary = " -> ".join(f"({w['lat']:.6f},{w['lon']:.6f})" for w in waypoints)
    print(f"  -> 多点导航 ({len(waypoints)}点): {summary}, 速度={vel}m/s")
    result = send_nav_and_wait({
        "cmd": "nav_waypoints",
        "coord_mode": "gps",
        "waypoints": waypoints,
        "target_vel": vel,
    })
    print()
    print("  ★ 全部航点完成!" if result == "REACHED" else
          "  ✗ 导航异常!" if result == "ERROR" else "  ✗ 等待超时")


def do_stop():
    print("  -> 取消导航")
    send_and_print({"cmd": "nav_cancel"})


def do_realtime_pose():
    """建立持久连接，被动接收 bridge 推送的位置广播，按回车退出"""
    print("  实时位置显示中，按回车退出...")
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((HOST, PORT))
            s.sendall((json.dumps({"cmd": "query_status"}) + "\n").encode())
            buf = b""
            while True:
                s.settimeout(0.2)
                try:
                    chunk = s.recv(4096)
                    if not chunk:
                        print("\n  ✗ 服务器断开")
                        break
                    buf += chunk
                except socket.timeout:
                    pass

                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        j = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    if j.get("type") == "pose":
                        state = j.get("state", "")
                        bat   = j.get("battery", 0.0)
                        vx    = j.get("vx", 0.0)
                        yaw   = j.get("yaw", 0.0)
                        pos = f"lat={j['lat']:.7f}  lon={j['lon']:.7f}" if "lat" in j \
                              else f"x={j.get('x',0.0):7.3f}  y={j.get('y',0.0):7.3f}"
                        print(f"\r  [{state}] {pos}  yaw={yaw:6.3f}"
                              f"  bat={bat:.1f}V  vx={vx:.2f}m/s  ",
                              end="", flush=True)

                if select.select([sys.stdin], [], [], 0.0)[0]:
                    sys.stdin.readline()
                    break
    except (ConnectionRefusedError, OSError) as e:
        print(f"\n  ✗ 连接失败: {e}")
    print()


# ==================== 主菜单 ====================

MENU = """
========== 导航控制 ==========
  1 - 单点导航
  2 - 多点导航
  3 - 取消导航
  4 - 实时位置显示
  q - 退出
=============================="""


def main():
    global HOST, PORT
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=9090)
    args = parser.parse_args()
    HOST, PORT = args.host, args.port

    print(f"Bridge: tcp://{HOST}:{PORT}")
    DISPATCH = {"1": do_goal, "2": do_waypoints, "3": do_stop, "4": do_realtime_pose}

    while True:
        print(MENU)
        choice = input("请选择: ").strip().lower()
        if choice in ("q", "quit", "exit"):
            print("退出")
            break
        fn = DISPATCH.get(choice)
        if fn:
            fn()
        else:
            print(f"  无效输入: {choice!r}")


if __name__ == "__main__":
    main()
