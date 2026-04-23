#!/usr/bin/env python3
"""
导航控制交互脚本 — 通过 TCP 向 BridgeNode 发送导航指令

启动后进入交互菜单:
  1 - 单点导航 (输入 x y yaw)
  2 - 多点导航 (逐个输入航点)
  3 - 停止导航
  4 - 查询状态
    6 - 下载实时点云文件 (trans.pcd)
  q - 退出

用法:
  python3 nav_cmd.py
  python3 nav_cmd.py --host 192.168.10.145 --port 9090
"""

import argparse
import base64
import json
import socket
import sys
import time
import threading


HOST = "localhost"
PORT = 9090
PCD_PORT = 9091


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


def pcd_send(data: dict, timeout: float = 5.0) -> str:
    msg = json.dumps(data, ensure_ascii=False) + "\n"
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(timeout)
        s.connect((HOST, PCD_PORT))
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


def wait_for_nav_done():
    """后台轮询导航状态，到达或出错时打印一次提示"""
    last_state = None
    while True:
        time.sleep(1.0)
        try:
            resp = tcp_send({"cmd": "query_status"}, timeout=2.0)
            j = json.loads(resp)
            nav = j.get("nav")
            if nav is None:
                continue
            state = nav.get("state", "")
            if state == last_state:
                continue
            last_state = state
            if state == "REACHED":
                dist = nav.get("dist_to_goal", "?")
                print(f"\n  ★ 导航完成! 已到达目标点 (剩余距离: {dist}m)")
                return
            elif state == "ERROR":
                print(f"\n  ✗ 导航异常!")
                return
            elif state == "IDLE":
                # 被外部取消或已结束
                return
        except (ConnectionRefusedError, socket.timeout, json.JSONDecodeError):
            continue


def start_nav_monitor():
    """启动后台线程监听导航完成"""
    t = threading.Thread(target=wait_for_nav_done, daemon=True)
    t.start()
    return t


def read_point(prompt="输入坐标 (x y yaw，yaw可省略默认0): "):
    while True:
        raw = input(prompt).strip()
        if not raw:
            return None
        parts = raw.replace(",", " ").split()
        if len(parts) < 2:
            print("  格式错误，至少输入 x y")
            continue
        try:
            x = float(parts[0])
            y = float(parts[1])
            yaw = float(parts[2]) if len(parts) >= 3 else 0.0
            return x, y, yaw
        except ValueError:
            print("  数值格式错误，请重新输入")


def do_goal():
    pt = read_point("输入目标点 (x y yaw): ")
    if pt is None:
        print("已取消")
        return
    x, y, yaw = pt
    data = {"cmd": "nav_goal", "x": x, "y": y, "yaw": yaw}
    print(f"  → 发送单点导航: x={x}, y={y}, yaw={yaw}")
    try:
        resp = tcp_send(data)
        print(f"  ← 应答: {resp}")
        print("  (后台监听导航状态，到达后会提示)")
        start_nav_monitor()
    except (ConnectionRefusedError, socket.timeout) as e:
        print(f"  ✗ 连接失败: {e}")


def do_waypoints():
    waypoints = []
    print("逐个输入航点 (直接回车结束输入):")
    idx = 1
    while True:
        pt = read_point(f"  航点{idx} (x y yaw): ")
        if pt is None:
            break
        waypoints.append({"x": pt[0], "y": pt[1], "yaw": pt[2]})
        idx += 1

    if not waypoints:
        print("未输入航点，已取消")
        return

    vel_str = input(f"目标速度 (m/s, 直接回车默认0.5): ").strip()
    vel = float(vel_str) if vel_str else 0.5

    data = {"cmd": "nav_waypoints", "waypoints": waypoints, "target_vel": vel}
    summary = " → ".join(f"({w['x']},{w['y']})" for w in waypoints)
    print(f"  → 发送多点导航 ({len(waypoints)}点): {summary}, 速度={vel} m/s")
    try:
        resp = tcp_send(data)
        print(f"  ← 应答: {resp}")
        print("  (后台监听导航状态，到达后会提示)")
        start_nav_monitor()
    except (ConnectionRefusedError, socket.timeout) as e:
        print(f"  ✗ 连接失败: {e}")


def do_stop():
    print("  → 发送停止导航")
    try:
        resp = tcp_send({"cmd": "nav_cancel"})
        print(f"  ← 应答: {resp}")
    except (ConnectionRefusedError, socket.timeout) as e:
        print(f"  ✗ 连接失败: {e}")


def do_realtime_pose():
    """实时查看位置，按回车退出"""
    print("  实时位置 (按回车退出):")
    import select
    try:
        while True:
            resp = tcp_send({"cmd": "query_status"}, timeout=2.0)
            j = json.loads(resp)
            slam = j.get("slam", {})
            nav = j.get("nav")
            nav_state = nav.get("state", "?") if nav else "?"
            x = slam.get("x", 0)
            y = slam.get("y", 0)
            yaw = slam.get("yaw", 0)
            line = f"  位置: x={x:7.3f}  y={y:7.3f}  yaw={yaw:6.3f}  导航状态: {nav_state}"
            print(f"\r{line}", end="", flush=True)
            # 检查用户是否按了回车
            if select.select([sys.stdin], [], [], 0.5)[0]:
                sys.stdin.readline()
                break
    except (ConnectionRefusedError, socket.timeout) as e:
        print(f"\n  ✗ 连接失败: {e}")
    except (json.JSONDecodeError, KeyError):
        pass
    print()  # 换行


def do_status():
    print("  → 查询状态")
    try:
        resp = tcp_send({"cmd": "query_status"})
        j = json.loads(resp)
        print(json.dumps(j, indent=2, ensure_ascii=False))
    except json.JSONDecodeError:
        print(f"  ← {resp}")
    except (ConnectionRefusedError, socket.timeout) as e:
        print(f"  ✗ 连接失败: {e}")


def do_download_trans_pcd():
    print("  → 查询 trans.pcd 文件信息")
    try:
        info_resp = pcd_send({"cmd": "pcd_info"}, timeout=3.0)
        info = json.loads(info_resp)
    except (ConnectionRefusedError, socket.timeout) as e:
        print(f"  ✗ PCD 端口连接失败: {e}")
        return
    except json.JSONDecodeError:
        print(f"  ✗ PCD 信息返回非法 JSON: {info_resp}")
        return

    if not info.get("exists", False):
        print("  ✗ trans.pcd 不存在，请先启动建图并开启实时保存")
        return

    total_size = int(info.get("size", 0))
    chunk_bytes = int(info.get("chunk_bytes", 65536))
    if total_size <= 0:
        print("  ✗ trans.pcd 文件大小为 0")
        return

    default_out = "trans_download.pcd"
    out_path = input(f"保存路径 (回车默认 {default_out}): ").strip()
    if not out_path:
        out_path = default_out

    print(f"  → 开始下载: {total_size} bytes, chunk={chunk_bytes} bytes")

    offset = 0
    with open(out_path, "wb") as f:
        while offset < total_size:
            req_len = min(chunk_bytes, total_size - offset)
            try:
                chunk_resp = pcd_send(
                    {"cmd": "pcd_chunk", "offset": offset, "length": req_len},
                    timeout=8.0,
                )
                chunk = json.loads(chunk_resp)
            except (ConnectionRefusedError, socket.timeout) as e:
                print(f"\n  ✗ 下载中断，连接失败: {e}")
                return
            except json.JSONDecodeError:
                print(f"\n  ✗ 下载中断，返回非法 JSON: {chunk_resp}")
                return

            if "error" in chunk:
                print(f"\n  ✗ 下载失败: {chunk['error']}")
                return

            data_b64 = chunk.get("data_b64", "")
            try:
                data = base64.b64decode(data_b64)
            except Exception as e:
                print(f"\n  ✗ Base64 解码失败: {e}")
                return

            got = len(data)
            if got == 0 and not chunk.get("eof", False):
                print("\n  ✗ 下载失败: 收到空分片")
                return

            f.write(data)
            offset += got
            progress = min(100.0, 100.0 * offset / total_size)
            print(f"\r  下载进度: {progress:6.2f}% ({offset}/{total_size})", end="", flush=True)

            if chunk.get("eof", False):
                break

    print(f"\n  ★ 下载完成: {out_path}")


MENU = """
========== 导航控制 ==========
  1 - 单点导航
  2 - 多点导航
  3 - 停止导航
  4 - 实时位置
  5 - 查询状态
    6 - 下载实时点云文件 (trans.pcd)
  q - 退出
=============================="""


def main():
    global HOST, PORT, PCD_PORT

    parser = argparse.ArgumentParser(description="导航控制交互脚本")
    parser.add_argument("--host", default="localhost", help="BridgeNode IP (默认 localhost)")
    parser.add_argument("--port", type=int, default=9090, help="BridgeNode 端口 (默认 9090)")
    parser.add_argument("--pcd-port", type=int, default=9091, help="PCD 传输端口 (默认 9091)")
    args = parser.parse_args()

    HOST = args.host
    PORT = args.port
    PCD_PORT = args.pcd_port

    print(f"连接目标: 控制 {HOST}:{PORT}, PCD {HOST}:{PCD_PORT}")

    while True:
        print(MENU)
        choice = input("请选择: ").strip()

        if choice == "1":
            do_goal()
        elif choice == "2":
            do_waypoints()
        elif choice == "3":
            do_stop()
        elif choice == "4":
            do_realtime_pose()
        elif choice == "5":
            do_status()
        elif choice == "6":
            do_download_trans_pcd()
        elif choice in ("q", "Q", "quit", "exit"):
            print("退出")
            break
        else:
            print("无效输入，请输入 1-6 或 q")


if __name__ == "__main__":
    main()
