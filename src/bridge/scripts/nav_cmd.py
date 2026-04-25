#!/usr/bin/env python3
"""
导航控制交互脚本 — 通过 TCP 向 BridgeNode 发送导航指令

用法:
  python3 nav_cmd.py [--host HOST] [--port PORT]

====== 所有支持的 bridge 接口 ======

【生命周期管理】
  start_mapping      → 启动 FAST-LIO 建图节点, 定位源切换为 /Odometry
  stop_mapping       → 停止建图节点
  start_indoor_loc   → 启动室内定位 (global_localization + transform_fusion), 定位源切换为 /localization
  stop_indoor_loc    → 停止室内定位节点
  start_outdoor      → 启动 RTK 节点, 定位源切换为 /outdoor/odom, 规划器切为直线
  stop_outdoor       → 停止 RTK 节点, 定位源切换回室内

【导航指令 — 室内（本地坐标 map 系）】
  nav_goal      {"cmd":"nav_goal","coord_mode":"local","x":1.0,"y":2.0,"yaw":0.0}
  nav_waypoints {"cmd":"nav_waypoints","coord_mode":"local",
                 "waypoints":[{"x":1.0,"y":2.0},{"x":3.0,"y":4.0,"yaw":1.57}],
                 "target_vel":0.5}

【导航指令 — 室外（WGS84 经纬度）】
  nav_goal      {"cmd":"nav_goal","coord_mode":"gps","lat":36.66123,"lon":117.01688}
  nav_waypoints {"cmd":"nav_waypoints","coord_mode":"gps",
                 "waypoints":[{"lat":36.66123,"lon":117.01688}],
                 "target_vel":0.8}

【通用指令】
  nav_cancel    {"cmd":"nav_cancel"}
  query_status  {"cmd":"query_status"}

【PCD 文件传输 (端口 9091)】
  pcd_info      {"cmd":"pcd_info"}
  pcd_chunk     {"cmd":"pcd_chunk","offset":0,"length":65536}
"""

import argparse
import base64
import json
import select
import socket
import sys
import time

HOST = "localhost"
PORT = 9090
PCD_PORT = 9091


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


def pcd_send(data: dict, timeout: float = 5.0) -> str:
    msg = json.dumps(data, ensure_ascii=False) + "\n"
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(timeout)
        s.connect((HOST, PCD_PORT))
        s.sendall(msg.encode("utf-8"))
        buf = b""
        try:
            while True:
                chunk = s.recv(65536 + 512)
                if not chunk:
                    break
                buf += chunk
                if b"\n" in buf:
                    break
        except socket.timeout:
            pass
    return buf.decode("utf-8", errors="replace").strip()


def send_and_print(data: dict):
    """发送指令并打印应答，返回解析后的 dict 或 None"""
    try:
        resp = tcp_send(data)
        try:
            j = json.loads(resp)
            print(f"  <- {json.dumps(j, ensure_ascii=False)}")
            return j
        except json.JSONDecodeError:
            print(f"  <- {resp}")
            return None
    except (ConnectionRefusedError, socket.timeout) as e:
        print(f"  ✗ 连接失败: {e}")
        return None


# ==================== 持久连接等待 nav_result ====================

def send_nav_and_wait(data: dict, timeout: float = 300.0) -> str:
    """
    发送导航指令，保持 TCP 连接等待 nav_result 推送。
    Bridge 在机器人到达（或出错）后向所有连接客户端广播 nav_result。
    期间会收到 5Hz 的位置推送，过滤掉，只等 nav_result。
    返回: 'REACHED' | 'ERROR' | 'TIMEOUT' | 'CONNECT_ERROR'
    """
    msg = json.dumps(data, ensure_ascii=False) + "\n"
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((HOST, PORT))
            s.sendall(msg.encode("utf-8"))

            buf = b""
            deadline = time.time() + timeout
            while time.time() < deadline:
                remaining = deadline - time.time()
                s.settimeout(min(remaining, 0.5))
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
                        # 实时位置推送: 在同一行刷新显示
                        state = j.get("state", "")
                        if "lat" in j:
                            pos = f"lat={j['lat']:.6f} lon={j['lon']:.6f}"
                        else:
                            pos = f"x={j.get('x',0.0):6.2f} y={j.get('y',0.0):6.2f}"
                        print(f"\r  [{state}] {pos}  yaw={j.get('yaw',0.0):.2f}  ",
                              end="", flush=True)
                    else:
                        # ack 或其他: 正常打印
                        print(f"  <- {json.dumps(j, ensure_ascii=False)}")

        return "TIMEOUT"
    except (ConnectionRefusedError, OSError) as e:
        print(f"  ✗ 连接失败: {e}")
        return "CONNECT_ERROR"


# ==================== 输入工具 ====================

def read_coord_mode() -> str:
    raw = input("坐标模式 [1=local 室内坐标, 2=gps 室外经纬度, 回车默认1]: ").strip()
    if raw in ("2", "gps", "g"):
        return "gps"
    return "local"


def read_local_point(prompt: str):
    while True:
        raw = input(prompt).strip()
        if not raw:
            return None
        parts = raw.replace(",", " ").split()
        if len(parts) < 2:
            print("  格式错误, 至少输入 x y")
            continue
        try:
            x = float(parts[0])
            y = float(parts[1])
            yaw = float(parts[2]) if len(parts) >= 3 else 0.0
            return x, y, yaw
        except ValueError:
            print("  数值格式错误")


def read_gps_point(prompt: str):
    while True:
        raw = input(prompt).strip()
        if not raw:
            return None
        parts = raw.replace(",", " ").split()
        if len(parts) < 2:
            print("  格式错误, 至少输入 lat lon")
            continue
        try:
            lat = float(parts[0])
            lon = float(parts[1])
            wp = {"lat": lat, "lon": lon}
            if len(parts) >= 3:
                wp["yaw"] = float(parts[2])
            return wp
        except ValueError:
            print("  数值格式错误")


def read_target_vel(default: float = 0.5) -> float:
    raw = input(f"目标速度 m/s [回车默认 {default}]: ").strip()
    try:
        return float(raw) if raw else default
    except ValueError:
        return default


# ==================== 命令处理函数 ====================

def do_lifecycle(subcmd: str):
    DESC = {
        "start_mapping":    "启动建图  (定位源 -> /Odometry)",
        "stop_mapping":     "停止建图",
        "start_indoor_loc": "切换室内定位模式  (定位源 -> /localization)",
        "stop_indoor_loc":  "停止室内定位节点",
        "start_outdoor":    "切换室外RTK模式  (定位源 -> /outdoor/odom, 直线规划)",
        "stop_outdoor":     "停止室外RTK, 切回室内定位",
    }
    print(f"  -> {DESC.get(subcmd, subcmd)}")
    send_and_print({"cmd": subcmd})


def do_goal():
    coord_mode = read_coord_mode()
    if coord_mode == "gps":
        wp = read_gps_point("目标点 (lat lon [yaw_rad], 回车取消): ")
        if wp is None:
            print("已取消")
            return
        data = {"cmd": "nav_goal", "coord_mode": "gps", **wp}
        print(f"  -> GPS单点导航: lat={wp['lat']}, lon={wp['lon']}")
    else:
        pt = read_local_point("目标点 (x y [yaw_rad], 回车取消): ")
        if pt is None:
            print("已取消")
            return
        x, y, yaw = pt
        data = {"cmd": "nav_goal", "coord_mode": "local", "x": x, "y": y, "yaw": yaw}
        print(f"  -> 本地单点导航: x={x}, y={y}, yaw={yaw}")
    print("  等待到达目标点 (含朝向对齐)...")
    result = send_nav_and_wait(data)
    print()  # 换行，避免覆盖实时位置行
    if result == "REACHED":
        print("  ★ 导航完成!")
    elif result == "ERROR":
        print("  ✗ 导航异常!")
    elif result == "TIMEOUT":
        print("  ✗ 等待超时")


def do_waypoints():
    coord_mode = read_coord_mode()
    waypoints = []
    print("逐个输入航点 (直接回车结束):")
    idx = 1
    while True:
        if coord_mode == "gps":
            wp = read_gps_point(f"  航点{idx} (lat lon [yaw_rad]): ")
            if wp is None:
                break
            waypoints.append(wp)
        else:
            pt = read_local_point(f"  航点{idx} (x y [yaw_rad]): ")
            if pt is None:
                break
            waypoints.append({"x": pt[0], "y": pt[1], "yaw": pt[2]})
        idx += 1

    if not waypoints:
        print("未输入航点, 已取消")
        return

    vel = read_target_vel()
    data = {
        "cmd": "nav_waypoints",
        "coord_mode": coord_mode,
        "waypoints": waypoints,
        "target_vel": vel,
    }
    if coord_mode == "gps":
        summary = " -> ".join(f"({w['lat']:.6f},{w['lon']:.6f})" for w in waypoints)
    else:
        summary = " -> ".join(f"({w['x']},{w['y']})" for w in waypoints)
    print(f"  -> 多点导航 ({len(waypoints)}点 {coord_mode}): {summary}, 速度={vel}m/s")
    print("  等待到达所有航点 (含末点朝向对齐)...")
    result = send_nav_and_wait(data)
    print()
    if result == "REACHED":
        print("  ★ 全部航点完成!")
    elif result == "ERROR":
        print("  ✗ 导航异常!")
    elif result == "TIMEOUT":
        print("  ✗ 等待超时")


def do_stop():
    print("  -> 取消导航")
    send_and_print({"cmd": "nav_cancel"})


def do_pause():
    print("  -> 暂停导航")
    send_and_print({"cmd": "nav_pause"})


def do_resume():
    print("  -> 继续导航")
    send_and_print({"cmd": "nav_resume"})


def do_realtime_pose():
    print("  实时位置显示中, 按回车退出...")
    try:
        while True:
            resp = tcp_send({"cmd": "query_status"}, timeout=2.0)
            j = json.loads(resp)
            # 新格式: {"type":"pose","x":...,"y":...,"yaw":...,"battery":...,"vx":...,"vy":...}
            #         或 GPS 模式: {"type":"pose","lat":...,"lon":...,"yaw":...,...}
            yaw = j.get("yaw", 0.0)
            bat = j.get("battery", 0.0)
            vx  = j.get("vx", 0.0)
            if "lat" in j:
                pos = f"lat={j['lat']:.7f}  lon={j['lon']:.7f}"
                mode = "gps"
            else:
                pos = f"x={j.get('x', 0.0):7.3f}  y={j.get('y', 0.0):7.3f}"
                mode = "slam"
            line = (f"\r  [{mode}] {pos}  yaw={yaw:6.3f}"
                    f"  bat={bat:.1f}V  vx={vx:.2f}m/s  ")
            print(line, end="", flush=True)
            if select.select([sys.stdin], [], [], 0.5)[0]:
                sys.stdin.readline()
                break
    except (ConnectionRefusedError, socket.timeout):
        print(f"\n  ✗ 连接失败")
    except (json.JSONDecodeError, KeyError):
        pass
    print()


def do_status():
    print("  -> 查询状态")
    try:
        resp = tcp_send({"cmd": "query_status"})
        print(json.dumps(json.loads(resp), indent=2, ensure_ascii=False))
    except json.JSONDecodeError:
        print(f"  <- {resp}")
    except (ConnectionRefusedError, socket.timeout) as e:
        print(f"  ✗ 连接失败: {e}")


def do_download_trans_pcd():
    print("  -> 查询 trans.pcd 文件信息")
    try:
        info = json.loads(pcd_send({"cmd": "pcd_info"}, timeout=3.0))
    except (ConnectionRefusedError, socket.timeout) as e:
        print(f"  ✗ PCD 端口连接失败: {e}")
        return
    except json.JSONDecodeError as e:
        print(f"  ✗ 返回非法 JSON: {e}")
        return

    if not info.get("exists", False):
        print("  ✗ trans.pcd 不存在")
        return

    total_size = int(info.get("size", 0))
    chunk_bytes = int(info.get("chunk_bytes", 65536))
    if total_size <= 0:
        print("  ✗ 文件大小为 0")
        return

    default_out = "trans_download.pcd"
    out_path = input(f"保存路径 [回车默认 {default_out}]: ").strip() or default_out
    print(f"  -> 开始下载: {total_size} bytes, chunk={chunk_bytes}")

    offset = 0
    with open(out_path, "wb") as f:
        while offset < total_size:
            req_len = min(chunk_bytes, total_size - offset)
            try:
                chunk = json.loads(pcd_send(
                    {"cmd": "pcd_chunk", "offset": offset, "length": req_len},
                    timeout=8.0,
                ))
            except Exception as e:
                print(f"\n  ✗ 下载中断: {e}")
                return

            if "error" in chunk:
                print(f"\n  ✗ 服务器错误: {chunk['error']}")
                return

            data = base64.b64decode(chunk.get("data_b64", ""))
            if not data and not chunk.get("eof", False):
                print("\n  ✗ 收到空分片")
                return

            f.write(data)
            offset += len(data)
            print(f"\r  进度: {100*offset/total_size:6.2f}% ({offset}/{total_size})",
                  end="", flush=True)
            if chunk.get("eof", False):
                break

    print(f"\n  ★ 下载完成: {out_path}")


# ==================== 主菜单 ====================

MENU = """
============ 导航控制 ============
  【生命周期】
  m1 - 开始建图
  m2 - 停止建图
  m3 - 切换室内定位模式
  m4 - 切换室外RTK模式
  m5 - 停止室外RTK (切回室内)
  【导航】
  1  - 单点导航 (local/gps)
  2  - 多点导航 (local/gps)
  3  - 取消导航 (清除任务)
  4  - 暂停导航 (车子停止保留任务)
  5  - 继续导航
  【状态】
  6  - 实时位置 (按回车退出)
  7  - 查询完整状态
  8  - 下载实时点云 trans.pcd
  q  - 退出
================================="""


def main():
    global HOST, PORT, PCD_PORT

    parser = argparse.ArgumentParser(description="导航控制交互脚本")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=9090)
    parser.add_argument("--pcd-port", type=int, default=9091)
    args = parser.parse_args()

    HOST = args.host
    PORT = args.port
    PCD_PORT = args.pcd_port

    print(f"Bridge: tcp://{HOST}:{PORT}  PCD: tcp://{HOST}:{PCD_PORT}")

    DISPATCH = {
        "m1": lambda: do_lifecycle("start_mapping"),
        "m2": lambda: do_lifecycle("stop_mapping"),
        "m3": lambda: do_lifecycle("start_indoor_loc"),
        "m4": lambda: do_lifecycle("start_outdoor"),
        "m5": lambda: do_lifecycle("stop_outdoor"),
        "1":  do_goal,
        "2":  do_waypoints,
        "3":  do_stop,
        "4":  do_pause,
        "5":  do_resume,
        "6":  do_realtime_pose,
        "7":  do_status,
        "8":  do_download_trans_pcd,
    }

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
