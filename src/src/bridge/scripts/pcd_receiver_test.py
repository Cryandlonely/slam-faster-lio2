#!/usr/bin/env python3
"""
PCD receiver test client for bridge PCD transfer port.

Features:
- Query remote trans.pcd metadata (pcd_info)
- Download file by chunks (pcd_chunk)
- Optional watch mode: poll metadata and auto-download when file changes

Protocol (JSON Lines):
  {"cmd":"pcd_info"}
  {"cmd":"pcd_chunk","offset":0,"length":65536}
"""

import argparse
import base64
import json
import os
import socket
import sys
import time
from typing import Dict, Any, Tuple, Optional


class PcdViewer:
    def __init__(self) -> None:
        self.o3d = None
        self.vis = None
        self.geom = None
        self.ready = False

    def init(self) -> None:
        if self.ready:
            return
        try:
            import open3d as o3d  # type: ignore
        except Exception as e:
            raise RuntimeError(
                "Open3D not available. Install with: pip3 install open3d"
            ) from e

        self.o3d = o3d
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name="trans.pcd receiver", width=1024, height=768)
        self.geom = o3d.geometry.PointCloud()
        self.vis.add_geometry(self.geom)
        self.ready = True

    def update_from_file(self, pcd_path: str) -> None:
        if not self.ready:
            self.init()
        assert self.o3d is not None
        assert self.vis is not None
        assert self.geom is not None

        cloud = self.o3d.io.read_point_cloud(pcd_path)
        self.geom.points = cloud.points
        self.geom.colors = cloud.colors
        self.geom.normals = cloud.normals
        self.vis.update_geometry(self.geom)
        self.vis.reset_view_point(True)
        self.vis.poll_events()
        self.vis.update_renderer()

    def spin_once(self) -> None:
        if self.ready and self.vis is not None:
            self.vis.poll_events()
            self.vis.update_renderer()

    def close(self) -> None:
        if self.ready and self.vis is not None:
            self.vis.destroy_window()
            self.ready = False


class PcdConnection:
    """持久化TCP连接，复用socket避免频繁建立/断开连接。"""

    def __init__(self, host: str, port: int, timeout: float = 8.0) -> None:
        self.host = host
        self.port = port
        self.timeout = timeout
        self._sock: Optional[socket.socket] = None
        self._buf = b""

    def _connect(self) -> None:
        if self._sock is not None:
            return
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(self.timeout)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.connect((self.host, self.port))
        self._sock = s
        self._buf = b""

    def _close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None
            self._buf = b""

    def send(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """发送请求并等待一行JSON响应，连接断开时自动重连一次。"""
        for attempt in range(2):
            try:
                self._connect()
                assert self._sock is not None
                msg = json.dumps(payload, ensure_ascii=False) + "\n"
                self._sock.sendall(msg.encode("utf-8"))

                while b"\n" not in self._buf:
                    chunk = self._sock.recv(65536)
                    if not chunk:
                        raise ConnectionResetError("server closed connection")
                    self._buf += chunk

                line, self._buf = self._buf.split(b"\n", 1)
                raw = line.decode("utf-8", errors="replace").strip()
                if not raw:
                    raise RuntimeError("empty response")
                return json.loads(raw)

            except (ConnectionResetError, BrokenPipeError, OSError) as e:
                self._close()
                if attempt == 1:
                    raise RuntimeError(f"connection failed after retry: {e}") from e

        raise RuntimeError("unreachable")

    def close(self) -> None:
        self._close()

    def __enter__(self) -> "PcdConnection":
        return self

    def __exit__(self, *_: Any) -> None:
        self._close()


def fetch_info(conn: "PcdConnection") -> Dict[str, Any]:
    rsp = conn.send({"cmd": "pcd_info"})
    if "error" in rsp:
        raise RuntimeError(f"pcd_info error: {rsp['error']}")
    return rsp


def download_once(conn: "PcdConnection", out_path: str) -> Tuple[int, int]:
    info = fetch_info(conn)
    if not info.get("exists", False):
        raise RuntimeError("remote trans.pcd not found")

    total_size = int(info.get("size", 0))
    chunk_bytes = int(info.get("chunk_bytes", 65536))
    if total_size <= 0:
        raise RuntimeError("remote trans.pcd size is 0")
    if chunk_bytes <= 0:
        chunk_bytes = 65536

    out_dir = os.path.dirname(out_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    offset = 0
    with open(out_path, "wb") as f:
        while offset < total_size:
            req_len = min(chunk_bytes, total_size - offset)
            rsp = conn.send({"cmd": "pcd_chunk", "offset": offset, "length": req_len})
            if "error" in rsp:
                raise RuntimeError(f"pcd_chunk error: {rsp['error']}")

            data_b64 = rsp.get("data_b64", "")
            try:
                data = base64.b64decode(data_b64)
            except Exception as e:
                raise RuntimeError(f"base64 decode failed: {e}") from e

            got = len(data)
            if got == 0 and not rsp.get("eof", False):
                raise RuntimeError("received empty chunk before eof")

            f.write(data)
            offset += got

            progress = min(100.0, 100.0 * offset / total_size)
            print(f"\rDownloading: {progress:6.2f}% ({offset}/{total_size})", end="", flush=True)

            if rsp.get("eof", False):
                break

    print()
    mtime_ns = int(info.get("mtime_ns", 0))
    return total_size, mtime_ns


def watch_mode(host: str, port: int, out_path: str, interval: float, view: bool = False) -> None:
    print(f"Watch mode: host={host}, pcd_port={port}, out={out_path}, interval={interval}s")
    last_sig = None
    viewer: Optional[PcdViewer] = PcdViewer() if view else None

    if viewer is not None:
        viewer.init()

    with PcdConnection(host, port) as conn:
      while True:
        try:
            info = fetch_info(conn)
            exists = info.get("exists", False)
            size = int(info.get("size", 0))
            mtime_ns = int(info.get("mtime_ns", 0))
            sig = (exists, size, mtime_ns)

            if not exists:
                print("[watch] remote file not found, waiting...")
            elif sig != last_sig:
                print(f"[watch] change detected: size={size}, mtime_ns={mtime_ns}")
                start = time.time()
                got_size, got_mtime = download_once(conn, out_path)
                dt = time.time() - start
                print(f"[watch] updated: {out_path} ({got_size} bytes, mtime_ns={got_mtime}, {dt:.2f}s)")
                last_sig = (True, got_size, got_mtime)
                if viewer is not None:
                    viewer.update_from_file(out_path)
            else:
                print(f"[watch] no change (size={size}, mtime_ns={mtime_ns})")

        except KeyboardInterrupt:
            print("\nStopped by user.")
            if viewer is not None:
                viewer.close()
            return
        except Exception as e:
            print(f"[watch] error: {e}")

        # 在等待interval期间持续驱动渲染，避免窗口卡死
        if viewer is not None:
            deadline = time.time() + interval
            while time.time() < deadline:
                viewer.spin_once()
                time.sleep(0.033)  # ~30fps
        else:
            time.sleep(interval)


def main() -> int:
    parser = argparse.ArgumentParser(description="PCD receiver test client")
    parser.add_argument("--host", default="127.0.0.1", help="Bridge host")
    parser.add_argument("--pcd-port", type=int, default=9091, help="PCD transfer port")
    parser.add_argument("--out", default="trans_download.pcd", help="Local output file")
    parser.add_argument("--watch", action="store_true", help="Watch remote changes and auto-download")
    parser.add_argument("--interval", type=float, default=1.0, help="Watch interval in seconds")
    parser.add_argument("--view", action="store_true", help="Visualize point cloud with Open3D")

    args = parser.parse_args()

    try:
        if args.watch:
            watch_mode(args.host, args.pcd_port, args.out, max(0.2, args.interval), args.view)
        else:
            print(f"Downloading from {args.host}:{args.pcd_port} -> {args.out}")
            with PcdConnection(args.host, args.pcd_port) as conn:
                size, mtime_ns = download_once(conn, args.out)
            print(f"Done: {args.out}, size={size}, mtime_ns={mtime_ns}")
            if args.view:
                viewer = PcdViewer()
                viewer.init()
                viewer.update_from_file(args.out)
                print("Viewer running. Press Ctrl+C to exit.")
                try:
                    while True:
                        viewer.spin_once()
                        time.sleep(0.03)
                except KeyboardInterrupt:
                    viewer.close()
        return 0
    except KeyboardInterrupt:
        print("\nCancelled.")
        return 130
    except Exception as e:
        print(f"Failed: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
