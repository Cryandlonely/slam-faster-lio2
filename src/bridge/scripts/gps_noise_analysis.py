#!/usr/bin/env python3
"""
GPS 静止噪声分析脚本 — 直接读取串口 Unicore 二进制协议
车辆静止时采集经纬度，分析噪声特性，辅助滤波参数整定

用法: python3 gps_noise_analysis.py [--port /dev/ttyUSB0] [--baud 115200] [--seconds 60]
"""

import argparse
import math
import struct
import time
import sys

SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE   = 115200

# ==================== Unicore 二进制协议 ====================
# 同步头
SYNC = bytes([0xAA, 0x44, 0x12])
HEADER_LEN   = 28   # BinaryHeader 固定 28 字节
BESTPOS_ID   = 42
BESTPOS_BODY = 72   # BestPosBody 字节数

# BinaryHeader layout (28 bytes, little-endian)
# sync1 sync2 sync3 header_length msg_id(u16) msg_type reserved1
# msg_length(u16) reserved2(u16) idle_time time_status
# week(u16) milliseconds(u32) reserved3(u32) bds_gps_offset(u16) reserved4(u16)
_HDR_FMT = "<BBBBBHBBHHBBHIII HH"

# BestPosBody layout (72 bytes)
# sol_status(u32) pos_type(u32) lat(f64) lon(f64) hgt(f64)
# undulation(f32) datum_id(u32)
# lat_sigma(f32) lon_sigma(f32) hgt_sigma(f32)
# stn_id(4s) diff_age(f32) sol_age(f32)
# num_svs num_soln_svs reserved*3 ext_sol_stat galileo_sig gps_glo_bds
_POS_FMT = "<IIdddfIfffI4sffBBBBBBBB"  # Note: stn_id as 4s

CRC32_TABLE = [0] * 256

def _init_crc32():
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = (crc >> 1) ^ 0xEDB88320 if (crc & 1) else crc >> 1
        CRC32_TABLE[i] = crc

def calc_crc32(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = CRC32_TABLE[(crc ^ b) & 0xFF] ^ (crc >> 8)
    return crc & 0xFFFFFFFF

_init_crc32()

# pos_type → fix_quality 映射 (参考 Unicore 表 9-47)
# 16=SINGLE, 17=PSRDIFF, 32=L1_FLOAT, 34=NARROW_FLOAT, 48=L1_INT, 50=NARROW_INT
_RTK_FIXED   = {50, 56}  # NARROW_INT / INS_RTKFIXED
_RTK_FLOAT   = {34, 49}

def pos_type_str(pt: int) -> str:
    if pt in _RTK_FIXED:  return "RTK固定"
    if pt in _RTK_FLOAT:  return "RTK浮动"
    if pt == 17:          return "差分"
    if pt == 16:          return "单点"
    return f"type={pt}"


# ==================== 数据采集 ====================

def collect_gps(port: str, baud: int, duration: float) -> list[dict]:
    try:
        import serial
    except ImportError:
        print("  ✗ 需要 pyserial: pip3 install pyserial")
        sys.exit(1)

    samples   = []
    buf       = bytearray()
    t0        = time.time()
    deadline  = t0 + duration
    last_print = t0

    print(f"  打开串口 {port}  波特率 {baud} ...")
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"  ✗ 串口打开失败: {e}")
        sys.exit(1)

    print(f"  采集中，共 {duration:.0f} 秒，按 Ctrl+C 提前结束...")
    try:
        while time.time() < deadline:
            raw = ser.read(512)
            if raw:
                buf.extend(raw)

            # 解析缓冲区
            while len(buf) >= HEADER_LEN + 4:
                # 搜索同步头
                idx = -1
                for i in range(len(buf) - 2):
                    if buf[i:i+3] == SYNC:
                        idx = i
                        break
                if idx < 0:
                    del buf[:-2]
                    break
                if idx > 0:
                    del buf[:idx]
                    continue

                if len(buf) < HEADER_LEN:
                    break

                header_len_field = buf[3]       # 通常 0x1C = 28
                msg_id  = struct.unpack_from("<H", buf, 4)[0]
                msg_len = struct.unpack_from("<H", buf, 8)[0]
                total   = header_len_field + msg_len + 4  # +4 CRC

                if len(buf) < total:
                    break

                # CRC 校验
                crc_data = bytes(buf[:header_len_field + msg_len])
                crc_recv = struct.unpack_from("<I", buf, header_len_field + msg_len)[0]
                if calc_crc32(crc_data) != crc_recv:
                    del buf[:3]   # 跳过坏帧同步头
                    continue

                # 只处理 BESTPOS
                if msg_id == BESTPOS_ID and msg_len >= BESTPOS_BODY:
                    body = bytes(buf[header_len_field : header_len_field + BESTPOS_BODY])
                    fields = struct.unpack_from("<IIdddfIfff", body, 0)
                    sol_status, pos_type, lat, lon, hgt, undulation, datum_id, \
                        lat_sigma, lon_sigma, hgt_sigma = fields

                    if sol_status == 0:   # SOL_COMPUTED
                        samples.append({
                            "t":         time.time() - t0,
                            "lat":       lat,
                            "lon":       lon,
                            "lat_sigma": lat_sigma,
                            "lon_sigma": lon_sigma,
                            "pos_type":  pos_type,
                        })

                del buf[:total]

            # 进度
            now = time.time()
            if now - last_print >= 1.0:
                remaining = max(0, deadline - now)
                pt_str = pos_type_str(samples[-1]["pos_type"]) if samples else "等待..."
                last_sigma = f"σ={samples[-1]['lat_sigma']*100:.1f}cm" if samples else ""
                print(f"\r  已采集 {len(samples):4d} 样本  剩余 {remaining:4.0f}s"
                      f"  [{pt_str}] {last_sigma}   ",
                      end="", flush=True)
                last_print = now

    except KeyboardInterrupt:
        print("\n  提前结束")
    finally:
        ser.close()

    print(f"\n  采集完成，共 {len(samples)} 个样本")
    return samples


# ==================== 数学分析 ====================

def deg_to_m(dlat: float, dlon: float, ref_lat: float) -> tuple[float, float]:
    """经纬度偏差(度) → 米"""
    dx = dlon * math.cos(math.radians(ref_lat)) * 111320.0
    dy = dlat * 111320.0
    return dx, dy


def analyze(samples: list[dict]) -> dict:
    if len(samples) < 2:
        return {}

    lats = [s["lat"] for s in samples]
    lons = [s["lon"] for s in samples]
    ts   = [s["t"]   for s in samples]

    ref_lat = sum(lats) / len(lats)
    ref_lon = sum(lons) / len(lons)

    # 转换为局部米坐标
    xs, ys = zip(*[deg_to_m(lat - ref_lat, lon - ref_lon, ref_lat)
                   for lat, lon in zip(lats, lons)])

    n     = len(xs)
    mean_x = sum(xs) / n
    mean_y = sum(ys) / n
    std_x  = math.sqrt(sum((v - mean_x)**2 for v in xs) / n)
    std_y  = math.sqrt(sum((v - mean_y)**2 for v in ys) / n)
    errors = [math.sqrt((x - mean_x)**2 + (y - mean_y)**2) for x, y in zip(xs, ys)]
    max_err   = max(errors)
    p95_err   = sorted(errors)[int(0.95 * n)]
    p99_err   = sorted(errors)[int(0.99 * n)]
    rms_err   = math.sqrt(sum(e**2 for e in errors) / n)

    # 硬件 sigma（RTK 接收机自报）
    lat_sigmas = [s.get("lat_sigma", 0) for s in samples]
    lon_sigmas = [s.get("lon_sigma", 0) for s in samples]
    avg_lat_sigma = sum(lat_sigmas) / n
    avg_lon_sigma = sum(lon_sigmas) / n

    # 定位类型分布
    from collections import Counter
    pt_counter = Counter(pos_type_str(s.get("pos_type", 0)) for s in samples)

    # 采样频率
    dt_list = [ts[i+1] - ts[i] for i in range(n - 1)]
    avg_dt  = sum(dt_list) / len(dt_list)
    avg_hz  = 1.0 / avg_dt if avg_dt > 0 else 0.0

    # 自相关 (lag=1)
    def autocorr(seq, lag=1):
        mean = sum(seq) / len(seq)
        denom = sum((v - mean)**2 for v in seq)
        if denom < 1e-12:
            return 0.0
        numer = sum((seq[i] - mean) * (seq[i - lag] - mean)
                    for i in range(lag, len(seq)))
        return numer / denom

    ac_x = autocorr(list(xs))
    ac_y = autocorr(list(ys))

    return {
        "n": n, "avg_hz": avg_hz,
        "ref_lat": ref_lat, "ref_lon": ref_lon,
        "mean_x": mean_x, "mean_y": mean_y,
        "std_x": std_x,   "std_y": std_y,
        "avg_lat_sigma": avg_lat_sigma, "avg_lon_sigma": avg_lon_sigma,
        "rms_err": rms_err, "max_err": max_err,
        "p95_err": p95_err, "p99_err": p99_err,
        "ac_x": ac_x, "ac_y": ac_y,
        "pt_counter": dict(pt_counter),
        "xs": list(xs), "ys": list(ys),
        "ts": list(ts), "errors": errors,
    }


def print_summary(r: dict):
    print()
    print("=" * 56)
    print("  GPS 静止噪声分析报告")
    print("=" * 56)
    print(f"  样本数:          {r['n']}")
    print(f"  平均采样率:      {r['avg_hz']:.2f} Hz")
    print(f"  参考点:          lat={r['ref_lat']:.8f}  lon={r['ref_lon']:.8f}")
    print()
    print("  ── 定位类型分布 ──────────────────────────────")
    for pt, cnt in sorted(r["pt_counter"].items(), key=lambda x: -x[1]):
        pct = cnt / r["n"] * 100
        print(f"    {pt:<12s} {cnt:4d} 次  ({pct:.1f}%)")
    print()
    print("  ── 实际误差统计 ──────────────────────────────")
    print(f"  X(东西) 标准差:   {r['std_x']*100:.1f} cm")
    print(f"  Y(南北) 标准差:   {r['std_y']*100:.1f} cm")
    print(f"  2D RMS 误差:     {r['rms_err']*100:.1f} cm")
    print(f"  最大偏差:         {r['max_err']*100:.1f} cm")
    print(f"  95% 误差范围:    {r['p95_err']*100:.1f} cm")
    print(f"  99% 误差范围:    {r['p99_err']*100:.1f} cm")
    print()
    print("  ── 接收机自报 σ（仅供参考） ──────────────────")
    print(f"  纬度 σ 均值:      {r['avg_lat_sigma']*100:.1f} cm")
    print(f"  经度 σ 均值:      {r['avg_lon_sigma']*100:.1f} cm")
    print()
    print(f"  X 自相关(lag1): {r['ac_x']:.3f}  {'⚠ 时序相关' if abs(r['ac_x']) > 0.5 else '✓ 接近白噪声'}")
    print(f"  Y 自相关(lag1): {r['ac_y']:.3f}  {'⚠ 时序相关' if abs(r['ac_y']) > 0.5 else '✓ 接近白噪声'}")
    print()
    cte_dead = round(r['p95_err'] * 1.5, 2)
    goal_tol = round(r['p95_err'] * 2.5, 2)
    print("  ── 导航参数建议 ──────────────────────────────")
    print(f"  cte_dead_zone:  {max(cte_dead, 0.3):.2f} m  (95%误差 × 1.5)")
    print(f"  goal_tolerance: {max(goal_tol, 0.5):.2f} m  (95%误差 × 2.5)")
    if abs(r['ac_x']) > 0.5 or abs(r['ac_y']) > 0.5:
        print("  噪声时序相关 → 建议卡尔曼滤波或增大低通时间常数")
    else:
        print("  噪声接近白噪声 → 简单低通滤波即可")
    print("=" * 56)


# ==================== 绘图 ====================

def plot_analysis(r: dict):
    try:
        import matplotlib.pyplot as plt
        import matplotlib.gridspec as gridspec
        import numpy as np
    except ImportError:
        print("  ✗ 需要 matplotlib/numpy: pip3 install matplotlib numpy")
        return

    xs  = np.array(r["xs"])
    ys  = np.array(r["ys"])
    ts  = np.array(r["ts"])
    err = np.array(r["errors"])

    fig = plt.figure(figsize=(14, 10))
    fig.suptitle("GPS 静止噪声分析", fontsize=14, fontweight="bold")
    gs  = gridspec.GridSpec(3, 3, figure=fig, hspace=0.45, wspace=0.35)

    # ── 1. 2D 散点图 ──────────────────────────────────────
    ax1 = fig.add_subplot(gs[0:2, 0:2])
    sc  = ax1.scatter(xs * 100, ys * 100, c=ts, cmap="viridis",
                      s=8, alpha=0.6, label="GPS 样本")
    plt.colorbar(sc, ax=ax1, label="时间 (s)")
    circle_95 = plt.Circle((r["mean_x"]*100, r["mean_y"]*100),
                             r["p95_err"]*100, color="orange",
                             fill=False, linestyle="--", label="95% 范围")
    circle_99 = plt.Circle((r["mean_x"]*100, r["mean_y"]*100),
                             r["p99_err"]*100, color="red",
                             fill=False, linestyle=":", label="99% 范围")
    ax1.add_patch(circle_95)
    ax1.add_patch(circle_99)
    ax1.axhline(0, color="gray", linewidth=0.5)
    ax1.axvline(0, color="gray", linewidth=0.5)
    ax1.set_xlabel("X 偏差 (cm)")
    ax1.set_ylabel("Y 偏差 (cm)")
    ax1.set_title("2D 位置散点")
    ax1.set_aspect("equal")
    ax1.legend(fontsize=7)
    ax1.grid(True, alpha=0.3)

    # ── 2. 误差时间序列 ──────────────────────────────────
    ax2 = fig.add_subplot(gs[0, 2])
    ax2.plot(ts, xs * 100, linewidth=0.8, label="X")
    ax2.plot(ts, ys * 100, linewidth=0.8, label="Y")
    ax2.axhline(0, color="gray", linewidth=0.5)
    ax2.set_xlabel("时间 (s)")
    ax2.set_ylabel("偏差 (cm)")
    ax2.set_title("X/Y 时间序列")
    ax2.legend(fontsize=7)
    ax2.grid(True, alpha=0.3)

    # ── 3. 2D 误差时间序列 ────────────────────────────────
    ax3 = fig.add_subplot(gs[1, 2])
    ax3.plot(ts, err * 100, linewidth=0.8, color="purple")
    ax3.axhline(r["p95_err"] * 100, color="orange", linestyle="--",
                linewidth=1, label=f"95% {r['p95_err']*100:.1f}cm")
    ax3.axhline(r["p99_err"] * 100, color="red", linestyle=":",
                linewidth=1, label=f"99% {r['p99_err']*100:.1f}cm")
    ax3.set_xlabel("时间 (s)")
    ax3.set_ylabel("2D误差 (cm)")
    ax3.set_title("2D 误差序列")
    ax3.legend(fontsize=7)
    ax3.grid(True, alpha=0.3)

    # ── 4. 误差直方图 ─────────────────────────────────────
    ax4 = fig.add_subplot(gs[2, 0])
    ax4.hist(err * 100, bins=40, color="steelblue", edgecolor="white",
             linewidth=0.3, density=True)
    # 叠加瑞利分布（2D 高斯的理论分布）
    sigma = (r["std_x"] + r["std_y"]) / 2 * 100
    x_fit = np.linspace(0, max(err) * 100 * 1.1, 200)
    rayleigh = (x_fit / sigma**2) * np.exp(-x_fit**2 / (2 * sigma**2))
    ax4.plot(x_fit, rayleigh, "r-", linewidth=1.5, label="Rayleigh 理论")
    ax4.set_xlabel("2D 误差 (cm)")
    ax4.set_ylabel("概率密度")
    ax4.set_title("误差分布")
    ax4.legend(fontsize=7)
    ax4.grid(True, alpha=0.3)

    # ── 5. 自相关图 ───────────────────────────────────────
    ax5 = fig.add_subplot(gs[2, 1])
    max_lag = min(50, len(xs) // 4)
    lags = range(1, max_lag + 1)

    def autocorr_series(seq, max_lag):
        mean = np.mean(seq)
        denom = np.sum((seq - mean)**2)
        return [np.sum((seq[lag:] - mean) * (seq[:-lag] - mean)) / denom
                for lag in range(1, max_lag + 1)]

    ac_xs = autocorr_series(xs, max_lag)
    ac_ys = autocorr_series(ys, max_lag)
    ax5.plot(list(lags), ac_xs, label="X", linewidth=1)
    ax5.plot(list(lags), ac_ys, label="Y", linewidth=1)
    conf = 1.96 / math.sqrt(len(xs))
    ax5.axhline(conf,  color="gray", linestyle="--", linewidth=0.8)
    ax5.axhline(-conf, color="gray", linestyle="--", linewidth=0.8)
    ax5.axhline(0, color="black", linewidth=0.5)
    ax5.set_xlabel("Lag (帧)")
    ax5.set_ylabel("自相关系数")
    ax5.set_title("自相关 (>虚线=相关)")
    ax5.legend(fontsize=7)
    ax5.grid(True, alpha=0.3)

    # ── 6. 功率谱 ─────────────────────────────────────────
    ax6 = fig.add_subplot(gs[2, 2])
    n   = len(xs)
    fft_x = np.abs(np.fft.rfft(xs - np.mean(xs)))**2
    fft_y = np.abs(np.fft.rfft(ys - np.mean(ys)))**2
    freqs = np.fft.rfftfreq(n, d=1.0 / r["avg_hz"])
    ax6.semilogy(freqs[1:], fft_x[1:], label="X", linewidth=0.8)
    ax6.semilogy(freqs[1:], fft_y[1:], label="Y", linewidth=0.8)
    ax6.set_xlabel("频率 (Hz)")
    ax6.set_ylabel("功率 (对数)")
    ax6.set_title("功率谱密度")
    ax6.legend(fontsize=7)
    ax6.grid(True, alpha=0.3)

    plt.savefig("/tmp/gps_noise_analysis.png", dpi=150, bbox_inches="tight")
    print("  图表已保存: /tmp/gps_noise_analysis.png")
    plt.show()


# ==================== 主入口 ====================

def main():
    parser = argparse.ArgumentParser(description="GPS 静止噪声分析（直读串口）")
    parser.add_argument("--port",    default=SERIAL_PORT, help="串口设备 (默认 /dev/ttyUSB0)")
    parser.add_argument("--baud",    type=int, default=BAUD_RATE, help="波特率 (默认 115200)")
    parser.add_argument("--seconds", type=float, default=60.0, help="采集时长，秒 (默认 60)")
    args = parser.parse_args()

    print(f"GPS 噪声分析 — 串口: {args.port}  波特率: {args.baud}")
    print("请保持车辆静止!")
    print()

    samples = collect_gps(args.port, args.baud, args.seconds)

    if len(samples) < 10:
        print("  样本数不足，无法分析")
        return

    r = analyze(samples)
    print_summary(r)
    plot_analysis(r)


if __name__ == "__main__":
    main()
