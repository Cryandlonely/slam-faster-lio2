#!/bin/bash
# ============================================================
#  室内SLAM导航 一键启动脚本
#  用法:
#    ./start_nav.sh          # 启动全部模块
#    ./start_nav.sh stop     # 停止全部模块
# ============================================================

set -e

WS_DIR="$(cd "$(dirname "$0")" && pwd)"
SETUP_FILE="$WS_DIR/install/setup.bash"
LOG_DIR="$WS_DIR/nav_logs"

# 颜色
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

# PID 文件
PID_FILE="$WS_DIR/.nav_pids"

# ============================================================
#  停止所有模块
# ============================================================
stop_all() {
    echo -e "${YELLOW}正在停止所有导航模块...${NC}"
    if [[ -f "$PID_FILE" ]]; then
        while read -r pid name; do
            if kill -0 "$pid" 2>/dev/null; then
                echo -e "  停止 ${CYAN}$name${NC} (PID=$pid)"
                kill -INT "$pid" 2>/dev/null || true
            fi
        done < "$PID_FILE"
        sleep 2
        # 确保全部终止
        while read -r pid name; do
            if kill -0 "$pid" 2>/dev/null; then
                kill -9 "$pid" 2>/dev/null || true
            fi
        done < "$PID_FILE"
        rm -f "$PID_FILE"
    fi
    echo -e "${GREEN}全部已停止${NC}"
}

# ============================================================
#  启动单个模块 (后台运行)
# ============================================================
launch_module() {
    local name="$1"
    local cmd="$2"
    local delay="${3:-0}"

    if [[ "$delay" -gt 0 ]]; then
        echo -e "  等待 ${delay}s..."
        sleep "$delay"
    fi

    mkdir -p "$LOG_DIR"
    local logfile="$LOG_DIR/${name}.log"

    echo -e "  启动 ${CYAN}$name${NC} → 日志: $logfile"
    bash -c "source $SETUP_FILE && $cmd" > "$logfile" 2>&1 &
    local pid=$!
    echo "$pid $name" >> "$PID_FILE"
}

# ============================================================
#  主逻辑
# ============================================================

# 停止命令
if [[ "$1" == "stop" ]]; then
    stop_all
    exit 0
fi

# 检查 install 目录
if [[ ! -f "$SETUP_FILE" ]]; then
    echo -e "${RED}错误: 未找到 $SETUP_FILE${NC}"
    echo "请先执行: cd $WS_DIR && colcon build && source install/setup.bash"
    exit 1
fi

# 先停掉旧进程
if [[ -f "$PID_FILE" ]]; then
    stop_all
fi

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}   室内SLAM导航系统 一键启动${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# 清空 PID 文件
> "$PID_FILE"

# ---- 1. LiDAR 驱动 ----
echo -e "${YELLOW}[1/6] LiDAR 驱动${NC}"
launch_module "livox_driver" "ros2 launch livox_ros_driver2 msg_MID360s_launch.py"

# ---- 2. SLAM 定位 ----
echo -e "${YELLOW}[2/6] SLAM 定位${NC}"
launch_module "location" "ros2 launch location velodyne_localization.launch.py" 2

# ---- 3. 地图服务 ----
echo -e "${YELLOW}[3/6] 地图服务${NC}"
launch_module "pcd2pgm" "ros2 launch pcd2pgm pcd2pgm_rviz_launch.py" 2

# ---- 4. 底盘驱动 ----
echo -e "${YELLOW}[4/6] 底盘驱动${NC}"
launch_module "chassis" "ros2 launch chassis chassis_launch.py" 1

# ---- 5. 导航规划 ----
echo -e "${YELLOW}[5/6] 导航规划${NC}"
launch_module "nav_planner" "ros2 launch nav_planner slam_nav_launch.py" 2

# ---- 6. TCP 桥接 ----
echo -e "${YELLOW}[6/6] TCP 桥接${NC}"
launch_module "bridge" "ros2 launch bridge bridge_launch.py" 1

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}   全部模块已启动!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "  日志目录: ${CYAN}$LOG_DIR/${NC}"
echo -e "  查看日志: ${CYAN}tail -f $LOG_DIR/<模块名>.log${NC}"
echo -e "  停止全部: ${CYAN}./start_nav.sh stop${NC}"
echo -e "  导航控制: ${CYAN}cd src/bridge/scripts && python3 nav_cmd.py${NC}"
echo ""

# 等待用户按 Ctrl+C
echo -e "${YELLOW}按 Ctrl+C 停止所有模块${NC}"
trap 'echo ""; stop_all; exit 0' INT TERM

# 监控进程状态
while true; do
    sleep 5
    failed=""
    while read -r pid name; do
        if ! kill -0 "$pid" 2>/dev/null; then
            failed="$failed $name"
        fi
    done < "$PID_FILE"
    if [[ -n "$failed" ]]; then
        echo -e "${RED}[警告] 以下模块已退出:${failed}${NC}"
    fi
done
