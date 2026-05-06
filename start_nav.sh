#!/bin/bash
# ============================================================
#  室内SLAM导航 一键启动脚本
#  用法:
#    ./start_nav.sh          # 启动全部模块
#    ./start_nav.sh stop     # 停止全部模块
# ============================================================

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

# 自动重启配置
MAX_RESTARTS=10   # 单个模块最多重启次数, 超过后不再重启
RESTART_DELAY=3   # 重启前等待秒数, 避免崩溃风暴

# 内存中的模块状态（关联数组）
declare -A MODULE_PIDS     # name → pid
declare -A MODULE_CMDS     # name → bash 命令
declare -A MODULE_RESTARTS # name → 已重启次数

# ============================================================
#  更新 PID 文件 (供 stop_all 使用)
# ============================================================
_update_pid_file() {
    > "$PID_FILE"
    for _name in "${!MODULE_PIDS[@]}"; do
        echo "${MODULE_PIDS[$_name]} $_name" >> "$PID_FILE"
    done
}

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
#  内部启动函数: 后台运行并记录到关联数组
# ============================================================
_do_start() {
    local name="$1"
    local cmd="$2"
    local logfile="$LOG_DIR/${name}.log"

    mkdir -p "$LOG_DIR"
    bash -c "source $SETUP_FILE && $cmd" >> "$logfile" 2>&1 &
    MODULE_PIDS["$name"]=$!
    MODULE_CMDS["$name"]="$cmd"
    _update_pid_file
}

# ============================================================
#  首次启动单个模块 (带延迟打印)
# ============================================================
launch_module() {
    local name="$1"
    local cmd="$2"
    local delay="${3:-0}"

    if [[ "$delay" -gt 0 ]]; then
        echo -e "  等待 ${delay}s..."
        sleep "$delay"
    fi

    echo -e "  启动 ${CYAN}$name${NC} → 日志: $LOG_DIR/${name}.log"
    MODULE_RESTARTS["$name"]=0
    _do_start "$name" "$cmd"
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
echo -e "${GREEN}   室外SLAM导航系统 一键启动${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# 清空 PID 文件
> "$PID_FILE"

# ---- 1. LiDAR 驱动 ----
echo -e "${YELLOW}[1/5] LiDAR 驱动${NC}"
launch_module "livox_driver" "ros2 launch livox_ros_driver2 msg_MID360s_launch.py"

# ---- 2. IMU 驱动 ----
echo -e "${YELLOW}[2/5] IMU 驱动${NC}"
launch_module "imu" "ros2 launch vru_sensor vru.launch.py" 1

# ---- 3. 底盘驱动 ----
echo -e "${YELLOW}[3/5] 底盘驱动${NC}"
launch_module "chassis" "ros2 launch chassis chassis_launch.py" 1

# ---- 4. 导航规划 + RTK ----
echo -e "${YELLOW}[4/5] 导航规划 + RTK 定位${NC}"
launch_module "nav_planner" "ros2 launch nav_planner slam_nav_debug_launch.py" 2

# ---- 5. TCP 桥接 ----
echo -e "${YELLOW}[5/5] TCP 桥接${NC}"
launch_module "bridge" "ros2 launch bridge bridge_launch.py" 1

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}   全部模块已启动! (RTK 已随导航节点自动启动)${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "  日志目录: ${CYAN}$LOG_DIR/${NC}"
echo -e "  查看日志: ${CYAN}tail -f $LOG_DIR/<模块名>.log${NC}"
echo -e "  停止全部: ${CYAN}./start_nav.sh stop${NC}"
echo -e "  导航控制: ${CYAN}cd src/bridge/scripts && python3 nav_cmd.py${NC}"
echo ""

# 注册退出信号
echo -e "${YELLOW}按 Ctrl+C 停止所有模块${NC}"
trap 'echo ""; stop_all; exit 0' INT TERM

# ============================================================
#  监控 + 自动重启循环
# ============================================================
while true; do
    sleep 5

    for mod_name in "${!MODULE_PIDS[@]}"; do
        mod_pid="${MODULE_PIDS[$mod_name]}"
        if ! kill -0 "$mod_pid" 2>/dev/null; then
            mod_count="${MODULE_RESTARTS[$mod_name]:-0}"
            if [[ "$mod_count" -lt "$MAX_RESTARTS" ]]; then
                MODULE_RESTARTS["$mod_name"]=$((mod_count + 1))
                echo -e "${YELLOW}[重启 $((mod_count+1))/$MAX_RESTARTS] ${CYAN}${mod_name}${YELLOW} 已退出，${RESTART_DELAY}s 后重启...${NC}"
                sleep "$RESTART_DELAY"
                _do_start "$mod_name" "${MODULE_CMDS[$mod_name]}"
                echo -e "${GREEN}[重启成功] ${mod_name} PID=${MODULE_PIDS[$mod_name]}${NC}"
            else
                echo -e "${RED}[放弃重启] ${mod_name} 已达最大重启次数 ($MAX_RESTARTS)，请检查日志: $LOG_DIR/${mod_name}.log${NC}"
                unset "MODULE_PIDS[$mod_name]"
                _update_pid_file
            fi
        fi
    done
done
