#!/bin/bash
# ============================================================
# 一键安装/卸载开机自启动服务
# 用法:
#   ./install_autostart.sh          # 安装并启用服务
#   ./install_autostart.sh remove   # 停止并卸载服务
# ============================================================

SERVICE_NAME="slam-autostart"
SERVICE_FILE="$(dirname "$(realpath "$0")")/slam-autostart.service"
SYSTEMD_DIR="/etc/systemd/system"
START_SCRIPT="$(dirname "$(realpath "$0")")/start_all.sh"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; }

if [[ "$EUID" -ne 0 ]]; then
    error "请使用 sudo 运行此脚本: sudo $0 $*"
    exit 1
fi

# ---------- 卸载 ----------
if [[ "$1" == "remove" ]]; then
    info "停止并禁用服务 ${SERVICE_NAME} ..."
    systemctl stop  "${SERVICE_NAME}" 2>/dev/null || true
    systemctl disable "${SERVICE_NAME}" 2>/dev/null || true
    rm -f "${SYSTEMD_DIR}/${SERVICE_NAME}.service"
    systemctl daemon-reload
    info "服务已卸载"
    exit 0
fi

# ---------- 安装 ----------
if [[ ! -f "$SERVICE_FILE" ]]; then
    error "未找到服务文件: $SERVICE_FILE"
    exit 1
fi

if [[ ! -f "$START_SCRIPT" ]]; then
    error "未找到启动脚本: $START_SCRIPT"
    exit 1
fi

# 确保脚本可执行
chmod +x "$START_SCRIPT"

info "复制服务文件到 ${SYSTEMD_DIR} ..."
cp "$SERVICE_FILE" "${SYSTEMD_DIR}/${SERVICE_NAME}.service"

info "重新加载 systemd ..."
systemctl daemon-reload

info "启用开机自启 ..."
systemctl enable "${SERVICE_NAME}"

info "立即启动服务 ..."
systemctl start "${SERVICE_NAME}"

echo ""
info "安装完成！常用管理命令:"
echo "  查看状态: sudo systemctl status ${SERVICE_NAME}"
echo "  查看日志: sudo journalctl -u ${SERVICE_NAME} -f"
echo "  手动停止: sudo systemctl stop ${SERVICE_NAME}"
echo "  手动启动: sudo systemctl start ${SERVICE_NAME}"
echo "  禁用自启: sudo systemctl disable ${SERVICE_NAME}"
echo "  卸载服务: sudo $0 remove"
