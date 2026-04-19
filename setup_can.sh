#!/bin/bash
# CAN 接口开机自启动配置脚本
# 用法: sudo bash setup_can.sh

set -e

CAN_IF="can0"
BITRATE=500000
SERVICE_NAME="can-setup"

if [ "$EUID" -ne 0 ]; then
    echo "请使用 sudo 运行: sudo bash $0"
    exit 1
fi

echo "=== 配置 ${CAN_IF} 开机自启动 (bitrate=${BITRATE}) ==="

# 创建 systemd 服务文件
cat > /etc/systemd/system/${SERVICE_NAME}.service << EOF
[Unit]
Description=CAN Bus Interface Setup
After=network-pre.target
Wants=network-pre.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/sbin/ip link set ${CAN_IF} type can bitrate ${BITRATE}
ExecStart=/sbin/ip link set ${CAN_IF} up
ExecStop=/sbin/ip link set ${CAN_IF} down

[Install]
WantedBy=multi-user.target
EOF

# 重载并启用服务
systemctl daemon-reload
systemctl enable ${SERVICE_NAME}.service

echo "=== 立即启动 ${CAN_IF} ==="
systemctl start ${SERVICE_NAME}.service

echo ""
echo "完成! ${CAN_IF} 已配置并启用开机自启动"
echo "  查看状态: systemctl status ${SERVICE_NAME}"
echo "  停用自启: sudo systemctl disable ${SERVICE_NAME}"
echo "  手动重启: sudo systemctl restart ${SERVICE_NAME}"

ip link show ${CAN_IF}
