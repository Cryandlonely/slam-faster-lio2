#!/bin/bash

# 一键永久固定 RTK 设备为 /dev/ttyACM3
# 适配你的设备：idProduct=0001 devpath=2

echo "========================================"
echo "  一键固定 RTK /dev/ttyACM3 中..."
echo "========================================"

# 写入 udev 规则
sudo tee /etc/udev/rules.d/99-rtk-force-ttyACM3.rules > /dev/null << EOF
# 强制固定 RTK 设备为 /dev/ttyACM3
SUBSYSTEM=="tty", SUBSYSTEMS=="usb", ATTRS{idProduct}=="0001", ATTRS{devpath}=="2", NAME="ttyACM3", MODE="0666", GROUP="dialout"
EOF

# 重新加载规则
sudo udevadm control --reload-rules
sudo udevadm trigger

echo ""
echo "✅ 配置完成！"
echo "✅ 设备已强制固定为：/dev/ttyACM3"
echo "✅ 以后无论怎么插拔、重枚举，都不会变成 ttyACM4 了！"
echo ""
echo "请重新插拔 RTK USB 设备生效！"