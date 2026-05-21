#!/bin/bash
echo "验证FastDDS共享内存配置..."
echo "=================================="

echo "环境变量检查:"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "FASTRTPS_DEFAULT_PROFILES_FILE: $FASTRTPS_DEFAULT_PROFILES_FILE"
echo "RMW_FASTRTPS_USE_QOS_FROM_XML: $RMW_FASTRTPS_USE_QOS_FROM_XML"
echo ""

echo "当前/dev/shm中的FastDDS文件:"
ls -lh /dev/shm | grep fastrtps
echo ""

echo "提示:"
echo "- 运行 'ros2 topic hz /rslidar_points' 检查点云帧率"
echo "- 正常情况下应看到约200MB大小的共享内存文件"
echo "- 帧率应稳定在10Hz左右"
