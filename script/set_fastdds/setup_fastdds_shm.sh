#!/bin/bash
# FastDDS共享内存环境变量设置
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/fastdds_shm.xml
export RMW_FASTRTPS_USE_QOS_FROM_XML=1

echo "FastDDS共享内存配置已加载"
echo "RMW实现: $RMW_IMPLEMENTATION"
echo "配置文件: $FASTRTPS_DEFAULT_PROFILES_FILE"
