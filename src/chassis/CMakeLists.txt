cmake_minimum_required(VERSION 3.10)
project(chassis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- 依赖 ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# ---- 头文件 ----
include_directories(include)

# ---- 串口底层库 ----
add_library(chassis_serial SHARED
  src/chassis_serial.cpp
)
target_include_directories(chassis_serial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(chassis_serial pthread)

# ---- ROS2 节点 ----
add_executable(chassis_node
  src/main.cpp
  src/chassis_node.cpp
)
target_link_libraries(chassis_node chassis_serial)
ament_target_dependencies(chassis_node
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
)

# ---- 安装 ----
install(TARGETS chassis_serial
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS chassis_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
