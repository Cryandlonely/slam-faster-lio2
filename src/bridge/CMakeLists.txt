cmake_minimum_required(VERSION 3.10)
project(bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- 依赖 ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

# ---- 头文件 ----
include_directories(include)

# ---- TCP 服务器库 ----
add_library(bridge_tcp SHARED
  src/tcp_server.cpp
)
target_include_directories(bridge_tcp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(bridge_tcp pthread)

# ---- ROS2 节点 ----
add_executable(bridge_node
  src/main.cpp
  src/bridge_node.cpp
)
target_link_libraries(bridge_node bridge_tcp nlohmann_json::nlohmann_json)
ament_target_dependencies(bridge_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
)

# ---- 安装 ----
install(TARGETS bridge_tcp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS bridge_node
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
