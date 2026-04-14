cmake_minimum_required(VERSION 3.10)
project(nav_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- 依赖 ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

# ---- 头文件 ----
include_directories(include)

# ---- 核心库 (四模块 + 公共) ----
add_library(slam_nav_core SHARED
  src/point_to_point_planner.cpp
  src/pure_pursuit_tracker.cpp
  src/robot_sdk_interface.cpp
  src/navigation_state_machine.cpp
)
target_include_directories(slam_nav_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(slam_nav_core
  rclcpp
)

# ---- 可执行文件 ----
add_executable(slam_nav_node
  src/main.cpp
  src/slam_nav_node.cpp
)
target_link_libraries(slam_nav_node slam_nav_core nlohmann_json::nlohmann_json)
ament_target_dependencies(slam_nav_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

# ---- 安装 ----
install(TARGETS slam_nav_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS slam_nav_node
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

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
  OPTIONAL
)

# ---- 测试 ----
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
