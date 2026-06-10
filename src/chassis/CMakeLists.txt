cmake_minimum_required(VERSION 3.10)
project(chassis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --------------------------
# zsibot SDK (静态库 + 头文件)
# --------------------------
set(ZSIBOT_LIB "/usr/local/lib/libzsibot.a")
set(ZSIBOT_INCLUDE "/usr/local/include/zsibot/include")

if(NOT EXISTS ${ZSIBOT_LIB})
    message(FATAL_ERROR "未找到 zsibot 库文件: ${ZSIBOT_LIB}")
endif()

message(STATUS "使用 zsibot SDK: ${ZSIBOT_LIB}")

# ---- 依赖 ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# ---- ROS2 节点 ----
add_executable(chassis_driver_node src/chassis_driver_node.cpp)
target_include_directories(chassis_driver_node PRIVATE
    ${ZSIBOT_INCLUDE}
)
target_link_libraries(chassis_driver_node
  ${ZSIBOT_LIB}
  ${rclcpp_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
)

# ---- 安装 ----
install(TARGETS chassis_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
