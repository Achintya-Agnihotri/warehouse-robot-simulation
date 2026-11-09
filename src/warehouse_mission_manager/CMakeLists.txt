cmake_minimum_required(VERSION 3.8)
project(warehouse_mission_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(warehouse_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

add_executable(mission_manager_node
  src/mission_manager.cpp
)

ament_target_dependencies(mission_manager_node
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  std_srvs
  warehouse_interfaces
  tf2_ros
  ros_gz_interfaces
)

target_include_directories(mission_manager_node PRIVATE include)

install(TARGETS
  mission_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()