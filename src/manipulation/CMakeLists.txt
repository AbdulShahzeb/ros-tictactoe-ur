cmake_minimum_required(VERSION 3.8)
project(manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(helper REQUIRED)
find_package(ompl REQUIRED)


add_executable(moveit_server
  src/moveit_server.cpp
)

ament_target_dependencies(moveit_server
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_geometry_msgs
  helper
)

install(TARGETS
  moveit_server
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
