cmake_minimum_required(VERSION 3.8)
project(helper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/CellPose.msg"
  "msg/CellPoseArray.msg"
)

set(srv_files
  "srv/MoveRequest.srv"
)

# set(action_files
#   "action/Example.action"
# )

rosidl_generate_interfaces(${PROJECT_NAME}
#   ${msg_files}
  ${srv_files}
#   ${action_files}
)

ament_package()