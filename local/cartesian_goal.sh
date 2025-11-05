#!/bin/bash

if [ "$ROS_DISTRO" == "humble" ]; then
  constraints="ORIEN"
elif [ "$ROS_DISTRO" == "jazzy" ]; then
  constraints="NONE"
else
  constraints="ORIEN"
fi

if [ "$1" == "home" ] && [ $# -eq 1 ]; then
    ros2 service call /moveit_path_plan helper/srv/MoveRequest "{command: 'cartesian', positions: [0.486, 0.132, 0.559, 180, 0, -90], constraints_identifier: '$constraints'}"
    exit 0
fi

if [ $# -eq 3 ]; then
    ros2 service call /moveit_path_plan helper/srv/MoveRequest "{command: 'cartesian', positions: [$1, $2, $3, 180, 0, -90], constraints_identifier: '$constraints'}"
    exit 0
fi

if [ $# -lt 6 ] || [ $# -gt 7 ]; then
  echo "Usage: $0 x y z roll pitch yaw [constraints_identifier]"
  exit 1
fi

constraints=${7:-$constraints}  # default based on ROS_DISTRO

ros2 service call /moveit_path_plan helper/srv/MoveRequest "{command: 'cartesian', positions: [$1, $2, $3, $4, $5, $6], constraints_identifier: '$constraints'}"
