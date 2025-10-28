#!/bin/bash

if [ "$1" == "home" ] && [ $# -eq 1 ]; then
    ros2 service call /moveit_path_plan helper/srv/MoveRequest "{command: 'cartesian', positions: [0.59, 0.133, 0.366, 180, 0, 90], constraints_identifier: 'NONE'}"
    exit 0
fi

if [ $# -lt 6 ] || [ $# -gt 7 ]; then
  echo "Usage: $0 x y z roll pitch yaw [constraints_identifier]"
  exit 1
fi

constraints=${7:-'NONE'}  # default 'NONE'

ros2 service call /moveit_path_plan helper/srv/MoveRequest "{command: 'cartesian', positions: [$1, $2, $3, $4, $5, $6], constraints_identifier: '$constraints'}"
