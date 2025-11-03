#!/bin/bash

if [ "$1" == "home" ] && [ $# -eq 1 ]; then
    ros2 service call /moveit_path_plan helper/srv/MoveRequest "{command: 'joint', positions: [0, -89.87, 79.39, -79.50, -90, 0], constraints_identifier: 'NONE'}"
    exit 0
fi

if [ $# -lt 6 ] || [ $# -gt 7 ]; then
  echo "Usage: $0 j1 j2 j3 j4 j5 j6 [constraints_identifier]"
  exit 1
fi

constraints=${7:-'NONE'}  # default 'NONE'

ros2 service call /moveit_path_plan helper/srv/MoveRequest "{command: 'joint', positions: [$1, $2, $3, $4, $5, $6], constraints_identifier: '$constraints'}"
