#!/bin/bash

if [ "$1" == "home" ] && [ $# -eq 1 ]; then
    ros2 service call /moveit_path_plan helper/srv/MoveRequest "{command: 'joint', positions: [0, -74.5, 90, -105, -90, 0], constraints_identifier: '1'}"
    exit 0
fi

if [ $# -ne 6 ]; then
  echo "Usage: $0 j1 j2 j3 j4 j5 j6 [constraints_identifier]"
  exit 1
fi

constraints=${7:-0}  # default '0'

ros2 service call /moveit_path_plan helper/srv/MoveRequest "{command: 'joint', positions: [$1, $2, $3, $4, $5, $6], constraints_identifier: '$constraints'}"
