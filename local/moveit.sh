#!/bin/bash

ros2 launch ur_moveit_config ur_moveit.launch.py \
    robot_ip:=192.168.0.100 \
    ur_type:=ur5e \
    launch_rviz:=true
