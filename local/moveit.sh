#!/bin/bash

IP=192.168.0.100

while [[ "$#" -gt 0 ]]; do
    case $1 in
        -f|--fake) IP=192.168.56.101 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
    shift
done

ros2 launch ur_moveit_config ur_moveit.launch.py \
    robot_ip:=$IP \
    ur_type:=ur5e \
    launch_rviz:=true
