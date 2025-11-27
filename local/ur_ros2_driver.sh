#!/bin/bash

UR_TYPE="ur5e"

while [[ "$#" -gt 0 ]]; do
    case $1 in
        -10|--ur10e) UR_TYPE="ur10e" ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
    shift
done

ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:="$UR_TYPE" \
    robot_ip:="192.168.0.100" \
    kinematics_params_file:="/home/abdul/${UR_TYPE}_calibration.yaml" \
    launch_rviz:=false
