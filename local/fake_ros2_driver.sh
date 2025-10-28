#!/bin/bash

ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.56.101 launch_rviz:=false kinematics_params_file:="/home/blink/ur5e_calibration.yaml"
