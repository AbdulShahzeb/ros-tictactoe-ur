#!/bin/bash

ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.56.101 launch_rviz:=false description_file:=/home/blink/mtrn4231/mtrn4231-project/src/end_effector/urdf/ur_with_pen_holder.xacro