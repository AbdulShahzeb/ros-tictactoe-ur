#!/bin/bash

ros2 launch realsense2_camera rs_launch.py \
    align_depth.enable:=true \
    enable_color:=true \
    enable_depth:=true \
    pointcloud.enable:=true