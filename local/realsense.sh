#!/bin/bash

#rgb_camera.color_profile:=1920x1080x30 \

ros2 launch realsense2_camera rs_launch.py \
    align_depth.enable:=true \
    enable_color:=true \
    enable_depth:=true \
    rgb_camera.color_profile:=1280x720x15 \
    pointcloud.enable:=false
