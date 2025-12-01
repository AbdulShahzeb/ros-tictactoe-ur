#!/usr/bin/env bash

# Usage: ./calibrate.sh <NUMBER>

if [ -z "$1" ]; then
    echo "Error: missing robot number."
    echo "Usage: $0 <NUMBER>"
    exit 1
fi

NUMBER="$1"
TARGET_FILE="${HOME}/ros-tictactoe-ur/src/brain/config/robot${NUMBER}_calib.yaml"

ros2 launch ur_calibration calibration_correction.launch.py \
    robot_ip:=192.168.0.100 \
    target_filename:="${TARGET_FILE}"