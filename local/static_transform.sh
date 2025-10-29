#!/bin/bash

# Default
x=0.85
y=0.04
z=0.92
qx=0
qy=1
qz=0
qw=0

# Handle user arguments
if [ $# -eq 3 ]; then
    x=$1
    y=$2
    z=$3
elif [ $# -eq 7 ]; then
    x=$1
    y=$2
    z=$3
    qx=$4
    qy=$5
    qz=$6
    qw=$7
elif [ $# -ne 0 ]; then
    echo "Usage: $0 [x y z] or [x y z qx qy qz qw]"
    exit 1
fi

ros2 run tf2_ros static_transform_publisher \
    $x $y $z $qx $qy $qz $qw base_link camera_link
