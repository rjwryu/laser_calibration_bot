#!/usr/bin/env bash

# MyActuator ROS2 setup

. /opt/ros/humble/setup.bash
. install/setup.bash

ros2 launch myactuator_rmd_bringup myactuator_rmd_control.launch.py \
    actuator:=X6_60 \
    simulation:=false
