#!/usr/bin/env bash

# Cable driven robot setup

. /opt/ros/humble/setup.bash
. install/setup.bash

ros2 launch cable_robot cable_robot.launch.py
