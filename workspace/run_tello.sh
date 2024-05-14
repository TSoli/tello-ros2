#!/usr/bin/env bash

source /opt/ros/humble/setup.sh
source ./install/local_setup.sh
cd src && ros2 launch launch.py
