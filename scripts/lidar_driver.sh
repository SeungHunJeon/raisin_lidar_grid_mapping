#!/bin/bash
set -e

CURRENT_DIR=$(pwd)

LIDAR_DIR="${CURRENT_DIR}/../jsh_ws"

#LIDAR_DIR="${CURRENT_DIR}/.."

source ${LIDAR_DIR}/ws_livox/install/setup.bash

ros2 launch livox_ros_driver2 msg_MID360_launch.py
