#!/bin/bash
# Run a tools/*.py script with the real-robot ROS environment (domain 33).
# Usage: run_tool.sh <script.py> [args...]
export PATH=$(echo $PATH | tr ':' '\n' | grep -v miniconda | paste -sd:)
source /opt/ros/humble/setup.bash
source ~/fr3_hungros2/ws_fr3_moveit/install/setup.bash
export ROS_DOMAIN_ID=33 ROS_LOCALHOST_ONLY=1 LC_NUMERIC=en_US.UTF-8
script=$1; shift
exec /usr/bin/python3 ~/fr3_hungros2/tools/$script "$@"
