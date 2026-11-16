#!/bin/bash
# Pointing test on the real FR3: plans, shows the path in RViz, moves only after y.
export PATH=$(echo $PATH | tr ':' '\n' | grep -v miniconda | paste -sd:)
source /opt/ros/humble/setup.bash
source ~/fr3_hungros2/ws_fr3_moveit/install/setup.bash
export ROS_DOMAIN_ID=33 ROS_LOCALHOST_ONLY=1 LC_NUMERIC=en_US.UTF-8
exec /usr/bin/python3 ~/fr3_hungros2/tools/point_test.py "$@"
