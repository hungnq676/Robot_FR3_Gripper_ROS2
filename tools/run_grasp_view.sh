#!/bin/bash
# Separate RViz window for tools/grasp_once.py: object cloud, 3D box and grasps.
export PATH=$(echo $PATH | tr ':' '\n' | grep -v miniconda | paste -sd:)
source /opt/ros/humble/setup.bash
source ~/fr3_hungros2/ws_fr3_moveit/install/setup.bash   # robot meshes
export ROS_DOMAIN_ID=33 ROS_LOCALHOST_ONLY=1 LC_NUMERIC=en_US.UTF-8
exec rviz2 -d ~/fr3_hungros2/tools/grasp_view.rviz
