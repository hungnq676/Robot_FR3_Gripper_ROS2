#!/bin/bash
# grasp_once.py in the graspnet conda env (GPU) with the real-robot ROS environment.
export PATH=$(echo $PATH | tr ':' '\n' | grep -v miniconda | paste -sd:)
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=33 ROS_LOCALHOST_ONLY=1 LC_NUMERIC=en_US.UTF-8
cd ~/graspnet_runtime   # ultralytics downloads mobile_sam.pt here on first use
exec ~/miniconda3/envs/graspnet/bin/python ~/fr3_hungros2/tools/grasp_once.py "$@"
