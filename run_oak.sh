#!/bin/bash
# OAK-D Pro W that (depthai-ros) tren may ROG, cung domain voi run_real.sh.
# Topic: /oak/rgb/image_raw, /oak/stereo/image_raw (depth canh theo RGB), /oak/points
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v miniconda | paste -sd:)
unset CONDA_PREFIX PYTHONPATH
export ROS_DOMAIN_ID=33 ROS_LOCALHOST_ONLY=1 LC_NUMERIC=en_US.UTF-8
source /opt/ros/humble/setup.bash
ros2 launch depthai_ros_driver camera.launch.py camera_model:=OAK-D-PRO-W params_file:=$HOME/fr3_hungros2/oak_params.yaml
