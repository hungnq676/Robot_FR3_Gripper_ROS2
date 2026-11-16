#!/bin/bash
# MTC tien toi tu the san sang kep (khong kep) tren robot that, tu ket qua grasp_once moi nhat.
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v miniconda | paste -sd:)
unset CONDA_PREFIX PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/fr3_hungros2/ws_fr3_moveit/install/setup.bash
export ROS_DOMAIN_ID=33 ROS_LOCALHOST_ONLY=1 LC_NUMERIC=en_US.UTF-8
exec /usr/bin/python3 ~/fr3_hungros2/tools/mtc_pregrasp.py "$@"
