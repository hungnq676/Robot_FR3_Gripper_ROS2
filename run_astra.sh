#!/bin/bash
# Orbbec Astra Pro Plus that (ros2_astra_camera) tren may ROG, cung domain voi run_real.sh.
# Topic: /camera/color/image_raw, /camera/depth/image_raw (depth canh theo mau), /camera/depth/points
# Xem: run_astra.sh --rviz (mo them RViz voi tools/astra_view.rviz)
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v miniconda | paste -sd:)
unset CONDA_PREFIX PYTHONPATH
export ROS_DOMAIN_ID=33 ROS_LOCALHOST_ONLY=1 LC_NUMERIC=en_US.UTF-8
source /opt/ros/humble/setup.bash
source $HOME/fr3_hungros2/ros2_ws/install/setup.bash
if [ "$1" = "--rviz" ]; then
    rviz2 -d $HOME/fr3_hungros2/tools/astra_view.rviz &
fi
ros2 launch astra_camera astro_pro_plus.launch.xml depth_registration:=true
