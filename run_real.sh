#!/bin/bash
# Robot FR3 that + MoveIt + RViz tren may ROG (gripper van la mock).
# Truoc khi chay: tren pendant robot da xoa loi va ENABLE (che do TAY hay TU DONG deu duoc).
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v miniconda | paste -sd:)
unset CONDA_PREFIX PYTHONPATH
export ROS_DOMAIN_ID=33 ROS_LOCALHOST_ONLY=1 LC_NUMERIC=en_US.UTF-8
source /opt/ros/humble/setup.bash
source ~/fr3_hungros2/ros2_ws/install/setup.bash
source ~/fr3_hungros2/ws_fr3_moveit/install/setup.bash

if ! ping -c 1 -W 1 192.168.58.2 > /dev/null; then
  echo "Khong ping duoc robot 192.168.58.2. Kiem tra day LAN va IP may (192.168.58.x)."
  exit 1
fi
# Dat toc do toan cuc 100% (vo hai; toc do that do MoveIt gioi han). Nguyen nhan
# that cua viec robot cham/ngat la hang doi ServoJ, da sua trong driver.
~/fr3_hungros2/tools/fr_speed100 || { echo "Khong dat duoc toc do 100% (driver khac dang giu ket noi?)"; exit 1; }

# Camera (Orbbec Astra Pro Plus, chay rieng bang run_astra.sh) vao chuoi TF:
# base_footprint -> camera_link tu ket qua hand-eye.
# Camera xe dich thi phai calibrate lai (tools/tag_check.py + tools/handeye_solve.py).
if [ -f ~/.ros/fr3_astra_handeye.yaml ]; then
  /usr/bin/python3 ~/fr3_hungros2/tools/cam_tf.py ~/.ros/fr3_astra_handeye.yaml &
  CAM_TF=$!
  trap 'kill $CAM_TF 2>/dev/null' EXIT
else
  echo "Chua co ~/.ros/fr3_astra_handeye.yaml: camera chua noi vao TF robot (can hieu chuan hand-eye)."
fi
ros2 launch fairino_fr3_moveit_config real_robot.launch.py
