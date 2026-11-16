#!/bin/bash
# Mot lan chay "tien toi tu the san sang kep" co ghi du lieu:
#   anh truoc (RGB / depth / RGB-D) -> bat quay camera (mau | depth) -> vat can -> MTC (hien RViz, hoi y)
#   -> robot dung -> anh sau -> dung quay. Ket qua: nhat_ky/du_lieu/<ngay_gio>_pregrasp_<nhan>/
# Dung: tools/run_pregrasp_demo.sh [nhan] [tham so mtc_pregrasp...]   (can ket qua grasp_once moi nhat)
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v miniconda | paste -sd:)
unset CONDA_PREFIX PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/fr3_hungros2/ws_fr3_moveit/install/setup.bash
export ROS_DOMAIN_ID=33 ROS_LOCALHOST_ONLY=1 LC_NUMERIC=en_US.UTF-8
T=~/fr3_hungros2/tools
LABEL=${1:-vat}; shift
DIR=~/fr3_hungros2/nhat_ky/du_lieu/$(date +%Y-%m-%d_%H%M%S)_pregrasp_$LABEL
mkdir -p "$DIR"
had_session=$( [ -f ~/.ros/fr3_session_dir ] && cat ~/.ros/fr3_session_dir )
echo "$DIR" > ~/.ros/fr3_session_dir          # anh cua lan chay nay vao $DIR/anh
cp ~/.ros/fr3_last_grasp.yaml "$DIR/grasp_once_ket_qua.yaml"
/usr/bin/python3 $T/snap_rgbd.py truoc
/usr/bin/python3 $T/record_camera.py "$DIR" & REC=$!
/usr/bin/python3 $T/scene_obstacles.py
/usr/bin/python3 -u $T/mtc_pregrasp.py "$@" 2>&1 | grep --line-buffered -vE '^\[INFO|^\[WARN' | tee "$DIR/mtc_pregrasp.txt"
sleep 2
/usr/bin/python3 $T/snap_rgbd.py sau
kill -INT $REC; wait $REC
if [ -n "$had_session" ]; then echo "$had_session" > ~/.ros/fr3_session_dir; else rm -f ~/.ros/fr3_session_dir; fi
echo "Ket qua: $DIR"; ls "$DIR" "$DIR/anh"
