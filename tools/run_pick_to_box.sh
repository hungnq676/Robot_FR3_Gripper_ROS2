#!/bin/bash
# Kich ban bao cao: HOME -> nhin ban -> gap VAT -> bo vao HOP -> HOME, co ghi du lieu.
#   1. ve HOME neu chua o do (hoi y; HOME = ~/.ros/fr3_home.yaml tu tools/save_home.py)
#   2. anh truoc (RGB / depth / RGB-D) + bat quay camera
#   3. Gemini + SAM + GraspNet tim VAT; Gemini + SAM tim HOP (~/.ros/fr3_place_box.yaml)
#   4. vat can -> MTC: lap ke hoach, hien RViz, hoi 1 lan -> gap, nhac, mang toi hop, tha, nhac, ve HOME
#   5. anh sau, dung quay. Ket qua: nhat_ky/du_lieu/<ngay_gio>_hop_<nhan>/
# Dung: tools/run_pick_to_box.sh "<vat>" "<hop>" [nhan] [tham so mtc_pregrasp...]
#   vd:  tools/run_pick_to_box.sh "the white object" "the cardboard box" gia_trang --deeper 0.015
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v miniconda | paste -sd:)
unset CONDA_PREFIX PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/fr3_hungros2/ws_fr3_moveit/install/setup.bash
export ROS_DOMAIN_ID=33 ROS_LOCALHOST_ONLY=1 LC_NUMERIC=en_US.UTF-8
T=~/fr3_hungros2/tools
OBJ=$1; BOX=$2; LABEL=${3:-vat}; shift 3
BOXFILE=~/.ros/fr3_place_box.yaml
DIR=~/fr3_hungros2/nhat_ky/du_lieu/$(date +%Y-%m-%d_%H%M%S)_hop_$LABEL
mkdir -p "$DIR"
/usr/bin/python3 -u $T/mtc_pregrasp.py --only-home 2>&1 | grep --line-buffered -vE '^\[INFO|^\[WARN'
had_session=$( [ -f ~/.ros/fr3_session_dir ] && cat ~/.ros/fr3_session_dir )
echo "$DIR" > ~/.ros/fr3_session_dir
/usr/bin/python3 $T/snap_rgbd.py truoc
/usr/bin/python3 $T/record_camera.py "$DIR" & REC=$!
$T/run_grasp.sh --gemini "$OBJ" --top 15 --no-show --no-hold 2>&1 | grep -E "^Vat|Hop bao|ngon an|Khong|^Anh"
$T/run_grasp.sh --gemini "$BOX" --save-as $BOXFILE --no-show --no-hold 2>&1 | grep -E "^Vat|Hop bao|^Anh"
cp ~/.ros/fr3_last_grasp.yaml "$DIR/vat.yaml"; cp $BOXFILE "$DIR/hop.yaml"
/usr/bin/python3 $T/scene_obstacles.py
/usr/bin/python3 -u $T/mtc_pregrasp.py --pick --into-box $BOXFILE "$@" 2>&1 \
    | grep --line-buffered -vE '^\[INFO|^\[WARN|^ +[0-9-]+ +- |^Failing|^IK \(|^tien vao \(|^min_' | tee "$DIR/mtc_pregrasp.txt"
sleep 2
/usr/bin/python3 $T/snap_rgbd.py sau
kill -INT $REC; wait $REC
if [ -n "$had_session" ]; then echo "$had_session" > ~/.ros/fr3_session_dir; else rm -f ~/.ros/fr3_session_dir; fi
$T/chia_se_video.sh "$DIR/camera_mau_depth.webm" 2>&1 | tail -1
echo "Ket qua: $DIR"; ls "$DIR" "$DIR/anh"
