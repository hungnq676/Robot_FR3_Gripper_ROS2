#!/bin/bash
# Ghi du lieu mot phien thi nghiem cho bao cao (chay song song voi run_real.sh + run_astra.sh):
#   camera_mau_depth.webm : video camera (mau | depth), anh dau phien
#   man_hinh.webm         : quay toan man hinh (RViz, terminal) qua GNOME
#   bag/                  : rosbag nhe (khop, TF, quy dao ke hoach, grasp, camera_info) de phat lai RViz
#   anh/                  : anh point_test / grasp_once trong phien (tu dong, qua ~/.ros/fr3_session_dir)
#   cau_hinh/             : ban sao hand-eye, mo hinh depth, URDF luc chay
# Dung: tools/record_session.sh [nhan]   (vd: graspnet_tay_cam) ; Ctrl+C de dung tat ca.
# Video robot that: quay bang dien thoai cung luc, dong ho trong video camera de khop thoi gian.
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v miniconda | paste -sd:)
unset CONDA_PREFIX PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/fr3_hungros2/ws_fr3_moveit/install/setup.bash
export ROS_DOMAIN_ID=33 ROS_LOCALHOST_ONLY=1 LC_NUMERIC=en_US.UTF-8

DIR=~/fr3_hungros2/nhat_ky/du_lieu/$(date +%Y-%m-%d_%H%M%S)${1:+_$1}
mkdir -p "$DIR/anh" "$DIR/cau_hinh"
echo "$DIR" > ~/.ros/fr3_session_dir
cp ~/.ros/fr3_astra_handeye.yaml ~/.ros/fr3_astra_depth_model.yaml "$DIR/cau_hinh/" 2>/dev/null
ros2 param get /robot_state_publisher robot_description 2>/dev/null > "$DIR/cau_hinh/robot_description.txt"
echo "bat dau: $(date '+%F %T')" > "$DIR/ghi_chu.txt"

pids=()
/usr/bin/python3 ~/fr3_hungros2/tools/record_camera.py "$DIR" & pids+=($!)
/usr/bin/python3 ~/fr3_hungros2/tools/screencast.py "$DIR/man_hinh.webm" & pids+=($!)
ros2 bag record -o "$DIR/bag" /joint_states /tf /tf_static /display_planned_path \
    /fr3_demo/grasp_markers /fr3_demo/object_cloud /camera/color/camera_info >/dev/null 2>&1 & pids+=($!)

stop() {
    trap - INT TERM
    echo; echo "Dung ghi..."
    kill -INT "${pids[@]}" 2>/dev/null
    wait "${pids[@]}" 2>/dev/null
    rm -f ~/.ros/fr3_session_dir
    echo "ket thuc: $(date '+%F %T')" >> "$DIR/ghi_chu.txt"
    echo "Da luu: $DIR"; ls -la "$DIR" "$DIR/anh"
    exit 0
}
trap stop INT TERM
echo "Dang ghi vao $DIR  (Ctrl+C de dung)"
wait
