#!/bin/bash
# Gap vat mau trang roi nhac len (nhu lan thanh cong 02/11 00:16):
#   1. ve HOME (hoi y) de robot khong che ban
#   2. Gemini + SAM + GraspNet tim vat "the white object"
#   3. MTC: lap ke hoach, hien RViz, hoi y mot lan -> mo, tien vao, kep, nhac len
# Dung: [VAT="the black game controller"] tools/run_gap_trang.sh [nhan] [tham so mtc_pregrasp them...]
#   VAT: cau mo ta vat cho Gemini (mac dinh "the white object")
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v miniconda | paste -sd:)
unset CONDA_PREFIX PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/fr3_hungros2/ws_fr3_moveit/install/setup.bash
export ROS_DOMAIN_ID=33 ROS_LOCALHOST_ONLY=1 LC_NUMERIC=en_US.UTF-8
T=~/fr3_hungros2/tools
LABEL=${1:-gap_trang}; shift
OBJ=${VAT:-the white object}
echo "== 1/3 Ve HOME =="
HOME_OUT=$(/usr/bin/python3 -u $T/mtc_pregrasp.py --only-home 2>&1 | grep --line-buffered -vE '^\[INFO|^\[WARN' | tee /dev/tty)
# 17:08 (02/11): HOME was aborted with j1 18 deg short; the arm can then hide the object. Stop instead.
if echo "$HOME_OUT" | grep -q "Ve HOME: loi"; then echo "DUNG: chua ve duoc HOME. Chay lai."; exit 1; fi
echo "== 2/3 Tim vat: $OBJ (Gemini + SAM + GraspNet) =="
START=$(date +%s)
$T/run_grasp.sh --gemini "$OBJ" --top 15 --no-show --no-hold 2>&1 | grep -E "^Vat|ngon an|Khong|^Anh|grasp|Grasp|depth|Qua it"
# 16:34 (02/11): grasp_once failed and mtc_pregrasp silently used the 02:17 result. Stop instead.
/usr/bin/python3 -c "
import os, sys, yaml
f = os.path.expanduser('~/.ros/fr3_last_grasp.yaml')
r = yaml.safe_load(open(f)) or {}
if os.path.getmtime(f) < $START: sys.exit(1)
if r.get('graspnet'): sys.exit(0)
sys.exit(2 if (r.get('geometric') or {}).get('ok') else 1)"
RC=$?
if [ $RC -eq 2 ]; then
  echo "Khong co tu the GraspNet (khong co GPU?): dung grasp hinh hoc tu tren xuong (--geometric)."
  EXTRA=--geometric
elif [ $RC -ne 0 ]; then
  echo "DUNG: buoc tim vat khong ra tu the gap moi (xem anh o tren). Robot khong gap."
  exit 1
fi
echo "== 3/3 Gap va nhac len (hoi y truoc khi chay) =="
$T/run_pregrasp_demo.sh "$LABEL" --pick --deeper 0.015 $EXTRA "$@"
