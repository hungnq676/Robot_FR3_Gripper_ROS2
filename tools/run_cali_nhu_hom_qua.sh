#!/bin/bash
# Cali hand-eye lam y nhu 02/11 00:05 (22 mau, LOO 0.81 px): ve tu the bat dau cua hom do
# (j1 ~133 deg giua tam quay, j6 -167.7 deg) roi de xuat tu the voi dung tham so hom do.
# tag_check.py (khong --append) phai dang chay o cua so khac. Moi buoc chi chay khi go y.
T=~/fr3_hungros2/tools
echo "== Ve tu the bat dau cua hom qua =="
$T/run_tool.sh goto_joints.py 132.9 -124.4 -56.9 -96.4 66.3 -167.7
echo; echo "== De xuat tu the (tham so hom qua) =="
$T/run_tool.sh handeye_guided.py --margin 40 --angle 20 40 --min-dist 0.33 --poses 20
