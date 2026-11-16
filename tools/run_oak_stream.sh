#!/bin/bash
# OAK-D Pro W live RGB / depth / RGBD with depthai (no ROS). Stop run_oak.sh first.
unset PYTHONPATH   # keep ROS' python packages out of the venv
exec ~/fr3_hungros2/venv_oak/bin/python ~/fr3_hungros2/tools/oak_stream.py "$@"
