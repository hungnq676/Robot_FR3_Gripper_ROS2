#!/bin/bash
# Cai Gazebo Harmonic + ros_gz cho Humble giong may hungros2, va pip cho Python he thong.
set -e

# 1. Repo OSRF (Gazebo)
sudo apt-get install -y curl lsb-release gnupg python3-pip
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update

# 2. Go ros_gz ban Fortress (xung dot voi ban Harmonic), cai Harmonic
sudo apt-get remove -y 'ros-humble-ros-gz*' 'ros-humble-ros-ign*' || true
sudo apt-get install -y gz-harmonic ros-humble-ros-gzharmonic

# 3. Thu vien Python cho cac node FR3 (Python he thong, cai vao user)
/usr/bin/python3 -m pip install --user numpy==1.26.4 opencv-python==4.10.0.84 scipy==1.11.4 google-genai==2.33.0

# 4. GZ_VERSION cho cac terminal moi
grep -q "GZ_VERSION=harmonic" ~/.bashrc || echo "export GZ_VERSION=harmonic" >> ~/.bashrc

echo "XONG. gz sim --version:"
gz sim --version
