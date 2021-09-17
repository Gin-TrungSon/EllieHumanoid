#!/bin/bash
# Copyright 2021 by Dong Trung Son, Nueremberg University.
# Email: trungsondo68839@th-nuernberg.de
# All rights reserved.
# This file is part of the Ellie-Project,
# and is released under the "MIT License Agreement". Please see the LICENSE
# file that should have been included as part of this package.

echo ""
echo "[Note] OS version  >>> Ubuntu 20.04 (Focal Fossa)"
echo "[Note] Target ROS version >>> ROS 2 Foxy Fitzroy"
echo "[Note] Ellie workspace   >>> $HOME/ellie_ws"
echo ""
echo "PRESS [ENTER] TO CONTINUE THE INSTALLATION"
echo "IF YOU WANT TO CANCEL, PRESS [CTRL] + [C]"
read

echo "[Set the target ROS version and name of ellie workspace]"
name_ros_version=${name_ros_version:="foxy"}
name_ellie_workspace=${name_ellie_workspace:="ellie_ws"}

echo "[Environment setup]"
source /opt/ros/$name_ros_version/setup.sh
sudo apt install -y python3-argcomplete python3-colcon-common-extensions python3-vcstool git wget
sudo apt install chromium-browser
sudo apt-get install flac
sudo apt-get install -y libhdf5-dev libc-ares-dev libeigen3-dev gcc gfortran libgfortran5 \
    libatlas3-base libatlas-base-dev libopenblas-dev libopenblas-base libblas-dev \
    liblapack-dev cython3 openmpi-bin libopenmpi-dev libatlas-base-dev python3-dev
sudo apt-get install python3-pyqt5
sudo apt install python3-pyqt5.qtwebengine

echo "[Virtual enviroment]"
cd $HOME
sudo apt install python3-virtualenv
virtualenv -p python3 ./venv
source ./venv/bin/activate
touch ./venv/COLCON_IGNORE
sh -c "echo \"source ~/venv/bin/activate\" >> ~/.bashrc"
sh -c "echo \"export PYTHONPATH=$PYTHONPATH:~/venv/lib/python3.8/site-packages\" >> ~/.bashrc"

echo "[Install requirements]"
sudo apt-get install ffmpeg
sudo apt-get install portaudio19-dev python3-pyaudio -y
sudo apt install python3-opencv

## install tensorflow
pip install gdown
gdown https://drive.google.com/u/0/uc?id=13_otGSGrsE1atBB2RjcPhOvONErKSFef&export=download
pip install tensorflow-2.6.0-cp38-cp38-linux_aarch64.whl


wget https://raw.githubusercontent.com/Gin-TrungSon/EllieHumanoid/devel/requirements_arm64.txt
pip install -r requirements_arm64.txt


echo "[Make the ellie workspace and test ellie build]"
mkdir -p $HOME/$name_ellie_workspace
cd $HOME/$name_ellie_workspace
git clone -b devel https://github.com/Gin-TrungSon/EllieHumanoid .
colcon build --symlink-install

echo "[Set the ROS evironment]"
sh -c "echo \"alias nb='nano ~/.bashrc'\" >> ~/.bashrc"
sh -c "echo \"alias sb='source ~/.bashrc'\" >> ~/.bashrc"
sh -c "echo \"alias gs='git status'\" >> ~/.bashrc"
sh -c "echo \"alias gp='git pull'\" >> ~/.bashrc"

sh -c "echo \"alias cw='cd ~/$name_ellie_workspace'\" >> ~/.bashrc"
sh -c "echo \"alias cs='cd ~/$name_ellie_workspace/src'\" >> ~/.bashrc"
sh -c "echo \"alias cb='cd ~/$name_ellie_workspace && colcon build --symlink-install && source ~/.bashrc'\" >> ~/.bashrc"

sh -c "echo \"source /opt/ros/$name_ros_version/setup.bash\" >> ~/.bashrc"
sh -c "echo \"source ~/$name_ellie_workspace/install/local_setup.bash\" >> ~/.bashrc"
sh -c "echo \"export QT_XCB_GL_INTEGRATION=none\" >> ~/.bashrc"
sh -c "echo \"export QTWEBENGINE_DISABLE_SANDBOX=1\" >> ~/.bashrc"

sh -c "echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer"
sudo apt-get update
exec bash

echo "[Complete!!!]"
exit 0
