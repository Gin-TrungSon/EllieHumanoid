#!/bin/bash

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

echo "[Set Locale]"
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "[Setup Sources]"
sudo rm -rf /var/lib/apt/lists/* && sudo apt update && sudo apt install -y curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null'

echo "[Install ROS 2 packages]"
sudo apt update && sudo apt install -y ros-$name_ros_version-desktop

echo "[Environment setup]"
source /opt/ros/$name_ros_version/setup.sh
sudo apt install -y python3-argcomplete python3-colcon-common-extensions python3-vcstool git wget

echo "[Make the ellie workspace and test ellie build]"
mkdir -p $HOME/$name_ellie_workspace/src
cd $HOME/$name_ellie_workspace
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

wget https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb
sudo apt install ./google-chrome-stable_current_amd64.deb
rm ./google-chrome-stable_current_amd64.deb

echo "[Virtual enviroment]"
cd $HOME/$name_ellie_workspace
sudo apt install python3-virtualenv
virtualenv -p python3 ./venv
source ./venv/bin/activate
touch ./venv/COLCON_IGNORE
sh -c "echo \"source ~/$name_ellie_workspace/venv/bin/activate\" >> ~/.bashrc"
sh -c "echo \"export PYTHONPATH=$PYTHONPATH:~/$name_ellie_workspace/venv/lib/python3.8/site-packages\" >> ~/.bashrc"

echo "[Install requirements]"
sudo apt-get install ffmpeg
sudo apt-get install portaudio19-dev python3-pyaudio -y
sudo apt install python3-opencv

wget https://raw.githubusercontent.com/Gin-TrungSon/EllieHumanoid/devel/requirements.txt
pip install -r requirements.txt

git clone -b devel https://github.com/Gin-TrungSon/EllieHumanoid .

sh -c "echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer"
sudo apt-get update
exec bash

echo "[Complete!!!]"
exit 0