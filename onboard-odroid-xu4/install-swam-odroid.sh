#!/bin/sh

# install linux image to use odroid-wifi5 usb stick
sudo apt-get install linux-image-xu3 -y

# vim and emacs, tmux, tmuxinator
sudo apt-get install vim emacs tmux tmuxinator -y

# ros kinetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update -y
sudo apt-get install ros-kinetic-desktop -y
sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras -y

# ros kinetic mavros catkin build
sudo apt-get install python-catkin-tools python-rosinstall-generator python-wstool build-essential -y
sudo apt-get install python-future python-lxml -y
sudo apt-get install ros-kinetic-control-toolbox
sudo easy_install pymavlink

# add swapfile to build mavros_extras
sudo fallocate -l 4G /swapfile
sudo mkswap /swapfile
sudo chmod 0600 /swapfile
sudo swapon /swapfile
sudo echo '/swapfile none swap sw 0 0' | tee -a /etc/fstab

# ros mavros catkin workspace
sudo -i -u odroid bash << EOF
whoami

source /opt/ros/kinetic/setup.bash

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
wstool init src

rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavros.rosinstall
rosinstall_generator --rosdistro kinetic --upstream mavros | tee -a /tmp/mavros.rosinstall

wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src -j4
rosdep install --from-paths src --ignore-src -y

sudo ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh

source devel/setup.bash

catkin build

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
echo "source $HOME/catkin_ws/devel/setup.bash" >> ~/.bashrc

EOF

sudo easy_install pypozyx
sudo easy_install phao-mqtt
