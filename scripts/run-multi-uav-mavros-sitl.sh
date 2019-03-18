#!/bin/sh

cd ~/src/Firmware
#./Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export GAZEBO_PLUGIN_PATH=/home/user/src/Firmware/build/px4_sitl_default/build_gazebo:$GAZEBO_PLUGIN_PATH
export GAZEBO_MODEL_PATH=/home/user/src/Firmware/Tools/sitl_gazebo/models:$GAZEBO_PLUGIN_PATH
export LD_LIBRARY_PATH=/opt/ros/kinetic/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu:/home/user/src/Firmware/build/px4_sitl_default/build_gazebo:$LD_LIBRARY_PATH

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo

roslaunch px4 multi_uav_mavros_sitl.launch gui:=false
