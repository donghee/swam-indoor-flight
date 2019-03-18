#!/bin/sh

cd ~/catkin_ws
rosrun swam mavros_offboard_posctl.py __ns:=/uav1 &
rosrun swam mavros_offboard_posctl.py __ns:=/uav2 &
rosrun swam mavros_offboard_posctl.py __ns:=/uav3 &
