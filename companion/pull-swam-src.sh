#!/bin/sh

if sudo ifconfig -a | grep -q "inet addr:192.168.88.243"; then
  echo I am master, do not need pull swam src
else
  scp -r donghee@192.168.88.239:/home/donghee/odroid_xu4/swam ~/catkin_ws/src
  cd ~/catkin_ws/src
  catkin build
fi
