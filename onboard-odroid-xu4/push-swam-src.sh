#!/bin/sh


if sudo ifconfig -a | grep -q "inet addr:192.168.88.243"; then
  echo Yes, I am master
  scp -r ~/catkin_ws/src/swam  donghee@192.168.88.239:/home/donghee/odroid_xu4/
else 
  echo No
fi
