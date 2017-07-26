#!/bin/bash
rm -rf build devel
cd src
catkin_init_workspace
cd ..

catkin_make
#catkin_make --pkg rplidar_ros

