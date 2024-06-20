#!/bin/bash
cd ${HOME}/catkin_workspace/
source /opt/ros/${ROS_DISTRO}/setup.bash
source ${HOME}/catkin_workspace/devel/setup.bash
echo "launching application, please wait..."
roslaunch whi_imu whi_imu_ns.launch robot_name:=whi

