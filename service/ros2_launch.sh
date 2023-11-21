#!/bin/bash

SCRIPTDIR=/home/pi/service/
ENVFILE=/home/pi/ros2_ws/install/local_setup.bash

echo "Loading ROS2 Env..."
source /opt/ros/humble/setup.bash
source ${ENVFILE}
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "ROS2 Launching..."
exec ros2 launch scramble_auto_robot hardware.launch.py 2>&1

