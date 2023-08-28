#!/bin/bash

set -e

source /opt/ros/noetic/setup.bash

export ROS_WS=/home/catkin_ws

cd $ROS_WS
catkin_make
source $ROS_WS/devel/setup.bash

exec "$@"