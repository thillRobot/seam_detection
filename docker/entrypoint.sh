#!/bin/bash

set -e

source /opt/ros/noetic/setup.bash

export SD_WS=/home/seamdetection_ws

cd $SD_WS
catkin build
source $SD_WS/devel/setup.bash

exec "$@"