#!/bin/bash

set -e

source /opt/ros/noetic/setup.bash

#systemctl start mongod
#systemctl status mongod

#service mongod start
#service mongod status

#mkdir -p $TVABOT_WS/src 

#cd $TVABOT_WS/src # use git for production not development
#git clone https://github.com/thillRobot/tvarobot_ros.git -b noetic-devel 

source $MOVEIT_WS/devel/setup.bash

export TVABOT_WS=/home/tvarobot_ws                
cd $TVABOT_WS
catkin build
source $TVABOT_WS/devel/setup.bash

#DEBIAN_FRONTEND=noninteractive rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} && \
#catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release && \
#catkin config --blacklist rviz_visual_tools && \

#catkin build
#source $TVABOT_WS/devel/setup.bash

#source /usr/share/gazebo/setup.sh
#export GAZEBO_RESOURCE_PATH="$GAZEBO_RESOURCE_PATH/$TVABOT_WS/src/tvarobot_ros/tvarobot_gazebo/worlds"

exec "$@"