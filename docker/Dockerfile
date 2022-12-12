FROM ros:noetic-robot-focal

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

RUN apt update && apt upgrade -y && apt update && apt install -y apt-utils build-essential vim git wget

#RUN apt update && apt install -y ros-noetic-rviz ros-noetic-joint-state-publisher-gui ros-noetic-gazebo-ros ros-noetic-gazebo-ros-pkgs  ros-noetic-gazebo-ros-control ros-noetic-rqt-reconfigure ros-noetic-rqt-publisher ros-noetic-controller-manager ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-rviz-visual-tools 

# RUN apt install python3-wstool python3-catkin-tools clang-format-10 python3-rosdep -y

RUN apt install -y python3-catkin-tools

ENV SD_WS=/home/seamdetection_ws
RUN mkdir -p $SD_WS/src 

#EXPOSE 11345

COPY ./entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"] 