FROM ros:noetic-robot-focal

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

RUN apt update && apt upgrade -y && apt update && apt install -y apt-utils build-essential vim git wget

RUN apt install -y python3-catkin-tools

RUN apt update && \
DEBIAN_FRONTEND=noninteractive apt install -y ros-noetic-laser-geometry ros-noetic-pcl-conversions

ENV SD_WS=/home/seamdetection_ws
RUN mkdir -p $SD_WS/src 

#EXPOSE 11345

COPY ./entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"] 