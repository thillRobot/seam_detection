FROM ros:noetic-robot-focal

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

RUN apt update && apt upgrade -y && apt update && apt install -y apt-utils build-essential vim git wget

RUN apt install -y python3-catkin-tools

RUN apt update && \
DEBIAN_FRONTEND=noninteractive apt install -y ros-noetic-pcl-ros ros-noetic-laser-geometry ros-noetic-pcl-conversions

RUN apt install -y ros-noetic-tf2-geometry-msgs ros-noetic-rviz python3-pip

RUN pip install --upgrade pip
RUN pip install --upgrade numpy scipy
RUN pip install --upgrade open3d

ENV SD_WS=/home/seamdetection_ws
RUN mkdir -p $SD_WS/src 

RUN apt install libeigen3-dev libboost-all-dev
RUN git clone https://github.com/MIT-SPARK/TEASER-plusplus.git $SD_WS/src/TEASER-plusplus
RUN cd $SD_WS/src/TEASER-plusplus && mkdir build && cd build && \
    cmake -DBUILD_TEASER_FPFH=ON .. && \
    make && \
    make install && \
    ldconfig
RUN cd $SD_WS/src/TEASER-plusplus/examples/teaser_cpp_ply && mkdir build && cd build && \
    cmake .. && make


#EXPOSE 11345

COPY ./entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"] 