#!/bin/bash

set -e

source /opt/ros/noetic/setup.bash

export SD_WS=/home/seamdetection_ws

exec "$@"