#!/usr/bin/env bash
set -e

source "/opt/ros/melodic/setup.bash"
source "/catkin_ws/install/setup.bash"

exec "$@"
