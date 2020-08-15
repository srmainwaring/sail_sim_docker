#!/usr/bin/env bash
set -e

source "/opt/ros/melodic/setup.bash"
source "/catkin_ws/install/setup.bash"

# Gazebo environment
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/catkin_ws/install/share/asv_wave_sim_gazebo/models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/catkin_ws/install/share/asv_wave_sim_gazebo/world_models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/catkin_ws/install/share/rs750_gazebo/models
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/catkin_ws/install/lib

# ArduPilot
#export PATH=/ardupilot/ardupilot/Tools/autotest

exec "$@"
