#!/usr/bin/env bash

# Start Gazebo
/gazebo_entrypoint.sh /bin/bash -c 'gazebo /catkin_ws/install/share/rs750_gazebo/worlds/triangle_course_ocean.world --verbose' &
