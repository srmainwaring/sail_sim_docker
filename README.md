# Docker files for a Gazebo simulation of a sailing robot

This project contains docker files to build and run an Ardupilot / Gazebo
simulation of a sailing robot.

![Racing Sparrow 750 Simulation](https://github.com/srmainwaring/sail_sim_docker/wiki/images/ocean_waves_rs750_fft.jpg)

## Quick start

```bash
# Pull the docker image from Docker Hub
docker pull rhysmainwaring/sail-sim-ardupilot

# Run it
docker-compose -f docker-compose.yaml up
```

If you have an AMD card or are running from a virtual machine use
`docker-compose-vmware.yaml`. For WSL use `docker-compose-wsl.yaml`
If you have an NVIDA card use `docker-compose-nvidia.yaml` instead.

You will need to provide the containers `xhost` access to you machine:
`xhost +` will do this but there are serious security concerns. For a
discussion on better approaches see the
[ROS docker tutorials](http://wiki.ros.org/docker/Tutorials/GUI).


## Usage

The simulation launches a number of screens:

- Gazebo
- ArduPilot sim_vehicle.py console
- ArduPilot Rover console
- MavProxy console
- MavProxy map

The [ArduPilot Sailboat](https://ardupilot.org/rover/docs/sailboat-home.html) documentation has a guide to
using sailing vehicles with ArduPilot, in particular the sailing modes and parameter tuning.

The [ArduPilot SITL Simulator](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)
documentation explains how to configure and use the simulator.

### Demo mission

Load the mission way points:

- From the MavProxy console 'Mission' menu select 'Editor'
- Click the 'Load WP File' button and open `/catkin_ws/install/share/rs750_gazebo/config/sailboat_missions.txt`
- Click 'Write WPs' to load them to the flight controller
- At this point you should see the waypoints in the MavProxy map.

![Mission Editor](https://github.com/srmainwaring/sail_sim_docker/wiki/images/mission_editor.jpg)

![Map Waypoints](https://github.com/srmainwaring/sail_sim_docker/wiki/images/map_waypoints.jpg)


Prepare the sailboat:

- In the `sim_vehicle.py` console arm the throttle: `MANUAL> arm throttle force` (to override PreArm EKF calibration warnings)
- Set the mode to auto: `MANUAL> mode auto`

Update the Gazebo wind environment:

- At initialisation the Gazebo environment settings are switched off. Without this the gyro and EFK calibration will not initialise correctly.
- Using the Gazebo sidebar set the wind `linear_velocity.x = 10.0`
- The sailboat should start moving under autopilot control around the triangular course.

![Gazebo Auto](https://github.com/srmainwaring/sail_sim_docker/wiki/images/gazebo_mip.jpg)

![Map Auto](https://github.com/srmainwaring/sail_sim_docker/wiki/images/map_mip.jpg)


## Dependencies

The project depends on the following:

- base docker image: `osrf/ros:melodic-desktop-full`
- marine simulation packages:
  - [asv_wave_sim](https://github.com/srmainwaring/asv_wave_sim.git)
  - [asv_sim](https://github.com/srmainwaring/asv_sim.git)
- robot model:
  - [rs750](https://github.com/srmainwaring/rs750.git)
- sensor plugins:
  - [hector-gazebo-plugins](http://wiki.ros.org/hector_gazebo_plugins)
- modified versions of ardupilot and ardupilot_gazebo:
  - [ardupilot](https://github.com/srmainwaring/ardupilot)
  - [ardupilot_gazebo](https://github.com/srmainwaring/ardupilot_gazebo)

In addition there are a number of maths libraries and hardware acceleration drivers layered into the image. The Dockerfile has the full details.

## Building

```bash
# Clone the repo
git clone https://github.com/srmainwaring/sail_sim_docker.git

# Build the image
docker build -t rhysmainwaring/sail-sim-ardupilot .
```

## Build Status

|    | Melodic |
|--- |--- |
| asv_wave_sim | [![Build Status](https://travis-ci.com/srmainwaring/asv_wave_sim.svg?branch=feature%2Ffft_waves)](https://travis-ci.com/srmainwaring/asv_wave_sim) |
| asv_sim | [![Build Status](https://travis-ci.com/srmainwaring/asv_sim.svg?branch=feature%2Fwrsc-devel)](https://travis-ci.com/srmainwaring/asv_sim) |
| rs750 | [![Build Status](https://travis-ci.com/srmainwaring/rs750.svg?branch=feature%2Fwrsc-devel)](https://travis-ci.com/srmainwaring/rs750) |


## License
This is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This software is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
