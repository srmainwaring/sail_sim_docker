# Docker files for a Gazebo simulation of a sailing robot

This project contains docker files to build and run a ROS / Gazebo
simulation of a sailing robot.

![Racing Sparrow 750 Simulation](https://github.com/srmainwaring/sail_sim_docker/wiki/images/ocean_waves_rs750_fft.jpg)

## Quick start

```bash
# Pull the docker image from Docker Hub
docker pull rhysmainwaring/sail-sim

# Run it
docker-compose -f docker-compose-nvidia.yaml up
```

If you have an AMD card or are running from a virtual machine use
`docker-compose-vmware.yaml` instead. For WSL use `docker-compose-wsl.yaml`

You will need to provide the containers `xhost` access to you machine:
`xhost +` will do this but there are serious security concerns. For a
discussion on better approaches see the [ROS docker tutorials](http://wiki.ros.org/docker/Tutorials/GUI).


The simulation launches a number of screens:

- Gazebo
- `rviz`
- `rqt`
- `rqt_robot_steering`

Set the wind using the Gazebo sidebar then unpause the simulation. The horizontal slider on `rqt_robot_steering` controls the rudder. The sails are trimmed automatically.

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

In addition there are a number of maths libraries and hardware acceleration drivers layered into the image. The Dockerfile has the full details.

## Building

```bash
# Clone the repo
git clone https://github.com/srmainwaring/sail_sim_docker.git

# Build the image
docker build -t rhysmainwaring/sail-sim .
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
