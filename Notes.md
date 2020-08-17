# Notes

A list of articles that were referred to when setting up the docker image.

Stack Overflow discussion:

- [Can you run GUI applications in a Docker container?](https://stackoverflow.com/questions/16296753/can-you-run-gui-applications-in-a-docker-container/25280523#25280523)
- [Running GUI apps with Docker](http://fabiorehm.com/blog/2014/09/11/running-gui-apps-with-docker/)

Security Concerns:

- Sharing X11 info with docker allows the application within docker to receive every X11 event in the
desktop, including all keys typed in all other apps.

Suggested solutions:

- x11docker [https://github.com/mviereck/x11docker](https://github.com/mviereck/x11docker)

macOS:

Using `socat` for X11 port forwarding:

- [Running GUI Apps in Docker on Mac OS X](https://darrensnotebook.blogspot.com/2016/04/running-gui-apps-in-docker-on-mac-os-x.html)
- [how to use -e DISPLAY flag on osx?](https://github.com/moby/moby/issues/8710)
- [BWC: GUI apps in Docker on OSX](http://blog.bennycornelissen.nl.s3-website-eu-west-1.amazonaws.com/post/bwc-gui-apps-in-docker-on-osx/)
- [how to use -e DISPLAY flag on osx? #8710](https://github.com/moby/moby/issues/8710)

Sharing the X11 socket with the container:

- [Running GUI apps with Docker](http://fabiorehm.com/blog/2014/09/11/running-gui-apps-with-docker/)

A review of Docker for Mac

- [Docker for Mac: neat, fast, and flawed.](https://blog.bennycornelissen.nl/post/docker-for-mac-neat-fast-and-flawed/)

ROS Answers:

- [Running ROS and its gui tools through a docker image](https://answers.ros.org/question/313786/running-ros-and-its-gui-tools-through-a-docker-image/)

Discussion of ROS entrypoint files:

- [Unable to override osrf/ros entrypoint](https://answers.ros.org/question/320375/unable-to-override-osrfros-entrypoint/)

ROS docker tutorials:

- [dockerTutorialsCompose](http://wiki.ros.org/docker/Tutorials/Compose)
- [dockerTutorialsNetwork](https://wiki.ros.org/docker/Tutorials/Network)
- [dockerTutorialsGUI](https://wiki.ros.org/docker/Tutorials/GUI)

ROS docker examples

- [wilselby/ouster_example](https://github.com/wilselby/ouster_example).


## ArduPilot specific

ArduPilot docker image:

- [edrdo/ardupilot-sitl-docker](https://github.com/edrdo/ardupilot-sitl-docker)

Run with:

```bash
docker run --rm -it \
  -v $(pwd):/external \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  -e "SIM_OPTIONS=--out=udpout:127.0.0.1:14550 \
  -m --target-system=0 $SIM_OPTIONS" \
  --entrypoint "/external/entryPoint.sh" \
  edrdo/ardupilot-sitl-docker:latest 0
```

ArduPilot Gazebo plugins:

Linux install location of Gazebo plugins

```bash
/usr/lib/x86_64-linux-gnu/gazebo-9/plugins/libArduPilotPlugin.so
```

ArduPilot SITL:

Infer the mavproxy command line arguments from `sim_vehicle.py`:

```bash
mavproxy.py --map --console \
  --out 127.0.0.1:14550 \
  --out 127.0.0.1:14551 \
  --master tcp:127.0.0.1:5760 \
  --sitl 127.0.0.1:5501
```

Infer the ardupilot SITL command line arguments from running `sim_vehicle.py`:

```console
SIM_VEHICLE: "/home/rhys/Code/ardupilot/ardupilot/Tools/autotest/run_in_terminal_window.sh" "Rover" \
  "/home/rhys/Code/ardupilot/ardupilot/build/sitl/bin/ardurover" "-S" \
  "--model" "gazebo-sailboat" \
  "--speedup" "1" \
  "--defaults" "/home/rhys/Code/ardupilot/ardupilot/Tools/autotest/default_params/rover.parm,/home/rhys/Code/ardupilot/ardupilot/Tools/autotest/default_params/sailboat.parm" \
  "-I0" \
  "--home" "51.5734550122,-3.99302875275,0.0,1.0"
```