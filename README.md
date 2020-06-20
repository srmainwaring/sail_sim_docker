# Docker file for Sailing Simulation

Building:

```bash
git clone ssh://rhys@diskstation.local:/volume1/git/robotics/sail_sim_docker.git
cd sail_sim_docker && checkout develop

mkdir src && cd src
git clone ssh://rhys@diskstation.local:/volume1/git/robotics/asv_sim.git
git clone ssh://rhys@diskstation.local:/volume1/git/robotics/asv_wave_sim.git
git clone ssh://rhys@diskstation.local:/volume1/git/robotics/rs750.git

cd ~/sail_sim_docker/src/rs750
git checkout feature/controller

cd ~/sail_sim_docker/src/asv_sim
git checkout feature/docker

cd ~/sail_sim_docker/src/asv_wave_sim
git checkout feature/fft_waves

cd ~/sail_sim_docker
sudo docker build -t rhysmainwaring/sail-sim .
```

Running (recklessly insecure):

```bash
# !!!!!!!!!
xhost +

sudo docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --name rhysmainwaring/sail-sim sail-sim
```

```bash
sudo docker exec -it sail-sim bash
```

## Links

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

## Appendix: runtime warnings

Open:

These warnings appear when running the session on an Ubuntu 18.04 VM running in
VMware Fusion 11.5.5. They are OpenGL / GPU / hardware acceleration issues.

- `QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-root'`
- `libGL error: failed to open drm device: No such file or directory`
- `libGL error: failed to load driver: vmwgfx`

Closed:

This issue was due to a missing dependency on `rviz_imu_plugin`. It has been fixed
by adding `ros-melodic-imu-tools` to the Dockerfile.

- `rviz_imu_plugin/Imu' failed to load.  Error: According to the loaded plugin descriptions the class rviz_imu_plugin/Imu with base class type rviz::Display does not exist.`

Ignore:

The following messages are from Gazebo and can be safely ignored.

- `[Wrn] [msgs.cc:1852] Conversion of sensor type[anemometer] not supported.`
- `[Err] [msgs.cc:2883] Unrecognized geometry type`
- `[Wrn] [msgs.cc:1852] Conversion of sensor type[anemometer] not supported.`

```console
$ sudo docker-compose up
Starting sailsimdocker_ros-master_1 ... 
Starting sailsimdocker_ros-master_1 ... done
Starting sailsimdocker_steering_1 ... 
Starting sailsimdocker_steering_1
Starting sailsimdocker_rviz_1 ... 
Starting sailsimdocker_twist-translate_1 ... 
Starting sailsimdocker_rviz_1
Starting sailsimdocker_twist-translate_1
Starting sailsimdocker_gazebo_1 ... 
Starting sailsimdocker_sail-controller_1 ... 
Starting sailsimdocker_gazebo_1
Starting sailsimdocker_steering_1 ... done
Attaching to sailsimdocker_ros-master_1, sailsimdocker_twist-translate_1, sailsimdocker_rviz_1, sailsimdocker_gazebo_1, sailsimdocker_sail-controller_1, sailsimdocker_steering_1
ros-master_1       | ... logging to /root/.ros/log/ef2f5c06-b32a-11ea-8017-0242ac120002/roslaunch-11da9c1e9c18-1.log
ros-master_1       | Checking log directory for disk usage. This may take a while.
ros-master_1       | Press Ctrl-C to interrupt
ros-master_1       | Done checking log file disk usage. Usage is <1GB.
ros-master_1       | 
ros-master_1       | started roslaunch server http://11da9c1e9c18:33871/
ros-master_1       | ros_comm version 1.14.5
ros-master_1       | 
ros-master_1       | 
ros-master_1       | SUMMARY
ros-master_1       | ========
ros-master_1       | 
ros-master_1       | PARAMETERS
ros-master_1       |  * /rosdistro: melodic
ros-master_1       |  * /rosversion: 1.14.5
ros-master_1       | 
ros-master_1       | NODES
ros-master_1       | 
ros-master_1       | auto-starting new master
ros-master_1       | process[master]: started with pid [63]
ros-master_1       | ROS_MASTER_URI=http://11da9c1e9c18:11311/
ros-master_1       | 
ros-master_1       | setting /run_id to ef2f5c06-b32a-11ea-8017-0242ac120002
ros-master_1       | process[rosout-1]: started with pid [74]
ros-master_1       | started core service [/rosout]
rviz_1             | ... logging to /root/.ros/log/ef2f5c06-b32a-11ea-8017-0242ac120002/roslaunch-ba9b9f4ea674-1.log
rviz_1             | Checking log directory for disk usage. This may take a while.
rviz_1             | Press Ctrl-C to interrupt
rviz_1             | Done checking log file disk usage. Usage is <1GB.
rviz_1             | 
rviz_1             | started roslaunch server http://rviz:45307/
rviz_1             | 
rviz_1             | SUMMARY
rviz_1             | ========
rviz_1             | 
rviz_1             | PARAMETERS
rviz_1             |  * /rosdistro: melodic
rviz_1             |  * /rosversion: 1.14.5
rviz_1             | 
rviz_1             | NODES
rviz_1             |   /
rviz_1             |     rviz (rviz/rviz)
rviz_1             | 
rviz_1             | ROS_MASTER_URI=http://ros-master:11311
rviz_1             | 
rviz_1             | process[rviz-1]: started with pid [62]
rviz_1             | QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-root'
rviz_1             | libGL error: failed to open drm device: No such file or directory
rviz_1             | libGL error: failed to load driver: vmwgfx
rviz_1             | libGL error: failed to open drm device: No such file or directory
rviz_1             | libGL error: failed to load driver: vmwgfx
rviz_1             | [ERROR] [1592680760.076940062]: PluginlibFactory: The plugin for class 'rviz_imu_plugin/Imu' failed to load.  Error: According to the loaded plugin descriptions the class rviz_imu_plugin/Imu with base class type rviz::Display does not exist. Declared types are  rviz/Axes rviz/Camera rviz/DepthCloud rviz/Effort rviz/FluidPressure rviz/Grid rviz/GridCells rviz/Illuminance rviz/Image rviz/InteractiveMarkers rviz/LaserScan rviz/Map rviz/Marker rviz/MarkerArray rviz/Odometry rviz/Path rviz/PointCloud rviz/PointCloud2 rviz/PointStamped rviz/Polygon rviz/Pose rviz/PoseArray rviz/PoseWithCovariance rviz/Range rviz/RelativeHumidity rviz/RobotModel rviz/TF rviz/Temperature rviz/WrenchStamped rviz_plugin_tutorials/Imu
gazebo_1           | ... logging to /root/.ros/log/ef2f5c06-b32a-11ea-8017-0242ac120002/roslaunch-589fa6084171-1.log
gazebo_1           | Checking log directory for disk usage. This may take a while.
gazebo_1           | Press Ctrl-C to interrupt
gazebo_1           | Done checking log file disk usage. Usage is <1GB.
gazebo_1           | 
gazebo_1           | started roslaunch server http://gazebo:42897/
gazebo_1           | 
gazebo_1           | SUMMARY
gazebo_1           | ========
gazebo_1           | 
gazebo_1           | PARAMETERS
gazebo_1           |  * /fore_sail_position/joint: fore_sail_joint
gazebo_1           |  * /fore_sail_position/type: position_controll...
gazebo_1           |  * /gazebo/enable_ros_network: True
gazebo_1           |  * /joint_state_publisher/publish_rate: 50
gazebo_1           |  * /joint_state_publisher/type: joint_state_contr...
gazebo_1           |  * /main_sail_position/joint: main_sail_joint
gazebo_1           |  * /main_sail_position/type: position_controll...
gazebo_1           |  * /robot_description: <?xml version="1....
gazebo_1           |  * /robot_state_publisher/publish_frequency: 50.0
gazebo_1           |  * /rosdistro: melodic
gazebo_1           |  * /rosversion: 1.14.5
gazebo_1           |  * /rudder_position/joint: rudder_joint
gazebo_1           |  * /rudder_position/type: position_controll...
gazebo_1           |  * /use_sim_time: True
gazebo_1           | 
gazebo_1           | NODES
gazebo_1           |   /
gazebo_1           |     controller_spawner (controller_manager/spawner)
gazebo_1           |     gazebo (gazebo_ros/gzserver)
gazebo_1           |     gazebo_gui (gazebo_ros/gzclient)
gazebo_1           |     robot_state_publisher (robot_state_publisher/robot_state_publisher)
gazebo_1           |     urdf_spawner (gazebo_ros/spawn_model)
gazebo_1           | 
gazebo_1           | ROS_MASTER_URI=http://ros-master:11311
gazebo_1           | 
gazebo_1           | process[gazebo-1]: started with pid [65]
gazebo_1           | process[gazebo_gui-2]: started with pid [69]
gazebo_1           | process[controller_spawner-3]: started with pid [123]
gazebo_1           | Gazebo multi-robot simulator, version 9.0.0
gazebo_1           | Copyright (C) 2012 Open Source Robotics Foundation.
gazebo_1           | Released under the Apache 2 License.
gazebo_1           | http://gazebosim.org
gazebo_1           | 
gazebo_1           | [Msg] RegisterMsg: Type: asv_msgs.msgs.Anemometer
gazebo_1           | [Msg] RegisterMsg: Type: asv_msgs.msgs.LiftDrag
gazebo_1           | [Msg] RegisterSensor: Type: Anemometer
gazebo_1           | [ INFO] [1592680761.640931756]: Finished loading Gazebo ROS API Plugin.
gazebo_1           | [Msg] Waiting for master.
gazebo_1           | [Msg] Connected to gazebo master @ http://127.0.0.1:11345
sail-controller_1  | [INFO] [1592680761.852677, 0.000000]: Starting sail controller
sail-controller_1  | [INFO] [1592680761.874673, 0.000000]: Starting control loop at 10.0 Hz
gazebo_1           | [Msg] Publicized address: 172.18.0.5
gazebo_1           | libGL error: failed to open drm device: No such file or directory
gazebo_1           | libGL error: failed to load driver: vmwgfx
gazebo_1           | [ INFO] [1592680761.656666770]: waitForService: Service [/gazebo/set_physics_properties] has not been advertised, waiting...
gazebo_1           | Gazebo multi-robot simulator, version 9.0.0
gazebo_1           | Copyright (C) 2012 Open Source Robotics Foundation.
gazebo_1           | Released under the Apache 2 License.
gazebo_1           | http://gazebosim.org
gazebo_1           | 
gazebo_1           | [Wrn] [GuiIface.cc:199] g/gui-plugin is really loading a SystemPlugin. To load a GUI plugin please use --gui-client-plugin 
gazebo_1           | process[robot_state_publisher-4]: started with pid [170]
gazebo_1           | [Msg] RegisterMsg: Type: asv_msgs.msgs.Anemometer
gazebo_1           | [Msg] RegisterMsg: Type: asv_msgs.msgs.LiftDrag
gazebo_1           | [Msg] RegisterSensor: Type: Anemometer
gazebo_1           | [ INFO] [1592680761.890275745]: Finished loading Gazebo ROS API Plugin.
gazebo_1           | [Msg] Waiting for master.
gazebo_1           | [ INFO] [1592680761.896081691]: waitForService: Service [/gazebo_gui/set_physics_properties] has not been advertised, waiting...
gazebo_1           | [INFO] [1592680761.896642, 0.000000]: Controller Spawner: Waiting for service controller_manager/load_controller
gazebo_1           | [Msg] Connected to gazebo master @ http://127.0.0.1:11345
gazebo_1           | [Msg] Publicized address: 172.18.0.5
gazebo_1           | libGL error: failed to open drm device: No such file or directory
gazebo_1           | libGL error: failed to load driver: vmwgfx
gazebo_1           | process[urdf_spawner-5]: started with pid [197]
gazebo_1           | ALSA lib confmisc.c:767:(parse_card) cannot find card '0'
gazebo_1           | ALSA lib conf.c:4528:(_snd_config_evaluate) function snd_func_card_driver returned error: No such file or directory
gazebo_1           | ALSA lib confmisc.c:392:(snd_func_concat) error evaluating strings
gazebo_1           | ALSA lib conf.c:4528:(_snd_config_evaluate) function snd_func_concat returned error: No such file or directory
gazebo_1           | ALSA lib confmisc.c:1246:(snd_func_refer) error evaluating name
gazebo_1           | ALSA lib conf.c:4528:(_snd_config_evaluate) function snd_func_refer returned error: No such file or directory
gazebo_1           | ALSA lib conf.c:5007:(snd_config_expand) Evaluate error: No such file or directory
gazebo_1           | ALSA lib pcm.c:2495:(snd_pcm_open_noupdate) Unknown PCM default
gazebo_1           | AL lib: (EE) ALCplaybackAlsa_open: Could not open playback device 'default': No such file or directory
gazebo_1           | [Err] [OpenAL.cc:84] Unable to open audio device[default]
gazebo_1           |  Audio will be disabled.
gazebo_1           | [Wrn] [GuiIface.cc:120] QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-root'
gazebo_1           | [ INFO] [1592680762.056190000]: waitForService: Service [/gazebo/set_physics_properties] is now available.
gazebo_1           | [Msg] Parameter found - setting <static> to <0>.
gazebo_1           | [Msg] Parameter found - setting <update_rate> to <30>.
gazebo_1           | [Msg] Parameter found - setting <wave_patch> to <0>.
gazebo_1           | [Msg] Parameter found - setting <wave_patch_size> to <4 4>.
gazebo_1           | [Msg] Parameter found - setting <size> to <1000 1000>.
gazebo_1           | [Msg] Parameter found - setting <cell_count> to <50 50>.
gazebo_1           | [Msg] Parameter found - setting <number> to <3>.
gazebo_1           | [Msg] Parameter found - setting <amplitude> to <1>.
gazebo_1           | [Msg] Parameter found - setting <period> to <8>.
gazebo_1           | [Msg] Parameter <phase> not found: Using default value of <0>.
gazebo_1           | [Msg] Parameter found - setting <direction> to <1 1>.
gazebo_1           | [Msg] Parameter found - setting <scale> to <2.5>.
gazebo_1           | [Msg] Parameter found - setting <angle> to <0.3>.
gazebo_1           | [Msg] Parameter found - setting <steepness> to <1>.
gazebo_1           | [Msg] Constructing WavefieldOceanTile...
gazebo_1           | [Msg] Creating WaveParameters.
gazebo_1           | [Msg] Creating OceanTile.
gazebo_1           | libGL error: failed to open drm device: No such file or directory
gazebo_1           | libGL error: failed to load driver: vmwgfx
gazebo_1           | [ INFO] [1592680762.120555445]: Physics dynamic reconfigure ready.
gazebo_1           | [Msg] Creating grid.
gazebo_1           | [Msg] Creating triangulated grid.
gazebo_1           | [Msg] Done constructing WavefieldOceanTile.
gazebo_1           | [Err] [REST.cc:205] Error in REST request
gazebo_1           | 
gazebo_1           | libcurl: (51) SSL: no alternative certificate subject name matches target host name 'api.ignitionfuel.org'
gazebo_1           | [INFO] [1592680762.674517, 0.000000]: Loading model XML from ros parameter robot_description
gazebo_1           | [INFO] [1592680762.685981, 0.000000]: Waiting for service /gazebo/spawn_urdf_model
gazebo_1           | [INFO] [1592680762.689848, 0.000000]: Calling service /gazebo/spawn_urdf_model
gazebo_1           | [ INFO] [1592680762.848388178]: <robotNamespace> set to: //
gazebo_1           | [ INFO] [1592680762.848428693]: <topicName> set to: //sensors/imu
gazebo_1           | [ INFO] [1592680762.848437486]: <frameName> set to: imu_link
gazebo_1           | [ INFO] [1592680762.848455085]: <updateRateHZ> set to: 50
gazebo_1           | [ INFO] [1592680762.848470599]: <gaussianNoise> set to: 0
gazebo_1           | [ INFO] [1592680762.848486709]: <xyzOffset> set to: 0 0 0
gazebo_1           | [ INFO] [1592680762.848510742]: <rpyOffset> set to: 0 -0 0
steering_1         | QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-root'
gazebo_1           | [Wrn] [msgs.cc:1852] Conversion of sensor type[anemometer] not supported.
gazebo_1           | [Msg] main_sail_liftdrag Plugin <cp> set to -0.30875 0 1.30625
gazebo_1           | [Msg] main_sail_liftdrag Plugin <link_name> set to main_sail_link
gazebo_1           | [Msg] main_sail_liftdrag Plugin <topic> set to lift_drag
gazebo_1           | [Msg] <fluid_density> set to 1.2
gazebo_1           | [Msg] <radial_symmetry> set to 1
gazebo_1           | [Msg] <forward> set to 1 0 0
gazebo_1           | [Msg] <upward> set to 0 1 0
gazebo_1           | [Msg] <area> set to 1.81
gazebo_1           | [Msg] <a0> set to 0
gazebo_1           | [Msg] <alpha_stall> set to 0.1592
gazebo_1           | [Msg] <cla> set to 6.2832
gazebo_1           | [Msg] <cla_stall> set to -0.7083
gazebo_1           | [Msg] <cda> set to 0.63662
gazebo_1           | [Msg] fore_sail_liftdrag Plugin <cp> set to -0.605667 0 1.20528
gazebo_1           | [Msg] fore_sail_liftdrag Plugin <link_name> set to fore_sail_link
gazebo_1           | [Msg] fore_sail_liftdrag Plugin <topic> set to lift_drag
gazebo_1           | [Msg] <fluid_density> set to 1.2
gazebo_1           | [Msg] <radial_symmetry> set to 1
gazebo_1           | [Msg] <forward> set to 1 0 0
gazebo_1           | [Msg] <upward> set to 0 1 0
gazebo_1           | [Msg] <area> set to 1.43
gazebo_1           | [Msg] <a0> set to 0
gazebo_1           | [Msg] <alpha_stall> set to 0.1592
gazebo_1           | [Msg] <cla> set to 6.2832
gazebo_1           | [Msg] <cla_stall> set to -0.7083
gazebo_1           | [Msg] <cda> set to 0.63662
gazebo_1           | [INFO] [1592680762.866268, 0.000000]: Spawn status: SpawnModel: Successfully spawned entity
gazebo_1           | [ INFO] [1592680762.875526686]: Loading gazebo_ros_control plugin
gazebo_1           | [ INFO] [1592680762.875610385]: Starting gazebo_ros_control plugin in namespace: /
gazebo_1           | [ INFO] [1592680762.876452124]: gazebo_ros_control plugin is waiting for model URDF in parameter [robot_description] on the ROS param server.
gazebo_1           | [ERROR] [1592680763.004118764]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/rudder_joint
gazebo_1           | [ERROR] [1592680763.005063096]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/main_sail_joint
gazebo_1           | [ERROR] [1592680763.005761450]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/fore_sail_joint
gazebo_1           | [ INFO] [1592680763.009962434]: Loaded gazebo_ros_control.
gazebo_1           | [Msg] Parameter found - setting <wave_model> to <ocean>.
gazebo_1           | [Msg] Parameter found - setting <damping_on> to <1>.
gazebo_1           | [Msg] Parameter found - setting <viscous_drag_on> to <1>.
gazebo_1           | [Msg] Parameter found - setting <pressure_drag_on> to <1>.
gazebo_1           | [Msg] Parameter <cDampL1> not found: Using default value of <1e-06>.
gazebo_1           | [Msg] Parameter <cDampL2> not found: Using default value of <1e-06>.
gazebo_1           | [Msg] Parameter <cDampR1> not found: Using default value of <1e-06>.
gazebo_1           | [Msg] Parameter <cDampR2> not found: Using default value of <1e-06>.
gazebo_1           | [Msg] Parameter <cPDrag1> not found: Using default value of <100>.
gazebo_1           | [Msg] Parameter <cPDrag2> not found: Using default value of <100>.
gazebo_1           | [Msg] Parameter <fPDrag> not found: Using default value of <0.4>.
gazebo_1           | [Msg] Parameter <cSDrag1> not found: Using default value of <100>.
gazebo_1           | [Msg] Parameter <cSDrag2> not found: Using default value of <100>.
gazebo_1           | [Msg] Parameter <fSDrag> not found: Using default value of <0.4>.
gazebo_1           | [Msg] Parameter <vRDrag> not found: Using default value of <1>.
gazebo_1           | [Msg] Parameter found - setting <update_rate> to <30>.
gazebo_1           | [Msg] Parameter found - setting <water_patch> to <0>.
gazebo_1           | [Msg] Parameter found - setting <waterline> to <0>.
gazebo_1           | [Msg] Parameter found - setting <underwater_surface> to <0>.
gazebo_1           | [Msg] Shape:      shape
gazebo_1           | [Msg] Scale:      1 1 1
gazebo_1           | [Msg] Type:       30000
gazebo_1           | [Msg] Type:       BOX_SHAPE
gazebo_1           | [Msg] Size:       0.01 0.01 0.01
gazebo_1           | [Msg] Vertex:     24
gazebo_1           | [Msg] Shape:      shape
gazebo_1           | [Msg] Scale:      1 1 1
gazebo_1           | [Msg] Type:       50000
gazebo_1           | [Msg] Type:       CYLINDER_SHAPE
gazebo_1           | [Msg] Radius:     0.02
gazebo_1           | [Msg] Length:     0.05
gazebo_1           | [Msg] Mesh:       rs750::base_link::base_link_fixed_joint_lump__anemometer_link_collision_1::cylinder
gazebo_1           | [Msg] Vertex:     134
gazebo_1           | [Msg] Shape:      shape
gazebo_1           | [Msg] Scale:      1 1 1
gazebo_1           | [Msg] Type:       2010000
gazebo_1           | [Msg] Type:       MESH_SHAPE
gazebo_1           | [Msg] MeshURI:    /catkin_ws/install/share/rs750_description/meshes/rs750_hull_collision_3.stl
gazebo_1           | [Msg] MeshStr:    /catkin_ws/install/share/rs750_description/meshes/rs750_hull_collision_3.stl
gazebo_1           | [Msg] Mesh:       /catkin_ws/install/share/rs750_description/meshes/rs750_hull_collision_3.stl
gazebo_1           | [Msg] Vertex:     717
gazebo_1           | [Msg] Shape:      shape
gazebo_1           | [Msg] Scale:      1 1 1
gazebo_1           | [Msg] Type:       30000
gazebo_1           | [Msg] Type:       BOX_SHAPE
gazebo_1           | [Msg] Size:       0.031 0.026 0.005
gazebo_1           | [Msg] Vertex:     24
gazebo_1           | [Msg] Shape:      shape
gazebo_1           | [Msg] Scale:      1 1 1
gazebo_1           | [Msg] Type:       50000
gazebo_1           | [Msg] Type:       CYLINDER_SHAPE
gazebo_1           | [Msg] Radius:     0.048
gazebo_1           | [Msg] Length:     0.688
gazebo_1           | [Msg] Mesh:       rs750::base_link::base_link_fixed_joint_lump__keel_bulb_link_collision_4::cylinder
gazebo_1           | [Msg] Vertex:     134
gazebo_1           | [Msg] Shape:      shape
gazebo_1           | [Msg] Scale:      1 1 1
gazebo_1           | [Msg] Type:       30000
gazebo_1           | [Msg] Type:       BOX_SHAPE
gazebo_1           | [Msg] Size:       0.031 0.026 0.005
gazebo_1           | [Msg] Vertex:     24
gazebo_1           | [Msg] Shape:      shape
gazebo_1           | [Msg] Scale:      1 1 1
gazebo_1           | [Msg] Type:       2010000
gazebo_1           | [Msg] Type:       MESH_SHAPE
gazebo_1           | [Msg] MeshURI:    /catkin_ws/install/share/rs750_description/meshes/rs750_fore_sail.stl
gazebo_1           | [Msg] MeshStr:    /catkin_ws/install/share/rs750_description/meshes/rs750_fore_sail.stl
gazebo_1           | [Msg] Mesh:       /catkin_ws/install/share/rs750_description/meshes/rs750_fore_sail.stl
gazebo_1           | [Msg] Vertex:     24
gazebo_1           | [Msg] Shape:      shape
gazebo_1           | [Msg] Scale:      1 1 1
gazebo_1           | [Msg] Type:       2010000
gazebo_1           | [Msg] Type:       MESH_SHAPE
gazebo_1           | [Msg] MeshURI:    /catkin_ws/install/share/rs750_description/meshes/rs750_keel_fin_collision_2.stl
gazebo_1           | [Msg] MeshStr:    /catkin_ws/install/share/rs750_description/meshes/rs750_keel_fin_collision_2.stl
gazebo_1           | [Msg] Mesh:       /catkin_ws/install/share/rs750_description/meshes/rs750_keel_fin_collision_2.stl
gazebo_1           | [Msg] Vertex:     387
gazebo_1           | [Msg] Shape:      shape
gazebo_1           | [Msg] Scale:      1 1 1
gazebo_1           | [Msg] Type:       2010000
gazebo_1           | [Msg] Type:       MESH_SHAPE
gazebo_1           | [Msg] MeshURI:    /catkin_ws/install/share/rs750_description/meshes/rs750_main_sail.stl
gazebo_1           | [Msg] MeshStr:    /catkin_ws/install/share/rs750_description/meshes/rs750_main_sail.stl
gazebo_1           | [Msg] Mesh:       /catkin_ws/install/share/rs750_description/meshes/rs750_main_sail.stl
gazebo_1           | [Msg] Vertex:     120
gazebo_1           | [Msg] Shape:      shape
gazebo_1           | [Msg] Scale:      1 1 1
gazebo_1           | [Msg] Type:       2010000
gazebo_1           | [Msg] Type:       MESH_SHAPE
gazebo_1           | [Msg] MeshURI:    /catkin_ws/install/share/rs750_description/meshes/rs750_rudder_collision_3.stl
gazebo_1           | [Msg] MeshStr:    /catkin_ws/install/share/rs750_description/meshes/rs750_rudder_collision_3.stl
gazebo_1           | [Msg] Mesh:       /catkin_ws/install/share/rs750_description/meshes/rs750_rudder_collision_3.stl
gazebo_1           | [Msg] Vertex:     129
gazebo_1           | [Msg] links:  5
gazebo_1           | [Msg] meshes: 5
gazebo_1           | [Msg] Water patch size: 13.0209
gazebo_1           | [Msg] Water patch size: 7.35919
gazebo_1           | [Msg] Water patch size: 2.27981
gazebo_1           | [Msg] Water patch size: 8.27735
gazebo_1           | [Msg] Water patch size: 1.23884
gazebo_1           | [Msg] keel_fin_liftdrag Plugin <cp> set to 0 0 0
gazebo_1           | [Msg] keel_fin_liftdrag Plugin <link_name> set to keel_fin_link
gazebo_1           | [Msg] keel_fin_liftdrag Plugin <topic> set to lift_drag
gazebo_1           | [Msg] <fluid_density> set to 1000
gazebo_1           | [Msg] <radial_symmetry> set to 1
gazebo_1           | [Msg] <forward> set to 1 0 0
gazebo_1           | [Msg] <upward> set to 0 1 0
gazebo_1           | [Msg] <area> set to 0.17
gazebo_1           | [Msg] <a0> set to 0
gazebo_1           | [Msg] <alpha_stall> set to 0.1592
gazebo_1           | [Msg] <cla> set to 6.2832
gazebo_1           | [Msg] <cla_stall> set to -0.7083
gazebo_1           | [Msg] <cda> set to 0.63662
gazebo_1           | [Msg] rudder_liftdrag Plugin <cp> set to -0.06 0 -0.2
gazebo_1           | [Msg] rudder_liftdrag Plugin <link_name> set to rudder_link
gazebo_1           | [Msg] rudder_liftdrag Plugin <topic> set to lift_drag
gazebo_1           | [Msg] <fluid_density> set to 1000
gazebo_1           | [Msg] <radial_symmetry> set to 1
gazebo_1           | [Msg] <forward> set to 1 0 0
gazebo_1           | [Msg] <upward> set to 0 1 0
gazebo_1           | [Msg] <area> set to 0.12
gazebo_1           | [Msg] <a0> set to 0
gazebo_1           | [Msg] <alpha_stall> set to 0.1592
gazebo_1           | [Msg] <cla> set to 6.2832
gazebo_1           | [Msg] <cla_stall> set to -0.7083
gazebo_1           | [Msg] <cda> set to 0.63662
gazebo_1           | [INFO] [1592680763.126444, 0.000000]: Controller Spawner: Waiting for service controller_manager/switch_controller
gazebo_1           | [INFO] [1592680763.130215, 0.000000]: Controller Spawner: Waiting for service controller_manager/unload_controller
gazebo_1           | [INFO] [1592680763.133384, 0.000000]: Loading controller: joint_state_publisher
gazebo_1           | [Err] [msgs.cc:2883] Unrecognized geometry type
gazebo_1           | [INFO] [1592680763.144899, 0.000000]: Loading controller: rudder_position
gazebo_1           | [INFO] [1592680763.160073, 0.000000]: Loading controller: main_sail_position
gazebo_1           | [Msg] Loading OceanVisualPlugin...
gazebo_1           | [Msg] Done loading OceanVisualPlugin.
gazebo_1           | [Msg] Initializing OceanVisualPlugin...
gazebo_1           | [INFO] [1592680763.170307, 0.000000]: Loading controller: fore_sail_position
gazebo_1           | [urdf_spawner-5] process has finished cleanly
gazebo_1           | log file: /root/.ros/log/ef2f5c06-b32a-11ea-8017-0242ac120002/urdf_spawner-5*.log
gazebo_1           | [INFO] [1592680763.184280, 0.000000]: Controller Spawner: Loaded controllers: joint_state_publisher, rudder_position, main_sail_position, fore_sail_position
steering_1         | libGL error: failed to open drm device: No such file or directory
steering_1         | libGL error: failed to load driver: vmwgfx
gazebo_1           | [Wrn] [msgs.cc:1852] Conversion of sensor type[anemometer] not supported.
gazebo_1           | [Msg] Done initializing OceanVisualPlugin.
gazebo_1           | [Msg] Initializing OceanVisualPlugin...
gazebo_1           | [Msg] Done initializing OceanVisualPlugin.
gazebo_1           | [Wrn] [Publisher.cc:141] Queue limit reached for topic /gazebo/ocean_world/user_camera/pose, deleting message. This warning is printed only once.
```



