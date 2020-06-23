# Issues and warnings

## Open

These warnings appear when running the session on an Ubuntu 18.04 VM running in
VMware Fusion 11.5.5. They are OpenGL / GPU / hardware acceleration issues.

- `QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-root'`
- `libGL error: failed to open drm device: No such file or directory`
- `libGL error: failed to load driver: vmwgfx`

## Closed

This issue was due to a missing dependency on `rviz_imu_plugin`. It has been fixed
by adding `ros-melodic-imu-tools` to the Dockerfile.

- `rviz_imu_plugin/Imu' failed to load.  Error: According to the loaded plugin descriptions the class rviz_imu_plugin/Imu with base class type rviz::Display does not exist.`

## Ignore

The following messages are from Gazebo and can be safely ignored.

- `[Wrn] [msgs.cc:1852] Conversion of sensor type[anemometer] not supported.`
- `[Err] [msgs.cc:2883] Unrecognized geometry type`
- `[Wrn] [msgs.cc:1852] Conversion of sensor type[anemometer] not supported.`

## Node conflicts

You have to take care when running more than one service that uses the `rqt_gui` framework to avoid name collisions.

Usually when running two `rqt_gui` applications, such as
`rqt_robot_steering` and `rqt` on a single machine they will be
assigned unique node ids, for example:

```bash
# Start rqt
$ rqt

# Start rqt_robot_steering
$ rosrun rqt_robot_steering rqt_robot_steering

# Examine the node list
$ rosnode list
/rosout
/rqt_gui_py_node_24960
/rqt_gui_py_node_24990
```

However when using `docker-compose` each node is started on an independent `clean` machine, so we wind up with both nodes being assigned the id `/rqt_gui_py_node_1`, and depending upon
which node starts first we'll either get no messages from the steering gui, or `rqt` will not
be able to subscribe to any messages.

The fix is to start one or both nodes in separate namespaces.
