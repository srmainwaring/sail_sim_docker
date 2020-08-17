#!/usr/bin/env bash

# Start xterm for SITL
/bin/bash -c 'xterm -fa \"Monospace\" -fs 12 -hold -e /ardupilot/Tools/autotest/sim_vehicle.py -N -v Rover -f gazebo-sailboat -L Mumbles --console --map' &
