#!/bin/bash

path=/home/kurt
firm=/home/kurt/Firmware

cd $firm

x-terminal-emulator -e $path/launch_build_gazebo.sh

x-terminal-emulator -e $path/launch_ros_gazebo.sh

x-terminal-emulator -e $path/launch_mavros.sh






















































































































