#!/bin/bash

# This script launches Gazebo with an empty world and the Iris quadrotor model.
# This script must be placed in the home folder along with its dependencies which are:
#	launch_build_gazebo.sh
#	launch_ros_gazebo.sh
#	launch_mavros.sh

# In order for this script to function, the users computer must have an install of ROS Kinetic, Ubuntu 16.04,
# and Gazebo 7
# Additionally, the user must have initialized the toolchain for the PX4 Flight stack, and cloned the firmware

echo "Bash Version ${BASH_VERSION}"

firm=$HOME/Firmware


roscore & #>/dev/null &

# Sleep to allow the roscore to finish initializing, otherwise the launch files will not be able to locate the port
sleep 6

x-terminal-emulator -e $HOME/launch_build_gazebo.sh &>/dev/null &
x-terminal-emulator -e $HOME/launch_ros_gazebo.sh &>/dev/null &
x-terminal-emulator -e $HOME/launch_mavros.sh &>/dev/null &

cd $HOME/Quadrotor_VelocityControl/Velocity_Control/





















































































































