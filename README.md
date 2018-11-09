# Quadrotor_VelocityControl
The purpose of this respository is to simulate a quadrotor in Gazebo, and then execute control scripts. As of right now the gazebo enviornment is simply an empty world with the Iris quadrotor model.

## TO BEGIN:
Ensure that the tool chain has been set up for the PX4 flight stack following the instructions on the wiki:

[Tool Chain Installation](https://dev.px4.io/en/setup/dev_env_linux_ubuntu.html)

Additionally, clone the Firmware into your home folder:

[PX4 Firmware](https://github.com/PX4/Firmware)

Finally, install Mavros globally using `apt get` on your machine. The bash scripts in this repository assume that mavros was not built from source in a catkin workspace.

[Mavros Installation](https://dev.px4.io/en/ros/mavros_installation.html)

## Using the code
### Launching Gazebo
To launch gazebo, navigate to the folder, and run the bash script, launch_gazebo.sh
```
$ ./launch_gazebo.sh
```
This script sets up a roscore, and then runs the scrips: launch_mavros.sh, launch_build_gazebo.sh, and launch_ros_gazebo.sh.
The reason a roscore is set up first is so that the following scripts do not try and access the port of a master that does not exist. The roslaunch file in launch_ros_gazebo.sh will try and make a master if there is none, but since the processes are being run simultaneously, it cannot be set up in time.

### Controlling the Quad
Once Gazebo is running, ensure that mavros is communicating the topics by running in a new terminal:
```
$ rostopic list
```
A lengthy list of topics prefixes by /mavros/... should show up

The control scrips can then be run by navigating to the folder /Quadrotor_VelocityControl/Velocity_Control and executing
```
$ python filename.py
```
