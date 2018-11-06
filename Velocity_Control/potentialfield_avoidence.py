#/usr/bin/python

import rospy
import math
import numpy as np
import mavros_msgs
from geometry_msgs.msg import PoseStamped, TwistStamped
import mavros
from mavros import command
from mavros_msgs.srv import CommandBool, ParamGet, SetMode
from mavros_msgs.msg import State

current_pose = PoseStamped()
set_vel = TwistStamped()
current_state = State()

def obstacles():
	o = []
	r = .25
	o.append((1,1,1.5))

	return o,r

def pos_sub_callback(pose_sub_data):
	global set_vel
	global current_pose
	global vel_pub
	current_pose = pose_sub_data

	# Current Position, renamed to shorter variables
	x = current_pose.pose.position.x
	y = current_pose.pose.position.y
	z = current_pose.pose.position.z

	# Goal position
	xg = 2
	yg = 2
	zg = 4

	# Position error between setpoint and current position
	x_error = xg - x
	y_error = yg - y
	z_error = zg - z
	#dist = math.sqrt(x_error**2 + y_error**2 + z_error**2)
	#print(dist)

	# Unit vecotor, variable means 'direction_command'
	x_c = x_error# / dist
	y_c = y_error# / dist
	z_c = z_error# / dist

	# Publist to TwistStamped
	set_vel.twist.linear.x = .5*x_c
	set_vel.twist.linear.y = .5*y_c
	set_vel.twist.linear.z = .7*z_c

	if abs(set_vel.twist.linear.x) > 2:
		set_vel.twist.linear.x = np.sign(set_vel.twist.linear.z)*2
	if abs(set_vel.twist.linear.y) > 2:
        set_vel.twist.linear.y = np.sign(set_vel.twist.linear.y)*2
	if abs(set_vel.twist.linear.z) > 2:
        set_vel.twist.linear.z = np.sign(set_vel.twist.linear.z)*2



	#Publish the commanded velocity
	vel_pub.publish(set_vel)

def state_callback(state_data):
	global current_state
	current_state = state_data

def main():
	global vel_pub
	rospy.init_node('Velocity_Control', anonymous='True')

	# Initialize the obstacles
	obstacles()

	# Set up publishers and subscribers
	my_state = rospy.Subscriber('/mavros/state',State,state_callback)
	vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size = 1)
	local_position_subscribe = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pos_sub_callback)

	# Set the timeout for the ROS service checks
	service_timeout = 30
        rospy.loginfo("waiting for ROS services")
	# Ensure all services are running, and switch Quad to offboard
	while current_state.mode != "OFFBOARD" or not current_state.armed:
        	arm = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        	arm(True)
        	set_mode = rospy.ServiceProxy('/mavros/set_mode',SetMode)
        	mode = set_mode(custom_mode='OFFBOARD')
		rospy.wait_for_service('mavros/set_mode', service_timeout)
		rospy.loginfo("ROS services are up")
		if not mode.mode_sent:
			rospy.logerr("failed to send mode command")

	# Wait for keyboard input
	x = raw_input('Press ENTER to disarm quad: ')

	# Disarm quad so the RTL function doesnt consume time
	arm = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
	arm(False)

	# Keep program alive until we stop it
	rospy.spin()

# Run
if __name__ == "__main__":
    main()
