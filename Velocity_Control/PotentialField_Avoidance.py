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

def create_obstacles():
	global obstacles,radius
	obstacles = []
	radius = .5  
	obstacles.append((3.5,3.5,1))

	#return obstacles,radius

def pos_sub_callback(pose_sub_data):
	global set_vel
	global current_pose
	global vel_pub
	global obstacles,radius
	current_pose = pose_sub_data

	# Current Position, renamed to shorter variables
	x = current_pose.pose.position.x
	y = current_pose.pose.position.y
	z = current_pose.pose.position.z

	# Goal position
	xg = 6
	yg = 6
	zg = 1.5

	# Position error between setpoint and current position
	x_error = xg - x
	y_error = yg - y
	z_error = zg - z

	# Velocity vector to get to goal
	gx = .5*x_error
	gy = .5*y_error
	gz = 1.5*z_error

	### Distance to obstacle
	c = 1   # Repulsion force
	k = 1
	# Vector of tuples corresponding to repulsive vectors
	o_dists = []
	for i in obstacles:
		x_error = abs(i[0] - x) #- radius
		y_error = abs(i[1] - y) #- radius
		z_error = abs(i[2] - z) #- radius
		if (x_error or y_error or z_error) < 0:
			print('ERROR')
		#print('Errors: {} {} {}'.format(x_error,y_error,z_error))
		o_x = c/np.power(x_error*k,3)
		o_y = c/np.power(y_error*k,3)
		o_z = c/np.power(z_error*k,3)

		o_dists.append((o_x,o_y,o_z))

	# Velocity commands x,y,z sent to quadrotor
	# Initially equal to the attractive force of the goal
	cx = gx
	cy = gy
	cz = gz
	# Add each component from the repulsive forces
	for k in o_dists:
		cx += -1*np.sign(cx)*k[0]
		cy += -1*np.sign(cy)*k[1]
		cz += -1*np.sign(cz)*k[2]


	# Set limits on the velocity the quad can have
	if abs(cx) > 2:
		cx = np.sign(cx)*2
	if abs(cy) > 2:
		cy = np.sign(cy)*2
	if abs(cz) > 2:
		cz = np.sign(cz)*2

	# Set the message values to the velocity commands
	set_vel.twist.linear.x = cx
	set_vel.twist.linear.y = cy
	set_vel.twist.linear.z = cz

	#Publish the commanded velocity
	vel_pub.publish(set_vel)

def state_callback(state_data):
	global current_state
	current_state = state_data

def main():
	global vel_pub
	rospy.init_node('Velocity_Control', anonymous='True')

	# Initialize the obstacles
	create_obstacles()

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
	x = raw_input('Press ENTER to land quad: ')

	while current_state.mode != "AUTO.LAND":
        	set_mode = rospy.ServiceProxy('/mavros/set_mode',SetMode)
        	mode = set_mode(custom_mode='AUTO.LAND')
		rospy.wait_for_service('mavros/set_mode', service_timeout)
		rospy.loginfo("ROS services are up")
		if not mode.mode_sent:
			rospy.logerr("failed to send mode command")

	x = raw_input('Press ENTER to disarm quad: ')
	# Disarm quad so the RTL function doesnt consume time
	arm(False)

	# Keep program alive until we stop it
	rospy.spin()

# Run
if __name__ == "__main__":
    main()
