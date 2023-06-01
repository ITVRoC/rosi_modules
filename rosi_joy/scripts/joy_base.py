#!/usr/bin/env python3
import sys
import rospy
import numpy as np

from sensor_msgs.msg import Joy
from controller.msg import Control

from controller.srv import RequestID
from rosi_joy.srv import * #SetNodeEnabled, SetNodeEnabledResponse, GetNodeEnabled, GetNodeEnabledResponse

from phd_common.rosi_tools import *

# TODO rever a matriz cinematica do rosi

class RosiNodeClass():

	# class constructor
	def __init__(self):

		''' ==== CLASS ATTRIBUTES ==== '''

		# parameters
		self.max_translational_speed = 25 # in [m/s]
		self.max_rotational_speed = 20 # in [rad/s]
		self.max_arms_rotational_speed = 3.1415/6 # in [rad/s]

		#  class variables initialization
		self.joint_correction = [1, -1, 1, -1, -1, 1, 1, -1]  # this is 4*[traction} + 4*[arms]
		self.node_enabled = True

		# how to obtain these values? see Mandow et al. COMPLETE THIS REFERENCE
		self.var_lambda = 0.965
		self.wheel_radius = 0.13
		self.ycir = 0.531

		''' ==== PREAMBLE ==== '''

		# initializing useful variables
		self.originID = 0
		
		self.pub_msg_modes = [ctrlType["NotControlled"]] * 8
		self.pub_msg_data = [0] * 8
		
		self.pub_msg_modes_halted = [2] * 8
		self.pub_msg_data_halted = [0.0] * 8

		# receiving ros parameters
		self._param_joy_arms_cmd = rospy.get_param("/rosi_joy_arms_cmd", 'true')
		rospy.loginfo("[PARAM] /rosi_joy_arms: " + str(self._param_joy_arms_cmd))

		# computing the kinematic A matrix
		self.kin_matrix_A = self.compute_kinematicAMatrix(self.var_lambda, self.wheel_radius, self.ycir)

		''' ==== ROS INTERFACING ==== '''

		# topic  publishers
		self.pub_rosi_cmd = rospy.Publisher('/rosi/mind/req_cmd', Control, queue_size=1)

		# topic  subscribers
		self.sub_joy = rospy.Subscriber('/joy', Joy, self.callback_Joy)

		# services
		srv_setNodeEnabled = rospy.Service("/rosi_joy/base/set_enabled", SetNodeEnabled, self.cllbckSrv_setNodeEnabled)
		srv_getNodeEnabled = rospy.Service("/rosi_joy/base/get_enabled", GetNodeEnabled, self.cllbckSrv_getNodeEnabled)

		# calling node main
		self.nodeMain()


	def nodeMain(self):
		''' Node main script'''

		# defining the eternal loop frequency
		node_sleep_rate = rospy.Rate(10)

		# true loop
		rospy.loginfo("Node in spin mode.")
		while not rospy.is_shutdown():

			if self.node_enabled: # only sends joystick commands if node is enabled

				# receiving ROS time
				time_ros = rospy.get_rostime()

				# mount pub message
				pub_msg = Control()
				pub_msg.header.stamp = time_ros
				pub_msg.header.frame_id = "rosi_joy2"
				pub_msg.originId = self.originID

				pub_msg.modes = self.pub_msg_modes
				pub_msg.data = self.pub_msg_data

				# publishing the message
				self.pub_rosi_cmd.publish(pub_msg)

			# sleep for a while
			node_sleep_rate.sleep()


	# ---- Support Methods --------
	# -- Method for compute the skid-steer A kinematic matrix
	@staticmethod
	def compute_kinematicAMatrix(var_lambda, wheel_radius, ycir):

		# kinematic A matrix 
		matrix_A = np.array([[var_lambda*wheel_radius/2, var_lambda*wheel_radius/2],
							[(var_lambda*wheel_radius)/(2*ycir), -(var_lambda*wheel_radius)/(2*ycir)]])

		return matrix_A


	# joystick callback function
	def callback_Joy(self, msg):

		# receives current rostime
		time_ros = rospy.get_rostime()

		# receiving and treating commands
		j_vel_lin = msg.axes[1]
		j_vel_ang = msg.axes[0]
		j_arm_cmd = msg.axes[4]
		j_arms_s = [1 if msg.axes[2] < 0 else 0, 1 if msg.axes[5] < 0 else 0, msg.buttons[4], msg.buttons[5]]

		''' Treating traction command'''
		# computing desired linear and angular velocities of the platform
		des_vel_linear_x = self.max_translational_speed * j_vel_lin
		des_vel_angular_z = self.max_rotational_speed * j_vel_ang

		#print('linear:',des_vel_linear_x,'  angular:',des_vel_angular_z)

		# b vector
		b = np.array([[des_vel_linear_x], [des_vel_angular_z]])

		# finds the joints control
		x = np.linalg.lstsq(self.kin_matrix_A, b, rcond=-1)[0]

		# query the sides velocities and converts to rad/s
		omega_right = np.ndarray.tolist(np.deg2rad(x[0]))[0]
		omega_left = np.ndarray.tolist(np.deg2rad(x[1]))[0]

		traction_modes = [ctrlType["Velocity"]] * 4
		traction_cmd = [omega_left, omega_right, omega_left, omega_right] # inverts omega_right considering rosi mountage

		''' Treating arms command'''
		if self._param_joy_arms_cmd:	# tests if it is desirable to command flippers joints from the joystick

			# computing rotational arms speed
			arm_rot_speed = j_arm_cmd * self.max_arms_rotational_speed

			# populating message class attribute
			flippers_modes = [ctrlType["Velocity"]] * 4
			flippers_cmd = [a*b for a, b in zip(j_arms_s, [arm_rot_speed, arm_rot_speed, arm_rot_speed, arm_rot_speed])]
			
		else:
			flippers_modes = [ctrlType["Unchanged"]] * 4
			flippers_cmd = [0, 0, 0, 0]
			

		''' Appending commands '''
		self.pub_msg_modes = traction_modes + flippers_modes
		self.pub_msg_data = self.correct_joints_signal(traction_cmd + flippers_cmd)


	def cllbckSrv_setNodeEnabled(self, req):
		''' set nodeEnabled flag service request callback'''
		
		rospy.loginfo("Setting node to %r state", req.data)

		# setting node enabled flag
		self.node_enabled = req.data

		# mounting response
		r = SetNodeEnabledResponse()
		r.success = True
		r.message = "Node now in " + str(self.node_enabled) + " state."

		return r


	def cllbckSrv_getNodeEnabled(self, req):
		'''get nodeEnabled flag service request callback '''

		r = GetNodeEnabledResponse()
		r.current_mode = self.node_enabled
		 
		return r


	# corrects joint signals
	def correct_joints_signal(self, l_in):
		return [a*b for a,b in zip(l_in, self.joint_correction)] 


# starting the node
if __name__ == '__main__':

	# initialize the node
	rospy.loginfo("RosiJoy node started.")
	rospy.init_node('joy_base', anonymous=True)

	# instantiate the class
	try:
		node_obj = RosiNodeClass()
	except rospy.ROSInterruptException: pass

