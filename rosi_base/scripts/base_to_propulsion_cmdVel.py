#!/usr/bin/env python3
'''This node receives to Twist command desired for {R} frame and computes
The velocity vector for each propulsion frame.'''
import rospy

import numpy as np

from rosi_common.msg import Vector3ArrayStamped, TwistStamped
from geometry_msgs.msg import Vector3

from rosi_common.rosi_tools import compute_J_c_dagger
from rosi_model.rosi_description import rotm_base_piFlp, tr_base_piFlp


class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name
        

        ##=== Useful variables
        self.msg_baseCmdVel = None


        ##=== One time calculations
        # computes the chassis jacobian
        self.J_c_dag = compute_J_c_dagger(rotm_base_piFlp.values(), tr_base_piFlp.values())


        ##=== ROS interfaces
        # publishers
        self.pub_propCmdVel = rospy.Publisher('/rosi/flippers/space/cmd_vel', Vector3ArrayStamped, queue_size=5)

        # subscribers
        sub_baseCmdVel = rospy.Subscriber('/rosi/base/space/cmd_vel', TwistStamped, self.cllbck_baseCmdVel)

        ##=== Main method
        self.nodeMain()

    
    def nodeMain(self):
        '''Node main method'''

        while not rospy.is_shutdown():

            # only runs if valid messages have been received
            if self.msg_baseCmdVel is not None:

                # converting ROS message to numpy array format
                xdot_c_in = np.array([self.msg_baseCmdVel.twist.linear.x,
                                      self.msg_baseCmdVel.twist.linear.y,
                                      self.msg_baseCmdVel.twist.linear.z,
                                      self.msg_baseCmdVel.twist.angular.x,
                                      self.msg_baseCmdVel.twist.angular.y, 
                                      self.msg_baseCmdVel.twist.angular.z,]).reshape(6,1)

                # computes the velocity command for each propulsion frame
                v_Pi_V = np.dot(self.J_c_dag, xdot_c_in)

                # mounting Vector3 array to publish
                v_Pi_V3arr = [Vector3(v_Pi_V[i*3][0], v_Pi_V[i*3+1][0], v_Pi_V[i*3+2][0]) for i in range(int(len(v_Pi_V)/3))]

                # mounting message to publish
                m = Vector3ArrayStamped()
                m.header.frame_id = self.node_name
                m.header.stamp = rospy.get_rostime()
                m.vec = v_Pi_V3arr
                self.pub_propCmdVel.publish(m)


    def cllbck_baseCmdVel(self, msg):
        ''' Callback for the robot base {R} frame cmd vel input
        '''
        self.msg_baseCmdVel = msg


if __name__ == '__main__':
    node_name = 'base_to_propulsion_cmdVel'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass