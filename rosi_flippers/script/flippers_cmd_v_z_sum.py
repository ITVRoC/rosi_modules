#!/usr/bin/env python3
'''this is a ROSI flippers node that sums velocities to the z vector in 
the propulsion frame.
'''
import rospy
import numpy as np
#from collections import deque

from rosi_common.msg import Vector3ArrayStamped
from geometry_msgs.msg import Vector3

from rosi_common.node_status_tools import nodeStatus

class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        ##=== Parameters

        # node status object
        self.ns = nodeStatus(node_name)
        #self.ns.resetActive() # this node is disabled by default

        # maximum input queue size
        self.deque_queue_max_elements = 10

        # time window to discard last received command
        self.timeWindowToDiscardCmd = rospy.Duration.from_sec(1)

        ##=== Useful variables
        # callback messages
        self.msg_cmdz_joy = None
        self.msg_cmdz_lev = None


        ##=== ROS interfaces
        # publishers
        self.pub_cmdVel_cmdVelVzPi = rospy.Publisher('/rosi/flippers/space/cmd_v_z', Vector3ArrayStamped, queue_size=5)

        # subscribers
        sub_cmdVel_cmdVelVzPi_joy = rospy.Subscriber('/rosi/propulsion/space/cmd_vel', Vector3ArrayStamped, self.cllbck_cmdVelVzPi_joy)
        sub_cmdVel_cmdVelVzPi_leveler = rospy.Subscriber('/rosi/flippers/space/cmd_v_z/leveler', Vector3ArrayStamped, self.cllbck_cmdVelVzPi_leveler)


        ##=== Main method 
        self.nodeMain()


    def nodeMain(self):
        '''Node main script'''

        # rate sleep
        node_rate_sleep = rospy.Rate(10)

        # eternal loop
        while not rospy.is_shutdown():

            if self.ns.getNodeStatus()['active']: # only runs if node is active

                # receives current ros time
                time_current = rospy.get_rostime()
                
                # empty command vector that will receive the sum
                cmd_v_z_sum = 4*[np.zeros(3)]

                # treats the command received by the joystick subsystem
                if self.msg_cmdz_joy is not None:
                    if (time_current - self.msg_cmdz_joy.header.stamp) <= self.timeWindowToDiscardCmd: 
                            cmd_v_z_sum = [x + np.array([v.x, v.y, v.z]) for x,v in zip(cmd_v_z_sum, self.msg_cmdz_joy.vec)]

                # treats the command received by the leveler subsystem
                if self.msg_cmdz_lev is not None:
                    if (time_current - self.msg_cmdz_lev.header.stamp) <= self.timeWindowToDiscardCmd: 
                            cmd_v_z_sum = [x + np.array([v.x, v.y, v.z]) for x,v in zip(cmd_v_z_sum, self.msg_cmdz_lev.vec)]

                # publishing message
                m = Vector3ArrayStamped()
                m.header.stamp = time_current
                m.header.frame_id = self.node_name
                m.vec = [Vector3(v[0], v[1], v[2]) for v in  cmd_v_z_sum]
                self.pub_cmdVel_cmdVelVzPi.publish(m)


    def cllbck_cmdVelVzPi_joy(self, msg): 
        """ Callback for a ROS topic"""
        self.msg_cmdz_joy = msg      


    def cllbck_cmdVelVzPi_leveler(self, msg): 
        """ Callback for a ROS topic"""
        self.msg_cmdz_lev = msg


if __name__ == '__main__':
    node_name = 'flippers_cmd_v_z_sum'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass