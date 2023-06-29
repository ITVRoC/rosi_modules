#!/usr/bin/env python3
''' This is a ROSI algorithm
It receives joy input and generates a Twist for the ROSI base.
'''
import rospy

from rosi_common.msg import TwistStamped
from sensor_msgs.msg import Joy

from rosi_common.node_status_tools import nodeStatus
from rosi_common.srv import SetNodeStatus, GetNodeStatusList

import numpy as np

class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name
        
        #==== Parameters
        # parameters
        self.p_max_lin_speed = {
            'x': 0.7, # in [m/s] -- 1.1 is the so far maximum
            'y': 0.0, # in [m/s]
            'z': 0.04 # in [m/s]
        }   # max linear speed per axis

        self.p_max_ang_speed = {
            'x': 0.3,  # in [rad/s]
            'y': 0.3,  # in [rad/s]
            'z': 1.5*0.3491   # in [rad/s]
        }   # max rotational speed per axis

        # contant linear forward velocity
        self.vel_lin_cte = 6

        ##=== Useful variables
        # node status object
        self.ns = nodeStatus(node_name)
        self.ns.resetActive() # this node is disabled by default


        ##=== ROS interfaces

        # publishers
        self.pub_baseCmdVel = rospy.Publisher('/rosi/base/space/cmd_vel/joy', TwistStamped, queue_size=5)

        # subscribers
        sub_Joy = rospy.Subscriber('/joy', Joy, self.cllbck_joy)

        # services
        srv_setActive = rospy.Service(self.ns.getSrvPath('active', rospy), SetNodeStatus, self.srvcllbck_setActive)
        srv_getStatus = rospy.Service(self.ns.getSrvPath('getNodeStatus', rospy), GetNodeStatusList, self.srvcllbck_getStatus)
        srv_setHaltCmd = rospy.Service(self.ns.getSrvPath('haltcmd', rospy), SetNodeStatus, self.srvcllbck_setHaltCmd)

        # infinity
        rospy.loginfo('entering in eternal loop.')
        rospy.spin()            


    def cllbck_joy(self, msg):
        '''Callback for the joystick'''

        if self.ns.getNodeStatus()['active']: # only runs if node is active

            # translating joy input to twist commands for the robot base
            v_lin_x = msg.axes[1] * self.p_max_lin_speed['x'] if msg.buttons[1]==0 else np.sign(msg.axes[1])*self.vel_lin_cte
            v_lin_y = 0.0   # base non-holonomic restriction unables base to generate y_{R} sense's movement
            v_lin_z = ((msg.axes[2] - msg.axes[5])/2) * self.p_max_lin_speed['z']

            v_ang_x = -msg.axes[3] * self.p_max_ang_speed['x'] 
            v_ang_y = msg.axes[4] * self.p_max_ang_speed['y'] 
            v_ang_z = msg.axes[0] * self.p_max_ang_speed['z'] 

            # mount message to publish
            m = TwistStamped()
            m.header.stamp = rospy.get_rostime()
            m.header.frame_id = self.node_name

            if not self.ns.getNodeStatus()['haltcmd']:  
                m.twist.linear.x = v_lin_x
                m.twist.linear.y = v_lin_y
                m.twist.linear.z = v_lin_z
                m.twist.angular.x = v_ang_x
                m.twist.angular.y = v_ang_y
                m.twist.angular.z = v_ang_z 
            else:   # when node is halted
                m.twist.linear.x = 0.0
                m.twist.linear.y = 0.0
                m.twist.linear.z = 0.0
                m.twist.angular.x = 0.0
                m.twist.angular.y = 0.0
                m.twist.angular.z = 0.0

            self.pub_baseCmdVel.publish(m)
            #print(m)


    ''' === Service Callbacks === '''
    def srvcllbck_setActive(self, req):
        ''' Method for setting the active node status flag'''
        return self.ns.defActiveServiceReq(req, rospy)

    def srvcllbck_getStatus(self, req):
        ''' Method for returning the node status flag list'''
        return self.ns.getNodeStatusSrvResponse()

    def srvcllbck_setHaltCmd(self, req):
        ''' Method for setting the haltCmd node status flag'''
        return self.ns.defHaltCmdServiceReq(req, rospy)


if __name__ == '__main__':
    node_name = 'joy_base_cmdVel'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass