#!/usr/bin/env python3
'''this is a ROSI flippers cmd directly from joy
It receives joy commands and converts it directly to flippers arms rotational velocity commands.

'''
import rospy

from sensor_msgs.msg import Joy
from controller.msg import Control

from rosi_common.rosi_tools import ctrlType, correctFlippersJointSignal

import numpy as np

from rosi_common.node_status_tools import nodeStatus
from rosi_common.srv import SetNodeStatus, GetNodeStatusList

class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        ##===== parameters

        # rosi direction side
        self.drive_side_param_path = '/rosi/forward_side'
        self.drive_side = self.getParamWithWait(self.drive_side_param_path)

        # flippers max rotational speed
        self.flippers_maxRotSpeed = 0.1 * 3.1415 # in [rad/s]

        ##===== useful variables
        # node status object
        self.ns = nodeStatus(node_name)
        self.ns.resetActive() # this node is disabled by default

        # default epos modes for publishing
        self.pubModesDefault = [ctrlType["Unchanged"]]*4+[ctrlType["Velocity"]]*4

        #==== ROS interfaces
        # publishers
        self.pub_cmdControllerReq = rospy.Publisher('/rosi/controller/req_cmd', Control, queue_size=5)
        
        # subscriber
        sub_Joy = rospy.Subscriber('/joy', Joy, self.cllbck_joy)

        # node management services
        srv_getStatus = rospy.Service(self.ns.getSrvPath('getNodeStatus', rospy), GetNodeStatusList, self.srvcllbck_getStatus)
        srv_setActive = rospy.Service(self.ns.getSrvPath('active', rospy), SetNodeStatus, self.srvcllbck_setActive)
        srv_setHaltCmd = rospy.Service(self.ns.getSrvPath('haltcmd', rospy), SetNodeStatus, self.srvcllbck_setHaltCmd)

        # spinning the node
        rospy.loginfo('entering in eternal loop.')
        rospy.spin()


    def cllbck_joy(self, msg):

        if self.ns.getNodeStatus()['active']: # only runs if node is active

            # takes which flippers to command from joystick input
            btn_flippersSelector = [1 if msg.axes[2] < 0 else 0, 1 if msg.axes[5] < 0 else 0, msg.buttons[4], msg.buttons[5]]

            # takes flippers command from joystick input
            axis_arm_cmd = -msg.axes[4]

            # computing rotational arms speed
            arm_rot_speed = axis_arm_cmd * self.flippers_maxRotSpeed

            # computing flippers command
            flippers_cmd = [a*b for a, b in zip(btn_flippersSelector, [arm_rot_speed, arm_rot_speed, arm_rot_speed, arm_rot_speed])]

            # if node is commanded to send halt commands
            if self.ns.getNodeStatus()['haltcmd']:  
                flippers_cmd = [0.0] * 4


            # updates drive param
            self.drive_side = rospy.get_param(self.drive_side_param_path)

            # mounting message to publish
            m = Control()
            m.header.stamp = rospy.get_rostime()
            m.header.frame_id = 'joy_flippers_cmdVel'
            m.originId = 0
            m.modes = self.pubModesDefault

            if self.drive_side == 'a':
                m.data = [0.0]*4 + np.ndarray.tolist(correctFlippersJointSignal(flippers_cmd))
                self.pub_cmdControllerReq.publish(m)

            elif self.drive_side == 'b':
                # reverses flippers cmd
                flippers_cmd = [val for val in reversed(flippers_cmd)]
                m.data = [0.0]*4 + np.ndarray.tolist(correctFlippersJointSignal(flippers_cmd))
                self.pub_cmdControllerReq.publish(m)
            
            else:
                rospy.logerr("[joy_flippers_cmdVel] wrong param "+self.drive_side_param_path+" value. Valid values are 'a' and 'b'")

            #print(m)


    ''' === Service Callbacks === '''
    def srvcllbck_getStatus(self, req):
        ''' Method for returning the node status flag list'''
        return self.ns.getNodeStatusSrvResponse()


    def srvcllbck_setActive(self, req):
        ''' Method for setting the active node status flag'''
        return self.ns.defActiveServiceReq(req, rospy)


    def srvcllbck_setHaltCmd(self, req):
        ''' Method for setting the haltCmd node status flag'''
        return self.ns.defHaltCmdServiceReq(req, rospy)
    

    @staticmethod
    def getParamWithWait(path_param):
        """Waits until a param exists so retrieves it"""
        while not  rospy.has_param(path_param):
            rospy.loginfo("[manager] Waiting for param: %s", path_param)
        return rospy.get_param(path_param)
			
            
if __name__ == '__main__':
    node_name = 'joy_flippers_cmdVel'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass



    