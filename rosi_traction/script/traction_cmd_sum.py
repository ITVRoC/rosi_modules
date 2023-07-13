#!/usr/bin/env python3
'''this is a ROSI traction sum
It receives traction joint cmd vel command from multiple nodes and sum commands to publish

'''
import rospy

from rosi_common.msg import Float32Array
from controller.msg import Control

from rosi_common.rosi_tools import ctrlType

import numpy as np

from rosi_common.node_status_tools import nodeStatus
from rosi_common.srv import SetNodeStatus, GetNodeStatusList

class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        ##=== Parameters
        # time window to discard last received command
        self.timeWindowToDiscardCmd = rospy.Duration.from_sec(0.1)

        ##=== Useful variables
        # node status object
        self.ns = nodeStatus(node_name)

        # possible frame_ids to publish into /rosi/flippers/joint/cmd_vel
        self.pubNodesId = (
            'base_space_2_traction_joint_cmd_vel',
            'traction_flipper_risal_compensator'
        )

        # default epos modes for publishing
        self.pubModesDefault = [ctrlType["Velocity"]]*4 + [ctrlType["Unchanged"]]*4

        # initializing the last cmd time stamp
        self.lastCmd_timeStamp = {
                                self.pubNodesId[0]: rospy.Time(),
                                self.pubNodesId[1]: rospy.Time()
        }
        
        # initializing the last Cmd data (the commands itself, four positions, one for each flipper) 
        self.lastCmd_data = {
                                self.pubNodesId[0]: np.array([0.0]*4),
                                self.pubNodesId[1]: np.array([0.0]*4)
        }

        # dummy time
        self.dummyTime = rospy.Time()

        ##=== ROS interfaces
        self.pub_CtrlInputReq = rospy.Publisher('/rosi/controller/req_cmd', Control, queue_size=5)

        self.sub_cmdNavigation = rospy.Subscriber('/rosi/traction/joint/cmd_vel/navigation', Float32Array, self.cllbck_cmdNavigation)
        self.sub_cmdCompensator = rospy.Subscriber('/rosi/traction/joint/cmd_vel/compensator', Float32Array, self.cllbck_cmdCompensator)

        # node management services
        srv_getStatus = rospy.Service(self.ns.getSrvPath('getNodeStatus', rospy), GetNodeStatusList, self.srvcllbck_getStatus)
        srv_setActive = rospy.Service(self.ns.getSrvPath('active', rospy), SetNodeStatus, self.srvcllbck_setActive)
        srv_setHaltCmd = rospy.Service(self.ns.getSrvPath('haltcmd', rospy), SetNodeStatus, self.srvcllbck_setHaltCmd)


        ##=== Main method
        self.nodeMain()


    def nodeMain(self):
        '''Node main method'''

        # rate sleep
        node_rate_sleep = rospy.Rate(10)

        # eternal loop
        while not rospy.is_shutdown():

            if self.ns.getNodeStatus()['active']: # only runs if node is active

                # cmd list
                cmd = np.array([0.0] * 4)

                # receiving current ros time
                time_current = rospy.get_rostime()

                # only runs when receives at least one message
                if self.lastCmd_timeStamp[self.pubNodesId[0]] != self.dummyTime or self.lastCmd_timeStamp[self.pubNodesId[1]] != self.dummyTime:
                    # sums last received value if they are in the non-discard time span
                    if (abs(time_current - self.lastCmd_timeStamp[self.pubNodesId[0]])) <= self.timeWindowToDiscardCmd: # navigation command
                        cmd += self.lastCmd_data[self.pubNodesId[0]]
                        #print('navigator')
                    if (abs(time_current - self.lastCmd_timeStamp[self.pubNodesId[1]])) <= self.timeWindowToDiscardCmd: # compensator command
                        cmd += self.lastCmd_data[self.pubNodesId[1]]
                        #print('compensator')


                # if node is commanded to send halt commands
                if self.ns.getNodeStatus()['haltcmd']:  
                    cmd = np.array([0.0]*4)

                # mounting message to publish
                m = Control()
                m.header.stamp = time_current
                m.header.frame_id = 'traction_cmd_sum'
                m.originId = 0
                m.modes = self.pubModesDefault
                m.data = np.ndarray.tolist(cmd) + [0.0]*4
                self.pub_CtrlInputReq.publish(m)

                #print(m)

            # sleeping node
            node_rate_sleep.sleep()


    def cllbck_cmdNavigation(self, msg):
        '''Callback method for storing traction commands that came from navigation nodes'''
        self.lastCmd_timeStamp[msg.header.frame_id] = msg.header.stamp
        self.lastCmd_data[msg.header.frame_id] = np.array(msg.data)


    def cllbck_cmdCompensator(self, msg):
        '''Callback method for storing traction commands that came from the 
        forward differential kinematics of linear velocity in y axis'''
        self.lastCmd_timeStamp[msg.header.frame_id] = msg.header.stamp
        self.lastCmd_data[msg.header.frame_id] = np.array(msg.data)

    
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


if __name__ == '__main__':
    node_name = 'traction_cmd_sum'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass

















