#!/usr/bin/env python3
'''this is a ROSI flippers sum
It receives flippers joint cmd vel command from multiple nodes and selects the most important to publish
to flippers mind

'''
import rospy
import genpy

import numpy as np

from controller.msg import Control
from rosi_common.msg import Float32Array

from rosi_common.rosi_tools import ctrlType, correctFlippersJointSignal

from rosi_common.node_status_tools import nodeStatus
from rosi_common.srv import SetNodeStatus, GetNodeStatusList

class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        ##=== Parameters
        # time window to discard last received command
        self.timeWindowToDiscardCmd = rospy.Duration.from_sec(0.3)

        ##=== Useful variables
        # node status object
        self.ns = nodeStatus(node_name)
        self.ns.resetActive() # this node is disabled by default

        # possible frame_ids to publish into /rosi/flippers/joint/cmd_vel
        self.pubNodesId = (
            "flippers_touch_granter",
            "flippers_space_2_joint_cmd_vel"
        )
        
        # default epos modes for publishing
        self.pubModesDefault = [ctrlType["Unchanged"]]*4+[ctrlType["Velocity"]]*4

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
        # ROS publisher
        self.pub_CtrlInputReq = rospy.Publisher('/rosi/flippers/joint/cmd_vel/sum', Control, queue_size=5)

        # ROS subscribere
        sub_cmdVel_leveler = rospy.Subscriber('/rosi/flippers/joint/cmd_vel/leveler', Float32Array, self.cllbck_cmdVel_leveler)
        sub_cmdVel_touchGranter = rospy.Subscriber('/rosi/flippers/joint/cmd_vel/touch_granter', Float32Array, self.cllbck_cmdVel_touchGranter)

        # services
        srv_setActive = rospy.Service(self.ns.getSrvPath('active', rospy), SetNodeStatus, self.srvcllbck_setActive)
        srv_getStatus = rospy.Service(self.ns.getSrvPath('getNodeStatus', rospy), GetNodeStatusList, self.srvcllbck_getStatus)
        srv_setHaltCmd = rospy.Service(self.ns.getSrvPath('haltcmd', rospy), SetNodeStatus, self.srvcllbck_setHaltCmd)

        ##=== Main method 
        self.nodeMain()
        

    def nodeMain(self):
        '''Node main script'''

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
                    if (time_current - self.lastCmd_timeStamp[self.pubNodesId[0]]) <= self.timeWindowToDiscardCmd: # touch granter

                        cmd += np.array([self.lastCmd_data[self.pubNodesId[0]]])[0]
                        #print('tg')
                    if (time_current - self.lastCmd_timeStamp[self.pubNodesId[1]]) <= self.timeWindowToDiscardCmd: # body leveler
                        cmd += np.array([self.lastCmd_data[self.pubNodesId[1]]])[0]
                        #print('bl')

                # mounting message to publish
                m = Control()
                m.header.stamp = time_current
                m.header.frame_id = 'flippers_cmd_sum'
                m.originId = 0
                m.modes = self.pubModesDefault
                m.data = [0.0]*4 + np.ndarray.tolist(cmd)
                self.pub_CtrlInputReq.publish(m)

                #print(m)
                #print('---')

            # sleeping node
            node_rate_sleep.sleep()


    def cllbck_cmdVel_leveler(self, msg):
        ''' Callback method when receives flippers command from the leveler loop'''
        self.lastCmd_timeStamp[msg.header.frame_id] = msg.header.stamp
        self.lastCmd_data[msg.header.frame_id] = msg.data


    def cllbck_cmdVel_touchGranter(self, msg):
        ''' Callback method when receives flippers command from the touch granter'''
        self.lastCmd_timeStamp[msg.header.frame_id] = msg.header.stamp
        self.lastCmd_data[msg.header.frame_id] = msg.data


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
    node_name = 'flippers_cmd_sum'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass



    