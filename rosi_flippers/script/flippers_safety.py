#!/usr/bin/env python3
'''this is a ROSI flippers safety node
It receives flippers cmd_vel and cuts it if unsafety commands are detected
Safety considers flippers max speed and operational range for the pose regulator.

'''
from time import time
import rospy

from controller.msg import Control
from sensor_msgs.msg import JointState
from rosi_common.msg import BoolArrayStamped

import numpy as np

from rosi_common.rosi_tools import clipFlipVel, clipFlipPos, jointStateData2dict, ctrlType, correctFlippersJointSignal
from rosi_model.rosi_description import dict_flprsPosLimits

from rosi_common.node_status_tools import nodeStatus
from rosi_common.srv import SetNodeStatus, GetNodeStatusList

class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        ##== Parameters
         # minimum velocity command to send for flippers in deg/s (absolute value around zero)
        self.p_flpCmd_minValue = np.deg2rad(0.3)   

         # flippers joints max rotational velocity in deg/s
        self.dict_flprsVelLimits = {'negative': -np.deg2rad(30), 'positive': np.deg2rad(30)}

        # flippers angular position limits
        #self.dict_flprsPosLimits = {'min': np.deg2rad(20), 'max': np.deg2rad(160)} 

        # note sleep rate
        self.p_nodeSleepRate = 20 # [Hz]

        ##==== useful variables
        # node status object
        self.ns = nodeStatus(node_name)
        self.ns.resetActive() # this node is disabled by default

        self.msg_flpJointCmdSum = None

        self.msg_flpJntState = None
        self.flag_maxPosReached = None

        # default epos modes for publishing
        self.pubModesDefault = [ctrlType["Unchanged"]]*4+[ctrlType["Velocity"]]*4

        ##==== ROS Interfaces

        # publisher

        # safety filtered flippers' joints command
        self.pub_cmdVelFlpJointSafety = rospy.Publisher('/rosi/controller/req_cmd', Control, queue_size=5)

        # safety-lock status
        self.pub_safetyLock_maxPos = rospy.Publisher('/rosi/flippers/status/safety/max_pos_lock', BoolArrayStamped, queue_size=5)


        # subscribers
        sub_cmdVelFlpJointSum = rospy.Subscriber('/rosi/flippers/joint/cmd_vel/sum', Control, self.cllbck_cmdVelFlpJointSum)
        sub_jointState = rospy.Subscriber('/rosi/rosi_controller/joint_state', JointState, self.cllbck_jointState)

        # services
        srv_getStatus = rospy.Service(self.ns.getSrvPath('getNodeStatus', rospy), GetNodeStatusList, self.srvcllbck_getStatus)
        srv_setActive = rospy.Service(self.ns.getSrvPath('active', rospy), SetNodeStatus, self.srvcllbck_setActive)
        srv_setBypass = rospy.Service(self.ns.getSrvPath('bypass', rospy), SetNodeStatus, self.srvcllbck_setBypass)
        srv_setHaltCmd = rospy.Service(self.ns.getSrvPath('haltcmd', rospy), SetNodeStatus, self.srvcllbck_setHaltCmd)
        srv_setTelemetry = rospy.Service(self.ns.getSrvPath('telemetry', rospy), SetNodeStatus, self.srvcllbck_setTelemetry)

        # spins the node
        self.nodeMain()


    def nodeMain(self):
        '''Node main method'''

        # node rate sleep
        node_rate_sleep =  rospy.Rate(self.p_nodeSleepRate)

        # waits until valid messages have been received
        rospy.loginfo('[%s] Waiting for valid messages to appear.')
        while not rospy.is_shutdown():
            if self.msg_flpJointCmdSum is not None and self.msg_flpJntState is not None:
                break
            node_rate_sleep.sleep()


        # initializing variables
        dt = 0.0
        time_last = rospy.get_rostime()
        flag_firstRun = True

        while not rospy.is_shutdown():

            if self.ns.getNodeStatus()['active'] or self.ns.getNodeStatus()['telemetry']: # only runs if node is active

                # extracting flippers velocity command
                flpJnt_cmdVel_l = correctFlippersJointSignal(list(self.msg_flpJointCmdSum.data[4:]))
                
                # treating time dt
                time_current = rospy.get_rostime()
                if flag_firstRun:
                    dt = 0.0
                    flag_firstRun = False
                else:
                    dt = (time_current - time_last).to_sec()
                time_last = time_current

                # cuts the command if it is too small 
                flpJnt_cmdVel_l =  [cmd if np.abs(cmd) >= self.p_flpCmd_minValue else 0.0 for cmd in flpJnt_cmdVel_l]

                # clips flippers command based on maximum rotational speed
                flpJnt_cmdVel_l = clipFlipVel(flpJnt_cmdVel_l, self.dict_flprsVelLimits)

                # cuts commands that would lead flippers outside safe operational range for the chassis kinematics
                flpJnt_cmdVel_l, flag_maxPosReached = clipFlipPos(flpJnt_cmdVel_l, self.msg_flpJntState["pos"], dict_flprsPosLimits , dt)
                
                
                ##=== Treating node status flags
                
                """if self.ns.getNodeStatus()['haltcmd']:  # if node is commanded to send halt commands
                    flpJnt_cmdVel_l = [0.0]*4
                elif self.ns.getNodeStatus()['bypass']: # if bypass is enabled, node only forwards received msg data
                    flpJnt_cmdVel_l = list(msg.data[4:])"""


                ##=== Publishing messages
                # mounting message to publish flippers command
               
                m = Control()
                m.header.stamp = time_current
                m.header.frame_id = 'flippers_safety'
                m.originId = 0
                m.modes = self.pubModesDefault
                m.data = [0.0]*4 + np.ndarray.tolist(correctFlippersJointSignal(flpJnt_cmdVel_l))
                self.pub_cmdVelFlpJointSafety.publish(m)
                #print(m)

                
                # mounting message to publish flippers position safety lock
                m = BoolArrayStamped()
                m.header.stamp = time_current
                m.header.frame_id = 'flippers_safety'
                if flag_maxPosReached is not None:
                    m.data = [0 if a==1 else 1 for a in flag_maxPosReached] # inverses the mask for suggesting a boolean that is 1 when a maxPosition cutout has occurred
                else:
                    m.data = [0, 0, 0, 0]
                self.pub_safetyLock_maxPos.publish(m)
                #print(m)

                        
            # sleeping the node
            node_rate_sleep.sleep()



    def cllbck_cmdVelFlpJointSum(self,msg):
        ''' ROS topic callback that receives flippers commands'''
        self.msg_flpJointCmdSum = msg

        
    def cllbck_jointState(self, msg):
        '''Callback method for jointState topic'''
        _, self.msg_flpJntState = jointStateData2dict(msg)


    ''' === Service Callbacks === '''
    def srvcllbck_getStatus(self, req):
        ''' Method for returning the node status flag list'''
        return self.ns.getNodeStatusSrvResponse()

    def srvcllbck_setActive(self, req):
        ''' Method for setting the active node status flag'''
        return self.ns.defActiveServiceReq(req, rospy)

    def srvcllbck_setBypass(self, req):
        ''' Method for setting the bypass node status flag'''
        return self.ns.defBypassServiceReq(req, rospy)

    def srvcllbck_setHaltCmd(self, req):
        ''' Method for setting the haltCmd node status flag'''
        return self.ns.defHaltCmdServiceReq(req, rospy)

    def srvcllbck_setTelemetry(self, req):
        ''' Method for setting the telemetry node status flag'''
        return self.ns.defTelemetryServiceReq(req, rospy)


if __name__ == '__main__':
    node_name = 'flippers_safety'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass


    