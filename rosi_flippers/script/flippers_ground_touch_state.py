#!/usr/bin/env python3
''' This is a ROSI flippers algorithm
It receives flippers joint torque generates a boolean array suggesting which flipper is touching the ground
It considers that flippers are in the extended semi-circle (pointing outwards considering the robot x axis)
'''
import rospy

import numpy as np

from collections import deque

from rosi_common.msg import Int8ArrayStamped
from sensor_msgs.msg import JointState

from rosi_common.rosi_tools import correctFlippersJointSignal

from rosi_common.node_status_tools import nodeStatus
from rosi_common.srv import SetNodeStatus, GetNodeStatusList

class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        ##=== PARAMETERS

        # Torque min threshold for considering flippers as touching the ground
        # notice that flippers assume positive values when they are touching the ground.
        self.torqueThreshold = 0 # N.m

        # time window to state that no further message has been received
        self.timeWindowToDiscard = rospy.Duration.from_sec(0.3)

        # number of last torque readings to perform the avg value of last received torque data
        self.torqueAvgQtd = 10

        # flipper joint position range within the touch inference is made. Outside it, we consider wheels as touching the ground.
        self.flpJnt_inferenceRange = {
            'min': np.deg2rad(50),
            'max': np.deg2rad(180)
        }

        ##=== Useful variables
        # node status object
        self.ns = nodeStatus(node_name)

        # flippers status message received from the topic
        self.flpMsg = None

        # dt from current ros time from last
        self.dt = None

        # accumulated dt from last received message
        self.dt_MsgAccum = None

        # last time
        self.time_last = None

        # last received message time sequence
        self.lastMsgSeq = None
        self.lastMsgTime = None

        # torque log variable
        self.torque_log_l = [deque(maxlen=self.torqueAvgQtd),
                             deque(maxlen=self.torqueAvgQtd),
                             deque(maxlen=self.torqueAvgQtd),
                             deque(maxlen=self.torqueAvgQtd)]

        ##=== ROS interfaces
        self.pub_touchStatus = rospy.Publisher('/rosi/flippers/status/touching_ground', Int8ArrayStamped, queue_size=5)

        sub_jointState = rospy.Subscriber('/rosi/rosi_controller/joint_state', JointState, self.cllbck_jointState)

        # services
        srv_setActive = rospy.Service(self.ns.getSrvPath('active', rospy), SetNodeStatus, self.srvcllbck_setActive)
        srv_getStatus = rospy.Service(self.ns.getSrvPath('getNodeStatus', rospy), GetNodeStatusList, self.srvcllbck_getStatus)


        # calling node main node
        self.nodeMain()

    
    def nodeMain(self):
        '''Node main method'''

        # node rate sleep
        node_rate_sleep = rospy.Rate(10) 

        # spins the node
        rospy.loginfo("Node in spin mode.")
        
        while not rospy.is_shutdown():

            if self.ns.getNodeStatus()['active']: # only runs if node is active

                # getting current ros time
                time_current = rospy.get_rostime()

                # computing delta time from last run
                if self.dt is None: # first run
                    self.dt = 0
                else:               # all other runs
                    self.dt = time_current - self.time_last
                self.time_last = time_current


                # using received message to compute flippers state
                if self.flpMsg == None: # it means no message has been received
                    ret = [-1]*4
                else: # when at least one flippers data message has been received

                    # this block discovers if received message sequence has changed, if not, it computes time duration from lat received valid sequence
                    if self.lastMsgSeq is None:    # in case of first received valid message
                        self.lastMsgSeq = self.flpMsg.header.seq
                        self.lastMsgTime = self.flpMsg.header.stamp
                        self.dt_MsgAccum = rospy.Duration.from_sec(0)
                    else:   # in case at least one valid message has already been received
                        if self.flpMsg.header.seq <= self.lastMsgSeq:  # in case of current message has same sequence than ancient one
                            self.dt_MsgAccum += self.dt
                        else:
                            self.lastMsgSeq = self.flpMsg.header.seq
                            self.lastMsgTime = self.flpMsg.header.stamp
                            self.dt_MsgAccum = rospy.Duration.from_sec(0)

                    # tests if last message is within a time window
                    if self.dt_MsgAccum > self.timeWindowToDiscard:
                        ret = [-1]*4
                    else:   # condition when there is a received valid value within time window

                        # flippers joint position
                        flpJnt_pos_l = correctFlippersJointSignal(self.flpMsg.position[4:])


                        #self.flpJnt_inferenceRange

                        # current torque reading
                        torque_sigFixed = correctFlippersJointSignal(list(self.flpMsg.effort[4:])) # only last 4 torque values are flipper ones

                        # adding current torque values to the torques log
                        for torque_val, torque_log in zip(torque_sigFixed, self.torque_log_l):
                            torque_log.append(torque_val)


                        # only executes when the proper number of torque values have been received
                        if len(self.torque_log_l[0]) >= self.torqueAvgQtd:

                            ret = []
                            for jTorqueLog, jPos  in zip(self.torque_log_l, flpJnt_pos_l):

                                pass

                                if self.flpJnt_inferenceRange['min'] < jPos and jPos < self.flpJnt_inferenceRange['max']:

                                    # computes the average of n last torque values
                                    torque_avg = sum(list(jTorqueLog))/self.torqueAvgQtd 

                                     # estimates the contact based on the torque average sign
                                    ret.append(0) if torque_avg < self.torqueThreshold else ret.append(1)
                                
                                else:   # in case of flippers outside the region to estimate the touch
                                    ret.append(0)                           

                        else: # in case of not enough values have been received
                            ret = 4*[-1] # sends a invalid value


                # publishing the message
                m = Int8ArrayStamped()
                m.header.stamp = time_current 
                m.header.frame_id = 'flippers_ground_touch_state'
                m.data = ret
                self.pub_touchStatus.publish(m)
                #print(m)

            # sleeps the node
            node_rate_sleep.sleep()


    def cllbck_jointState(self, msg):
        '''Callback method for jointState topic'''
        self.flpMsg = msg


    ''' === Service Callbacks === '''
    def srvcllbck_setActive(self, req):
        ''' Method for setting the active node status flag'''
        return self.ns.defActiveServiceReq(req, rospy)

    def srvcllbck_getStatus(self, req):
        ''' Method for returning the node status flag list'''
        return self.ns.getNodeStatusSrvResponse()


if __name__ == '__main__':
    node_name = 'flippers_ground_touch_state'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass



    