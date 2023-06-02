#!/usr/bin/env python3
''' This is a ROSI flippers algorithm
It receives flippers joint torque generates a boolean array suggesting which flipper is touching the ground
It considers that flippers are in the extended semi-circle (pointing outwards considering the robot x axis)
'''
import rospy

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

                print(self.flpMsg)

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
                        # computes the values to return 
                        torque_sigFixed = correctFlippersJointSignal(list(self.flpMsg.effort[4:])) # only last 4 torque values are flipper ones
                        ret = [0 if torque < self.torqueThreshold else 1 for torque in torque_sigFixed]

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



    