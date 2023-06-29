#!/usr/bin/env python3
''' thisis a ROSI flippers algorithm
It receives flippers joint torque and generate flippers commands based on it.
'''
import rospy

from rosi_common.msg import Float32Array, Int8ArrayStamped

from rosi_common.rosi_tools import *


from rosi_common.node_status_tools import nodeStatus
from rosi_common.srv import SetNodeStatus, GetNodeStatusList

class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        ##=== PARAMETERS
        # flippers rotational velocity while searching for the floor
        self.searchFlpVel = 0.05

        ##===seful variables
        # node status object
        self.ns = nodeStatus(node_name)

        self.flpTouchStatus = None

        ##=== ROS Interfaces
        # publishers
        self.pub_cmdVelJoint = rospy.Publisher('/rosi/flippers/joint/cmd_vel/touch_granter', Float32Array, queue_size=5)

        # subscribers
        sub_flpTouchState = rospy.Subscriber('/rosi/flippers/status/touching_ground', Int8ArrayStamped, self.cllbck_flpTouchState)

        # services
        srv_setActive = rospy.Service(self.ns.getSrvPath('active', rospy), SetNodeStatus, self.srvcllbck_setActive)
        srv_getStatus = rospy.Service(self.ns.getSrvPath('getNodeStatus', rospy), GetNodeStatusList, self.srvcllbck_getStatus)
        srv_setHaltCmd = rospy.Service(self.ns.getSrvPath('haltcmd', rospy), SetNodeStatus, self.srvcllbck_setHaltCmd)

        # calling node main method
        self.nodeMain()


    def nodeMain(self):
        ''' Node main script'''

        # node rate sleep
        node_rate_sleep = rospy.Rate(10)

        # spins the node
        rospy.loginfo("Node in spin mode.")

        # eternal loop
        while not rospy.is_shutdown():

            if self.ns.getNodeStatus()['active']: # only runs if node is active

                # only runs if valid states has been received by the touch status node
                if self.flpTouchStatus != None and self.flpTouchStatus[0] != -1:

                    # computing flippers command considering flippers' joint torque the read torque
                    cmd_flprs = [self.searchFlpVel if touchStatus==0 else 0.0 for touchStatus in self.flpTouchStatus]

                    # correcting joint signals for ROSI
                    cmd_flprs = correctFlippersJointSignal(cmd_flprs)

                    # receives rostime
                    ros_time = rospy.get_rostime()
                    
                    # mounting publishing message
                    mp = Float32Array()
                    mp.header.stamp = ros_time
                    mp.header.frame_id = 'flippers_touch_granter'
                    mp.data = cmd_flprs
            
                    # publishing message
                    self.pub_cmdVelJoint.publish(mp)

                    #print(mp)
            
            # sleeps the node
            node_rate_sleep.sleep()


    def cllbck_flpTouchState(self, msg):
        '''Callback method for jointState topic'''
        self.flpTouchStatus = list(msg.data)


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
    node_name = 'flippers_touch_granter'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass



    