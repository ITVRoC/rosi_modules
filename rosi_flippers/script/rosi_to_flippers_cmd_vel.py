#!/usr/bin/env python3
''' this is a ROSI flippers algorithm 
It receives linear/angular velocity commands to ROSI base frame and
generate corrective director vectors for flippers frame
'''

import rospy

import numpy as np

from rosi_common.msg import TwistStamped, Float32Array

from rosi_common.srv import SetNodeStatus, GetNodeStatusList

from rosi_model.rosi_description import tr_base_piFlp
from rosi_common.node_status_tools import nodeStatus
from rosi_common.rosi_tools import compute_J_art_dagger


class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        ##=== Useful variables
        # node status object
        self.ns = nodeStatus(node_name)

        # message received from ros topic
        self.msg_baseCmdVel = None

        # obtains the chassis kinematics
        self.J_art_dagger = compute_J_art_dagger([x for x in tr_base_piFlp.values() ])

        ##=== ROS interfaces
        # publishers 
        self.pub_cmdVelFlipperSpace = rospy.Publisher('/rosi/flippers/space/cmd_v_z/joy', Float32Array, queue_size=5)

        # subscriber
        sub_baseCmdVel = rospy.Subscriber('/rosi/base/space/cmd_vel', TwistStamped, self.cllbck_baseCmdVel) # base frame cmd vel input

        # services
        srv_setActive = rospy.Service(self.ns.getSrvPath('active', rospy), SetNodeStatus, self.srvcllbck_setActive)
        srv_getStatus = rospy.Service(self.ns.getSrvPath('getNodeStatus', rospy), GetNodeStatusList, self.srvcllbck_getStatus)
        srv_setHaltCmd = rospy.Service(self.ns.getSrvPath('haltcmd', rospy), SetNodeStatus, self.srvcllbck_setHaltCmd)

        # Node main
        self.nodeMain()


    def nodeMain(self):
        '''Node main method'''

        # defining the eternal loop rate
        node_rate_sleep = rospy.Rate(10)

        rospy.loginfo('Entering in ethernal loop.')
        while not rospy.is_shutdown():
            pass

            if self.ns.getNodeStatus()['active']: # only runs if node is active
                
                if self.msg_baseCmdVel is not None: # only runs if valid messages have been received

                    # mounting the input articulation velocity vector for the chassis in the chassis frame
                    # v_art_R_in = [v_z omega_x omega_y]
                    v_art_R_in = np.array([ 
                                       self.msg_baseCmdVel.twist.linear.z, 
                                       self.msg_baseCmdVel.twist.angular.x, 
                                       self.msg_baseCmdVel.twist.angular.y]
                                      ).reshape(3,1)


                    # computing the v_z_Pi velocity for each flipper frame given the input chassis input velocity vector
                    v_z_Pi_out = self.J_art_dagger.dot(v_art_R_in)

                    # publishing message
                    m = Float32Array()
                    m.header.stamp = rospy.get_rostime()
                    m.header.frame_id = self.node_name
                    m.data = [x[0] for x in v_z_Pi_out]
                    self.pub_cmdVelFlipperSpace.publish(m)
            

    def cllbck_baseCmdVel(self, msg):
        ''' Callback for the robot base {R} frame cmd vel input
        '''
        self.msg_baseCmdVel = msg
       

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
    node_name = 'rosi_to_flippers_cmd_vel'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass


    