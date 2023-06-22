#!/usr/bin/env python3
''' this is a ROSI traction algorithm 
It receives linear/angular velocity commands to ROSI base frame and
generate corrective director vectors for traction frames
'''
import rospy

import numpy as np

from geometry_msgs.msg import Vector3
from rosi_common.msg import Vector3Array, TwistStamped

from rosi_model.rosi_description import tr_base_piTra, rotm_base_piTra

from rosi_common.node_status_tools import nodeStatus
from rosi_common.srv import SetNodeStatus, GetNodeStatusList

class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        ##=== Useful variables
        # node status object
        self.ns = nodeStatus(node_name)
        self.ns.resetActive() # this node is disabled by default


        ##=== ROS interfaces
        # publishers 
        self.pub_cmdVelTractionSpace = rospy.Publisher('/rosi/traction/space/cmd_vel', Vector3Array, queue_size=5)

        # subscriber
        sub_baseCmdVel = rospy.Subscriber('/rosi/base/space/cmd_vel', TwistStamped, self.cllbck_baseCmdVel) # base frame cmd vel input

        # services
        srv_setActive = rospy.Service(self.ns.getSrvPath('active', rospy), SetNodeStatus, self.srvcllbck_setActive)
        srv_getStatus = rospy.Service(self.ns.getSrvPath('getNodeStatus', rospy), GetNodeStatusList, self.srvcllbck_getStatus)
        srv_setHaltCmd = rospy.Service(self.ns.getSrvPath('haltcmd', rospy), SetNodeStatus, self.srvcllbck_setHaltCmd)

        # infinity
        rospy.loginfo('entering in eternal loop.')
        rospy.spin()


    def cllbck_baseCmdVel(self, msg):
        ''' Callback for the robot base {R} frame cmd vel input
        '''

        if self.ns.getNodeStatus()['active']: # only runs if node is active

            # extracting commands from received msg
            # traction only acts upon linear x and angular z velocities
            v_lin_R = np.array([[msg.twist.linear.x], [0.0], [0.0]])
            v_ang_R = np.array([[0.0], [0.0], [msg.twist.angular.z]])

            # computing corrective director vector for each propellant {Pi} frame
            # !!! por enquanto consideramos a pose do frame {Pi} da tracao identico ao dos flippers 
            v_corrDirVec = [rotm.T.dot(v_lin_R + np.cross(v_ang_R.T, tr.T).T) for tr, rotm in zip(tr_base_piTra.values(), rotm_base_piTra.values())]

            # publishing message
            mp = Vector3Array([Vector3(vec[0][0], vec[1][0], vec[2][0]) for vec in v_corrDirVec])
            self.pub_cmdVelTractionSpace.publish(mp)

            #print(mp)

    
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
    node_name = 'rosi_to_traction_cmd_vel'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass


    