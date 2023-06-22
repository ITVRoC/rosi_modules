#!/usr/bin/env python3
''' this is a ROSI traction algorithm 
It receives the corrective vector for traction frames and converts it to 
traction joints commands.
'''
import rospy

import numpy as np

from rosi_common.msg import Float32Array, Vector3ArrayStamped, Int8ArrayStamped

from rosi_common.node_status_tools import nodeStatus
from rosi_common.srv import SetNodeStatus, GetNodeStatusList

from rosi_common.rosi_tools import compute_J_traction, tractionJointSpeedGivenLinVel, correctTractionJointSignal

from rosi_model.rosi_description import coefs_baseLinVel_wrt_trJointSpeed_tracks, coefs_baseLinVel_wrt_trJointSpeed_wheels, rotm_base_piFlp, tr_base_piFlp

class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        ##=== Useful variables
        # node status object
        self.ns = nodeStatus(node_name)

        self.msg_v_Pi_V = None 
        self.msg_flpTouchStatus = None
        
        self.x_pi = np.array([1, 0, 0])

        # traction systems effective radius
        self.track_radius = coefs_baseLinVel_wrt_trJointSpeed_tracks['a']
        self.wheel_radius = coefs_baseLinVel_wrt_trJointSpeed_wheels['a']


        ##=== ROS Interfaces

        #publisher
        self.pub_jointCmdVel = rospy.Publisher('/rosi/traction/joint/cmd_vel/navigation', Float32Array, queue_size=5)

        # subscriber
        sub_baseSpaceCmdVel = rospy.Subscriber('/rosi/propulsion/space/cmd_vel', Vector3ArrayStamped, self.cllbck_spaceCmdVel)
        sub_flpTouchState = rospy.Subscriber('/rosi/flippers/status/touching_ground', Int8ArrayStamped, self.cllbck_flpTouchState)

        # services
        srv_setActive = rospy.Service(self.ns.getSrvPath('active', rospy), SetNodeStatus, self.srvcllbck_setActive)
        srv_getStatus = rospy.Service(self.ns.getSrvPath('getNodeStatus', rospy), GetNodeStatusList, self.srvcllbck_getStatus)

        ##=== Node main method
        self.nodeMain()

    
    def nodeMain(self):
        '''Node main method'''

        # node rate sleep
        node_rate_sleep = rospy.Rate(10)

        # spins the node
        rospy.loginfo("Node in spin mode.")

        # eternal loop
        while not rospy.is_shutdown():

            # only runs if node is active, or required to send telemetry
            if self.ns.getNodeStatus()['active']:
                
                # only runs if a valid flippers touch status message has been received
                if self.msg_v_Pi_V is not None and self.msg_flpTouchStatus is not None:
                    pass

                    for tchStatus in self.msg_flpTouchStatus:

                        if tchStatus == 0: # flipper not touching the ground
                            # updates the wheel jacobian
                            J_w = compute_J_traction(self.wheel_radius, )

                    
                    #cmdVelJ = [tractionJointSpeedGivenLinVel(vel,'wheel') if val==0 else tractionJointSpeedGivenLinVel(vel,'flipper') for vel,val in zip(v_x, self.msg_flpTouchStatus)]

                    
                    

                    # publishing the message
                    m = Float32Array()
                    m.header.stamp = rospy.get_rostime()
                    m.header.frame_id = self.node_name
                    m.data = correctTractionJointSignal(cmdVelJ)
                    self.pub_jointCmdVel.publish(m)

                    """
                    # computing velocity vectors norm
                    vel_norm_l = [np.linalg.norm(vel) for vel in self.v_corrDirVec]

                    # angle cosine between v_pi and x_pi
                    cosTheta = [np.dot(np.array(vel), self.x_pi)/ vel_norm if vel_norm != 0 else 0 for vel, vel_norm in zip(self.v_corrDirVec, vel_norm_l)]
                    
                    # computing needed velocity in v_x projection to attend desired v
                    v_x = [np.linalg.norm(vel)/cost if cost != 0 else 0 for vel,cost in zip(self.v_corrDirVec, cosTheta)]

                    # computing appliable velocity to joints
                    cmdVelJ = [tractionJointSpeedGivenLinVel(vel,'wheel') if val==0 else tractionJointSpeedGivenLinVel(vel,'flipper') for vel,val in zip(v_x, self.msg_flpTouchStatus)]
                
                    # publishing the message
                    m = Float32Array()
                    m.header.stamp = rospy.get_rostime()
                    m.header.frame_id = self.node_name
                    m.data = correctTractionJointSignal(cmdVelJ)
                    self.pub_jointCmdVel.publish(m)

                    #print(m)"""
                
                
    def cllbck_flpTouchState(self, msg):
        '''Callback method for jointState topic'''
        self.msg_flpTouchStatus = msg.data          


    def cllbck_spaceCmdVel(self, msg):
        '''Callback for the traction propellant frame corrective vector'''
        # transforming input ROS msg into a python list of lists
        self.msg_v_Pi_V = msg


    ''' === Service Callbacks === '''
    def srvcllbck_setActive(self, req):
        ''' Method for setting the active node status flag'''
        return self.ns.defActiveServiceReq(req, rospy)


    def srvcllbck_getStatus(self, req):
        ''' Method for returning the node status flag list'''
        return self.ns.getNodeStatusSrvResponse()


if __name__ == '__main__':
    node_name = 'traction_space_2_joint_cmd_vel'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass