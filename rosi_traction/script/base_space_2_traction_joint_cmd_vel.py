#!/usr/bin/env python3
''' this is a ROSI traction algorithm 
It receives the corrective vector for traction frames and converts it to 
traction joints commands.
'''
import rospy

import numpy as np

from rosi_common.msg import Float32Array, Vector3ArrayStamped, Int8ArrayStamped, TwistStamped
from geometry_msgs.msg import Vector3Stamped

from rosi_common.node_status_tools import nodeStatus
from rosi_common.srv import SetNodeStatus, GetNodeStatusList

from rosi_common.rosi_tools import compute_J_traction, tractionJointSpeedGivenLinVel, correctTractionJointSignal, compute_J_mnv_dagger

from rosi_model.rosi_description import coefs_baseLinVel_wrt_trJointSpeed_tracks, coefs_baseLinVel_wrt_trJointSpeed_wheels, rotm_base_piFlp, tr_base_piFlp

class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        ##=== Parameters

        # dof axis of traction system
        self.tract_dof_axis = np.array([1, 0, 0]).reshape(3,1) # we consider that traction can generate velocity only along x_axis.

        ##=== Useful variables
        # node status object
        self.ns = nodeStatus(node_name)

        self.msg_baseCmdVel = None
        self.msg_flpTouchStatus = None
        self.msg_n_cp = None
        self.msg_cntctPnt = None
        
        self.x_pi = np.array([1, 0, 0])

        # traction systems effective radius
        self.track_radius = coefs_baseLinVel_wrt_trJointSpeed_tracks['a']
        self.wheel_radius = coefs_baseLinVel_wrt_trJointSpeed_wheels['a']


        ##=== ROS Interfaces

        #publisher
        self.pub_jointCmdVel = rospy.Publisher('/rosi/traction/joint/cmd_vel/navigation', Float32Array, queue_size=5)

        # subscriber
        #sub_baseSpaceCmdVel = rospy.Subscriber('/rosi/propulsion/space/cmd_vel', Vector3ArrayStamped, self.cllbck_spaceCmdVel)
        sub_cntctPnt = rospy.Subscriber('/rosi/model/contact_point_wrt_base', Vector3ArrayStamped, self.cllbck_cntctPnt)
        sub_baseCmdVel = rospy.Subscriber('/rosi/base/space/cmd_vel', TwistStamped, self.cllbck_baseCmdVel)
        sub_flpTouchState = rospy.Subscriber('/rosi/flippers/status/touching_ground', Int8ArrayStamped, self.cllbck_flpTouchState)
        sub_cntctPlaneNVec = rospy.Subscriber('/rosi/model/contact_plane_normal_vec', Vector3Stamped, self.cllbck_cntctPlaneNVec)

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
                if self.msg_baseCmdVel is not None and self.msg_flpTouchStatus is not None and self.msg_n_cp is not None and self.msg_cntctPnt is not None:

                    # extracts contact points
                    p_R_cp_l = [np.array([v.x, v.y, v.z]).reshape(3,1) for v in self.msg_cntctPnt.vec]

                    # creates the input velocity command vector
                    vl_R_input = np.array([self.msg_baseCmdVel.twist.linear.x, self.msg_baseCmdVel.twist.angular.z]).reshape(2,1)

                    # computes the maneuvering jacobian
                    J_mnvx  = compute_J_mnv_dagger(p_R_cp_l)

                    # computing contact point frame velocity for attaining input baseCmdVel
                    vl_Pi_l = np.dot(J_mnvx, vl_R_input)

                    # mounting the contact point frames velocity vector 
                    vl_cp_l = [ np.array([vl_Pi_l[2*i][0], vl_Pi_l[2*i+1][0], 0]).reshape(3,1) for i in range(int(len(vl_Pi_l)/2)) ]

                    # preparing the contact plane vector
                    n_cp = np.array([self.msg_n_cp.vector.x, self.msg_n_cp.vector.y, self.msg_n_cp.vector.z]).reshape(3,1)


                    # iterates for all four locomotion mechanisms
                    jCmdVel_l = []
                    for tchStatus, v_Pi in zip(self.msg_flpTouchStatus, vl_cp_l ):

                        if tchStatus == 0: # flipper not touching the ground
                            # computes the wheel jacobian
                            J_w = compute_J_traction(self.wheel_radius, n_cp)
                        else:
                            # computes tracks jacobian
                            J_w = compute_J_traction(self.track_radius, n_cp)

                        # computing the contact point (Pi) velocity vector norm
                        v_Pi_norm = np.linalg.norm(v_Pi)

                        if v_Pi_norm != 0: # only executes if a valid command has been received

                            # angle between the dof axis and the input velocity vector
                            cos_theta = (np.dot(self.tract_dof_axis.T, v_Pi))  / v_Pi_norm 

                            # computing required velocity along x_axis to match desired v_Pi
                            # this computation is based on the idea to compensate along x the velocity that is not performed
                            # by the traction mechanism along y_axis (since v_Pi has components along x and y).
                            if cos_theta != 0:
                                v_x_Pi = (v_Pi_norm / cos_theta)[0][0]
                            else:
                                v_x_Pi = 0.0  

                            # prepares the velocity input for the traction joint
                            #v_Pi_v = np.array([v_Pi.x, v_Pi.y, v_Pi.z]).reshape(3,1)
                            v_Pi_v = np.array([v_x_Pi, 0, 0]).reshape(3,1)
                        
                        else:
                            v_Pi_v = np.zeros(3).reshape(3,1)

                        # computing the joint velocity
                        jCmdVel_l.append(np.dot(np.linalg.pinv(J_w), v_Pi_v)[0][0])

                    # publishing the message
                    m = Float32Array()
                    m.header.stamp = rospy.get_rostime()
                    m.header.frame_id = self.node_name
                    m.data = correctTractionJointSignal(jCmdVel_l)
                    self.pub_jointCmdVel.publish(m)

                
    def cllbck_flpTouchState(self, msg):
        '''Callback method for jointState topic'''
        self.msg_flpTouchStatus = msg.data          


    def cllbck_cntctPnt(self, msg):
        '''Callback message for received contact points'''
        self.msg_cntctPnt = msg


    """def cllbck_spaceCmdVel(self, msg):
        '''Callback for the traction propellant frame corrective vector'''
        # transforming input ROS msg into a python list of lists
        self.msg_v_Pi_V = msg"""
    

    def cllbck_baseCmdVel(self, msg):
        ''' Callback for the robot base {R} frame cmd vel input
        '''
        self.msg_baseCmdVel = msg

    def cllbck_cntctPlaneNVec(self, msg):
        '''Callback for the contact plane n vector'''
        self.msg_n_cp = msg


    ''' === Service Callbacks === '''
    def srvcllbck_setActive(self, req):
        ''' Method for setting the active node status flag'''
        return self.ns.defActiveServiceReq(req, rospy)


    def srvcllbck_getStatus(self, req):
        ''' Method for returning the node status flag list'''
        return self.ns.getNodeStatusSrvResponse()


if __name__ == '__main__':
    node_name = 'base_space_2_traction_joint_cmd_vel'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass