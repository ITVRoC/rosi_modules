#!/usr/bin/env python3
''' This is a ROSI package that converts flippers frame y velocity computed by forward
direction kinematics to inverse traction commands in order to cancel it. 
'''
#from platform import java_ver
import rospy

from rosi_common.msg import BoolArrayStamped, Float32Array, Int8ArrayStamped, Vector3ArrayStamped
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3Stamped

from rosi_model.rosi_description import propKeys, rotm_qi_pi, coefs_baseLinVel_wrt_trJointSpeed_tracks
from rosi_common.rosi_tools import compute_J_flpLever, compute_J_traction, jointStateData2dict, correctTractionJointSignal
from rosi_common.dq_tools import angleAxis2dqRot, dqExtractTransV3

from rosi_common.node_status_tools import nodeStatus
from rosi_common.srv import SetNodeStatus, GetNodeStatusList

import numpy as np

class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        ##=== Parameters
        # joint vel deadband
        self.jVel_deadband = 0.01

        # flipper joint limits for enabling the compensator
        self.flpJntLim_min = np.radians([70, 70, 70, 70])
        self.flpJntLim_max = np.radians([170, 170, 170, 170])


        ##=== Useful variables
        # node status object
        self.ns = nodeStatus(node_name)

        # for storing received linear velocities in y axis of {Pi}
        self.flp_frame_lin_vel_y_l = {propKeys[0]: 0.0, propKeys[1]: 0.0, propKeys[2]: 0.0, propKeys[3]: 0.0,}

        self.msg_q_Pi_cp = None
        self.msg_n_cp = None

        self.flag_maxPosReached = None
        self.flpJointState = None
        self.flpTouchStatus = None

        ##=== One-time calculations

        # tracks effective radius
        self.track_radius = coefs_baseLinVel_wrt_trJointSpeed_tracks['a']

        ##=== ROS interfaces
        self.pub_CtrlInputReq = rospy.Publisher('/rosi/traction/joint/cmd_vel/compensator', Float32Array, queue_size=5)

        # subscribers
        sub_contactPointPi = rospy.Subscriber('/rosi/model/contact_point_wrt_pi', Vector3ArrayStamped, self.cllbck_contactPointPi)
        sub_cntctPlaneNVec = rospy.Subscriber('/rosi/model/contact_plane_normal_vec', Vector3Stamped, self.cllbck_cntctPlaneNVec)
        sub_safetyLock_maxPos = rospy.Subscriber('/rosi/flippers/status/safety/max_pos_lock', BoolArrayStamped, self.cllbck_safetyLock_maxPos)
        sub_jointState = rospy.Subscriber('/rosi/rosi_controller/joint_state', JointState, self.cllbck_jointState)
        sub_flpTouchState = rospy.Subscriber('/rosi/flippers/status/touching_ground', Int8ArrayStamped, self.cllbck_flpTouchState)


        # services
        srv_setActive = rospy.Service(self.ns.getSrvPath('active', rospy), SetNodeStatus, self.srvcllbck_setActive)
        srv_getStatus = rospy.Service(self.ns.getSrvPath('getNodeStatus', rospy), GetNodeStatusList, self.srvcllbck_getStatus)

        ##=== Node main method
        self.nodeMain()


    def nodeMain(self):
        '''Node main method'''

        # defining the eternal loop rate
        node_sleep_rate = rospy.Rate(20)

        # eternal loop
        while not rospy.is_shutdown():

            if self.ns.getNodeStatus()['active']: # only runs if node is active
            
                # only execture commands if valid values have been received from ROS topics
                #if self.msg_q_Pi_cp is not None and self.msg_n_cp  is not None and self.flpJointState is not None and self.flpTouchStatus is not None:
                if self.msg_q_Pi_cp is not None and \
                    self.msg_n_cp  is not None and \
                    self.flpJointState is not None and \
                    self.flpTouchStatus is not None and \
                    self.flag_maxPosReached is not None:

                    # extracts ROSI joint state date
                    _,flp_jointState = jointStateData2dict(self.flpJointState)

                    # correcting the velocity signal considering ROSI construction characteristics
                    flp_jVel_l = [x*y for x,y in zip(flp_jointState['vel'], [1,1,-1,-1])]

                    # converts the contact plane normal vector to the numpy format
                    n_cp = np.array([self.msg_n_cp.vector.x, self.msg_n_cp.vector.y, self.msg_n_cp.vector.z]).reshape(3,1)

                    # converts the individual flipper contact point to the numpy array format
                    p_Pi_cp_l = [np.array([p.x, p.y, p.z]) for p in  self.msg_q_Pi_cp.vec]

                    # computes the flipper lever joint Jacobian for x axis
                    J_flpLever_x_l = [compute_J_flpLever(rotm_qi_pi, p_Pi_cp, 'x')  for p_Pi_cp in p_Pi_cp_l]

                    # computes the advance velocity considering the flipper lever joint velocity given by its motor controller
                    v_Pi_x_l = [np.dot(J_flpLever_x, jVel)[0][0] for J_flpLever_x, jVel in zip(J_flpLever_x_l, flp_jVel_l) ]

                    # net velocity between flippers contact point from each robot side
                    # but it only generates velocities if flippers are touching the ground (in theory! thats why i added an angle span restriction)
                    v_cp_wrt_pi_net_l = [(v_Pi_x_l[0]-v_Pi_x_l[2])/2 * self.flpTouchStatus[0],
                                        (v_Pi_x_l[1]-v_Pi_x_l[3])/2 * self.flpTouchStatus[1],
                                        (v_Pi_x_l[2]-v_Pi_x_l[0])/2 * self.flpTouchStatus[2],
                                        (v_Pi_x_l[3]-v_Pi_x_l[1])/2 * self.flpTouchStatus[3]]

                    # computes the traction jacobian
                    J_traction = compute_J_traction(self.track_radius, n_cp)

                    # traction joint speed given flipper linear velocity on the floor
                    jVelCmd_l = []
                    for jVel, jPos, lim_min, lim_max in zip(v_cp_wrt_pi_net_l, flp_jointState['pos'], self.flpJntLim_min, self.flpJntLim_max):
                        if lim_min <= jPos and jPos <= lim_max:

                            # mounts the input velocity vector
                            v_Pi_cmd = np.array([jVel, 0, 0]).reshape(3,1)

                            # computes the joint velocity
                            jVelCmd_l.append( -1 * np.dot(np.linalg.pinv(J_traction), v_Pi_cmd)[0][0]  ) # we 

                        else:
                            jVelCmd_l.append(0.0)
                            

                    # mounting message to publish
                    m = Float32Array()
                    m.header.stamp = rospy.get_rostime()
                    m.header.frame_id = self.node_name
                    m.data = correctTractionJointSignal(jVelCmd_l)
                    self.pub_CtrlInputReq.publish(m)

            # sleeps the node
            node_sleep_rate.sleep()


    def cllbck_contactPointPi(self, msg):
        '''Callback for the ROSI contact points '''
        self.msg_q_Pi_cp = msg


    def cllbck_cntctPlaneNVec(self, msg):
        '''Callback for the contact plane n vector'''
        self.msg_n_cp = msg


    def cllbck_safetyLock_maxPos(self, msg):
        '''Callback for receiving flippers safety lock due to max position reached'''
        self.flag_maxPosReached = msg.data

    
    def cllbck_jointState(self, msg):
        '''Callback for the joints state  signal'''
        self.flpJointState = msg


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


if __name__ == '__main__':
    node_name = 'traction_flipper_risal_compensator'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass



    