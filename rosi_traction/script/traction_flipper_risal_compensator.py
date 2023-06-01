#!/usr/bin/env python3
''' This is a ROSI package that converts flippers frame y velocity computed by forward
direction kinematics to inverse traction commands in order to cancel it. 
'''
from platform import java_ver
import rospy

from rosi_common.msg import BoolArrayStamped, Float32Array, Int8ArrayStamped
from sensor_msgs.msg import JointState

from rosi_model.rosi_description import propKeys, dq_qi_flpContact
from rosi_common.rosi_tools import tractionJointSpeedGivenLinVel, jointStateData2dict, correctFlippersJointSignal
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
        self.flpJntLim_min = np.radians([70, -70, -70, 70])
        self.flpJntLim_max = np.radians([180, -180, -180, 180])


        ##=== Useful variables
        # node status object
        self.ns = nodeStatus(node_name)

        # for storing received linear velocities in y axis of {Pi}
        self.flp_frame_lin_vel_y_l = {propKeys[0]: 0.0, propKeys[1]: 0.0, propKeys[2]: 0.0, propKeys[3]: 0.0,}

        self.flag_maxPosReached = None
        self.flpJointState = None
        self.flpTouchStatus = None

        ##=== ROS interfaces
        self.pub_CtrlInputReq = rospy.Publisher('/rosi/traction/joint/cmd_vel/compensator', Float32Array, queue_size=5)

        # subscribers
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

            if self.ns.getNodeStatus()['active'] and self.flpJointState is not None and self.flpTouchStatus is not None: # only runs if node is active

                ##=== Working with related frames
                _,joint_state = jointStateData2dict(self.flpJointState)
                flp_pos = correctFlippersJointSignal(joint_state['pos'])    # this time, wee need flippers joint position accordingly to real joints (i.e., no tweak for positive angles for flippers extended)

                # dual-quaternion of frame Qi w.r.t. Pi (its a rotation around z of the flipper joint value)
                dq_pi_qi_l = [angleAxis2dqRot(jointPos, [0,0,1]) for jointPos in flp_pos] # rotation between Pi and Qi is only about z axis

                # retrieving flipper touch contact w.r.t. Qi frame
                dq_qi_cp_l = [dq_qi_flpContact[key] for key in propKeys]

                # computing contact point w.r.t. {Pi} frame
                dq_pi_cp_l = [dq_pi_qi * dq_qi_cp for dq_pi_qi, dq_qi_cp in zip(dq_pi_qi_l, dq_qi_cp_l)]

                # extracting the translation vector of every contact point ci
                tr_pi_cp_l = [dqExtractTransV3(dq.normalize()) for dq in dq_pi_cp_l]

                # computing the linear velocity at the contact point
                w_pi_l = [j_vel * np.array([0,0,1]).reshape(3,1) for j_vel in correctFlippersJointSignal(joint_state['vel'])]
                v_cp_wrt_pi_l = [np.cross(w_pi.T, tr_pi_cp.T) for w_pi, tr_pi_cp in zip(w_pi_l, tr_pi_cp_l)]

                # net velocity between flippers contact point from each robot side
                # but it only generates velocities if flippers are touching the ground (in theory! thats why i added an angle span restriction)
                avl = v_cp_wrt_pi_l
                v_cp_wrt_pi_net_l = [(avl[0]-avl[2])/2 * self.flpTouchStatus[0],
                                     (avl[1]-avl[3])/2 * self.flpTouchStatus[1],
                                     (avl[2]-avl[0])/2 * self.flpTouchStatus[2],
                                     (avl[3]-avl[1])/2 * self.flpTouchStatus[3]]


                # traction joint speed given flipper linear velocity on the floor
                jVel_l = []
                for v, jPos, lim_min, lim_max in zip(v_cp_wrt_pi_net_l, flp_pos, self.flpJntLim_min, self.flpJntLim_max):
                    if lim_min <= jPos and jPos <= lim_max:
                        jVel_l.append(tractionJointSpeedGivenLinVel(v[0][1], 'flipper'))
                    else:
                        jVel_l.append(0.0)
                        
                #jVel_l = [tractionJointSpeedGivenLinVel(v[0][1], 'flipper') for v in v_cp_wrt_pi_net_l]

                # applying flippers safety lock due to maximum flipper joint position reached
                if self.flag_maxPosReached is not None:
                    jVel_l = [0.0 if flag is True else vel for vel, flag in zip(jVel_l, self.flag_maxPosReached)]

                # mounting message to publish
                m = Float32Array()
                m.header.stamp = rospy.get_rostime()
                m.header.frame_id = 'traction_flipper_risal_compensator'
                #m.data = correctTractionJointSignal(jVel_l)
                m.data = jVel_l
                self.pub_CtrlInputReq.publish(m)

                #print(m)

            # sleeping the node
            node_sleep_rate.sleep()


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



    