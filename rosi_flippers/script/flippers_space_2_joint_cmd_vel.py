#!/usr/bin/env python3
''' this is a ROSI flippers algorithm 
It receives linear/angular velocity commands to flippers frame
and converts it to flippers joyt rotational velocities.
'''
import rospy

from std_msgs.msg import Float32MultiArray
from rosi_common.msg import Float32Array, Vector3ArrayStamped
from sensor_msgs.msg import JointState, Imu

import numpy as np
from dqrobotics import *
import quaternion

from rosi_model.rosi_description import rotm_qi_pi, propKeys, dq_qi_flpContact, dq_base_piFlp
from rosi_common.dq_tools import angleAxis2dqRot, dqExtractRotM, dqExtractTransV3
from rosi_common.rosi_tools import compute_J_flpLever, gravityVecProjectedInPlaneXZ, jointStateData2dict, flippersContactPoint, correctFlippersJointSignal
from rosi_common.dq_tools import trAndOri2dq

from rosi_common.node_status_tools import nodeStatus
from rosi_common.srv import SetNodeStatus, GetNodeStatusList

class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name


        ##== Useful variables
        # node status object
        self.ns = nodeStatus(node_name)

        self.v_z_P_l = None
        self.msg_q_Pi_cp = None

        # useful vectors
        self.x_pi = np.array([1, 0, 0])
        self.y_pi = np.array([0, 1, 0])
        self.z_pi = np.array([0, 0, 1])

        # unchanging calculations
        self.x_pi_dot_x_pi = np.dot(self.x_pi, self.x_pi)
        self.y_pi_dot_y_pi = np.dot(self.y_pi, self.y_pi)

        ##== One-time calculations

        # list of {Qi} frames rotation matrix wrt {Pi}
        self.rotm_qi_pi_l = [rotm_qi_pi, rotm_qi_pi, rotm_qi_pi, rotm_qi_pi]

        ##== ROS interfaces

        # publishers
        self.pub_cmdVelFlpJnt = rospy.Publisher('/rosi/flippers/joint/cmd_vel/leveler', Float32Array, queue_size=5)

        # subscribers
        sub_cmdVel_cmdVelVzPi = rospy.Subscriber('/rosi/flippers/space/cmd_v_z', Vector3ArrayStamped, self.cllbck_cmdVelVzPi)
        sub_contactPointPi = rospy.Subscriber('/rosi/model/contact_point_wrt_pi', Vector3ArrayStamped, self.cllbck_contactPointPi)

        # services
        srv_getStatus = rospy.Service(self.ns.getSrvPath('getNodeStatus', rospy), GetNodeStatusList, self.srvcllbck_getStatus)
        srv_setActive = rospy.Service(self.ns.getSrvPath('active', rospy), SetNodeStatus, self.srvcllbck_setActive)
        srv_setTelemetry = rospy.Service(self.ns.getSrvPath('telemetry', rospy), SetNodeStatus, self.srvcllbck_setTelemetry)

        ##== Node main loop
        self.nodeMain()


    def nodeMain(self):
        '''Node main script'''

        # node rate sleep
        node_rate_sleep = rospy.Rate(10)

        # spins the node
        rospy.loginfo("Node in spin mode.")

        # eternal loop
        while not rospy.is_shutdown():

            # only runs if node is active, or required to send telemetry
            if self.ns.getNodeStatus()['active'] or self.ns.getNodeStatus()['telemetry']:

                # only runs if valid messages have been received from topics
                if self.v_z_P_l is not None and self.msg_q_Pi_cp is not None:

                    # converting contact points to numpy format
                    p_Pi_cp_l = [np.array([p.x, p.y, p.z]) for p in self.msg_q_Pi_cp.vec]

                    # computes the flipper lever joint Jacobian for z axis
                    J_flpLever_z_l = [ compute_J_flpLever(rotm_qi_pi, p_Pi_cp, 'z') for rotm_qi_pi, p_Pi_cp in zip(self.rotm_qi_pi_l, p_Pi_cp_l)]

                    # mounting velocity command vector for z_P
                    v_P_z_l = [np.array([p.x, p.y, p.z]).reshape(3,1) for p in self.v_z_P_l.vec]

                    # flippers joint angular velocity computed using the flipper jacobian
                    dotq_flp_l = [np.dot(np.linalg.pinv(J_flpLever_z_i), v_P_z_i)[0][0] for J_flpLever_z_i, v_P_z_i in zip(J_flpLever_z_l, v_P_z_l) ]


                    # computing the sign joints commands
                    dotq_flp_sign_l = np.sign(dotq_flp_l)

                    # doubles the command velocity if flippers rotate in the same direction
                    # this is due ROSI flippers specific characteristics when reorienting the flipper couple of each side
                    if dotq_flp_sign_l[0] * dotq_flp_sign_l[2] > 0:
                        dotq_flp_l[0] = dotq_flp_l[0] * 2
                        dotq_flp_l[2] = dotq_flp_l[2] * 2

                    if dotq_flp_sign_l[1] * dotq_flp_sign_l[3] > 0:
                        dotq_flp_l[1] = dotq_flp_l[1] * 2
                        dotq_flp_l[3] = dotq_flp_l[3] * 2


                    # correct joints signal considering ROSI motors specific mounting
                    dotq_flp_l = [x*y for x,y in zip(dotq_flp_l, [1, -1, 1, -1])] 
                   
                    # mounting publishing message flipper joints cmd message
                    if self.ns.getNodeStatus()['active']: # only runs if node is active
                        # receives rostime
                        ros_time = rospy.get_rostime()

                        # publishing the message
                        mp = Float32Array()
                        mp.header.stamp = ros_time
                        mp.header.frame_id = 'flippers_space_2_joint_cmd_vel'
                        mp.data = dotq_flp_l
                        self.pub_cmdVelFlpJnt.publish(mp)
                        #print(mp)

            # sleeps the node
            node_rate_sleep.sleep()


    def cllbck_cmdVelVzPi(self, msg):
        '''Callback for the velocity input for each propulsion frame'''
        self.v_z_P_l = msg


    def cllbck_contactPointPi(self, msg):
        '''Callback for the contact point '''
        self.msg_q_Pi_cp = msg


    ''' === Service Callbacks === '''
    def srvcllbck_getStatus(self, req):
        ''' Method for returning the node status flag list'''
        return self.ns.getNodeStatusSrvResponse()

    def srvcllbck_setActive(self, req):
        ''' Method for setting the active node status flag'''
        return self.ns.defActiveServiceReq(req, rospy)

    def srvcllbck_setTelemetry(self, req):
        ''' Method for setting the telemetry node status flag'''
        return self.ns.defTelemetryServiceReq(req, rospy)
    
        
if __name__ == '__main__':
    node_name = 'flippers_space_2_joint_cmd_vel'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass

