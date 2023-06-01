#!/usr/bin/env python3
''' this is a ROSI flippers algorithm 
It receives linear/angular velocity commands to flippers frame
and converts it to flippers joyt rotational velocities.
'''
import rospy

from std_msgs.msg import Float32MultiArray
from rosi_common.msg import Float32Array
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

        ##== Parameters
        self.v_x_flprs = np.array([1, 0, 0])    # flippers vertical axis
        self.p_ctrl_kp = 0.2


        ##== Useful variables
        # node status object
        self.ns = nodeStatus(node_name)

        self.v_corrDirVec = [[0,0,0],[0,0,0],[0,0,0],[0,0,0]] # initially, there is no correction to make, until some signal is received
        self.flpJointState = None
        self.d_imu = None
        self.v_z_P_l = None

        # useful vectors
        self.x_pi = np.array([1, 0, 0])
        self.y_pi = np.array([0, 1, 0])
        self.z_pi = np.array([0, 0, 1])

        # unchanging calculations
        self.x_pi_dot_x_pi = np.dot(self.x_pi, self.x_pi)
        self.y_pi_dot_y_pi = np.dot(self.y_pi, self.y_pi)

        ##== One-time calculations

        # list of {Qi} frames rotation matrix wrt {Pi}
        self.rotm_qi_pi_l = 4*[rotm_qi_pi]

        ##== ROS interfaces

        # publishers
        #self.pub_cmdVelFlipperSpace = rospy.Publisher('/rosi/flippers/space/cmd_v_z', Float32MultiArray, queue_size=5)
        self.pub_cmdVelFlpJnt = rospy.Publisher('/rosi/flippers/joint/cmd_vel/leveler', Float32Array)

        # subscribers
        sub_cmdVel_cmdVelVzPi = rospy.Subscriber('/rosi/flippers/space/cmd_v_z', Float32Array, self.cllbck_cmdVelVzPi)

        #sub_cmdVelFlpSpace = rospy.Subscriber('/rosi/flippers/space/cmd_vel', Vector3ArrayStamped, self.cllbck_cmdVelFlpSpace)
        sub_jointState = rospy.Subscriber('/rosi/rosi_controller/joint_state', JointState, self.cllbck_jointState)
        sub_imu = rospy.Subscriber('/mti/sensor/imu', Imu, self.cllbck_imu)

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
                if self.flpJointState is not None and self.d_imu  is not None and self.v_z_P_l is not None:

                    # obtains the gravity vector projected onto XZ
                    gxz = gravityVecProjectedInPlaneXZ(self.d_imu)

                    # obtains flippers lever joint angular position
                    _,f_j_l = jointStateData2dict(self.flpJointState)
                    # TODO Verificar se o sinal de posicao dos flippers esta OK

                    # obtains flippers contact points
                    c_f_l_Pi_l = flippersContactPoint(f_j_l['pos'], gxz)

                    # computes the flipper lever joint Jacobian for z axis
                    J_flpLever_z_l = [ compute_J_flpLever(rotm_qi_pi, c_f_l_Pi, 'z') for rotm_qi_pi,c_f_l_Pi in zip(self.rotm_qi_pi_l, c_f_l_Pi_l)]

                    # mounting velocity command vector for z_P
                    v_P_z_l = [np.array([0, 0, vi]).reshape(3,1) for vi in self.v_z_P_l ]

                    # flippers joint angular velocity computed using the flipper jacobian
                    dotq_fl_l = correctFlippersJointSignal([np.dot(np.linalg.pinv(J_flpLever_z_i), v_P_z_i)[0][0] for J_flpLever_z_i, v_P_z_i in zip(J_flpLever_z_l, v_P_z_l) ] )

                    # mounting publishing message flipper joints cmd message
                    if self.ns.getNodeStatus()['active']: # only runs if node is active

                        # receives rostime
                        ros_time = rospy.get_rostime()

                        # publishing the message
                        mp = Float32Array()
                        mp.header.stamp = ros_time
                        mp.header.frame_id = 'flippers_space_2_joint_cmd_vel'
                        mp.data = dotq_fl_l
                        self.pub_cmdVelFlpJnt.publish(mp)

            # sleeps the node
            node_rate_sleep.sleep()


    def cllbck_cmdVelVzPi(self, msg):
        self.v_z_P_l = msg.data


    def cllbck_jointState(self, msg):
        ''' Callback for flippers state'''
        self.flpJointState = msg


    def cllbck_imu(self, msg):
        ''' Callback message for imu message'''
        self.d_imu = msg

        #self.dq_w_R = trAndOri2dq([0,0,0], [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z], 'trfirst')


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

