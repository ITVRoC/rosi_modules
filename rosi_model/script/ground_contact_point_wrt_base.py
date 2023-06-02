#!/usr/bin/env python3
''' This is a ROSI model algorithm
It receives flippers joint position, flippers ground contact condition,
rosi model parameters, and computes ROSI ground contacts.
'''
import rospy

from rosi_common.msg import Int8ArrayStamped, Vector3ArrayStamped
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3, Vector3Stamped
from sensor_msgs.msg import Imu

from rosi_common.rosi_tools import jointStateData2dict, correctFlippersJointSignal
from rosi_model.rosi_description import *
from rosi_common.dq_tools import angleAxis2dqRot, tr2dq, dqExtractTransV3, dqExtractQuaternions, dqExtractRotM
from rosi_common.geometry_tools import projectionV1toV2_norm

from rosi_common.node_status_tools import nodeStatus
from rosi_common.srv import SetNodeStatus, GetNodeStatusList

import numpy as np
import quaternion

class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        ##=== Useful variables

        # node status object
        self.ns = nodeStatus(node_name)

        self.flpJointState = None
        self.flpTouchState = None
        self.msg_imu = None

        ##=== Retrieving ROS parameters
        self.p_dist_sprockets = rospy.get_param('/rosi_model/dist_primary2secondary_sprockets')
        self.p_primarySprocket_radius = rospy.get_param('/rosi_model/primary_sprocket_radius')
        self.p_secondarySprocket_radius = rospy.get_param('/rosi_model/secondary_sprocket_radius')


        ##=== Dimensional auxiliary variables

        # vector from primary sprocket to the secondary
        self.dim_v_q_distSprockets = (np.array([0,0,1]) * self.p_dist_sprockets).reshape(3,1) # the secondary sprocket is aligned to the primary by the x_Qi vector (1,0,0)

        # an standard x vector
        self.v_x = np.array([1,0,0]).reshape(3,1)

        # default gravity g vector in quaternion format
        self.q_g_def = np.quaternion(0,0,0,-1)

        ##=== ROS interfaces
        # publisher
        self.pub_cntctPnt = rospy.Publisher('/rosi/model/contact_point_wrt_base', Vector3ArrayStamped, queue_size=5)
        self.pub_gvector = rospy.Publisher('/rosi/model/grav_vec_wrt_frame_r', Vector3Stamped, queue_size=5)

        # subscribers
        sub_flpTouchState = rospy.Subscriber('/rosi/flippers/status/touching_ground', Int8ArrayStamped, self.cllbck_flpTouchState)
        sub_jointState = rospy.Subscriber('/rosi/rosi_controller/joint_state', JointState, self.cllbck_jointState)
        sub_imu = rospy.Subscriber('/mti/sensor/imu', Imu, self.cllbck_imu)

        # services
        srv_setActive = rospy.Service(self.ns.getSrvPath('active', rospy), SetNodeStatus, self.srvcllbck_setActive)
        srv_getStatus = rospy.Service(self.ns.getSrvPath('getNodeStatus', rospy), GetNodeStatusList, self.srvcllbck_getStatus)

        # node main method
        self.nodeMain()



    def nodeMain(self):
        '''Node main method'''

        # node rate slee;
        node_rate_sleep = rospy.Rate(10)

        # spins
        rospy.loginfo('Node in spin mode.')
        while not rospy.is_shutdown():

            if self.ns.getNodeStatus()['active']: # only runs if node is active

                 # only runs if messages have been received
                if self.flpJointState is not None and self.flpTouchState is not None and self.msg_imu  is not None: 
                    if self.flpTouchState.data[0] != -1: # only runs if the touch state node is sending valid values

                        #finding the gravity vector w.r.t. {R} frame
                        q_w_r = np.quaternion(self.msg_imu.orientation.w, self.msg_imu.orientation.x, self.msg_imu.orientation.y, self.msg_imu.orientation.z) # finding the {R} orientation wrt {W}                        
                        q_r_g = q_w_r.conj() * self.q_g_def * q_w_r     # gravity vector w.r.t. {R}
                        # TODO publicar o vetor de velocidade
                        
                        # extracting flippers needed data
                        _, flp_data = jointStateData2dict(self.flpJointState)
                        flp_pos = [x*y for x,y in zip(flp_data['pos'], [1, 1, -1, -1])] # correcting joints pos signal from real axis value to the modeled concept (which considers all propulsion frames alined wrt {R})


                        # flipper ground touch data
                        flp_touch = self.flpTouchState.data

                        # numpy vector array with accumulating contact points
                        cp_l = []

                        # computing contact point w.r.t. frame Pi
                        for key, touch, jointPos in zip(propKeys, flp_touch, flp_pos):  # iterates for all four propellants

                            if touch == 0:  # when wheel is touching the ground 
                                cp_l.append(dq_pi_wheelContact[key])

                            else: # when flipper is touching the ground

                                # dual-quaternion of frame Qi w.r.t. Pi (its a rotation around z of the flipper joint value)
                                dq_pi_qi = angleAxis2dqRot(jointPos, [0,1,0]) # rotation between Pi and Qi is always about y axis

                                #-- first flipper contact point candidate: elbow
                                # changing representation of elbow contact point from Qi to Pi
                                dq_pi_cp_elbow = dq_pi_qi * dq_qi_flpContactElbow[key]
                                v_pi_cp1 = dqExtractTransV3(dq_pi_cp_elbow)


                                #-- second flipper contact point candidate: tip
                                # obtaining the gravitational vector expressed in {Qi}
                                q_pi_g =  q_base_piFlp[key].conj() * q_r_g * q_base_piFlp[key]  # expressing the gravitational vector in {Pi}
                                q_pi_qi = dqExtractRotQuaternion(dq_pi_qi)      # extracting orientation quarternion of {Qi} wrt {Pi}
                                q_qi_g = q_pi_qi.conj() * q_pi_g * q_pi_qi      # expressing gravitational vector in the {Qi} frame
                                v_qi_g = quat2tr(q_qi_g)

                                # computing the gravitational vector projection in the xz plane of {Qi{}}
                                v_qi_g_projxz = np.array([v_qi_g[0], 0, v_qi_g[2]]).reshape(3,1)  # the projection of the gravitational vector in the xz plane of {Qi}
                                v_qi_g_projxz = (v_qi_g_projxz / np.linalg.norm(v_qi_g_projxz))  # normalizing the projected vector

                                # computing the secondary contact candidate point wrt {Qi}
                                v_qi_cp2 = self.dim_v_q_distSprockets + v_qi_g_projxz*self.p_secondarySprocket_radius # summing the vectors from {Qi} to the contact point considering the gravitational vector on the second sprocket

                                # expressing the second candidate in pi frame
                                dq_pi_cp2 = dq_pi_qi * tr2dq(v_qi_cp2)
                                v_pi_cp2 = dqExtractTransV3(dq_pi_cp2)
                           
                                #-- deciding which candidate is the farthest from Pi along g vector
                                # projects both candidates onto g vector
                                v_pi_g = quat2tr(q_pi_g).reshape(3,1)
                                proj_c1_g = projectionV1toV2_norm(v_pi_cp1.T, v_pi_g)
                                proj_c2_g = projectionV1toV2_norm(v_pi_cp2.T, v_pi_g)
                                
                                # decides which has the smaller biggest norm (farthest from Pi along g vector)
                                if proj_c1_g > proj_c2_g:
                                    cp_l.append(tr2dq(v_pi_cp1))
                                else:
                                    cp_l.append(tr2dq(v_pi_cp2))


                        # transforming contact point w.r.t. Pi  to frame R 
                        dq_base_cp_l = [dq_base_pi * dq_pi_cp for dq_base_pi, dq_pi_cp in zip(dq_base_piFlp.values(), cp_l)]

                        # converting obtained dual-quaternions to position vectors
                        p_base_cp_l = [dq.translation().vec3() for dq in dq_base_cp_l]

                        # receiving ros time
                        time_current = rospy.get_rostime()

                        # publishing contact points message message
                        m = Vector3ArrayStamped()
                        m.header.stamp = time_current
                        m.header.frame_id = self.node_name
                        for v in p_base_cp_l:
                            m.vec.append(Vector3(x=v[0],y=v[1],z=v[2]))
                        self.pub_cntctPnt.publish(m)

                        #print(m)

                        # publishing the gravitational vector wrt {R}
                        m = Vector3Stamped()
                        m.header.stamp = time_current
                        m.header.frame_id = self.node_name
                        m.vector.x = q_r_g.components[1]
                        m.vector.y = q_r_g.components[2]
                        m.vector.z = q_r_g.components[3]
                        self.pub_gvector.publish(m)

            # sleeps the node
            node_rate_sleep.sleep()


    def cllbck_flpTouchState(self, msg):
        '''Callback for the flippers touching ground information'''
        self.flpTouchState = msg


    def cllbck_jointState(self, msg):
        ''' Callback for flippers state'''
        self.flpJointState = msg

    
    def cllbck_imu(self, msg):
        '''Callback for the IMU messages.'''
        # creating a numpy quaternion element
        self.msg_imu = msg


    ''' === Service Callbacks === '''
    def srvcllbck_setActive(self, req):
        ''' Method for setting the active node status flag'''
        return self.ns.defActiveServiceReq(req, rospy)


    def srvcllbck_getStatus(self, req):
        ''' Method for returning the node status flag list'''
        return self.ns.getNodeStatusSrvResponse()


if __name__ == '__main__':

    node_name = 'ground_contact_point_wrt_base'

    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')

    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass