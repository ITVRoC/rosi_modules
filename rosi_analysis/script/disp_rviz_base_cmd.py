#!/usr/bin/env python3
'''this is a ROSI analysis node
It generates topics proper for RVIZ plot containing rosi base meshes and vectors

'''
import rospy

from rosi_common.msg import TwistStamped, Vector3ArrayStamped
from sensor_msgs.msg import Imu, JointState
from visualization_msgs.msg import Marker, MarkerArray

from rosi_common.tf_tools import BroadcastRvizTransform, BroadcastRvizVector, listColors, BroadcastRvizMesh
from rosi_common.dq_tools import removeYaw, trAndOri2dq, angleAxis2dqRot, imuROSData2dq, rpy2quat
from rosi_common.rosi_tools import correctFlippersJointSignal2, jointStateData2dict

from rosi_model.rosi_description import dq_base_piFlp


import numpy as np
import quaternion


class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        ##=== Parameters
        self.ang_vec_dist = 0.2
        self.ang_vec_height = 0.05

        ##=== Meshes pose correction

        #-- base mesh offset
        #offset_base_tr = [-0.39520, -0.1950, -0.07]
        offset_base_tr = [-0.3920, -0.1950, -0.07]
        offset_base_rpy = np.deg2rad([90.0, 0.0, 90.0])

        #--- flp1 mesh offset
        offset_flp1_tr = [0.0, 0.0, 0.0]
        offset_flp1_rpy = np.deg2rad([90.0, 0.0, 180.0])

        #--- flp2 mesh offset
        offset_flp2_tr = [0.0, 0.0, 0.0]
        offset_flp2_rpy = np.deg2rad([90.0, 0.0, 0.0])

        #--- flp3 mesh offset
        offset_flp3_tr = [0.0, 0.0, 0.0]
        offset_flp3_rpy = np.deg2rad([90.0, 0.0, 180.0])

        #--- flp4 mesh offset
        offset_flp4_tr = [0.0, 0.0, 0.0]
        offset_flp4_rpy = np.deg2rad([90.0, 0.0, 0.0])

       
        ##=== Useful variables
        self.msg_baseSpaceCmdVel = None
        self.dq_world_base = None
        self.p_grnd = None
        self.flpJointState = None

        ##=== One time calculations

        # generating meshes ofsset dictionaries
        self.mesh_poseOffset_base = self.getMeshOffset(offset_base_tr, offset_base_rpy)
        self.mesh_poseOffset_flippers = {
                                'fl': self.getMeshOffset(offset_flp1_tr, offset_flp1_rpy),
                                'fr': self.getMeshOffset(offset_flp2_tr, offset_flp2_rpy),
                                'rl': self.getMeshOffset(offset_flp3_tr, offset_flp3_rpy),
                                'rr': self.getMeshOffset(offset_flp4_tr, offset_flp4_rpy)
        }


        ##=== ROS interfaces
        # publishers
        self.pub_vis_baseCmdVel_lin = rospy.Publisher('/rosi/visual/base_cmd_vel/lin', Marker, queue_size=5)
        self.pub_vis_baseCmdVel_ang_x = rospy.Publisher('/rosi/visual/base_cmd_vel/ang_x', Marker, queue_size=5)
        self.pub_vis_baseCmdVel_ang_y = rospy.Publisher('/rosi/visual/base_cmd_vel/ang_y', Marker, queue_size=5)
        self.pub_vis_baseCmdVel_ang_z = rospy.Publisher('/rosi/visual/base_cmd_vel/ang_z', Marker, queue_size=5)

        self.pub_vis_meshBase = rospy.Publisher('/rosi/visual/mesh/chassis', Marker, queue_size=5)
        self.pub_vis_meshFlippers = rospy.Publisher('/rosi/visual/mesh/flippers', MarkerArray, queue_size=5)


        # subscribers
        sub_baseSpaceCmdVel = rospy.Subscriber('/rosi/base/space/cmd_vel', TwistStamped, self.cllbck_baseSpaceCmdVel)
        sub_grndDist = rospy.Subscriber('/rosi/model/base_ground_distance', Vector3ArrayStamped, self.cllbck_grndDist)
        sub_imu = rospy.Subscriber('/sensor/imu_corrected', Imu, self.cllbck_imu) 
        sub_jointState = rospy.Subscriber('/rosi/rosi_controller/joint_state', JointState, self.cllbck_jointState)

        ##=== Node main method
        self.nodeMain()


    def nodeMain(self):
        '''Node main method'''

        # rate sleep
        node_rate_sleep = rospy.Rate(10)

        # auxiliary variables
        aux_originZero = np.array([0, 0, 0])
        aux_id_base_flpP_l = ['dq_base_flpP'+str(i+1) for i in range(4)]
        aux_id_flpP_flpQ_l = ['dq_flpP_flpQ'+str(i+1) for i in range(4)]

        rospy.loginfo('['+self.node_name+'] Node in spin mode.')
        while not rospy.is_shutdown():

            if self.msg_baseSpaceCmdVel is not None and self.dq_world_base is not None and self.p_grnd is not None and self.flpJointState is not None:

                # receives and treating ROS time
                timeRos = rospy.get_rostime()

                ##=== Computing poses

                # removing yaw from dq world base
                dq_world_base_noYaw = removeYaw(self.dq_world_base)  

                # computing flippers frame pose
                dq_base_flp_l = [dq_world_base_noYaw*dq_flp for dq_flp in dq_base_piFlp.values()]

                # dual-quaternion of frame Qi w.r.t. Pi (its a rotation around z of the flipper joint value)
                _,joint_state = jointStateData2dict(self.flpJointState)
                flp_pos = correctFlippersJointSignal2(joint_state['pos'])
                dq_pi_qi = [angleAxis2dqRot(jointPos, [0,1,0]) for jointPos in flp_pos] # rotation between Pi and Qi is always about y axis


                ##=== Treating variables

                # extracting orientation quaternion
                aux = dq_world_base_noYaw.normalize().rotation().vec4()
                q_world_base_noYaw = np.quaternion(aux[0], aux[1], aux[2], aux[3])

                # creating current pose dual-quaternion
                dq_pose_world_base = trAndOri2dq(self.p_grnd, q_world_base_noYaw, 'trfirst')

                baseSpaceCmdVel_lin = np.array([self.msg_baseSpaceCmdVel.twist.linear.x, self.msg_baseSpaceCmdVel.twist.linear.y, self.msg_baseSpaceCmdVel.twist.linear.z])

                # angular cmd x
                baseSpaceCmdVel_ang_x_origin = np.array([self.ang_vec_dist, self.msg_baseSpaceCmdVel.twist.angular.x/2, self.ang_vec_height])
                baseSpaceCmdVel_ang_x_vec = np.array([self.ang_vec_dist, -self.msg_baseSpaceCmdVel.twist.angular.x, self.ang_vec_height])

                 # angular cmd y
                baseSpaceCmdVel_ang_y_origin = np.array([-self.msg_baseSpaceCmdVel.twist.angular.y/2, self.ang_vec_dist, self.ang_vec_height])
                baseSpaceCmdVel_ang_y_vec = np.array([self.msg_baseSpaceCmdVel.twist.angular.y, self.ang_vec_dist, self.ang_vec_height])

                 # angular cmd z
                baseSpaceCmdVel_ang_z_origin = np.array([-self.ang_vec_height, self.msg_baseSpaceCmdVel.twist.angular.z/2, self.ang_vec_dist])
                baseSpaceCmdVel_ang_z_vec = np.array([-self.ang_vec_height, -self.msg_baseSpaceCmdVel.twist.angular.z, self.ang_vec_dist])
                

                ##=== Broadcasting to RVIZz

                # transforms
                BroadcastRvizTransform(timeRos, "world", "dq_world_base", dq_pose_world_base)    # dq world base no yaw
                BroadcastRvizTransform(timeRos, "dq_world_base", aux_id_base_flpP_l, dq_base_piFlp)  # flippers frame
                BroadcastRvizTransform(timeRos, aux_id_base_flpP_l, aux_id_flpP_flpQ_l, dq_pi_qi)  # flippers frame

                # vectors
                BroadcastRvizVector(timeRos, 'dq_world_base', 'base_cmd_vel', 0 , self.pub_vis_baseCmdVel_lin, aux_originZero, baseSpaceCmdVel_lin, listColors['base_cmd_vel_lin'])    # base cmd lin vel
                BroadcastRvizVector(timeRos, 'dq_world_base', 'base_cmd_ang_x', 0 , self.pub_vis_baseCmdVel_ang_x, baseSpaceCmdVel_ang_x_origin, baseSpaceCmdVel_ang_x_vec, listColors['red'])    # base cmd ang vel x
                BroadcastRvizVector(timeRos, 'dq_world_base', 'base_cmd_ang_y', 0 , self.pub_vis_baseCmdVel_ang_y, baseSpaceCmdVel_ang_y_origin, baseSpaceCmdVel_ang_y_vec, listColors['green'])    # base cmd ang vel y
                BroadcastRvizVector(timeRos, 'dq_world_base', 'base_cmd_ang_z', 0 , self.pub_vis_baseCmdVel_ang_z, baseSpaceCmdVel_ang_z_origin, baseSpaceCmdVel_ang_z_vec, listColors['blue'])    # base cmd ang vel z

                # meshes
                BroadcastRvizMesh(timeRos, 'dq_world_base', self.pub_vis_meshBase , 'chassis', '.stl', self.mesh_poseOffset_base)
                BroadcastRvizMesh(timeRos, aux_id_flpP_flpQ_l, self.pub_vis_meshFlippers, 'flipper', '.stl', self.mesh_poseOffset_flippers)

                #print(timeRos)

            ##=== Sleeping the node
            node_rate_sleep.sleep()


    def cllbck_imu(self, msg):
        '''Callback for the IMU message'''
        self.dq_world_base, _, _ = imuROSData2dq(msg)


    def cllbck_baseSpaceCmdVel(self, msg):
        ''' Callback for the base space cmd vel'''
        self.msg_baseSpaceCmdVel = msg

    
    def cllbck_grndDist(self, msg):
        '''Callback for received distance to the ground info'''
        # stores received distance to the ground as a 3D vector
        self.p_grnd = np.array([msg.vec[0].x, msg.vec[0].y, msg.vec[0].z])


    def cllbck_jointState(self, msg):
        ''' Callback for flippers state'''
        self.flpJointState = msg


    @staticmethod
    def getMeshOffset(tr, rpy):
        ''' Creates an offset dictionary based on translation and rpy vectors
        Input:
            - tr <list/np.array>: the translation offset
            - rpy <list/np.array>: the roll/pitch/yaw offset
        Output: <dict> the offset dictionary
        '''
        rpy = rpy2quat(rpy)
        return  {'px': tr[0], 'py': tr[1], 'pz': tr[2], 'qw': rpy[0], 'qx': rpy[1], 'qy': rpy[2],'qz': rpy[3]}
    

if __name__ == '__main__':
    node_name = 'disp_rviz_base_cmd'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass

