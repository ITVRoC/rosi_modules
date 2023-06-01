#!/usr/bin/env python3
'''this is a ROSI analysis node
It generates topics proper for RVIZ displaying rosi mesh

'''
import rospy

from rosi_common.msg import TwistStamped, Vector3ArrayStamped
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker

from rosi_common.tf_tools import BroadcastRvizTransform, BroadcastRvizVector, listColors
from rosi_common.dq_tools import removeYaw, trAndOri2dq, imuROSData2dq

import numpy as np
import quaternion


class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        ##=== Parameters

        self.ang_vec_dist = 0.2
        self.ang_vec_height = 0.05


        ##=== Useful variables
        self.msg_baseSpaceCmdVel = None
        self.dq_world_base = None
        self.p_grnd = None


        ##=== ROS interfaces

        # publishers
        self.pub_vis_baseCmdVel_lin = rospy.Publisher('/rosi/visual/base_cmd_vel/lin', Marker, queue_size=1)
        self.pub_vis_baseCmdVel_ang_x = rospy.Publisher('/rosi/visual/base_cmd_vel/ang_x', Marker, queue_size=1)
        self.pub_vis_baseCmdVel_ang_y = rospy.Publisher('/rosi/visual/base_cmd_vel/ang_y', Marker, queue_size=1)
        self.pub_vis_baseCmdVel_ang_z = rospy.Publisher('/rosi/visual/base_cmd_vel/ang_z', Marker, queue_size=1)

        sub_baseSpaceCmdVel = rospy.Subscriber('/rosi/base/space/cmd_vel', TwistStamped, self.cllbck_baseSpaceCmdVel)
        sub_grndDist = rospy.Subscriber('/rosi/model/base_ground_distance', Vector3ArrayStamped, self.cllbck_grndDist)
        sub_imu = rospy.Subscriber('/mti/sensor/imu', Imu, self.cllbck_imu) 

        ##=== Node main method
        self.nodeMain()


    def nodeMain(self):
        '''Node main method'''

        # rate sleep
        node_rate_sleep = rospy.Rate(10)

        # auxiliary variables
        aux_originZero = np.array([0, 0, 0])


        while not rospy.is_shutdown():

            if self.msg_baseSpaceCmdVel is not None and self.dq_world_base is not None and self.p_grnd is not None:

                # receives and treating ROS time
                timeRos = rospy.get_rostime()

                ##=== Treating variables

                # removing yaw from dq world base
                dq_world_base_noYaw = removeYaw(self.dq_world_base)  

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

                # vectors
                BroadcastRvizVector(timeRos, 'dq_world_base', 'base_cmd_vel', 0 , self.pub_vis_baseCmdVel_lin, aux_originZero, baseSpaceCmdVel_lin, listColors['base_cmd_vel_lin'])    # base cmd lin vel
                BroadcastRvizVector(timeRos, 'dq_world_base', 'base_cmd_ang_x', 0 , self.pub_vis_baseCmdVel_ang_x, baseSpaceCmdVel_ang_x_origin, baseSpaceCmdVel_ang_x_vec, listColors['red'])    # base cmd ang vel x
                BroadcastRvizVector(timeRos, 'dq_world_base', 'base_cmd_ang_y', 0 , self.pub_vis_baseCmdVel_ang_y, baseSpaceCmdVel_ang_y_origin, baseSpaceCmdVel_ang_y_vec, listColors['green'])    # base cmd ang vel y
                BroadcastRvizVector(timeRos, 'dq_world_base', 'base_cmd_ang_z', 0 , self.pub_vis_baseCmdVel_ang_z, baseSpaceCmdVel_ang_z_origin, baseSpaceCmdVel_ang_z_vec, listColors['blue'])    # base cmd ang vel z

                ##=== Sleeping the node
                node_rate_sleep.sleep()


    def cllbck_imu(self, msg):
        '''Callback for the IMU message'''
        self.dq_world_base, _, _ = imuROSData2dq(msg)


    def cllbck_baseSpaceCmdVel(self, msg):
        ''' Callback for the base space cmd vel'''
        self.msg_baseSpaceCmdVel = msg
        #print(msg)

    
    def cllbck_grndDist(self, msg):
        '''Callback for received distance to the ground info'''
        # stores received distance to the ground as a 3D vector
        self.p_grnd = np.array([msg.vec[0].x, msg.vec[0].y, msg.vec[0].z])


if __name__ == '__main__':
    node_name = 'disp_rviz_base_cmd'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass