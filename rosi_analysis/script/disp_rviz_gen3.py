#!/usr/bin/env python3
'''this is a ROSI analysis node
It generates topics proper for RVIZ plot containing gen3 meshes

'''
import rospy

import numpy as np
import quaternion

from dqrobotics import *

from rosi_common.tf_tools import BroadcastRvizMesh, BroadcastRvizTransform
from rosi_common.gen3_tools import gen3Fkin
from rosi_common.dq_tools import trAndOri2dq

from sensor_msgs.msg import JointState, Imu
from visualization_msgs.msg import Marker, MarkerArray
from rosi_common.msg import Vector3ArrayStamped

class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        ###=== PARAMETERS
        self.dq_base_gen3b = trAndOri2dq(np.array([-0.138, -0.005, 0.01]), np.quaternion(0.7071, 0.0, 0.0, -0.7071), 'trfirst')

        self.mesh_poseOffset_gen3links = {
                                'l0': {'px': 0.0, 'py': 0.0, 'pz': 0.0, 'qw': 1.0, 'qx': 0.0,'qy': 0.0,'qz': 0.0},
                                'l1': {'px': 0.0, 'py': 0.0, 'pz': 0.0, 'qw': 1.0, 'qx': 0.0,'qy': 0.0,'qz': 0.0},
                                'l2': {'px': 0.0, 'py': 0.0, 'pz': 0.0, 'qw': 1.0, 'qx': 0.0,'qy': 0.0,'qz': 0.0},
                                'l3': {'px': 0.0, 'py': 0.0, 'pz': 0.0, 'qw': 1.0, 'qx': 0.0,'qy': 0.0,'qz': 0.0},
                                'l4': {'px': 0.0, 'py': 0.0, 'pz': 0.0, 'qw': 1.0, 'qx': 0.0,'qy': 0.0,'qz': 0.0},
                                'l5': {'px': 0.0, 'py': 0.0, 'pz': 0.0, 'qw': 1.0, 'qx': 0.0,'qy': 0.0,'qz': 0.0},
                                'l6': {'px': 0.0, 'py': 0.0, 'pz': 0.0, 'qw': 1.0, 'qx': 0.0,'qy': 0.0,'qz': 0.0},
                                'l7': {'px': 0.0, 'py': 0.0, 'pz': 0.0, 'qw': 1.0, 'qx': 0.0,'qy': 0.0,'qz': 0.0},
        }

        # gen3 stereo camera point cloud reference frame pose offset wrt gen3 link7 (tcp)
        self.pcl_offset = DQ(0,0,0,1,0,0,0.035,0)

        ###=== USEFUL VARIABLES
        self.msg_gen3JointState = None
        self.p_grnd = None
        self.dq_world_base = None

        ###=== ROS INTERFACES
        self.pub_vis_gen3Mesh = {'l'+str(i): rospy.Publisher('/rosi/visual/mesh/gen3/l'+str(i), Marker, queue_size=5) for i in range(8)}

        ### DELETE
        self.pub_vis_meshBase = rospy.Publisher('/rosi/visual/mesh/chassis2', Marker, queue_size=5)

        self.sub_gen3JointState = rospy.Subscriber('/base_feedback/joint_state', JointState, self.cllbck_gen3JointState)

        ###=== Node main method
        self.nodeMain()


    def nodeMain(self):

        # rate sleep
        node_rate_sleep = rospy.Rate(10)
        
        rospy.loginfo('['+self.node_name+'] Node in spin mode.')
        while not rospy.is_shutdown():
            
            if self.msg_gen3JointState is not None: # only runs if valid messages have been received
                
                # receives and treating ROS time
                timeRos = rospy.get_rostime()


                # computing gen3 transforms
                d_dq_jb, d_dq_lb  = gen3Fkin(self.msg_gen3JointState, 'all')

                # broadcasting gen3 base wrt {R}
                BroadcastRvizTransform(timeRos, 'dq_world_base', 'dq_base_gen3b', self.dq_base_gen3b)


                # broadcasting joints pose wrt gen3 base
                for (id, dq) in d_dq_jb.items():
                    # broadcasting transform
                    BroadcastRvizTransform(timeRos, 'dq_base_gen3b', 'dq_gen3b_'+id[:-2], dq)


                # broadcasting links pose wrt gen3 base
                for id, dq, pub, off in zip(d_dq_lb.keys(), d_dq_lb.values(), self.pub_vis_gen3Mesh.values(), self.mesh_poseOffset_gen3links.values()):

                    aux_tf_dq_id = 'dq_gen3b_'+id[0:2]
                    aux_mesh_name = 'gen3_'+id[0:2]

                    # broadcasting transform
                    BroadcastRvizTransform(timeRos, 'dq_base_gen3b', aux_tf_dq_id, dq)

                    # broadcasting mesh
                    BroadcastRvizMesh(timeRos, aux_tf_dq_id, pub , aux_mesh_name, '.stl', off)

                # TF for the arm point-cloud
                #BroadcastRvizTransform(timeRos, 'world', 'camera_depth_frame', d_dq_lb['l7-b'])
                BroadcastRvizTransform(timeRos, 'dq_gen3b_l7', 'camera_depth_frame', self.pcl_offset)
        
            ##=== Sleeping the node
            node_rate_sleep.sleep()


    def cllbck_gen3JointState(self, msg):
        """Callback method for gen3 joint state data"""
        self.msg_gen3JointState = msg.position[0:7]
        #print(self.msg_gen3JointState )
        

if __name__ == '__main__':
    node_name = 'disp_rviz_gen3'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass