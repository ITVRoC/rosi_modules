#!/usr/bin/env python3
'''this is a ROSI analysis node
It generates topics proper for RVIZ plot containing pose controller metrics

'''
import rospy

from rosi_common.msg import TwistStamped, Vector3Array
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker, MarkerArray

from rosi_common.tf_tools import BroadcastRvizTransform, BroadcastRvizVector, listColors
from rosi_common.dq_tools import removeYaw, imuROSData2dq
from rosi_model.rosi_description import dq_base_pi

class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        ##=== Initializations
        self.dq_world_base = None
        self.flp_corrDirVec = None

        ##=== ROS interfaces

        # publishers
        self.pub_vis_corrDirVec = rospy.Publisher('/rosi/visual/pose_contro/corr_dir_vec_array', MarkerArray, queue_size=1)

        # subscribers
        sub_baseSpaceCmdVel = rospy.Subscriber('/rosi/base/space/cmd_vel', TwistStamped, self.cllbck_baseSpaceCmdVel)
        sub_imu = rospy.Subscriber('/mti/sensor/imu', Imu, self.cllbck_imu) 
        sub_flipperSpaceCmdVel = rospy.Subscriber('/rosi/flippers/space/cmd_vel', Vector3Array, self.cllbck_flipperSpaceCmdVel)
        sub_flipperSpaceFkinVelLin = rospy.Subscriber('/rosi/flippers/space/fkin/vel/lin', Vector3Array, self.cllbck_flipperSpaceFkinVelLin)

        ##=== Node main
        self.nodeMain()

    
    def nodeMain(self):
        '''Node main method'''

        # rate sleep
        node_rate_sleep = rospy.Rate(10)

        # auxiliary variables
        aux_idCount = [i for i in range(0,4)]
        aux_flpIdList = [id for id in dq_base_pi]
        aux_originZero = [[0, 0, 0]] * 4

        while not rospy.is_shutdown():

            # only runs when valid messages have been received
            if self.dq_world_base is not None and self.flp_corrDirVec is not None:

                # receives and treating ROS time
                timeRos = rospy.get_rostime()  

                # removing yaw from dq world base
                dq_world_base_noYaw = removeYaw(self.dq_world_base)

                # computing flippers frame considering base orientation
                dq_base_flp_error = [dq_world_base_noYaw*dq_flp for dq_flp in dq_base_pi.values()]
                #print(dq_base_flp_error)

                ##=== Broadcasting to RVIZ

                # transforms
                BroadcastRvizTransform(timeRos, "world", "dq_world_base", dq_world_base_noYaw)    # dq world base no yaw
                BroadcastRvizTransform(timeRos, "world", aux_flpIdList, dq_base_flp_error)   # flippers frame with error

                # vectors
                BroadcastRvizVector(timeRos, aux_flpIdList, "corr_dir_vec", aux_idCount , self.pub_vis_corrDirVec, aux_originZero, self.flp_corrDirVec, listColors['corr_dir_vec'])    # corrective vectors

                ##=== Sleeping the node
                node_rate_sleep.sleep()



    def cllbck_baseSpaceCmdVel(self, msg):
        '''Callback for the base space cmd vel'''
        pass


    def cllbck_imu(self, msg):
        '''Callback for the IMU message'''
        self.dq_world_base, _, _ = imuROSData2dq(msg)


    def cllbck_flipperSpaceCmdVel(self, msg):
        '''Callback for the flippers space cmd vel'''
        
        self.flp_corrDirVec = [[v.x, v.y, v.z] for v in msg.vec]


    def cllbck_flipperSpaceFkinVelLin(self, msg):
        '''Callback for the flippers space fkin cmd vel'''
        pass


if __name__ == '__main__':
    node_name = 'disp_rviz_pose_reg_metrics'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass