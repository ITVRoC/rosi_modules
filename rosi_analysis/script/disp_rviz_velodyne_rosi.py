#!/usr/bin/env python3
'''this is a ROSI analysis node
It generates the TF for proper displaying velodyne with ROSI 

'''
import rospy

from dqrobotics import *

from rosi_common.tf_tools import BroadcastRvizTransform

class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        # gen3 stereo camera point cloud reference frame pose offset wrt gen3 link7 (tcp)
        self.pcl_offset = DQ(0.043619,0,0,0.999048,0,0,0.12,0)

        ###=== Node main method
        self.nodeMain()


    def nodeMain(self):

        # rate sleep
        node_rate_sleep = rospy.Rate(10)

        rospy.loginfo('['+self.node_name+'] Node in spin mode.')

        while not rospy.is_shutdown():

            # receives and treating ROS time
            timeRos = rospy.get_rostime()

            # broadcasting velodyne transform
            BroadcastRvizTransform(timeRos, 'dq_world_base', 'velodyne', self.pcl_offset)

            ##=== Sleeping the node
            node_rate_sleep.sleep()


if __name__ == '__main__':
    node_name = 'disp_rviz_velodyne_rosi'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass