#!/usr/bin/env python3
""" Nde for printing in the console the vicon marker linear and angular speeds"""
import rospy

import numpy as np

from geometry_msgs.msg import TransformStamped

from rosi_common.dq_tools import twist2Dq, dqExtractTransV3, dq2rpy

class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        ##=== Parameters

        self.p_rateSleep = 20

        ##=== Useful variables
        self.basePoseGT_msg = None

        ##=== ROS Interfaces
        # publisher
        self.pubVel = rospy.Publisher('/vicon/rosi_base/vel', TransformStamped, queue_size=5)

        # subscribers
        sub_basePoseGT = rospy.Subscriber('/vicon/rosi_base/rosi_base', TransformStamped, self.cllbck_basePoseGT)

        ##=== Node main method
        self.nodeMain()


    def nodeMain(self):
        '''Node main method'''

        # node rate sleep
        node_rate_sleep = rospy.Rate(self.p_rateSleep)

        flag_firstRun = True
        while not rospy.is_shutdown():

            if self.basePoseGT_msg is not None:

                if flag_firstRun: # in case of this is the first run
                    dq_last = twist2Dq(self.basePoseGT_msg)
                    time_last = self.basePoseGT_msg.header.stamp
                    flag_firstRun = False
                
                else: # for all other runs

                    # current pose
                    dq_curr = twist2Dq(self.basePoseGT_msg)

                    # current time
                    time_curr = self.basePoseGT_msg.header.stamp

                    # delta time
                    dt = time_curr - time_last

                    # computing the displacement dq
                    ddq = dq_last.conj() * dq_curr

                    # the linear velocity 
                    vl = dqExtractTransV3(ddq) / dt.to_sec()

                    # the angular velocity
                    va = np.array(dq2rpy(ddq)).reshape(3,1) / dt.to_sec()


                    #print(vl.flatten().tolist(), va.flatten().tolist())
                    # publishing velocities
                    m = TransformStamped()
                    m.header.stamp = rospy.get_rostime()
                    m.header.frame_id = self.node_name
                    m.transform.translation.x = vl[0][0]
                    m.transform.translation.y = vl[1][0]
                    m.transform.translation.z = vl[2][0]
                    m.transform.rotation.x = va[0][0]
                    m.transform.rotation.y = va[1][0]
                    m.transform.rotation.z = va[2][0]
                    self.pubVel.publish(m)

                    # updating last variables
                    time_last = time_curr
                    dq_last = dq_curr


            node_rate_sleep.sleep()



    def cllbck_basePoseGT(self, msg):
        '''Callback for the ROSI base twist'''
        self.basePoseGT_msg = msg


if __name__ == '__main__':
    node_name = 'print_vicon'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass