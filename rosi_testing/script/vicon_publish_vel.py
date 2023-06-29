#!/usr/bin/env python3
""" Nde for printing in the console the vicon marker linear and angular speeds"""
import rospy

import numpy as np

from rosi_common.msg import Vector3ArrayStamped
from geometry_msgs.msg import TransformStamped, Vector3

from rosi_common.dq_tools import twist2Dq, dqExtractTransV3, dq2rpy
from rosi_common.vicon_tools import getBasePoseFromMarkerDq

from rosi_model.rosi_description import dq_base_piFlp

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
        self.pubVelR = rospy.Publisher('/vicon/rosi_base/vel', TransformStamped, queue_size=5)
        self.pubVelPi = rospy.Publisher('/vicon/propulsion/vel_lin', Vector3ArrayStamped, queue_size=5)

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
                    dq_R_last = getBasePoseFromMarkerDq(twist2Dq(self.basePoseGT_msg))
                    dq_Pi_last_l = [dq_R_last * dq_pi for dq_pi in dq_base_piFlp.values()]
                    time_last = self.basePoseGT_msg.header.stamp
                    flag_firstRun = False
                
                else: # for all other runs

                    # retrieving the current base pose dq
                    dq_R_curr = getBasePoseFromMarkerDq(twist2Dq(self.basePoseGT_msg))

                    # computing the pose of each {Pi} in space using the {R} as base
                    dq_Pi_curr_l = [dq_R_curr * dq_pi for dq_pi in dq_base_piFlp.values()]

                    # current time
                    time_curr = self.basePoseGT_msg.header.stamp

                    # delta time
                    dt = time_curr - time_last

                    # computing the displacement dq of {R}
                    ddq_R = dq_R_last.conj() * dq_R_curr

                    # computing the displacement of {Pi} frames
                    ddq_Pi_l = [dq_last.conj() * dq_curr for dq_last, dq_curr in zip(dq_Pi_last_l, dq_Pi_curr_l)]

                    # the linear velocity of {R}
                    vl_R = dqExtractTransV3(ddq_R) / dt.to_sec()

                    # the angular velocity of {Pi}
                    va_R = np.array(dq2rpy(ddq_R)).reshape(3,1) / dt.to_sec()

                    # the linear velocity of {Pi}
                    vl_Pi_l = [dqExtractTransV3(ddq) / dt.to_sec() for ddq in ddq_Pi_l]

                    # the angular velocity of {Pi}
                    #va_Pi_l = [np.array(dq2rpy(ddq)).reshape(3,1) / dt.to_sec() for ddq in ddq_Pi_l]


                    ros_time = rospy.get_rostime()

                    ##--- Publishing velocities of {R}
                    m = TransformStamped()
                    m.header.stamp = ros_time
                    m.header.frame_id = self.node_name
                    m.transform.translation.x = vl_R[0][0]
                    m.transform.translation.y = vl_R[1][0]
                    m.transform.translation.z = vl_R[2][0]
                    m.transform.rotation.x = va_R[0][0]
                    m.transform.rotation.y = va_R[1][0]
                    m.transform.rotation.z = va_R[2][0]
                    self.pubVelR.publish(m)

                    ##--- Publishing velocities of {Pi}
                    m = Vector3ArrayStamped()
                    m.header.frame_id = self.node_name
                    m.header.stamp = ros_time
                    m.vec = [Vector3(v[0][0], v[1][0], v[2][0]) for v in vl_Pi_l]
                    self.pubVelPi.publish(m)

                    # updating last variables
                    time_last = time_curr
                    dq_R_last = dq_R_curr
                    dq_Pi_last_l = dq_Pi_curr_l


            node_rate_sleep.sleep()



    def cllbck_basePoseGT(self, msg):
        '''Callback for the ROSI base twist'''
        self.basePoseGT_msg = msg


if __name__ == '__main__':
    node_name = 'vicon_publish_vel'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass