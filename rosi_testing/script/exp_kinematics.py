#!/usr/bin/env python3
"""Node for performing the kinematics evaluation experiment
rosbag record /rosi/rosi_controller/input /rosi/base/space/cmd_vel /rosi/rosi_controller/joint_state /vicon/rosi_base/rosi_base /rosi/base/space/cmd_vel/autnav
"""

import rospy

import numpy as np
from dqrobotics import DQ
from datetime import datetime, date
from time import strftime
from xml.dom.pulldom import parseString
import rospkg
import csv
import os

from rosi_common.msg import TwistStamped
from geometry_msgs.msg import TransformStamped

from rosi_common.dq_tools import trAndOri2dq, dq2rpy, dqExtractTransV3
from rosi_common.vicon_tools import getBasePoseFromMarkerDq

class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        ##=== Parameters

        # base velocities to apply
        self.vl = [0.0, 0.0, 0.2]
        self.va = [0.0, 0.0, 0.0]

        # node rate sleep [Hz]
        self.p_rateSleep = 50

        # time to apply the command to the base
        self.p_timeToApplyTwist = rospy.Duration.from_sec(3)

        # folder to save results
        test_name = 'kinematics'

        #=== Handling folders for saving

        # save file name
        today = date.today()
        now = datetime.now()
        d = today.strftime("%Y-%m-%d")
        n = now.strftime("%H-%M-%S")
        saveFileName = d + '_' + n + '.csv'

         # directory to save results
        saveDir = rospkg.RosPack().get_path('rosi_testing') + '/output/' + test_name + '/'

        # savepath
        self.savePath = saveDir + saveFileName

        # creates the target directory if it does not exists
        if not os.path.exists(saveDir):
            os.mkdir(saveDir)

        # deletes results csv if it exists
        if os.path.exists(self.savePath):
            os.remove(self.savePath)


        ##=== Useful variables and initializations
        self.basePoseGT_msg = None

        # header for the saving file
        self.saveFile_header = ['time', 
                                'in_vl_x', 'in_vl_y', 'in_vl_z', 'in_va_x', 'in_va_y', 'in_va_z',
                                'gt_dq_pw', 'gt_dq_px', 'gt_dq_py', 'gt_dq_pz', 'gt_dq_dw', 'gt_dq_dx', 'gt_dq_dy', 'gt_dq_dz']

        
        ##=== ROS interfaces

        # publishers
        self.pub_baseCmdVel = rospy.Publisher('/rosi/base/space/cmd_vel/autnav', TwistStamped, queue_size=5)

        # subscribers
        self.sub_basePoseGT = rospy.Subscriber('/vicon/rosi_base/rosi_base', TransformStamped, self.cllbck_basePoseGT)

        ##=== Main loop
        self.nodeMain()


    def nodeMain(self):
        '''Node main method'''
        
        
        while not rospy.is_shutdown():

            # only runs if a valid message has been received
            if self.basePoseGT_msg is not None:

                # node rate sleep
                node_rate_sleep = rospy.Rate(self.p_rateSleep)

                # applies the velocity to joints before starting measurement so motion enters in steady state.
                rospy.loginfo('Applying velocity.')
                f4_time_spent = rospy.Duration(0)
                f4_time_ini = rospy.get_rostime()
                while f4_time_spent < rospy.Duration(1):
                    m = self.createTwistMsg(self.vl, self.va)
                    self.pub_baseCmdVel.publish(m)
                    node_rate_sleep.sleep()
                    f4_time_spent = rospy.get_rostime() - f4_time_ini

                # variables ini
                time_ini = rospy.get_rostime()
                test_time = rospy.Duration.from_sec(0)
                #time_last = time_ini

                # logging variable
                log_data = []

                rospy.loginfo('Initiating measurements...')
                while self.p_timeToApplyTwist > test_time:

                    # retrieving current ros time
                    time_current = rospy.get_rostime()

                    # computing the delta time
                    #dt = (time_current - time_last).to_sec()

                    # applies the test velocity
                    m = self.createTwistMsg(self.vl, self.va)
                    self.pub_baseCmdVel.publish(m)
                    
                    # obtaining the vicon marker pose
                    pose_marker_dq = self.twist2Dq(self.basePoseGT_msg)

                    # obtains {R} pose from the vicon marker pose
                    pose_r_dq_curr = getBasePoseFromMarkerDq(pose_marker_dq)

                    # computes the pose displacement from last and current observations
                    #pose_r_delta_dq = pose_r_dq_last.conj() * pose_r_dq_curr
                    #rpy_r_delta = np.array(dq2rpy(pose_r_delta_dq)).reshape(3,1)
                    #tr_r_delta = dqExtractTransV3(pose_r_delta_dq)

                    # computes the velocity from deltas
                    #rpy_r_spd = rpy_r_delta / dt
                    #tr_r_spd = tr_r_delta / dt

                    # test delayed time
                    test_time = time_current - time_ini

                    # saving velocity values to the log variable
                    log_data.append([test_time.to_sec()] 
                                    + self.vl + self.va
                                    + pose_r_dq_curr.vec8().tolist())

                    # updating last variables
                    #pose_r_dq_last = pose_r_dq_curr
                    #time_last = time_current

                    # sleeps the node
                    node_rate_sleep.sleep()

                rospy.loginfo('Writing results in %s'%self.savePath)
                with open(self.savePath, "w", newline="") as f:
                    writer = csv.writer(f)
                    writer.writerow(self.saveFile_header)
                    writer.writerows(log_data)

                rospy.loginfo('End of experiment.')
                break


    def cllbck_basePoseGT(self, msg):
        '''Callback for the ROSI base twist'''
        self.basePoseGT_msg = msg


    @staticmethod
    def twist2Dq(twist):
        '''Returns the base position as a numpy array
        Output
            - a <nd.array> element with the chassis base position'''
        tr = [twist.transform.translation.x, twist.transform.translation.y, twist.transform.translation.z]
        ori_q = [twist.transform.rotation.w, twist.transform.rotation.x, twist.transform.rotation.y, twist.transform.rotation.z]
        return trAndOri2dq(tr, ori_q, 'trfirst')

    
    @staticmethod
    def createTwistMsg(vl, va):
        '''Creates a Twist message'''
        m = TwistStamped()
        m.header.stamp = rospy.get_rostime()
        m.header.frame_id = 'autnav'
        m.twist.linear.x = vl[0]
        m.twist.linear.y = vl[1]
        m.twist.linear.z = vl[2]
        m.twist.angular.x = va[0]
        m.twist.angular.y = va[1]
        m.twist.angular.z = va[2] 
        return m


if __name__ == '__main__':
    node_name = 'exp_kinematics'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass