#!/usr/bin/env python3
''' This node performs the ground height validation with the Vicon system'''
import rospy

import csv
import os
from datetime import datetime, date

from rosi_common.dq_tools import twist2Dq, dqExtractTransV3
from rosi_common.vicon_tools import getBasePoseFromMarkerDq
from sensor_msgs.msg import JointState

from geometry_msgs.msg import TransformStamped
from rosi_common.msg import Vector3ArrayStamped, TwistStamped


class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name


        ###==== Runtime parameters 

        # base velocities to applys
        self.vl = [0.0, 0.0, 0.07]
        self.va = [0.0, 0.0, 0.0]

        # node rate sleep [Hz]
        self.p_rateSleep = 10

        # path to the folder where results are going to be stored
        self.p_expFolderPath = '/home/filipe/pCloud_sync/DOC/DOC/pratico/experimentos-estudos/2023-07-07_groundHeightValidationVicon/data'


        ###==== Useful variables

        self.gt_basePose_msg = None
        self.msg_grndDist = None
        self.msg_flpJntState = None

        ###==== ROS Interfaces
        # publishers
        self.pub_baseCmdVel = rospy.Publisher('/rosi/base/space/cmd_vel/autnav', TwistStamped, queue_size=5)

        # subscribers
        self.sub_gt_basePose = rospy.Subscriber('/vicon/rosi_base/rosi_base', TransformStamped, self.cllbck_gt_basePose)
        sub_grndDist = rospy.Subscriber('/rosi/model/base_ground_distance', Vector3ArrayStamped, self.cllbck_grndDist)
        sub_jointState = rospy.Subscriber('/rosi/rosi_controller/joint_state', JointState, self.cllbck_jointState)

        ##=== Main loop
        self.nodeMain()


    def nodeMain(self):
        '''Node main method'''
        
        # node rate sleep
        node_rate_sleep = rospy.Rate(self.p_rateSleep)      

        # log variable
        logData = []


        rospy.loginfo('[%s] Entering in ethernal loop.', self.node_name)
        while not rospy.is_shutdown():
             
            if self.gt_basePose_msg is not None and self.msg_grndDist  is not None and self.msg_flpJntState is not None:
                 
                time_ini = rospy.get_rostime()

                rospy.loginfo('[%s] Starting measurement.', self.node_name)
                while self.msg_grndDist.vec[0].z < 0.4:

                    # ros time
                    ros_time = rospy.get_rostime()

                    # experiment time spent
                    time_running = (ros_time - time_ini).to_sec()

                    # applies the test velocity
                    m = self.createTwistMsg(self.vl, self.va, ros_time)
                    self.pub_baseCmdVel.publish(m)

                    # obtaining the vicon marker pose in dq format
                    pose_marker_dq = twist2Dq(self.gt_basePose_msg)

                    # obtains {R} pose from the vicon marker pose
                    pose_r_dq_curr = getBasePoseFromMarkerDq(pose_marker_dq)

                    # extracting variables from the vicon pose
                    pose_r_tr = dqExtractTransV3(pose_r_dq_curr)

                    # extracting flipper joints position
                    flp_joint_pos = [x*y for x,y in zip(self.msg_flpJntState.position[4:], [1,-1,-1,1] ) ]

                    # mounting the log
                    logData.append( [time_running] + [self.vl[2]] + [self.msg_grndDist.vec[0].z] + [pose_r_tr[2][0]] + list(flp_joint_pos)  )

                    # sleeping the node
                    node_rate_sleep.sleep()

                rospy.loginfo('[%s] End of the experiment.', self.node_name)
                break
            
            # sleeping the node
            node_rate_sleep.sleep()

        # save file name
        today = date.today()
        now = datetime.now()
        d = today.strftime("%Y-%m-%d")
        n = now.strftime("%H-%M-%S")
        dateStr = d + '_' + n
        saveDataFolder = self.p_expFolderPath + '/' + dateStr
        rospy.loginfo('[%s] Saving data into %s', self.node_name, saveDataFolder)

        # creates the target directory if it does not exists
        if not os.path.exists(saveDataFolder):
            os.mkdir(saveDataFolder)

        # saving data
        csv_header = ['time', 'vel_z', 'pz model', 'pz vicon', 'flp1_pos', 'flp2_pos', 'flp3_pos', 'flp4_pos']
        with open(saveDataFolder+'/data_model.csv', "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(csv_header)
            writer.writerows(logData)


    def cllbck_gt_basePose(self, msg):
            '''Callback for the ROSI base twist'''
            self.gt_basePose_msg = msg


    def cllbck_grndDist(self, msg):
            '''Callback for received distance to the ground info'''
            # stores received distance to the ground as a 3D vector
            self.msg_grndDist = msg

    def cllbck_jointState(self, msg):
        '''Callback method for jointState topic'''
        self.msg_flpJntState = msg


    @staticmethod
    def createTwistMsg(vl, va, ros_time):
        '''Creates a Twist message'''
        m = TwistStamped()
        m.header.stamp = ros_time
        m.header.frame_id = 'autnav'
        m.twist.linear.x = vl[0]
        m.twist.linear.y = vl[1]
        m.twist.linear.z = vl[2]
        m.twist.angular.x = va[0]
        m.twist.angular.y = va[1]
        m.twist.angular.z = va[2] 
        return m

if __name__ == '__main__':
    node_name = 'exp_ground_height'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass