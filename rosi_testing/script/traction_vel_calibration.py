#!/usr/bin/env python3
'''this is a ROSI testing node
It receives ground-truth velocities of rosi base and joint states.
By applying rotational velocities to traction joints, it finds the function 
v_R = a * v_t + b, that relates base forward-linear velocity with traction joint velocities

'''
from time import strftime
from xml.dom.pulldom import parseString
import rospy

from controller.msg import Control
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

from rosi_common.rosi_tools import jointStateData2dict, ctrlType, correctTractionJointSignal

import numpy as np
import csv
import rospkg
import os
from datetime import datetime, date

class NodeClass():

    def __init__(self):
        '''Class constructor'''

        ##=== Parameters

        # time to apply each joint velocity command
        self.timeToApplySingleCommand = rospy.Duration.from_sec(4) # in seconds

        # minimal joint speed to test
        jointSpeedMin = 0.1

        # minimal joint speed to test
        jointSpeedMax = 4

        # number of points to test
        nPoints = 30

        # resolution of inner points
        jointSpeedTetstResolution = 0.2

        # file name to save
        saveFileBaseName = 'rosiSim'

        ##=== Useful variables

        self.tractionJointState = None
        self.baseLinVel = None

        # creating test joint speed lin space
        self.testJointVel_l = np.ndarray.tolist(np.linspace(jointSpeedMin, jointSpeedMax, nPoints))


        ##=== Handling saving file

        # save file name
        today = date.today()
        now = datetime.now()
        d = today.strftime("%Y-%m-%d")
        n = now.strftime("%H-%M-%S")
        saveFileName = saveFileBaseName + '_' + d + '_' + n + '.csv'

         # directory to save results
        saveDir = rospkg.RosPack().get_path('rosi_testing') + '/output/'

        # savepath
        self.savePath = saveDir + saveFileName

        # creates the target directory if it does not exists
        if not os.path.exists(saveDir):
            os.mkdir(saveDir)

        # deletes results csv if it exists
        if os.path.exists(self.savePath):
            os.remove(self.savePath)

        ##=== ROS interfaces

        # for sending joint velocities commands
        self.pub_cmdReq = rospy.Publisher('/rosi/controller/req_cmd', Control, queue_size=5)

        self.sub_jointState = rospy.Subscriber('/rosi/rosi_controller/joint_state', JointState, self.cllbck_jointState)
        self.sub_basePoseTwist = rospy.Subscriber('/rosi/cheat/pose_vel_rosi/twist', Twist, self.cllbck_basePoseTwist)

        self.nodeMain()


    def nodeMain(self):
        '''Node main method'''

        # node rate slee;
        node_rate_sleep = rospy.Rate(10)

        while not rospy.is_shutdown():

            # wait until valid messages has been received
            if self.tractionJointState is not None and self.baseLinVel is not None:

                result_l = []
                velMod = -1 # alternating velocity modulus

                # performing the test for each desired joint velocity to test
                for jointVel in self.testJointVel_l:

                    # applyed velocity alternates between positive and negative, so the robot does not go too far
                    velMod = 1 if velMod==-1 else -1

                    # performing the test
                    res = self.performCalTest(velMod*jointVel, self.timeToApplySingleCommand, node_rate_sleep)
                    
                    rospy.loginfo('Result for jvel: %f: linvel: %f'%(res[0],res[1]))
                    rospy.loginfo('---------')
                    result_l.append(res)

                # saves final result to csv
                rospy.loginfo('Writing results in %s'%self.savePath)
                with open(self.savePath, "w", newline="") as f:
                    writer = csv.writer(f)
                    writer.writerows(result_l)

                break

            # sleeps the node
            node_rate_sleep.sleep()

        # shutting down the node
        rospy.loginfo('End of calibrations tessst.')
        rospy.signal_shutdown('End of calibration test.')


    def cllbck_jointState(self, msg):
        '''Callback for the joint state message'''
        self.tractionJointState,_ = jointStateData2dict(msg)


    def cllbck_basePoseTwist(self, msg):
        '''Callback for the ROSI base twist'''
        self.baseLinVel = {'x':msg.linear.x, 'y':msg.linear.y, 'z':msg.linear.z}


    def applyTractionJointVel(self, jointVel):
        '''Method that receives a given jointVelocity and sends 
        commands to ROSI'''

        m = Control()
        m.header.stamp = rospy.get_rostime()
        m.header.frame_id = 'traction_vel_calibration'
        m.originId = 0
        m.modes = [ctrlType["Velocity"]]*4 + [ctrlType["Unchanged"]]*4
        m.data = correctTractionJointSignal([jointVel]*4) + [0.0]*4
        self.pub_cmdReq.publish(m)


    def performCalTest(self, jVel, timeSpanToRun, node_rate_sleep):
        '''Performs a calibration test. 
        It receives a joint velocity as input. Performs it for a given time and then
        returns base linear velocity mean as output'''
        
        time_ini = rospy.get_rostime()
        dt = rospy.Duration(0)

        rospy.loginfo(f'Applying velocity %f'%jVel)
        self.applyTractionJointVel(jVel)

        linVel_l = []

        while timeSpanToRun > dt:

            # computing the linear velocity norm and appending value to a list
            linVel = np.sqrt(pow(self.baseLinVel['x'],2) + pow(self.baseLinVel['y'],2) + pow(self.baseLinVel['z'],2))
            linVel_l.append(linVel)

            # updating time has passed from test init
            dt = rospy.get_rostime() - time_ini
            node_rate_sleep.sleep()

        # stopping the robot
        rospy.loginfo('Stopping the robot')
        self.applyTractionJointVel(0)
        rospy.sleep(1.)

        # computing linVel average value
        linVel_av = sum(linVel_l) / len(linVel_l)

        # returing test result
        return [abs(jVel), linVel_av]


if __name__ == '__main__':

    rospy.loginfo("traction_vel_calibration node initiated.")
    rospy.init_node("traction_vel_calibration", anonymous=True)

    try:
        node_obj = NodeClass()
    except rospy.ROSInternalException: pass