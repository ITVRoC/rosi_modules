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
from geometry_msgs.msg import TransformStamped

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

        # min joint speed to test
        jointSpeedMin = 0.1

        # max joint speed to test
        jointSpeedMax = 4

        # number of points to test
        nPoints = 30

        # resolution of inner points
        jointSpeedTetstResolution = 0.2

        # file name to save
        test_name = 'rosi_vicon'

        ##=== Useful variables
        self.tractionJointState_msg = None
        self.basePoseGT_msg = None


        # publishing useful variables
        self.m_null4Array = 4 * [0.0]
        self.m_modes = [ctrlType["Velocity"]]*4 + [ctrlType["Unchanged"]]*4

        # creating test joint speed lin space
        self.testJointVel_l = np.ndarray.tolist(np.linspace(jointSpeedMin, jointSpeedMax, nPoints))


        ##=== Handling saving file

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

        ##=== ROS interfaces

        # for sending joint velocities commands
        self.pub_cmdReq = rospy.Publisher('/rosi/controller/req_cmd', Control, queue_size=5)

        self.sub_jointState = rospy.Subscriber('/rosi/rosi_controller/joint_state', JointState, self.cllbck_jointState)
        self.sub_basePoseGT = rospy.Subscriber('/vicon/rosi_base/rosi_base', TransformStamped, self.cllbck_basePoseGT)

        self.nodeMain()


    def nodeMain(self):
        '''Node main method'''

        # node rate slee;
        node_rate_sleep = rospy.Rate(10)

        while not rospy.is_shutdown():

            # wait until valid messages has been received
            if self.tractionJointState_msg is not None and self.basePoseGT_msg is not None:

                result_l = []
                velMod = -1 # alternating velocity modulus

                # performing the test for each desired joint velocity to test
                for i, jointVel in enumerate(self.testJointVel_l):

                    # applied velocity alternates between positive and negative, so the robot does not go too far
                    velMod = 1 if velMod==-1 else -1

                    # messages to the user
                    rospy.loginfo('---------')
                    rospy.loginfo('[stage %d/%d] Applying joint angular speed: %.2f.', i+1, len(self.testJointVel_l), jointVel)

                    # performing the test
                    res = self.performCalTest(velMod*jointVel, self.timeToApplySingleCommand, node_rate_sleep)
                    
                    # messages to the user
                    rospy.loginfo('Chassis avg lin spd: %.2f', res[1])
                    
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
        #self.tractionJointState,_ = jointStateData2dict(msg)
        self.tractionJointState_msg = msg


    def cllbck_basePoseGT(self, msg):
        '''Callback for the ROSI base twist'''
        self.basePoseGT_msg = msg


    def applyTractionJointVel(self, jointVel, ros_time):
        '''Method that receives a given jointVelocity and sends 
        commands to ROSI'''
        
        tr_vel = correctTractionJointSignal([jointVel]*4).tolist()

        m = Control()
        m.header.stamp = ros_time
        m.header.frame_id = 'traction_vel_calibration'
        m.originId = 0
        m.modes = self.m_modes
        m.data = tr_vel + self.m_null4Array
        self.pub_cmdReq.publish(m)


    def performCalTest(self, jVel, timeSpanToRun, node_rate_sleep):
        '''Performs a calibration test. 
        It receives a joint velocity as input. Performs it for a given time and then
        returns base linear velocity mean as output'''

        # applies the velocity to joints before starting measurement so motion enters in steady state.
        rospy.loginfo('Applying joint velocity.')
        f4_time_spent = rospy.Duration(0)
        f4_time_ini = rospy.get_rostime()
        while f4_time_spent < rospy.Duration(2):
            self.applyTractionJointVel(jVel, rospy.get_rostime())
            node_rate_sleep.sleep()
            f4_time_spent = rospy.get_rostime() - f4_time_ini

        # initial time
        time_ini = rospy.get_rostime()
        test_time = rospy.Duration(0)

        # initial position
        pos_ini = self.getPosCurr()

        # useful variables
        delta_p_l = []
        linVel_l = []
        time_l = []

        # initializing last variables
        pos_last = pos_ini
        time_last = time_ini

        # perfroming the experiment
        rospy.loginfo('Initiating measurement...')
        while timeSpanToRun > test_time:

            # retrieving current ros time
            time_current = rospy.get_rostime()

            # applies current velocity to traction joints
            self.applyTractionJointVel(jVel, rospy.get_rostime())

            # obtaining the displacement norm
            pos_current = self.getPosCurr()
            delta_p_norm = np.linalg.norm(pos_current - pos_last)

            # computing the delta time
            dt = (time_current - time_last).to_sec()

            # computing the velocity norm
            linVel = delta_p_norm / dt

            # updating tables
            delta_p_l.append(delta_p_norm)
            time_l.append(time_current)
            linVel_l.append(linVel)

            # updating variables
            pos_last = pos_current
            time_last = time_current

            # updating passed time since test init
            test_time = time_current - time_ini
            node_rate_sleep.sleep()

        # stopping the robot
        rospy.loginfo('Stopping the robot')
        self.applyTractionJointVel(0, rospy.get_rostime())
        rospy.sleep(1.)

        # computing linVel average value
        linVel_av = sum(linVel_l) / len(linVel_l)

        # returning test result
        return [abs(jVel), linVel_av]
    

    def getPosCurr(self):
        '''Returns the base position as a numpy array
        Output
            - a <nd.array> element with the chassis base position'''
        return np.array([self.basePoseGT_msg.transform.translation.x, self.basePoseGT_msg.transform.translation.y,self.basePoseGT_msg.transform.translation.z])


if __name__ == '__main__':

    rospy.loginfo("traction_vel_calibration node initiated.")
    rospy.init_node("traction_vel_calibration", anonymous=True)

    try:
        node_obj = NodeClass()
    except rospy.ROSInternalException: pass