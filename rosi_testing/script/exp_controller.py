#!/usr/bin/env python3
'''Node for performing the controller evaluation experiment

rosbag record /chassis_control/pose_current /chassis_control/pose_sp /chassis_control/pose_error /chassis_control/gain_dq /rosi/flippers/space/cmd_v_z/leveler /rosi/propulsion/space/cmd_vel /rosi/flippers/space/cmd_v_z /rosi/flippers/joint/cmd_vel/leveler /rosi/flippers/joint/cmd_vel/touch_granter /rosi/flippers/joint/cmd_vel/sum /rosi/rosi_controller/input /rosi/traction/joint/cmd_vel/navigation /rosi/base/space/cmd_vel /rosi/base/space/cmd_vel/autnav /rosi/base/space/cmd_vel/joy /joy /mti/sensor/imu /sensor/imu_corrected /rosi/rosi_controller/joint_state /rosi/model/contact_point_wrt_pi /rosi/model/contact_point_wrt_base /rosi/model/grav_vec_wrt_frame_r /rosi/model/flipper_tip_wrt_pi /rosi/model/base_ground_distance /rosi/model/contact_plane_normal_vec /vicon/rosi_base/rosi_base

'''
import rospy

import numpy as np
from dqrobotics import *
import matplotlib.pyplot as plt


from rosi_common.dq_tools import DualQuaternionStampedMsg2dq, dq2rpy, dqExtractTransV3, quat2rpy, trAndOri2dq
from rosi_common.vicon_tools import getBasePoseFromMarkerDq


from rosi_common.msg import DualQuaternionStamped
from geometry_msgs.msg import TransformStamped

from rosi_common.srv import SetFloat, SetFloatRequest, SetInt, setPoseSetPointVec, setPoseCtrlGainRequest, setPoseCtrlGain, setPoseSetPointVecRequest

class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        ##=== Parameters

        ##-------------- Controller parameters -------------------------

        # desired control type
        self.ctrlTypeDes = "articulation"

        # dof to evaluate the error
        self.errorMit_dof = 'rot_y'  # possible values are 'tr_z', 'rot_x' and 'rot_y'


        ##------- Experiment Pose set-points
        # The controller will go first to 'p1', then to 'p2', and finally returning to 'p1'
        # position set-point
        self.sp_tr = { # in [m]
            'p1': [0.0, 0.0, 0.3],
            'p2': [0.0, 0.0, 0.3]
        }

         # orientation set-point
        self.sp_ori = { # rpy in [rad]
            'p1': np.deg2rad([-20, 0, 0]),
            'p2': np.deg2rad([20, 0, 0])
        }

        # flipper joints mu function set-point
        self.sp_muF = {
            'p1': np.deg2rad(110),
            'p2': np.deg2rad(110)
        }

        # ground distance mu function set-point
        self.sp_muG = {
            'p1': 0.25,
            'p2': 0.25
        }


        ##------- Pose control gains ---------------

        #---> Proportional gains
        # translation Proportional control gain per DOF
        self.kp_tr_v = [0.0, 0.0, 0.0]

        # orientation Proportional controller gain per DOF
        self.kp_rot_v = [2, 0, 0]

        
        #---> Integrator gains
        # translational Integrator control gain per DOF
        self.ki_tr_v = [0.0, 0.0, 0.0]

        # orientation Integrator control gain per DOF
        self.ki_rot_v = [0.0, 0.0, 0.0]


        #---> Mu functions gains
        # flipper Mu function gain
        self.muF_kmu = 0.0

        # ground distance Mu function gain
        self.muG_kmu = 0.0



        ##------- Home set-points -------------------
        # position home set-point
        self.sp_tr_home = [0.0, 0.0, 0.25]

        # orientation set-point
        self.sp_ori_home = np.deg2rad([0, 0, 0])

        # flipper joints mu function set-point
        self.sp_muF_home = np.deg2rad(110)

        # ground distance mu function set-point
        self.sp_muG_home = 0.25 # in [m]



        ##------------- Runtime parameters ---------------------
        # node rate sleep [Hz]
        self.p_rateSleep = 25

        # chassis control service prefix
        cc_pr = '/chassis_control'

        # defaul sleep time after service calls
        self.slpSrvCll = 0.1

        # time window that the error should be below the threshold so we consider that the objective has been atteint
        self.error_time_window = rospy.Duration.from_sec(3)

        # max time for waiting the controller to reach the set-point
        self.error_time_max = rospy.Duration.from_sec(10)
        
        # threshold for considering the error as acceptable
        self.threshold = {
            'tr_z': 0.001,
            'rot_x': np.deg2rad(2),
            'rot_y': np.deg2rad(2) 
        }


        #------------------ Plotting parameters -------------------

        # Colors for plotting
        self.c1 = '#ff00ff'
        self.c2 = '#589f9c'
        self.c3 = '#7600be'
        self.c4 = '#7600be'
        self.c5 = '#bbaa00'
        self.c6 = '#009500'

        ##=== Useful variables

        self.gt_basePose_msg = None
        self.ctrl_basePose_msg = None
        self.ctrl_basePoseSp_msg = None
        self.ctrl_basePoseError = None
        self.cllbck_ctrl_CtrlGaiDq = None

        # Available control types
        self.chassisCtrlType = {
            "orientation": 1,
            "orientationNullSpace_FlpJnt": 2,
            "orientationNullSpace_GrndHght": 3,
            "articulation": 4
        }

        # log variable
        self.log = {
            'model_time': [],
            'model_rot_x': [],
            'model_rot_y': [],
            'model_tr_z': [],
            'vicon_time': [],
            'vicon_rot_x': [],
            'vicon_rot_y': [],
            'vicon_tr_z': [],
            'sp_time': [],
            'sp_rot_x': [],
            'sp_rot_y': [],
            'sp_tr_z': []
        }

        # flag indicating if is the first log to each logged callback
        self.flag_logFirstRun = {
            'model': True,
            'vicon': True,
            'sp': True
        }

        # flag indicating if logging is active
        self.flag_logging = False

        # initial time of each received variable
        self.loggingTimeIni = {
            'model': None,
            'vicon': None,
            'sp': None
        }

        ##=== ROS interfaces
        # publishers

        # services
        self.srvPrx_setCtrlType = rospy.ServiceProxy(cc_pr+'/set_ctrl_type', SetInt)

        self.srvPrx_setPoseSp = rospy.ServiceProxy(cc_pr+'/set_pose_set_point', setPoseSetPointVec)
        self.srvPrx_setPoseCtrlGain = rospy.ServiceProxy(cc_pr+'/set_pose_ctrl_gain', setPoseCtrlGain)

        self.srvPrx_setMuFlp_jntSp = rospy.ServiceProxy(cc_pr+'/set_mu_flp_set_point', SetFloat)
        self.srvPrx_setMuFlp_gain = rospy.ServiceProxy(cc_pr+'/set_mu_flp_gain', SetFloat)
        
        self.srvPrx_setMuGrndDst_dstSp = rospy.ServiceProxy(cc_pr+'/set_mu_grndist_set_point', SetFloat)
        self.srvPrx_setMuGrndDst_gain = rospy.ServiceProxy(cc_pr+'/set_mu_grndist_gain', SetFloat)


        # subscribers
        self.sub_gt_basePose = rospy.Subscriber('/vicon/rosi_base/rosi_base', TransformStamped, self.cllbck_gt_basePose)
        self.sub_ctrl_basePose = rospy.Subscriber('/chassis_control/pose_current', DualQuaternionStamped, self.cllbck_ctrl_basePose)
        self.sub_ctrl_basePoseSp = rospy.Subscriber('/chassis_control/pose_sp', DualQuaternionStamped, self.cllbck_ctrl_basePoseSp)
        self.sub_ctrl_basePoseError = rospy.Subscriber('/chassis_control/pose_error', DualQuaternionStamped, self.cllbck_ctrl_basePoseError)
        #self.sub_ctrl_CtrlGaiDq = rospy.Subscriber('/chassis_control/gain_dq', DualQuaternionStamped, self.cllbck_ctrl_CtrlGaiDq)


        ##=== Main loop
        self.nodeMain()


    def nodeMain(self):
        '''Node main method'''
        
        # node rate sleep
        node_rate_sleep = rospy.Rate(self.p_rateSleep)        

        # main loop
        while not rospy.is_shutdown():

            # only runs if a valid message has been received
            if self.ctrl_basePoseError is not None and self.gt_basePose_msg  is not None and self.ctrl_basePose_msg is not None:
                
                # setting the controller to the desired type
                self.setCtrlType(self.chassisCtrlType[self.ctrlTypeDes])

                # setting controller gains
                self.setBulkGains(self.kp_tr_v, self.kp_rot_v, self.ki_tr_v, self.ki_rot_v, self.muF_kmu, self.muG_kmu)

                # sending the robot to home position
                rospy.loginfo('[%s] Going to Home pose.', self.node_name)
                self.setBulkSP(self.sp_tr_home, self.sp_ori_home, self.sp_muF_home, self.sp_muG_home) 

                # activating logging
                self.flag_logging = True

                # condition for going to another point
                self.waitErrorMitigation(node_rate_sleep)


                # going to SP1
                rospy.loginfo('[%s] Going to P1.', self.node_name)
                self.setBulkSP(self.sp_tr['p1'], self.sp_ori['p1'], self.sp_muF['p1'], self.sp_muG['p1'])

                self.waitErrorMitigation(node_rate_sleep)

                # going to SP2
                rospy.loginfo('[%s] Going go P2.', self.node_name)
                self.setBulkSP(self.sp_tr['p2'], self.sp_ori['p2'], self.sp_muF['p2'], self.sp_muG['p2'])

                # condition for going to another point
                self.waitErrorMitigation(node_rate_sleep)

                # going to back SP1
                rospy.loginfo('[%s] Going back to P1.', self.node_name)
                self.setBulkSP(self.sp_tr['p1'], self.sp_ori['p1'], self.sp_muF['p1'], self.sp_muG['p1'])

                # condition for going to another point
                self.waitErrorMitigation(node_rate_sleep)

                # deactivating logging
                self.flag_logging = False

                # sending the robot to home position
                rospy.loginfo('[%s] Going to Home pose.', self.node_name)
                self.setBulkSP(self.sp_tr_home, self.sp_ori_home, self.sp_muF_home, self.sp_muG_home) 

                rospy.loginfo('[%s] End of the experiment', self.node_name)

                ##--- plotting results
                fig, axes = plt.subplots(3,1)

                axes[0].set_title('rot x')
                axes[0].plot(self.log['model_time'], np.rad2deg( self.log['model_rot_x']  ), color=self.c1)
                axes[0].plot(self.log['vicon_time'], np.rad2deg(  self.log['vicon_rot_x']  ), color=self.c2)
                axes[0].plot(self.log['sp_time'], np.rad2deg(  self.log['sp_rot_x']  ), color=self.c3, linestyle='dashed')
                

                axes[1].set_title('rot y')
                axes[1].plot(self.log['model_time'], np.rad2deg(  self.log['model_rot_y']  ), color=self.c1)
                axes[1].plot(self.log['vicon_time'], np.rad2deg(  self.log['vicon_rot_y']  ), color=self.c2)
                axes[1].plot(self.log['sp_time'], np.rad2deg(  self.log['sp_rot_y']  ), color=self.c3, linestyle='dashed')
                
                axes[2].set_title('tr z')
                axes[2].plot(self.log['model_time'], self.log['model_tr_z'], color=self.c1)
                axes[2].plot(self.log['vicon_time'], self.log['vicon_tr_z'], color=self.c2)
                axes[2].plot(self.log['sp_time'], self.log['sp_tr_z'], color=self.c3, linestyle='dashed')

                plt.tight_layout()
                plt.grid(True, color='gray', linestyle='--', linewidth=0.1)
                plt.show()
                


                break
         
        # sleeps the node
        node_rate_sleep.sleep()



    def cllbck_gt_basePose(self, msg):
        '''Callback for the ROSI base twist'''
        self.gt_basePose_msg = msg

        # logs received data
        if self.flag_logging:

            # saving first logged time
            if self.flag_logFirstRun['vicon']:
                self.flag_logFirstRun['vicon'] = False
                self.loggingTimeIni['vicon'] = msg.header.stamp
            
            # creates a dq from received values
            dq = trAndOri2dq([msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z],
                             [msg.transform.rotation.w, msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z],
                             'trfirst')
            
            # transforms the marker pose to {R}
            dq = getBasePoseFromMarkerDq(dq)

            # extracts variables from dq
            tr = dqExtractTransV3(dq)
            rpy = dq2rpy(dq)

            # logs variables
            self.log['vicon_time'].append(  (msg.header.stamp - self.loggingTimeIni['vicon']).to_sec()  )
            self.log['vicon_rot_x'].append(rpy[0])
            self.log['vicon_rot_y'].append(rpy[1])
            self.log['vicon_tr_z'].append(tr[2])



    def cllbck_ctrl_basePose(self, msg):
        '''Callback for the controller estimated pose'''
        self.ctrl_basePose_msg = msg

        # logs received data
        if self.flag_logging:

            # saving first logged time
            if self.flag_logFirstRun['model']:
                self.flag_logFirstRun['model'] = False
                self.loggingTimeIni['model'] = msg.header.stamp
            
            # creating a DQ element with received msg
            dq = DQ(msg.wp, msg.xp, msg.yp, msg.zp, msg.wd, msg.xd, msg.yd, msg.zd)

            # extracts variable from dq
            tr = dqExtractTransV3(dq)
            rpy = dq2rpy(dq)

            # logs variables
            self.log['model_time'].append(  (msg.header.stamp - self.loggingTimeIni['model']).to_sec()  )
            self.log['model_rot_x'].append(rpy[0])
            self.log['model_rot_y'].append(rpy[1])
            self.log['model_tr_z'].append(tr[2])


    
    def cllbck_ctrl_basePoseSp(self, msg):
        '''Callback for the controller pose set-point'''
        self.ctrl_basePoseSp_msg = msg

        # logs received data
        if self.flag_logging:

            # saving first logged time
            if self.flag_logFirstRun['sp']:
                self.flag_logFirstRun['sp'] = False
                self.loggingTimeIni['sp'] = msg.header.stamp

            # creating a DQ element with received msg
            dq = DQ(msg.wp, msg.xp, msg.yp, msg.zp, msg.wd, msg.xd, msg.yd, msg.zd)

            # extracts variable from dq
            tr = dqExtractTransV3(dq)
            rpy = dq2rpy(dq)

            # logs variables
            self.log['sp_time'].append(  (msg.header.stamp - self.loggingTimeIni['sp']).to_sec()  )
            self.log['sp_rot_x'].append(rpy[0])
            self.log['sp_rot_y'].append(rpy[1])
            self.log['sp_tr_z'].append(tr[2])


    
    def cllbck_ctrl_basePoseError(self, msg):
        '''Callback for the controller pose error'''
        self.ctrl_basePoseError = msg

    
    def cllbck_ctrl_CtrlGaiDq(self, msg):
        '''Callback for the controller gain dq'''
        self.cllbck_ctrl_CtrlGaiDq = msg

    
    def setCtrlType(self, ctrlType):
        '''Sends to the controller node the desired controller type'''
        ret = self.srvPrx_setCtrlType(ctrlType)
        if ret.ret !=  ctrlType:
            rospy.logerr('[%s] wrong desired control type informed.')
        return ret.ret
    

    def setBulkGains(self, kp_tr, kp_rot, ki_tr, ki_rot, kmuF, kmuG):
        '''Sends to the controller node desired gains
        Input:
            - ktr <list>: a 3-sized list containing the translation gain.
            - krot <list>: a 3-sized list containing the rotation gain.
            - kmuG <float>: the flipper mu function gain
            - kmuG <floag>: the ground distance mu function gain    
        '''
        
        # setting the controller control gains
        aux = setPoseCtrlGainRequest()
        aux.kp_tr = kp_tr
        aux.kp_ori = kp_rot
        aux.ki_tr = ki_tr
        aux.ki_ori = ki_rot
        self.srvPrx_setPoseCtrlGain(aux)

        # setting the flipper mu function gain
        self.srvPrx_setMuFlp_gain( SetFloatRequest(kmuF) )

        # setting the ground disttance mu function gain
        self.srvPrx_setMuGrndDst_gain( SetFloatRequest(kmuG) )


    def setBulkSP(self, trsp, orisp, muFsp, muGsp):
        '''Sends to the controller node pose and mu function set-points
        Input:
            - trsp <list>: a 3-sized list containing translation set-points in [m]
            - orisp <list>: a 3-sized list containing orientation set-points as a rpy array in [rad]
            - muFsp <float>: the flippers joint set-point angle for the flippers mu function 
            - muGsp <float>: the ground distance set-point for the ground-distance mu function
        '''

        # sending to the controller node the pose set-point
        self.srvPrx_setPoseSp( setPoseSetPointVecRequest(trsp, orisp) )

        # sending to the controller node the Flipper mu function set-point
        self.srvPrx_setMuFlp_jntSp(muFsp)

        # sending to the controller node the Ground Distance mu function set-point
        self.srvPrx_setMuGrndDst_dstSp(muGsp)


    def waitErrorMitigation(self, node_rate_sleep):
        '''Forces the code to wait until the error is within an acceptable range'''

        # ini time of waiting
        time_ini = self.ctrl_basePoseError.header.stamp

        # initializing variables
        time_windowIni = time_ini

        while not rospy.is_shutdown():
                
            # current time
            time_curr = self.ctrl_basePoseError.header.stamp

            # evaluating if the max time of waiting has been achieved
            if time_curr - time_ini > self.error_time_max:
                break

            # extracting error components
            auxdq = DualQuaternionStampedMsg2dq(self.ctrl_basePoseError)
            rpy = dq2rpy(auxdq)
            tr = dqExtractTransV3(auxdq)

            # figures out the current error variable and its threshold
            if self.errorMit_dof == 'tr_z':
                e = tr[0]
                threshold = self.threshold['tr_z']
            elif self.errorMit_dof == 'rot_x':
                e = rpy[0]
                threshold = self.threshold['rot_x']
            elif self.errorMit_dof == 'rot_y':
                e = rpy[1]
                threshold = self.threshold['Zrot_y']


            # if the error is above the threshold, the error time count zerates
            if e > threshold:
                time_windowIni = time_curr

            # in case of the error is below the threshold
            else:
                # verifies if the error is below a threshold for long enough
                if time_curr - time_windowIni > self.error_time_window:
                    #rospy.loginfo('[%s] error objective achieved.', self.node_name)
                    break

            # sleeps the node
            node_rate_sleep.sleep()


if __name__ == '__main__':
    node_name = 'exp_kinematics'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass