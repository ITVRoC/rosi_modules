#!/usr/bin/env python3
'''Node for performing the controller evaluation experiment

--> Para gravar com as vicon
rosbag record /vicon/rosi_base/rosi_base /rosi/rosi_controller/joint_state /sensor/imu_corrected /joy /rosi/base/space/cmd_vel /chassis_control/gain_dq /rosi/rosi_controller/input /rosi/controller/req_cmd /rosi/model/contact_point_wrt_base /rosi/model/grav_vec_wrt_frame_r /rosi/model/base_ground_distance /rosi/model/contact_plane_normal_vec /chassis_control/pose_current /chassis_control/pose_sp /chassis_control/pose_error /chassis_control/sp_dq /rosi/flippers/space/cmd_v_z/leveler /rosi/flippers/space/cmd_vel /rosi/flippers/space/cmd_v_z /rosi/flippers/joint/cmd_vel/leveler /rosi/flippers/status/touching_ground /rosi/flippers/joint/cmd_vel/touch_granter /rosi/flippers/joint/cmd_vel/sum /rosi/flippers/status/safety/max_pos_lock /rosi/traction/joint/cmd_vel/navigation /rosi/traction/joint/cmd_vel/compensator

--> Para gravar sem as vicon
rosbag record /rosi/rosi_controller/joint_state /sensor/imu_corrected /joy /rosi/base/space/cmd_vel /rosi/rosi_controller/input /chassis_control/gain_dq /rosi/controller/req_cmd /rosi/model/contact_point_wrt_base /rosi/model/grav_vec_wrt_frame_r /rosi/model/base_ground_distance /rosi/model/contact_plane_normal_vec /chassis_control/pose_current /chassis_control/pose_sp /chassis_control/pose_error /chassis_control/sp_dq /rosi/flippers/space/cmd_v_z/leveler /rosi/flippers/space/cmd_vel /rosi/flippers/space/cmd_v_z /rosi/flippers/joint/cmd_vel/leveler /rosi/flippers/status/touching_ground /rosi/flippers/joint/cmd_vel/touch_granter /rosi/flippers/joint/cmd_vel/sum /rosi/flippers/status/safety/max_pos_lock /rosi/traction/joint/cmd_vel/navigation /rosi/traction/joint/cmd_vel/compensator

'''
import rospy

import numpy as np
from dqrobotics import *
import matplotlib.pyplot as plt
from datetime import datetime, date
import os
import csv

from rosi_common.dq_tools import DualQuaternionStampedMsg2dq, dq2rpy, dqExtractTransV3, quat2rpy, trAndOri2dq, rpy2dq
from rosi_common.vicon_tools import getBasePoseFromMarkerDq

from rosi_common.msg import DualQuaternionStamped
from geometry_msgs.msg import TransformStamped

from rosi_common.srv import SetFloat, SetFloatRequest, SetInt, setPoseSetPointVec, setPoseCtrlGainRequest, setPoseCtrlGain, setPoseSetPointVecRequest

class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        ##==================  Parameters ==================

        ##-------------- Controller parameters -------------------------

        # desired control type
        self.ctrlTypeDes = "articulation" # possible values are 'orientation', 'orientationNullSpace_FlpJnt', 'orientationNullSpace_GrndHght', 'articulation'

        # experiment type
        self.p_ExperimentType = 'circle' # possible values are: 'step', 'circle'


        ##------- Controller gains ---------------
        # ---> translation gains
        # translation Proportional control gain per DOF
        self.kp_tr_v = [0.0, 0.0, 1.5]

        # translational Integrator control gain per DOF
        self.ki_tr_v = [0.0, 0.0, 0.0]


        # ---> Rotation gains
        # orientation Proportional controller gain per DOF
        self.kp_rot_v = [2.4, 2.2, 0.0]

        # orientation Integrator control gain per DOF
        self.ki_rot_v = [0.0, 0.0, 0.0]       

        
        #---> Mu functions gains
        # flipper Mu function gain
        self.muF_kmu = 0.26

        # ground distance Mu function gain
        self.muG_kmu = 0.9
        




        ##------- Home set-points -------------------
        # position home set-point
        self.sp_tr_home = [0.0, 0.0, 0.31]

        # orientation set-point
        self.sp_ori_home = np.deg2rad([0, 0, 0])

        # flipper joints mu function set-point
        self.sp_muF_home = np.deg2rad(130)

        # ground distance mu function set-point
        self.sp_muG_home = 0.31 # in [m]





        ##------- STEP EXPERIMENT PARAMETERS --- Step Experiment Pose parameters
        # dof to evaluate the error
        self.errorMit_dof = 'tr_z'  # possible values are: 'tr_z', 'rot_x', 'rot_y'

        # In the step experiment, the set-points sequency are home -> p1 -> p2 -> p1 -> home
        # The controller will go first to 'p1', then to 'p2', and finally returning to 'p1'
        # position set-point
        self.sp_tr = { # in [m]
            'p1': [0.0, 0.0, 0.22],
            'p2': [0.0, 0.0, 0.38]
        }

        # orientation set-point
        self.sp_ori = { # rpy in [rad]
            'p1': np.deg2rad([0, 0, 0]),
            'p2': np.deg2rad([0, 0, 0])
        }

        # flipper joints mu function set-point
        self.sp_muF = {
            'p1': np.deg2rad(130),
            'p2': np.deg2rad(130)
        }

        # ground distance mu function set-point
        self.sp_muG = {
            'p1': 0.31,
            'p2': 0.31
        }

        # time for wait within error range
        self.p_timeWithin = 5 #[s]

        # time for wait outside error range
        self.p_timeOutside = 10 #[s]



        ##------------- Runtime parameters ---------------------
        # node rate sleep [Hz]
        self.p_rateSleep = 30

        # chassis control service prefix
        cc_pr = '/chassis_control'
        
        # threshold for considering the error as acceptable
        self.threshold = {
            'tr_z': 0.01,
            'rot_x': np.deg2rad(1),
            'rot_y': np.deg2rad(1) 
        }





        #------------------ Saving/Plotting parameters -------------------
        # flag for saving or not the plottings
        self.p_flagSavingPic = False

        # path to the folder where results are going to be stored
        self.p_expFolderPath = '/home/filipe/pCloud_sync/DOC/DOC/pratico/experimentos-estudos/2023-07-03_controlLabVicon/ctrl_validation'

        # axes labels resolution
        self.p_yLabelRes = 12
        self.p_xLabelRes = 20

        # line width
        self.p_lw_model = 1.5
        self.p_lw_vicon = 1.0
        self.p_lw_sp = 1.2

        # Colors for plotting
        self.c1 = '#ff00ff'
        self.c2 = '#589f9c'
        self.c3 = '#7600be'
        self.c4 = '#bbaa00'
        self.c5 = '#009500'
        self.c_black = '#000000'
        self.c_gray = '#828282'
        self.c_blue = '#274381'
        self.c_red = '#9e1210'
        self.c_redLight = '#9c6566'



        ##==================== Useful variables ====================
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
        self.flag_logging = True

        # initial time of each received variable
        self.loggingTimeIni = {
            'model': None,
            'vicon': None,
            'sp': None
        }

        ##==================== ROS interfaces ====================
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


                # performs the desired experiment
                if self.p_ExperimentType == 'step':
                    self.expStep(node_rate_sleep)
                else:
                    self.expCircle(node_rate_sleep)
                rospy.loginfo('[%s] End of the experiment', self.node_name)


                # preparing the plot
                self.plotPrepare()

                #---> Saving figures and data
                if self.p_flagSavingPic:
                    self.savePlotAndData()


                # displays the figure
                plt.show()
                

                break

        # sleeps the node
        node_rate_sleep.sleep()


    def expStep(self, node_rate_sleep):
        '''Performs the step experiment.'''

        # sending the robot to home position
        rospy.loginfo('[%s] Going to Home pose.', self.node_name)
        self.setBulkSP(self.sp_tr_home, self.sp_ori_home, self.sp_muF_home, self.sp_muG_home) 

        # activating logging
        self.flag_logging = True

        # condition for going to another point
        self.waitErrorMitigation(node_rate_sleep, self.p_timeWithin, self.p_timeOutside)

        # going to SP1
        rospy.loginfo('[%s] Going to P1.', self.node_name)
        self.setBulkSP(self.sp_tr['p1'], self.sp_ori['p1'], self.sp_muF['p1'], self.sp_muG['p1'])

        self.waitErrorMitigation(node_rate_sleep, self.p_timeWithin, self.p_timeOutside)

        # going to SP2
        rospy.loginfo('[%s] Going go P2.', self.node_name)
        self.setBulkSP(self.sp_tr['p2'], self.sp_ori['p2'], self.sp_muF['p2'], self.sp_muG['p2'])

        # condition for going to another point
        self.waitErrorMitigation(node_rate_sleep, self.p_timeWithin, self.p_timeOutside)

        # going to back SP1
        rospy.loginfo('[%s] Going back to P1.', self.node_name)
        self.setBulkSP(self.sp_tr['p1'], self.sp_ori['p1'], self.sp_muF['p1'], self.sp_muG['p1'])

        # condition for going to another point
        self.waitErrorMitigation(node_rate_sleep, self.p_timeWithin, self.p_timeOutside)

        # sending the robot to home position
        rospy.loginfo('[%s] Going to Home pose.', self.node_name)
        self.setBulkSP(self.sp_tr_home, self.sp_ori_home, self.sp_muF_home, self.sp_muG_home) 

        # condition for going to another point
        self.waitErrorMitigation(node_rate_sleep, self.p_timeWithin, self.p_timeOutside)

        # deactivating logging
        self.flag_logging = False



    def expCircle(self, node_rate_sleep):
        
        # parameters
        n_points = 100
        circAmplitude = np.deg2rad(15)       # in [rad]
        heightAmplitude = 0.0               # in [m] 
        timeNextPoint = rospy.Duration.from_sec(0.1)                  # in [s]

        # creates the circle set-point for each dof
        iter_l = np.linspace(0, 2*np.pi, n_points)
        angx_sp_l = circAmplitude * np.cos(iter_l)
        angy_sp_l = circAmplitude * np.sin(iter_l)
        trz_sp_l = (heightAmplitude * np.sin(iter_l) ) + self.sp_tr_home[2]

        # sending the robot to home position
        rospy.loginfo('[%s] Going to Home pose.', self.node_name)
        self.setBulkSP(self.sp_tr_home, self.sp_ori_home, self.sp_muF_home, self.sp_muG_home) 
        self.waitErrorMitigation(node_rate_sleep, 5, 20)

        # activating logging
        self.flag_logging = True

        # performs the experiment
        rospy.loginfo('[%s] Starting the tracking experiment.', self.node_name)
        for sp_trz, sp_rotx, sp_roty, i in zip(trz_sp_l, angx_sp_l, angy_sp_l, range(n_points)):

            # sends current set-point
            rospy.loginfo('[%s] Sending point set-point %d/%d.', self.node_name, i+1, n_points)
            self.setBulkSP([0,0,sp_trz], [sp_rotx, sp_roty, 0], self.sp_muF_home, self.sp_muG_home)

            # sleeps for the specified time
            time_ini = rospy.get_rostime()
            while rospy.get_rostime() - time_ini < timeNextPoint:
                node_rate_sleep.sleep()
            
        # deactivating logging
        self.flag_logging = False


    def plotPrepare(self):
        
        fig, axes = plt.subplots(3,1, sharex=True)

        #---> rot x
        axes[0].plot(self.log['model_time'], np.rad2deg( self.log['model_rot_x']  ), color=self.c_blue, label='ctrl', lw=self.p_lw_model)
        axes[0].plot(self.log['vicon_time'], np.rad2deg(  self.log['vicon_rot_x']  ), color=self.c_redLight, linestyle='dashed', label='vicon', lw=self.p_lw_vicon)
        axes[0].plot(self.log['sp_time'], np.rad2deg(  self.log['sp_rot_x']  ), color=self.c_black, linestyle='dashed', label='sp', lw=self.p_lw_sp)

        axes[0].set_title(  'rot x  -  kp:'+format(self.kp_rot_v[0], '.2f')+',  ki:'+format(self.ki_rot_v[0], '.2f')+', kmuFlp:'+format(self.muF_kmu, '.2f')+', kmuGrn:'+format(self.muG_kmu,'.2f')  )
        axes[0].set_ylabel('angle [deg]', color=self.c_gray)

        x_locator = axes[0].xaxis.get_major_locator()
        x_locator.set_params(nbins=self.p_xLabelRes)

        y_locator = axes[0].yaxis.get_major_locator()
        y_locator.set_params(nbins=self.p_yLabelRes)

        


        #---> rot y
        axes[1].plot(self.log['model_time'], np.rad2deg(  self.log['model_rot_y']  ), color=self.c_blue, label='ctrl', lw=self.p_lw_model) # converting log data to degrees
        axes[1].plot(self.log['vicon_time'], np.rad2deg(  self.log['vicon_rot_y']  ), color=self.c_redLight, linestyle='dashed', label='vicon', lw=self.p_lw_vicon) # converting log data to degrees
        axes[1].plot(self.log['sp_time'], np.rad2deg(  self.log['sp_rot_y']  ), color=self.c_black, linestyle='dashed', label='sp', lw=self.p_lw_sp)

        axes[1].set_title(  'rot y  -  kp:'+format(self.kp_rot_v[1], '.2f')+',  ki:'+format(self.ki_rot_v[1], '.2f')  )
        axes[1].set_ylabel('angle [deg]', color=self.c_gray)

        x_locator = axes[1].xaxis.get_major_locator()
        x_locator.set_params(nbins=self.p_xLabelRes)

        y_locator = axes[1].yaxis.get_major_locator()
        y_locator.set_params(nbins=self.p_yLabelRes)
        

        #---> tr z
        axes[2].plot(self.log['model_time'], 100 * np.array( self.log['model_tr_z'] ), color=self.c_blue, label='ctrl', lw=self.p_lw_model) # converting log data to centimeters
        axes[2].plot(self.log['vicon_time'], 100 * np.array( self.log['vicon_tr_z'] ), color=self.c_redLight, linestyle='dashed', label='vicon', lw=self.p_lw_vicon) # converting log data to centimeters
        axes[2].plot(self.log['sp_time'], 100 * np.array( self.log['sp_tr_z'] ), color=self.c_black, linestyle='dashed', label='sp', lw=self.p_lw_sp)

        axes[2].set_title(  'tr z  -  kp:'+format(self.kp_tr_v[2], '.2f')+',  ki:'+format(self.ki_tr_v[2], '.2f')  )
        axes[2].set_ylabel('distance [cm]', color=self.c_gray)
        axes[2].set_xlabel('time [s]', color=self.c_gray)

        x_locator = axes[2].xaxis.get_major_locator()
        x_locator.set_params(nbins=self.p_xLabelRes)

        y_locator = axes[2].yaxis.get_major_locator()
        y_locator.set_params(nbins=self.p_yLabelRes)

        #---> plotting parameters
        plt.tight_layout(pad=1.5, w_pad=0.5, h_pad=1.0)
        for ax in axes.flatten():
            ax.grid(True, color='gray', linestyle='--', linewidth=0.1)


    def savePlotAndData(self):
        #--> preamble
        # obtaining the date string for saving
        # save file name
        today = date.today()
        now = datetime.now()
        d = today.strftime("%Y-%m-%d")
        n = now.strftime("%H-%M-%S")
        dateStr = d + '_' + n
        saveDataFolder = self.p_expFolderPath + '/' + dateStr

        # creates the target directory if it does not exists
        if not os.path.exists(saveDataFolder):
            os.mkdir(saveDataFolder)

        rospy.loginfo('[%s] Saving figures and data to %s', self.node_name, saveDataFolder)

        #--> saving figures

        # saving in high-res eps format
        plt.savefig(saveDataFolder+'/plot.svg', format='svg')

        # saving in png format
        plt.savefig(self.p_expFolderPath+'/'+dateStr+'.png', dpi=300, format='png')


        #--> saving data

        # header of csv file
        csv_header = [s for s in self.log.keys()]

        # csv data to export
        csv_data_model = [[time, x, y, z] for time, x, y, z in zip(  self.log['model_time'], self.log['model_rot_x'], self.log['model_rot_y'] , self.log['model_tr_z']   )]
        csv_data_vicon = [[time, x, y, z] for time, x, y, z in zip(  self.log['vicon_time'], self.log['vicon_rot_x'], self.log['vicon_rot_y'] , self.log['vicon_tr_z']   )]
        csv_data_sp = [[time, x, y, z] for time, x, y, z in zip(  self.log['sp_time'], self.log['sp_rot_x'], self.log['sp_rot_y'] , self.log['sp_tr_z']   )]

        # saving model data
        with open(saveDataFolder+'/data_model.csv', "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(csv_header[0:4])
            writer.writerows(csv_data_model)

        # saving vicon data
        with open(saveDataFolder+'/data_vicon.csv', "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(csv_header[4:8])
            writer.writerows(csv_data_vicon)

        # saving sp data
        with open(saveDataFolder+'/data_sp.csv', "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(csv_header[8:])
            writer.writerows(csv_data_sp)



    def cllbck_gt_basePose(self, msg):
        '''Callback for the ROSI base twist measured by Vicon'''
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
            self.log['vicon_tr_z'].append(tr[2][0])


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
            self.log['model_tr_z'].append(tr[2][0])

    
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
            self.log['sp_tr_z'].append(tr[2][0])


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


    def waitErrorMitigation(self, node_rate_sleep, timeWithinError, timeMax):
        '''Forces the code to wait until the error is within an acceptable range'''

        # converting times
        timeWithinError = rospy.Duration().from_sec(timeWithinError)
        timeMax = rospy.Duration(timeMax)

        # ini time of waiting
        time_ini = self.ctrl_basePoseError.header.stamp

        # initializing variables
        time_windowIni = time_ini

        while not rospy.is_shutdown():
                
            # current time
            time_curr = self.ctrl_basePoseError.header.stamp

            # evaluating if the max time of waiting has been achieved
            if time_curr - time_ini > timeMax:
                break

            # extracting error components
            auxdq = DualQuaternionStampedMsg2dq(self.ctrl_basePoseError)
            rpy = dq2rpy(auxdq)
            tr = dqExtractTransV3(auxdq)

            # figures out the current error variable and its threshold
            if self.errorMit_dof == 'tr_z':
                e = tr[2]
                threshold = self.threshold['tr_z']
                rospy.loginfo("error curr: %.3f cm,  error thr: %.3f cm", 100 * np.abs(e), 100 * threshold )

            elif self.errorMit_dof == 'rot_x':
                e = rpy[0]
                threshold = self.threshold['rot_x']
                rospy.loginfo("[%s] error curr: %.2f deg,  error thr: %.2f deg", self.node_name, np.rad2deg( np.abs(e) ), np.rad2deg( threshold ) )

            elif self.errorMit_dof == 'rot_y':
                e = rpy[1]
                threshold = self.threshold['rot_y']
                rospy.loginfo("[%s] error curr: %.2f deg,  error thr: %.2f deg", self.node_name, np.rad2deg( np.abs(e) ), np.rad2deg( threshold ) )
            
            
            # if the error is above the threshold, the error time count zerates
            if np.abs(e) > threshold:
                time_windowIni = time_curr

            # in case of the error is below the threshold
            else:

                rospy.loginfo('->>')
                # verifies if the error is below a threshold for long enough
                if time_curr - time_windowIni > timeWithinError:
                    rospy.loginfo('[%s] error objective achieved.', self.node_name)
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