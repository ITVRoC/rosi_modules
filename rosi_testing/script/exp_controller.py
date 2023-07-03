#!/usr/bin/env python3
'''Node for performing the controller evaluation experiment'''
import rospy

import numpy as np

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
        self.ctrlTypeDes = "orientation"

        ##------- Home set-points
        # position home set-point
        self.sp_tr_home = [0.0, 0.0, 0.25]

        # orientation set-point
        self.sp_ori_home = np.deg2rad([0, 0, 0])

        # flipper joints mu function set-point
        self.sp_muF_home = np.deg2rad(110)

        # ground distance mu function set-point
        self.sp_muG_home = 0.25 # in [m]


        ##------- Pose set-points
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


        ##------- Pose control gains
        # orientation controller gain per DOF
        self.kp_rot_v = [2, 2, 0.0]

        # translation control gain per DOF
        self.kp_tr_v = [0.0, 0.0, 0.0]

        ##------- Flipper Mu function gain
        self.muF_kmu = 0.0

        ##------- Ground distance Mu function gain
        self.muG_kmu = 0.0



        ##------------- Runtime parameters ---------------------
        # node rate sleep [Hz]
        self.p_rateSleep = 25

        # chassis control service prefix
        cc_pr = '/chassis_control'

        # defaul sleep time after service calls
        self.slpSrvCll = 0.1

        

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
        self.sub_ctrl_CtrlGaiDq = rospy.Subscriber('/chassis_control/gain_dq', DualQuaternionStamped, self.cllbck_ctrl_CtrlGaiDq)


        ##=== Main loop
        self.nodeMain()


    def nodeMain(self):
        '''Node main method'''
        
        # node rate sleep
        node_rate_sleep = rospy.Rate(self.p_rateSleep)        
        
        # main loop
        while not rospy.is_shutdown():

            # only runs if a valid message has been received
            if self.gt_basePose_msg is not None and self.ctrl_basePose_msg is not None:
                
                # setting the controller to the desired type
                self.setCtrlType(self.chassisCtrlType[self.ctrlTypeDes])

                # setting controller gains
                self.setBulkGains(self.kp_tr_v, self.kp_rot_v, self.muF_kmu, self.muG_kmu)

                # sending the robot to home position
                rospy.loginfo('[%s] Going to Home pose.', self.node_name)
                self.setBulkSP(self.sp_tr_home, self.sp_ori_home, self.sp_muF_home, self.sp_muG_home) 

                # condition for going to another point
                rospy.sleep(3)

                # going to SP1
                rospy.loginfo('[%s] Going to P1.', self.node_name)
                self.setBulkSP(self.sp_tr['p1'], self.sp_ori['p1'], self.sp_muF['p1'], self.sp_muG['p1'])

                # condition for going to another point
                rospy.sleep(3)

                # going to SP2
                rospy.loginfo('[%s] Going go P2.', self.node_name)
                self.setBulkSP(self.sp_tr['p2'], self.sp_ori['p2'], self.sp_muF['p2'], self.sp_muG['p2'])

                # condition for going to another point
                rospy.sleep(3)

                # going to back SP1
                rospy.loginfo('[%s] Going back to P1.', self.node_name)
                self.setBulkSP(self.sp_tr['p1'], self.sp_ori['p1'], self.sp_muF['p1'], self.sp_muG['p1'])

                # condition for going to another point
                rospy.sleep(3)

                # sending the robot to home position
                rospy.loginfo('[%s] Going to Home pose.', self.node_name)
                self.setBulkSP(self.sp_tr_home, self.sp_ori_home, self.sp_muF_home, self.sp_muG_home) 

                rospy.loginfo('[%s] End of the experiment', self.node_name)

            
                break
         
        # sleeps the node
        node_rate_sleep.sleep()



    def cllbck_gt_basePose(self, msg):
        '''Callback for the ROSI base twist'''
        self.gt_basePose_msg = msg


    def cllbck_ctrl_basePose(self, msg):
        '''Callback for the controller estimated pose'''
        self.ctrl_basePose_msg = msg
        
    
    def cllbck_ctrl_basePoseSp(self, msg):
        '''Callback for the controller pose set-point'''
        self.ctrl_basePoseSp_msg = msg

    
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
    

    def setBulkGains(self, ktr, krot, kmuF, kmuG):
        '''Sends to the controller node desired gains
        Input:
            - ktr <list>: a 3-sized list containing the translation gain.
            - krot <list>: a 3-sized list containing the rotation gain.
            - kmuG <float>: the flipper mu function gain
            - kmuG <floag>: the ground distance mu function gain    
        '''
        
        # setting the controller control gain
        self.srvPrx_setPoseCtrlGain( setPoseCtrlGainRequest(ktr, krot) )

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



if __name__ == '__main__':
    node_name = 'exp_kinematics'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass