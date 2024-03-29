#!/usr/bin/env python3
''' This is a ROSI algorithm
It controls the chassis
based on different control strategies
'''
import rospy
import numpy as np
import quaternion
from dqrobotics import *

from rosi_common.dq_tools import quat2rpy, rpy2quat, trAndOri2dq, dqElementwiseMul, dq2trAndQuatArray, dqExtractTransV3, dq2rpy, quatAssurePosW, dq2DualQuaternionStampedMsg, dq2trAndQuatArray
from rosi_common.rosi_tools import correctFlippersJointSignal, compute_J_ori_dagger, compute_J_art_dagger
from rosi_common.node_status_tools import nodeStatus
from rosi_model.rosi_description import tr_base_piFlp
from rosi_common.math_tools import quatExpFromMatlab

from rosi_common.msg import  Vector3ArrayStamped, DualQuaternionStamped
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Vector3

from rosi_common.srv import SetNodeStatus, GetNodeStatusList, setPoseSetPointVec, setPoseCtrlGain, getPoseCtrlGain, getPoseSetPointVec, SetInt, SetIntResponse, GetInt, GetIntResponse, SetFloat, SetFloatResponse, GetFloat, GetFloatResponse, getPoseCtrlGainResponse


class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        ##==== PARAMETERS ============================================

        # node rate sleep [Hz]
        self.p_rateSleep = 50

        #----------------- SET-POINT ------------------------------
        # orientation
        x_sp_ori_rpy = np.deg2rad([0, 0, 0]) # final vector is in radians RPY
        
        # ground distance
        x_sp_tr = [0.0, 0.0, 0.3] # in meters
       

        #----------------- CONTROL GAINS ------------------------------
        #--> Translational gains
        # translation control gain per DOF
        kp_tr_v = [0.0, 0.0, 1.5]

        # translation control gain per DOF
        ki_tr_v = [0.0, 0.0, 0.0]


        #--> Rotation gains
        # orientation controller Proportional gain per DOF
        kp_rot_v = [2.4, 2.2, 0.0]

        # orientation controller Integrative gain per DOF
        ki_rot_v = [0.0, 0.0, 0.0] 



        #------ Mu function for the Flippers lever angle optimization function
        # propulsion joints angular set-point for the null-space
        self.muf_flpJointPosSp_l = 4*[np.deg2rad(130)]

        # mu function gain
        self.muf_kmu_l = 4*[0.26]



        #------ Mu function for the Chassis ground height optimization function
        # ground distance set-point
        self.mug_grndDstncSp_l = 0.85
        
        # mu function gain 
        self.mug_kmu_l = 0.9
    



        ##=== Controller parameters
        #----- Method for generating the control signal
        # this variable dictactes how the proportional and integrative control signal are generates
        self.oriCtrlPropSigMthd = 'quaternion' # possible values are 'rpy' and 'quaternion'

        #------ Error integration parameters
        # orientation error integration method
        self.ctrlntgrMthd = 'rpy' # possible values are 'rpy' and 'quaternion'


        #---- Clipping 
        # value for clipping the command if it is to small (below the variable)
        self.u_Pi_clip_threshold = 0.01


        #---- Divers
        # rosi direction side
        self.drive_side_param_path = '/rosi/forward_side'
        self.drive_side = self.getParamWithWait(self.drive_side_param_path)

    
        ##=== Useful variables
        # node status object
        self.ns = nodeStatus(node_name)
        self.ns.resetActive() # this node is disabled by default

        # ROS topic message variables
        self.msg_grndDist = None
        self.msg_jointState = None
        self.msg_imu = None

        # Available control types
        self.chassisCtrlType = {
            "orientation": 1,
            "orientationNullSpace_FlpJnt": 2,
            "orientationNullSpace_GrndHght": 3,
            "articulation": 4
        }

        # default control type
        self.ctrlType_curr = self.chassisCtrlType['orientation']

        # for storing joints and w function last values
        self.last_jointPos = None
        self.last_f_w = None

        #  Integrative signal useful variables
        self.ctrlOriIntrg_eRpyAccu = np.zeros([3,1])
        self.ctrlOriIntrg_eRpyLast = np.zeros([3,1])
        self.ctrlArtIntrg_eTrZAccu = 0.0
        self.ctrlArtIntrg_eTrZLast = 0.0

        # testing
        self.q_e_acc = np.quaternion(*[1,0,0,0])


        ##========= One-time calculations ===================================
        # computing the set-points in orientation quaternion and pose dual quaternion
        self.x_sp_ori_q, self.x_sp_dq  = self.convertSetPoints2DqFormat(x_sp_tr, x_sp_ori_rpy)

        # orientation controller Proportional gain in vector format
        self.kp_o_v = np.array(kp_rot_v[0:2]).reshape(2,1)

        # articulation controller Proportional gain in vector format
        self.kp_a_v = np.array( [kp_tr_v[2], kp_rot_v[0], kp_rot_v[1]]  ).reshape(3,1)

        # orientation controller Proportional gain in quaternion format
        self.kp_o_q = np.quaternion(1, kp_rot_v[0], kp_rot_v[1], kp_rot_v[2])

        # articulation  controller Proportional gain in dual quaternion format
        self.kp_a_dq = DQ(1, kp_rot_v[0], kp_rot_v[1], kp_rot_v[2], 1, kp_tr_v[0], kp_tr_v[1], kp_tr_v[2])  

        # orientation controller Integrative gain in vector format
        self.ki_rot_v = np.array(ki_rot_v).reshape(3,1)

        # articulation controller Integrative gain in vector format
        self.ki_tr_v = np.array(ki_tr_v).reshape(3,1)

        # chassis orientation kinematics considering that propulsion and chassis frames have all the same orientation (identity matrix)
        self.J_ori_dagger = compute_J_ori_dagger(tr_base_piFlp.values())

        # the orientation jacobian
        self.J_ori = np.linalg.pinv(self.J_ori_dagger)

        # the orientation jacobian null-space projector
        self.J_ori_nsproj = np.eye(4) - np.dot(self.J_ori_dagger, self.J_ori)

        # chassis articulation kinematics considering that propulsion and chassis frames have all the same orientation (identity matrix)
        self.J_art_dagger = compute_J_art_dagger(tr_base_piFlp.values())


        ##==== ROS interfaces
        # publishers
        self.pub_cmdVelFlipperSpace = rospy.Publisher('/rosi/flippers/space/cmd_v_z/leveler', Vector3ArrayStamped, queue_size=5)

        self.pub_dqPoseCurr = rospy.Publisher('/chassis_control/pose_current', DualQuaternionStamped, queue_size=5)
        self.pub_dqSetPoint = rospy.Publisher('/chassis_control/pose_sp', DualQuaternionStamped, queue_size=5)
        self.pub_dqError = rospy.Publisher('/chassis_control/pose_error', DualQuaternionStamped, queue_size=5)
        self.pub_gain_dq = rospy.Publisher('/chassis_control/gain_dq', DualQuaternionStamped, queue_size=5)

        # subscribers
        sub_imu = rospy.Subscriber('/sensor/imu_corrected', Imu, self.cllbck_imu)
        sub_grndDist = rospy.Subscriber('/rosi/model/base_ground_distance', Vector3ArrayStamped, self.cllbck_grndDist)
        sub_jointState = rospy.Subscriber('/rosi/rosi_controller/joint_state', JointState, self.cllbck_jointState)

        # services
        srv_setActive = rospy.Service(self.ns.getSrvPath('active', rospy), SetNodeStatus, self.srvcllbck_setActive)
        srv_getStatus = rospy.Service(self.ns.getSrvPath('getNodeStatus', rospy), GetNodeStatusList, self.srvcllbck_getStatus)
        srv_setHaltCmd = rospy.Service(self.ns.getSrvPath('haltcmd', rospy), SetNodeStatus, self.srvcllbck_setHaltCmd)
        
        srv_setPoseSetPoint = rospy.Service(self.node_name+'/set_pose_set_point', setPoseSetPointVec, self.srvcllbck_setPoseSetPoint) 
        srv_setPoseCtrlGain = rospy.Service(self.node_name+'/set_pose_ctrl_gain', setPoseCtrlGain, self.srvcllbck_setPoseCtrlGain) 
        srv_setCtrlType = rospy.Service(self.node_name+'/set_ctrl_type', SetInt, self.srvcllbck_setCtrlType) 

        srv_setMuFGain = rospy.Service(self.node_name+'/set_mu_flp_gain', SetFloat, self.srvcllbck_setMuFGain)  
        srv_setMuFJntSetPoint = rospy.Service(self.node_name+'/set_mu_flp_set_point', SetFloat, self.srvcllbck_setMuFJntSetPoint )   

        srv_setMuGGain = rospy.Service(self.node_name+'/set_mu_grndist_gain', SetFloat, self.srvcllbck_setMuGGain)  
        srv_setMuGVertDistSetPoint = rospy.Service(self.node_name+'/set_mu_grndist_set_point', SetFloat, self.srvcllbck_setMuGVertDistSetPoint)   

        srv_getPoseSetPoint = rospy.Service(self.node_name+'/get_pose_set_point', getPoseSetPointVec, self.srvcllbck_getPoseSetPoint) 
        srv_getPoseCtrlGain = rospy.Service(self.node_name+'/get_pose_ctrl_gain', getPoseCtrlGain, self.srvcllbck_getPoseCtrlGain) 
        srv_getCtrlType = rospy.Service(self.node_name+'/get_ctrl_type', GetInt, self.srvcllbck_getCtrlType)

        srv_getMuFGain = rospy.Service(self.node_name+'/get_mu_flp_gain', GetFloat, self.srvcllbck_getMuFGain)  
        srv_getMuFJntSetPoint = rospy.Service(self.node_name+'/get_mu_flp_set_point', GetFloat, self.srvcllbck_getMuFJntSetPoint )   

        srv_getMuGGain = rospy.Service(self.node_name+'/get_mu_grndist_gain', GetFloat, self.srvcllbck_getMuGGain) 
        srv_getMuGVertDistSetPoint = rospy.Service(self.node_name+'/get_mu_grndist_set_point', GetFloat, self.srvcllbck_getMuGVertDistSetPoint)   

        # Node main
        self.nodeMain()

    
    def nodeMain(self):
        '''Node main method'''

        # defining the eternal loop rate
        node_rate_sleep = rospy.Rate(self.p_rateSleep)

        # variables for computing dt
        dt = rospy.Duration(0)
        time_last = rospy.get_rostime()
        

        rospy.loginfo('[%s] Entering in ethernal loop.', self.node_name)
        while not rospy.is_shutdown():

            # only runs if node is active
            if self.ns.getNodeStatus()['active']: 

                # only runs the control when all needed input variables are available
                if self.msg_grndDist is not None and self.msg_jointState is not None and self.msg_imu is not None:

                    #=== Required computations independently of the current control mode

                    # receiving ROS time
                    ros_time = rospy.get_rostime()

                    # updates dt
                    dt = ( ros_time - time_last ).to_sec()

                    # converting the imu ROS data in the numpy quaternion format
                    q_imu = quatAssurePosW(  np.quaternion(self.msg_imu.orientation.w, self.msg_imu.orientation.x, self.msg_imu.orientation.y, self.msg_imu.orientation.z)  )

                    # creating a orientation quaternion without the yaw component
                    rpy_imu = quat2rpy(q_imu)
                    x_o_R_q = np.quaternion(*rpy2quat([rpy_imu[0], rpy_imu[1], 0]) )

                    # defining the articulation pose state
                    x_a_R_dq = trAndOri2dq([0, 0, self.msg_grndDist.vec[0].z], x_o_R_q, 'trfirst')

                    #=== Control modes implementation
                    # If the control mode is orientation
                    if self.ctrlType_curr == self.chassisCtrlType['orientation'] \
                        or self.ctrlType_curr == self.chassisCtrlType['orientationNullSpace_FlpJnt'] \
                        or self.ctrlType_curr == self.chassisCtrlType['orientationNullSpace_GrndHght']:
                        
                        #--> Error
                        # computing the orientation error
                        e_o_R_q = self.x_sp_ori_q.conj() * x_o_R_q

                        #--> Control signal
                        # the variable to receive the control signal
                        u_o_R_v = np.zeros([2,1])

                        # computing the proportional control signal
                        u_o_R_v += self.OriPropCtrlSig_compute(e_o_R_q, self.oriCtrlPropSigMthd)

                        # computing the integrative control signal component
                        u_o_R_v += self.OriIntegrCtrlSig_compute(e_o_R_q, self.ki_rot_v, dt, self.ctrlntgrMthd )

                        # transforming the control signal from {R} space to {Pi}
                        u_Pi_v =  np.dot(self.J_ori_dagger, u_o_R_v)

                        # treats the case when the null-space optimization control mode is enabled
                        if self.ctrlType_curr == self.chassisCtrlType['orientationNullSpace_FlpJnt'] or \
                           self.ctrlType_curr == self.chassisCtrlType['orientationNullSpace_GrndHght']:

                            # computes the mu function for the Flippers lever angle optimization function case
                            if self.ctrlType_curr == self.chassisCtrlType['orientationNullSpace_FlpJnt']:

                                # treating flippers position
                                flpJPos_l = correctFlippersJointSignal(self.msg_jointState.position[4:])

                                # computing mu function
                                mu = np.array([ ki * (jsp - jcurr) for jcurr, jsp, ki in zip(flpJPos_l, self.muf_flpJointPosSp_l, self.muf_kmu_l)]).reshape(4,1)


                             # computes the mu function for the Chassis ground height optimization function case
                            elif self.ctrlType_curr == self.chassisCtrlType['orientationNullSpace_GrndHght']:
                        
                                # computing the mu function
                                mu = np.array( 4* [self.mug_kmu_l * (self.mug_grndDstncSp_l - self.msg_grndDist.vec[0].z) ] ).reshape(4,1)


                            # computing the null-space projector control signal component
                            aux_u = np.dot(self.J_ori_nsproj, mu)

                            # summing the component to the current orientation signal
                            u_Pi_v += aux_u


                    # If the control mode is articulation
                    elif self.ctrlType_curr == self.chassisCtrlType['articulation']:

                        #--> Error
                        # computing the pose error
                        e_a_R_dq = self.x_sp_dq.conj() * x_a_R_dq


                        #--> Control signal
                        # variable to receive the control signal in {R} space
                        u_a_R = np.zeros((3,1))
                        
                        # computing the Proportional control signal
                        u_a_R += self.ArtPropCtrlSig_compute(e_a_R_dq, self.oriCtrlPropSigMthd)

                        # computing the Integrator control signal
                        u_a_R += self.ArtIntegrCtrlSig_compute(e_a_R_dq, self.ki_rot_v, self.ki_tr_v, dt, self.ctrlntgrMthd )

                        # transforming the control signal from {R} space to {Pi}
                        u_Pi_v = np.dot(self.J_art_dagger, u_a_R)
    

                    # If the selected control mode is invalid
                    else:
                        u_Pi_v = np.array([0, 0, 0, 0]).reshape(4,1)


                    #=== Publishing the ROS message for the controller

                    # updates drive param
                    self.drive_side = rospy.get_param(self.drive_side_param_path)

                    # mounting and publishing
                    m = Vector3ArrayStamped()
                    m.header.stamp = ros_time
                    m.header.frame_id = self.node_name
                    m.vec = [Vector3(0, 0, u_Pi) for u_Pi in u_Pi_v]
                    self.pub_cmdVelFlipperSpace.publish(m)     


                    #=== Publishing control metrics
                    #current pose
                    m = dq2DualQuaternionStampedMsg(x_a_R_dq, ros_time, self.node_name)
                    self.pub_dqPoseCurr.publish(m)

                    # pose set-point
                    m = dq2DualQuaternionStampedMsg(self.x_sp_dq, ros_time, self.node_name)
                    self.pub_dqSetPoint.publish(m)

                    # pose error
                    if self.ctrlType_curr == self.chassisCtrlType['orientation'] \
                        or self.ctrlType_curr == self.chassisCtrlType['orientationNullSpace_FlpJnt'] \
                        or self.ctrlType_curr == self.chassisCtrlType['orientationNullSpace_GrndHght']:
                        # creates the dual quaternion error if it still does not exists (in case of orientation control)
                        aux = e_o_R_q.components
                        e_a_R_dq = DQ(aux[0], aux[1], aux[2], aux[3], 0, 0, 0, 0)

                    m = dq2DualQuaternionStampedMsg(e_a_R_dq, ros_time, self.node_name)
                    self.pub_dqError.publish(m)

                    # controller gain
                    m = dq2DualQuaternionStampedMsg(self.kp_a_dq , ros_time, self.node_name)
                    self.pub_gain_dq.publish(m)

                    # updates time variable
                    time_last = ros_time


                else: # in case of no valid messages has been received
                    # resets the init time
                    time_last = rospy.get_rostime()

            # sleeping the node
            node_rate_sleep.sleep()


    ''' === Support methods callbacks ==='''

    def OriPropCtrlSig_compute(self, e_q, oriCtrlSigMthd):
        '''Method that computes the proportional controller gain for the orientation controller
        Input
            - e_q <np.quaternion>: the orientation error in unit quaternion format
            - oriCtrlSigMthd <string>: the method for generating the orientation control signal. It can bey 'rpy' or 'quaternion'
        Output
            The control signal vector <np.array>(2,1) 
        '''
        if oriCtrlSigMthd == 'rpy':

            # mounting the orientation error vector
            e_rpy = quat2rpy(e_q)
            e_o_v = np.array([e_rpy[0], e_rpy[1]]).reshape(2,1)

            # computing the control signal
            u = self.kp_o_v * e_o_v

        elif oriCtrlSigMthd == 'quaternion':

            # computing the control signal
            u_q = np.multiply(self.kp_o_q.conj().components, e_q.components)

            # mounting the orientation control signal vector
            u = np.array([u_q[1], u_q[2]]).reshape(2,1)

        else:
            rospy.logerr('[%s] orientation control signal generation type not recognized')
            return -1

        return u
        

    def ArtPropCtrlSig_compute(self, e_dq, oriCtrlSigMthd):
        '''Method that computes the proportional controller gain for the articulation controller
        Input
            - e_qq <dqrobotics.DQ>: the articulation error in unit dual quaternion format
            - oriCtrlSigMthd <string>: the method for generating the orientation control signal. It can bey 'rpy' or 'quaternion'
        Output
            The control signal vector <np.array>(3,1) 
        '''
        if oriCtrlSigMthd == 'rpy':

            # mounting the articulation error vector
            e_tr, e_q = dq2trAndQuatArray(e_dq)
            e_rpy = quat2rpy(e_q)             
            e_v = np.array( [e_tr[2][0], e_rpy[0], e_rpy[1]] ).reshape(3,1)[ 8]

            # computing the control signal
            u  = np.array([-1,1,1]).reshape(3,1) * self.kp_a_v * e_v


        elif oriCtrlSigMthd == 'quaternion':
            
            # computing Proportional control signal
            u_dq = dqElementwiseMul(self.kp_a_dq.conj(), e_dq) # kp_dq.conj

            # defining the control signal vector
            u_dq_aux = u_dq.vec8().tolist()
            u = np.array([u_dq_aux[7], u_dq_aux[1], u_dq_aux[2]]).reshape(3,1)

        else:
            rospy.logerr('[%s] orientation control signal generation type not recognized')
            return -1
        
        return u


    def OriIntegrCtrlSig_compute(self, e_q, ki, dt, intgrMthd):
        '''Computes the Integrative control signal for the orientation control
        based on the trapezoidal rule
        Input:
            - e_q <np.quaternion>: current error in quaternion format
            - ki <np.array>[3,1]: integrative control gain per dof
            - dt <float>: time step
        Output:
            <np.array>[2,1] containing the integrative control signal for roll and pitch angles
        '''
        
        if intgrMthd == 'rpy':  # if the orientation integrator method is rpy
            # integrates the orientation error in roll-pitch-yaw format by the trapezoidal method

            # converts the orientation error quaternion to the rpy format
            e_rpy = np.array( quat2rpy(e_q) ).reshape(3,1)

            # accumulates by trapezoidal integration the orientation error in rpy format
            self.ctrlOriIntrg_eRpyAccu += 0.5 * dt * (self.ctrlOriIntrg_eRpyLast + e_rpy) 

            # computes the control signal
            u = (ki * self.ctrlOriIntrg_eRpyAccu).flatten()

            # updating the last rpy error variable
            self.ctrlOriIntrg_eRpyLast = e_rpy


        elif intgrMthd == 'quaternion': # if the orientation integrator method is quaternion
            # integrates the orientation error using the quaternion format and the Runge-Kutta method integrator method.

            # creating the omega quaternion considering that each error quaternion orthogonal component is an angular corrective signal for {R}
            q_omega = np.quaternion( *np.hstack([0, e_q.components[1:]]) ) 

            # computes the average q_omega
            k1 = q_omega * dt
            k2 = (q_omega + 0.5*k1) * dt
            k3 = (q_omega + 0.5*k2) * dt
            k4 = (q_omega + k3) * dt
            k_avg = (k1 + 2*k2 + 2*k3 + k4) / 6

            # integrates the current orientation error by accumulation 
            self.q_e_acc = self.q_e_acc * quatExpFromMatlab(0.5 * k_avg)

            # treates the obtained signal
            self.q_e_acc = (quatAssurePosW(self.q_e_acc)).normalized()

            # computes the integrator control signal component
            u = -1* ki * self.q_e_acc.components[1:4].reshape(3,1)


        else: # if the input integrator method has not been recognized
            rospy.logerr('[%s] requested orientation integration method not recognized')
            return -1


        # isolating control signal components related to rotations around x and y axes of {R}
        u = np.array([u[0], u[1]]).reshape(2,1)

        return u 
    

    def ArtIntegrCtrlSig_compute(self, e_dq, ki_ori, ki_tr, dt, intgrMthd):
        '''Computes the Integrative control signal for the articulation control'''

        # extract relevant variables from error dq
        e_tr, e_q = dq2trAndQuatArray(e_dq)


        #--- orientation integrative control signal component
        # computes the orientation Integrative control signal
        u_o = self.OriIntegrCtrlSig_compute(e_q, ki_ori, dt, intgrMthd)


        #--- vertical translation control signal component
        # accumulates by trapezoidal integration the vertical translation error in rpy format
        self.ctrlArtIntrg_eTrZAccu += 0.5 * dt * (self.ctrlArtIntrg_eTrZLast + e_tr[2])

        # computes the vertical translation control signal
        u_trZ = -1 * ki_tr[2] * self.ctrlArtIntrg_eTrZAccu

        #--- posamble
        # updating the last translation error component
        self.ctrlArtIntrg_eTrZLast = e_tr[2]

        # composed control signal
        u = np.vstack([u_trZ, u_o])

        # returns the articulation integrative control signal
        return u
        

    def IntegrCtrlSig_zerateAcc(self):
        '''Zerates the integrative signal accumulator variable'''
        self.ctrlOriIntrg_eRpyAccu = np.zeros([3,1])
        self.ctrlArtIntrg_eTrZAccu = 0.0
    

    ''' === Topics callbacks ==='''
    def cllbck_imu(self, msg):
        '''Callback for the IMU messages.'''
        self.msg_imu = msg      


    def cllbck_grndDist(self, msg):
        '''Callback for received distance to the ground info'''
        # stores received distance to the ground as a 3D vector
        self.msg_grndDist = msg

    
    def cllbck_jointState(self, msg):
        ''' Callback for flippers state'''
        self.msg_jointState = msg
    

    ''' === Service Callbacks === '''
    def srvcllbck_setActive(self, req):
        ''' Method for setting the active node status flag'''

        # zerate integrator accumulator variables
        self.IntegrCtrlSig_zerateAcc()

        return self.ns.defActiveServiceReq(req, rospy)


    def srvcllbck_getStatus(self, req):
        ''' Method for returning the node status flag list'''
        return self.ns.getNodeStatusSrvResponse()
    
    
    def srvcllbck_setHaltCmd(self, req):
        ''' Method for setting the haltCmd node status flag'''
        return self.ns.defHaltCmdServiceReq(req, rospy)    


    def srvcllbck_setPoseSetPoint(self, req):
        ''' Service callback method for redefining the pose set-point given 
        two 3D vectors (translation [m] + orientation in euler-RPY [rad]'''
        # updating set-point variables

        # zerate integrator accumulator variables
        self.IntegrCtrlSig_zerateAcc()

        self.x_sp_ori_q, self.x_sp_dq  = self.convertSetPoints2DqFormat(list(req.translation), list(req.orientation))
        return True


    def srvcllbck_setPoseCtrlGain(self, req):
        '''Service that sets the controller gains'''

        # zerate integrator accumulator variables
        self.IntegrCtrlSig_zerateAcc()

        # orientation controller proportional gain
        self.kp_o_v = np.array( [req.kp_ori[0], req.kp_ori[1] ] ).reshape(2,1)

        # articulation controller proportional gain
        self.kp_a_v = np.array( [req.kp_tr[2], req.kp_ori[0], req.kp_ori[1]] ).reshape(3,1)

        # orientation controller proportional gain in quaternion format
        self.kp_o_q = np.quaternion(1, req.kp_ori[0], req.kp_ori[1], req.kp_ori[2])

        # articulation  controller proportional gain in dual quaternion format
        self.kp_a_dq = DQ(1, req.kp_ori[0], req.kp_ori[1], req.kp_ori[2], 1, req.kp_tr[0], req.kp_tr[1], req.kp_tr[2])  

        # translation controller integrator gain
        self.ki_tr_v = np.array(req.ki_tr).reshape(3,1)

        # orientation controller integrator gain
        self.ki_rot_v = np.array(req.ki_ori).reshape(3,1)

        return True
    

    def srvcllbck_getPoseSetPoint(self, req):
        ''' Service callback method for retrieving the pose set-point given 
        two 3D vectors (translation + orientation in euler-XYZ'''
        # setting new set-point
        return [dqExtractTransV3(self.x_sp_dq ).reshape(1,3).tolist()[0] ,dq2rpy(self.x_sp_dq)]
    

    def srvcllbck_getPoseCtrlGain(self, req):
        '''Service that gets controller gains'''

        # extracting proportional gains
        aux = self.kp_a_dq.vec8().tolist()

        ret = getPoseCtrlGainResponse()
        ret.kp_tr = aux[5:]
        ret.kp_ori = aux[1:4]
        ret.ki_tr = self.ki_tr_v.flatten().tolist()
        ret.ki_ori = self.ki_rot_v.flatten().tolist()

        return ret
    

    def srvcllbck_setCtrlType(self, req):
        ''' Callback for changing current control type'''
        # preparing the response
        resp = SetIntResponse()

        # confirming if the requested control type is a valid one
        if req.value >= 1 and req.value <= len(self.chassisCtrlType):
            self.ctrlType_curr = req.value
            resp.ret = self.ctrlType_curr 
            rospy.loginfo('[%s] Setting control type to: %s.', self.node_name, self.get_key_by_value(self.chassisCtrlType, self.ctrlType_curr))

            # zerating integrator variables
            self.IntegrCtrlSig_zerateAcc()

        # in case of the received control mode is unavailable
        else:
            resp.ret = -1
            rospy.logerr('[%s] Received a bad control type: %s.', self.node_name, req.value)

        return resp


    def srvcllbck_getCtrlType(self, req):
        ''' Callback to inform the current control type'''
        ret = GetIntResponse()
        ret.ret = int(self.ctrlType_curr)
        return ret

    
    def srvcllbck_setMuFGain(self, req):
        ''' Method for setting the flippers mu function gain'''

        # zerate integrator accumulator variables
        self.IntegrCtrlSig_zerateAcc()

        self.muf_kmu_l = 4*[req.value]
        return SetFloatResponse(float(self.muf_kmu_l[0]))


    def srvcllbck_setMuFJntSetPoint(self, req):
        '''Method for setting the flippers mu function set-point'''

        # zerate integrator accumulator variables
        self.IntegrCtrlSig_zerateAcc()

        self.muf_flpJointPosSp_l = 4 * [req.value]
        return SetFloatResponse(float(self.muf_flpJointPosSp_l[0]))


    def srvcllbck_setMuGGain(self, req):
        '''Method for setting the ground distance mu function gain'''
        self.mug_kmu_l = req.value
        return SetFloatResponse(self.mug_kmu_l)
    

    def srvcllbck_setMuGVertDistSetPoint(self, req):
        ''' Method for setting the ground distance mu function set-point'''
        self.mug_grndDstncSp_l = req.value
        return SetFloatResponse(self.mug_grndDstncSp_l)

    
    def srvcllbck_getMuFGain(self, req):
        ''' Method for getting the flipper mu function gain'''
        return GetFloatResponse(self.muf_kmu_l[0])

    
    def srvcllbck_getMuFJntSetPoint(self, req):
        ''' Method for getting the flipper mu function set-point'''
        return GetFloatResponse(self.muf_flpJointPosSp_l[0])


    def srvcllbck_getMuGGain(self, req):
        ''' Method for getting the ground-distance mu function gain'''
        return GetFloatResponse(self.mug_kmu_l)

    
    def srvcllbck_getMuGVertDistSetPoint(self, req):
        ''' MEthof for getting the ground-distance mu function set-point'''
        return GetFloatResponse(self.mug_grndDstncSp_l)

       

    #=== Static methods
    @staticmethod
    def getParamWithWait(path_param):
        """Waits until a param exists so retrieves it"""
        while not  rospy.has_param(path_param):
            rospy.loginfo("[manager] Waiting for param: %s", path_param)
        return rospy.get_param(path_param)


    @staticmethod
    def convertSetPoints2DqFormat(tr, rpy):
        '''Converts two arrays containing translation and orientation set-points to the orientation quaternion
        and pose dual quaternion formats. Useful for defining and updating the set-points
        Input
            - tr <list/np.array>: the translation set-point vector
            - rpy <list/np.array>: the orientation set-point in Roll Pitch Yaw format
        Output
            - ori_q <np.quaternion>: the orientation set-point in quaternion format
            - pose_dq <dqrobotics.DQ>: the pose set-point in dual quaternion format   
        '''
        
        # orientation in quaternion format
        ori_q = rpy2quat(rpy)
        ori_q = np.quaternion(ori_q[0], ori_q[1], ori_q[2], ori_q[3])

        # pose dual quaternion
        pose_dq = trAndOri2dq(tr, ori_q, 'trfirst')

        return ori_q, pose_dq


    @staticmethod
    def get_key_by_value(dictionary, value):
        ''' Gets the key of a dictionary given a value'''
        for key, val in dictionary.items():
            if val == value:
                return key
        return None  # Value not found


    

if __name__ == '__main__':
    node_name = 'chassis_control'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass



    