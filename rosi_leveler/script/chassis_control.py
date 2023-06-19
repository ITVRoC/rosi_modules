#!/usr/bin/env python3
''' This is a ROSI algorithm
It controls the chassis
based on different control strategies
'''
import rospy
import numpy as np
import quaternion
from dqrobotics import *

from rosi_common.dq_tools import quat2rpy, rpy2quat, trAndOri2dq, dqElementwiseMul, dq2trAndQuatArray, dqExtractTransV3, dq2rpy
from rosi_common.rosi_tools import correctFlippersJointSignal, compute_J_ori_dagger, compute_J_art_dagger
from rosi_common.node_status_tools import nodeStatus
from rosi_model.rosi_description import tr_base_piFlp

from rosi_common.msg import Float32Array, Vector3ArrayStamped
from sensor_msgs.msg import Imu, JointState

from rosi_common.srv import SetNodeStatus, GetNodeStatusList, setPoseSetPointVec, setPoseCtrlGain, getPoseCtrlGain, getPoseSetPointVec, SetInt, SetIntResponse, GetInt, GetIntResponse


class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        ##==== PARAMETERS ============================================

        #----------------- SET-POINT ------------------------------
        # orientation
        x_sp_ori_rpy = np.deg2rad([0, 0, 0]) # final vector is in radians RPY
        
        # ground distance
        x_sp_tr = [0.0, 0.0, 0.3] # in meters
       

        #----------------- CONTROL GAINS ------------------------------
        # orientation controller gain per DOF
        kp_rot_v = [2.0, 4.0, 1.0]

        # translation control gain per DOF
        kp_tr_v = [1.0, 1.0, 0.8]


        #------ Mu function for the null-space controller parameters
        # propulsion joints angular set-point for the null-space
        self.flpJointPosSp_l = 4*[np.deg2rad(110)]

        # mu function gain
        self.kmu_l = 4*[0.3]

        #---- Divers
        # imu orientation rotation  correction
        aux_imu_correct = rpy2quat(np.deg2rad([0, 0, -90]))
        self.q_imu_offset = np.quaternion(aux_imu_correct[0], aux_imu_correct[1], aux_imu_correct[2], aux_imu_correct[3]) 

        # rosi direction side
        self.drive_side_param_path = '/rosi/forward_side'
        self.drive_side = self.getParamWithWait(self.drive_side_param_path)

    
        ##=== Useful variables
        # node status object
        self.ns = nodeStatus(node_name)
        #self.ns.resetActive() # this node is disabled by default

        # ROS topic message variables
        self.msg_grndDist = None
        self.msg_jointState = None
        self.msg_imu = None

        # Available control types
        self.chassisCtrlType = {
            "orientation": 1,
            "orientationNullSpace": 2,
            "articulation": 3
        }

        # current control type
        self.ctrlType_curr = self.chassisCtrlType['orientation']

        # for storing joints and w function last values
        self.last_jointPos = None
        self.last_f_w = None


        ##========= One-time calculations ===================================
        # computing the set-points in orientation quaternion and pose dual quaternion
        self.x_sp_ori_q, self.x_sp_dq  = self.convertSetPoints2DqFormat(x_sp_tr, x_sp_ori_rpy)

        # orientation controller gain in quaternion format
        self.kp_o_q = np.quaternion(1, kp_rot_v[0], kp_rot_v[1], kp_rot_v[2])

        # articulation  controller gain in dual quaternion format
        self.kp_a_dq = DQ(1, kp_rot_v[0], kp_rot_v[1], kp_rot_v[2], 1, kp_tr_v[0], kp_tr_v[1], kp_tr_v[2])  

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
        self.pub_cmdVelFlipperSpace = rospy.Publisher('/rosi/flippers/space/cmd_v_z/leveler', Float32Array, queue_size=5)
        #self.pub_imuCtrlSig = rospy.Publisher('/rosi/base/space/cmd_vel/ctrl_signal', TwistStamped, queue_size=5)
        #self.pub_dqError = rospy.Publisher('/rosi/base/pose_reg/de_o_R_qrror', DualQuaternionStamped, queue_size=5)
        #self.pub_dqSetPoint = rospy.Publisher('/rosi/base/pose_reg/set_point', DualQuaternionStamped, queue_size=5)
        #self.pub_dqPoseCurr = rospy.Publisher('/rosi/base/pose', DualQuaternionStamped, queue_size=5)
        #self.pub_ctrlGain = rospy.Publisher('/rosi/base/pose_reg/ctrl_gain', TwistStamped, queue_size=5)

        # subscribers
        sub_imu = rospy.Subscriber('/mti/sensor/imu', Imu, self.cllbck_imu)
        sub_grndDist = rospy.Subscriber('/rosi/model/base_ground_distance', Vector3ArrayStamped, self.cllbck_grndDist)
        sub_jointState = rospy.Subscriber('/rosi/rosi_controller/joint_state', JointState, self.cllbck_jointState)

        # services
        srv_setActive = rospy.Service(self.ns.getSrvPath('active', rospy), SetNodeStatus, self.srvcllbck_setActive)
        srv_getStatus = rospy.Service(self.ns.getSrvPath('getNodeStatus', rospy), GetNodeStatusList, self.srvcllbck_getStatus)
        srv_setHaltCmd = rospy.Service(self.ns.getSrvPath('haltcmd', rospy), SetNodeStatus, self.srvcllbck_setHaltCmd)
        
        srv_setPoseSetPoint = rospy.Service(self.node_name+'/set_pose_set_point', setPoseSetPointVec, self.srvcllbck_setPoseSetPoint) 
        srv_setPoseCtrlGain = rospy.Service(self.node_name+'/set_pose_ctrl_gain', setPoseCtrlGain, self.srvcllbck_setPoseCtrlGain) 
        srv_setCtrlType = rospy.Service(self.node_name+'/set_ctrl_type', SetInt, self.srvcllbck_setCtrlType) 

        srv_getPoseSetPoint = rospy.Service(self.node_name+'/get_pose_set_point', getPoseSetPointVec, self.srvcllbck_getPoseSetPoint) 
        srv_getPoseCtrlGain = rospy.Service(self.node_name+'/get_pose_ctrl_gain', getPoseCtrlGain, self.srvcllbck_getPoseCtrlGain) 
        srv_getCtrlType = rospy.Service(self.node_name+'/get_ctrl_type', GetInt, self.srvcllbck_getCtrlType)


        # Node main
        self.nodeMain()


    
    def nodeMain(self):
        '''Node main method'''

        # defining the eternal loop rate
        node_rate_sleep = rospy.Rate(20)

        rospy.loginfo('[%s] Entering in ethernal loop.', self.node_name)
        while not rospy.is_shutdown():

            # only runs if node is active
            if self.ns.getNodeStatus()['active']: 

                # only runs the control when all needed input variables are available
                if self.msg_grndDist is not None and self.msg_jointState is not None and self.msg_imu is not None:

                    #=== Required computations independently of the current control mode
                    # setting the imu ROS data in the numpy quaternion format
                    q_imu = np.quaternion(self.msg_imu.orientation.w, self.msg_imu.orientation.x, self.msg_imu.orientation.y, self.msg_imu.orientation.z)

                    # correcting imu physical missorientation
                    q_imu_corrected = q_imu * self.q_imu_offset

                    # computing the chassis orientation without the yaw component
                    rpy = quat2rpy(q_imu_corrected)
                    q_yawcorr = rpy2quat([0, 0, -rpy[2]])
                    q_yawcorr = np.quaternion(q_yawcorr[0], q_yawcorr[1], q_yawcorr[2], q_yawcorr[3])
                    x_o_R_q = q_imu_corrected * q_yawcorr

            
                    #=== Control modes implementation
                    # If the control mode is orientation
                    if self.ctrlType_curr == self.chassisCtrlType['orientation'] or self.ctrlType_curr == self.chassisCtrlType['orientationNullSpace']:
                        
                        # computing the orientation error
                        e_o_R_q = self.x_sp_ori_q.conj() * x_o_R_q

                        # control signal component due to the orientation error
                        u_o_R_q = np.multiply(self.kp_o_q.conj().components, e_o_R_q.components)
                        u_o_R_v = np.array([u_o_R_q[1], u_o_R_q[2]]).reshape(2,1)
                        u_Pi_v =  np.dot(self.J_ori_dagger, u_o_R_v)

                        # if the joints optimization is enabled, computes the null space component if this control mode is enabled
                        if self.ctrlType_curr == self.chassisCtrlType['orientationNullSpace']:

                            # treating flippers position
                            flpJPos_l = correctFlippersJointSignal(self.msg_jointState.position[4:])

                            # computing the null-space projector function component
                            mu = np.array([ ki * (jsp - jcurr) for jcurr, jsp, ki in zip(flpJPos_l, self.flpJointPosSp_l, self.kmu_l)]).reshape(4,1)

                            # computing the null-space projector control signal component
                            aux_u = np.dot(self.J_ori_nsproj, mu)

                            # summing the component to the current orientation signal
                            u_Pi_v = u_Pi_v + aux_u


                    # If the control mode is articulation
                    elif self.ctrlType_curr == self.chassisCtrlType['articulation']:

                        # defining the articulation pose 
                        x_a_R_dq = trAndOri2dq([0, 0, self.msg_grndDist.vec[0].z], x_o_R_q, 'trfirst')

                        # computing the pose error
                        e_a_R_dq = self.x_sp_dq .conj() * x_a_R_dq

                        # computing articulation the control signal
                        u_a_R_dq = dqElementwiseMul(self.kp_a_dq.conj(), e_a_R_dq) # kp_dq.conj

                        # converting the control signal to vector (translation) and quaternion (orientation) formats
                        u_a_R_tr, u_a_R_q = dq2trAndQuatArray(u_a_R_dq)

                        # defining the control signal vector
                        u_a_R = np.array([u_a_R_tr[2][0], u_a_R_q.components[1], u_a_R_q.components[2]]).reshape(3,1)

                        # control signal for each propulsion mechanisms vertical axis
                        u_Pi_v = np.dot(self.J_art_dagger, u_a_R)
     

                    # If the selected control mode is invalid
                    else:
                        u_Pi_v = np.array([0, 0, 0, 0]).reshape(4,1)
                    

                    #=== Publishing the ROS message
                    # receiving ROS time
                    ros_time = rospy.get_rostime()

                    # updates drive param
                    self.drive_side = rospy.get_param(self.drive_side_param_path)

                    # mounting and publishing
                    m = Float32Array()
                    m.header.stamp = ros_time
                    m.header.frame_id = self.node_name
                    m.data = u_Pi_v.flatten().tolist()
                    self.pub_cmdVelFlipperSpace.publish(m)     
                    #print(m)



                    """# extracting flippers needed data
                    _,joint_state = jointStateData2dict(self.flpJointState)
                    flp_pos = joint_state['pos']    

                    ##=== Computing the error signal
                    # converting the orientation quaternion to roll-pitch-yaw format
                    rpy = quat2rpy(self.q_ori.components)

                    # creates a dual-quaternion with the compensation of the yaw component, as the pose reg controls only rol, pitch angles and vertical position
                    aux = rpy2quat([0, 0, -rpy[2]])
                    q_transform_yaw = np.quaternion(aux[0], aux[1], aux[2], aux[3])

                    # rotates the orientation quaternion by the yaw compensating element to zerate yaw component
                    q_x_noYaw = self.q_ori * q_transform_yaw

                    # computing the orientation error
                    e_o_R_q = quatAssurePosW(self.sp_ori.conj() * q_x_noYaw)


                    ##=== Generating rotational control signal

                    # computing rotational velocity control signal (aroung x, y, and z base axis)
                    ctrlSig_rot = np.multiply(self.kp_u, np.multiply(-1, e_o_R_q.components[1:3]).tolist() + [0] )# puts a zero to the p_z component, which is not controlled in this control mode

                    print('---')
                    print(ctrlSig_rot)

                    # applyes the dead band to the rotational control signal  (controller do not correct if it is below a threshold)
                    ctrlSig_rot_db = [cmd if abs(cmd) >= self.deadBand_rot else 0.0 for cmd in ctrlSig_rot]
    

                    ##=== COMPUTING \mu function value

                    # first, computes the weight function value 
                    # currently implemented the joints mean value
                    aux = [((flp_pos_i - self.flpjoint_mean)/self.flpjoint_span)**2 for flp_pos_i in flp_pos]
                    n = len(flp_pos)
                    f_w = -1/(2*n) * sum(aux)

                    if self.last_jointPos is None: # for when it is first control loop run

                        # mu function is zero 
                        f_mu = 4*[0]

                    else: # for when the controller already run onde

                        # computing delta joint pos and f function
                        delta_jointPos = [x-y for x,y in zip(flp_pos, self.last_jointPos)]
                        delta_f_w = f_w - self.last_f_w

                        #  computing the mu function
                        f_mu = [k_mu_i * (delta_jointPos_i/delta_f_w) for delta_jointPos_i, k_mu_i in zip(delta_jointPos, self.kp_mu)]
                        
                        # avoids NaN in f_mu
                        f_mu = [0 if np.isnan(f_mu_i) else f_mu_i for f_mu_i in f_mu]

                    # updating last value variables
                    self.last_jointPos = flp_pos
                    self.last_f_w = f_w


                    ## === CALCULATING THE CONTROL SIGNAL
                    u1 = np.dot(self.J_art_dagger, np.array(ctrlSig_rot_db).reshape(3,1)) 
                    u2 = np.dot(  np.eye(4) - np.dot(self.J_art_dagger, np.linalg.pinv(self.J_art_dagger)), np.array(f_mu).reshape(4,1) )
                    #u_l = u1+u2
                    u_l = u1

                    print('----')
                    print(u1)
                    print(u2)
                    print(u_l)

                    ## === PUBLISHING THE CONTROL SIGNAL
                    # receiving ROS time
                    ros_time = rospy.get_rostime()

                    # updates drive param
                    self.drive_side = rospy.get_param(self.drive_side_param_path)

                    ##=== Publishing the control command signal
                    
                    # publishing message
                    m = Float32Array()
                    m.header.stamp = ros_time
                    m.header.frame_id = self.node_name
                    m.data = [aux[0] for aux in u_l]
                    self.pub_cmdVelFlipperSpace.publish(m)     
                    """            


                    """m = Vector3ArrayStamped()
                    m.header.stamp = rospy.get_rostime()
                    m.header.frame_id = self.node_name
                    m.vec = [Vector3(vec[0][0], vec[1][0], vec[2][0]) for vec in dotxp_l]
                    self.pub_cmdVelFlipperSpace.publish(m)
                    
                    m = TwistStamped()
                    m.header.stamp = ros_time
                    m.header.frame_id = self.node_name

                    if self.drive_side == 'a':
                        m.twist.linear.x = baseTrCtrlSig[0]
                        m.twist.linear.y = baseTrCtrlSig[1]
                        m.twist.linear.z = baseTrCtrlSig[2]
                        m.twist.angular.x = baseRotCtrlSig_db[0]
                        m.twist.angular.y = baseRotCtrlSig_db[1]
                        m.twist.angular.z = baseRotCtrlSig_db[2] 
                        self.pub_imuCtrlSig.publish(m)
                        #print(m)
                    elif self.drive_side == 'b':
                        m.twist.linear.x = -baseTrCtrlSig[0]
                        m.twist.linear.y = baseTrCtrlSig[1]
                        m.twist.linear.z = baseTrCtrlSig[2]
                        m.twist.angular.x = -baseRotCtrlSig_db[0]
                        m.twist.angular.y = -baseRotCtrlSig_db[1]
                        m.twist.angular.z = baseRotCtrlSig_db[2] 
                        self.pub_imuCtrlSig.publish(m)
                        #printprint(m)
                    else:
                        rospy.logerr("[pose_reg_base_cmd_vel] wrong param "+self.drive_side_param_path+" value. Valid values are 'a' and 'b'")

                    ##=== Publishing error dual quaternion message
                    aux_dqe = de_o_R_qrror.vec8()
                    m = DualQuaternionStamped()
                    m.header.stamp = ros_time
                    m.header.frame_id = self.node_name
                    m.wp = aux_dqe[0]
                    m.xp = aux_dqe[1]
                    m.yp = aux_dqe[2]
                    m.zp = aux_dqe[3]
                    m.wd = aux_dqe[4]
                    m.xd = aux_dqe[5]
                    m.yd = aux_dqe[6]
                    m.zd = aux_dqe[7]
                    self.pub_dqError.publish(m)

                    ##=== Publishing pose set-point dual quaternion message
                    aux_dqsp = self.dq_sp.vec8()
                    m = DualQuaternionStamped()
                    m.header.stamp = ros_time
                    m.header.frame_id = self.node_name
                    m.wp = aux_dqsp[0]
                    m.xp = aux_dqsp[1]
                    m.yp = aux_dqsp[2]
                    m.zp = aux_dqsp[3]
                    m.wd = aux_dqsp[4]
                    m.xd = aux_dqsp[5]
                    m.yd = aux_dqsp[6]
                    m.zd = aux_dqsp[7]
                    self.pub_dqSetPoint.publish(m)

                    ##=== Publishing current pose dual quaternion message
                    aux_dqpose = dq_x_noYaw.vec8()
                    m = DualQuaternionStamped()
                    m.header.stamp = ros_time
                    m.header.frame_id = self.node_name
                    m.wp = aux_dqpose[0]
                    m.xp = aux_dqpose[1]
                    m.yp = aux_dqpose[2]
                    m.zp = aux_dqpose[3]
                    m.wd = aux_dqpose[4]
                    m.xd = aux_dqpose[5]
                    m.yd = aux_dqpose[6]
                    m.zd = aux_dqpose[7]
                    self.pub_dqPoseCurr.publish(m)

                    ##=== Publishing the control command signal
                    m = TwistStamped()
                    m.header.stamp = ros_time
                    m.header.frame_id = self.node_name
                    m.twist.linear.x = self.kp_tr[0]
                    m.twist.linear.y = self.kp_tr[1]
                    m.twist.linear.z = self.kp_tr[2]
                    m.twist.angular.x = self.kp_rot[0]
                    m.twist.angular.y = self.kp_rot[1]
                    m.twist.angular.z = self.kp_rot[2]
                    self.pub_ctrlGain.publish(m)"""
                    


            # sleeping the node
            node_rate_sleep.sleep()

    
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
        self.x_sp_ori_q, self.x_sp_dq  = self.convertSetPoints2DqFormat(list(req.translation), list(req.orientation))
        return True


    def srvcllbck_setPoseCtrlGain(self, req):
        '''Service that sets the controller gains'''

        # orientation controller gain in quaternion format
        self.kp_o_q = np.quaternion(1, req.kp_ori[0], req.kp_ori[1], req.kp_ori[2])

        # articulation  controller gain in dual quaternion format
        self.kp_a_dq = DQ(1, req.kp_ori[0], req.kp_ori[1], req.kp_ori[2], 1, req.kp_tr[0], req.kp_tr[1], req.kp_tr[2])  

        return True
    

    def srvcllbck_getPoseSetPoint(self, req):
        ''' Service callback method for retrieving the pose set-point given 
        two 3D vectors (translation + orientation in euler-XYZ'''
        # setting new set-point
        return [dqExtractTransV3(self.x_sp_dq ).reshape(1,3).tolist()[0] ,dq2rpy(self.x_sp_dq)]
    

    def srvcllbck_getPoseCtrlGain(self, req):
        '''Service that gets controller gains'''
        aux = self.kp_a_dq.vec8().tolist()
        return [aux[1:4], aux[5:]]
    

    def srvcllbck_setCtrlType(self, req):
        ''' Callback for changing current control type'''
        # preparing the response
        resp = SetIntResponse()

        # confirming if the requested control type is a valid one
        if req.value >= 1 and req.value <= len(self.chassisCtrlType):
            self.ctrlType_curr = req.value
            resp.ret = self.ctrlType_curr 
            rospy.loginfo('[%s] Setting control type to: %s.', self.node_name, self.get_key_by_value(self.chassisCtrlType, self.ctrlType_curr))

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
    rospy.loginfo('Actually, articulationC_control_1 node initiated!!! Testing purposes only.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass



    