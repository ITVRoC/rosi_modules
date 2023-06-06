#!/usr/bin/env python3
''' This is a ROSI algorithm
It controls the body attitude by actively controlling the chassis orientation
and using the height as an optimization parameter. 
It sends as the output control signal the linear velocity on v_z_Pi, on the flipper's space
'''
import rospy

from rosi_common.msg import Float32Array
from sensor_msgs.msg import Imu, JointState

import numpy as np
import quaternion
from dqrobotics import *

from rosi_common.dq_tools import quat2rpy, rpy2quat, quatAssurePosW, trAndOri2dq, quat2rpy, dq2rpy, dq2trAndQuatArray, dqElementwiseMul
from rosi_common.dq_tools import *

from rosi_common.rosi_tools import compute_J_ori_dagger, correctFlippersJointSignal
from rosi_common.node_status_tools import nodeStatus

from rosi_common.msg import Vector3ArrayStamped
from rosi_common.srv import SetNodeStatus, GetNodeStatusList, setPoseSetPointVec, setPoseCtrlGain, getPoseCtrlGain, getPoseSetPointVec

from rosi_model.rosi_description import dict_flprsPosLimits, tr_base_piFlp

class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        ##==== PARAMETERS

        #-----------------------------------------------
        # Pose set-point
        aux_sp = rpy2quat(np.deg2rad([0, 0, 0]))
        self.q_sp = np.quaternion(aux_sp[0], aux_sp[1], aux_sp[2], aux_sp[3])
        #-----------------------------------------------

        # rotational controller kp for each dof [rol, pitch, yaw]
        self.kp_rot = np.quaternion(1.0, 4.0, 8.0, 1)

        # command deadband for each subclass
        self.deadBand_rot = 0.01
        self.deadBand_tr = 0.01

        # imu orientation rotation  correction
        aux_imu_correct = rpy2quat(np.deg2rad([0, 0, 0]))
        self.q_imu_correct = np.quaternion(aux_imu_correct[0], aux_imu_correct[1], aux_imu_correct[2], aux_imu_correct[3]) 

        #--- null space optimizer

        # propulsion joints angular set-point
        self.flpJointPosSp_l = 4*[np.deg2rad(110)]

        # mu function gain
        self.kmu_l = 4*[0.3]

        # rosi direction side
        self.drive_side_param_path = '/rosi/forward_side'
        self.drive_side = self.getParamWithWait(self.drive_side_param_path)
    
        ##=== Useful variables
        # node status object
        self.ns = nodeStatus(node_name)
        #self.ns.resetActive() # this node is disabled by default

        # stores IMU current orientation
        self.q_ori = None

        # stores the robot base distance from the ground
        self.p_grnd = None

        # ROSI joint states
        self.flpJointState = None

        # for storing joints and w function last values
        self.last_jointPos = None
        self.last_f_w = None


        ##==== One-time calculations

        # operational angular range of flippers joint axis
        self.flpjoint_span = dict_flprsPosLimits['max'] - dict_flprsPosLimits['min']

        # operational angular mean of flippers joint axis
        self.flpjoint_mean = (dict_flprsPosLimits['max'] + dict_flprsPosLimits['min']) / 2

        # chassis orientaiton kinematics considering that propulsion and chassis frames have all the same orientation (identity matrix)
        self.J_ori_dagger = compute_J_ori_dagger(tr_base_piFlp.values())

        # the orientation jacobian
        self.J_ori = np.linalg.pinv(self.J_ori_dagger)

        # the orientation jacobian null-space projector
        self.J_ori_nsproj = np.eye(4) - np.dot(self.J_ori_dagger, self.J_ori)

        ##==== ROS interfaces

        # publishers
        self.pub_cmdVelFlipperSpace = rospy.Publisher('/rosi/flippers/space/cmd_v_z/leveler', Float32Array, queue_size=5)
        #self.pub_imuCtrlSig = rospy.Publisher('/rosi/base/space/cmd_vel/ctrl_signal', TwistStamped, queue_size=5)
        #self.pub_dqError = rospy.Publisher('/rosi/base/pose_reg/dq_error', DualQuaternionStamped, queue_size=5)
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
        srv_getPoseSetPoint = rospy.Service(self.node_name+'/get_pose_set_point', getPoseSetPointVec, self.srvcllbck_getPoseSetPoint)
        srv_getPoseCtrlGain = rospy.Service(self.node_name+'/get_pose_ctrl_gain', getPoseCtrlGain, self.srvcllbck_getPoseCtrlGain)

        # Node main
        self.nodeMain()

    
    def nodeMain(self):
        '''Node main method'''

        # defining the eternal loop rate
        node_rate_sleep = rospy.Rate(10)

        rospy.loginfo('Entering in ethernal loop.')
        while not rospy.is_shutdown():

            if self.ns.getNodeStatus()['active']: # only runs if node is active

                # only runs when required data has been arrived
                if self.q_ori is not None and self.p_grnd is not None and self.flpJointState is not None:

                    #=== State definition
                    # correcting imu physical missorientation
                    q_ori_corrected = self.q_ori*self.q_imu_correct

                    # copmuting the yaw correction
                    rpy = quat2rpy(q_ori_corrected)
                    q_yawcorr = rpy2quat([0, 0, -rpy[2]])
                    q_yawcorr = np.quaternion(q_yawcorr[0], q_yawcorr[1], q_yawcorr[2], q_yawcorr[3])
                    q_ori_noYaw = q_ori_corrected * q_yawcorr

                    #=== Computing error
                    # computing the pose error
                    q_e = self.q_sp.conj() * q_ori_noYaw

                    
                    #=== Control signal
                

                    # control signal component due to the orientation error
                    u1_q_e_gain = np.multiply(self.kp_rot.conj().components, q_e.components)
                    u1_R_e_gain = np.array([u1_q_e_gain[1], u1_q_e_gain[2]]).reshape(2,1)
                    u1 = np.dot(self.J_ori_dagger, u1_R_e_gain)

                    # control signal component that optimizes flipper joints angular position
                    flpJPos = correctFlippersJointSignal(self.flpJointState.position[4:])
                    u2 = np.dot(self.J_ori_nsproj, self.computeU2(flpJPos, self.flpJointPosSp_l, self.kmu_l))

                    # control signal
                    u_Pi_ori = u1 + u2


                    #=== Publishing the ROS message
                    # receiving ROS time
                    ros_time = rospy.get_rostime()

                    # updates drive param
                    # TODO implement the drive side correction 
                    self.drive_side = rospy.get_param(self.drive_side_param_path)

                    # mounting and publishing
                    m = Float32Array()
                    m.header.stamp = ros_time
                    m.header.frame_id = self.node_name
                    m.data = u_Pi_ori.flatten().tolist()
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
                    q_e = quatAssurePosW(self.sp_ori.conj() * q_x_noYaw)


                    ##=== Generating rotational control signal

                    # computing rotational velocity control signal (aroung x, y, and z base axis)
                    ctrlSig_rot = np.multiply(self.kp_u, np.multiply(-1, q_e.components[1:3]).tolist() + [0] )# puts a zero to the p_z component, which is not controlled in this control mode

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
                    aux_dqe = dq_error.vec8()
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

    
    def cllbck_imu(self, msg):
        '''Callback for the IMU messages.'''
        # creating a numpy quaternion element
        self.q_ori = np.quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)


    def cllbck_grndDist(self, msg):
        '''Callback for received distance to the ground info'''
        # stores received distance to the ground as a 3D vector
        self.p_grnd = np.array([msg.vec[0].x, msg.vec[0].y, msg.vec[0].z])
    

    def cllbck_jointState(self, msg):
        ''' Callback for flippers state'''
        self.flpJointState = msg


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
        two 3D vectors (translation + orientation in euler-XYZ'''
        # setting new set-point
        self.dq_sp = trAndOri2dq(list(req.translation), rpy2quat(np.deg2rad(list(req.orientation))), 'trfirst')
        return True


    def srvcllbck_setPoseCtrlGain(self, req):
        '''Service that sets the controller gains'''
        self.kp_tr = req.kp_tr
        self.kp_rot = req.kp_ori
        return True
    

    def srvcllbck_getPoseSetPoint(self, req):
        ''' Service callback method for retrieving the pose set-point given 
        two 3D vectors (translation + orientation in euler-XYZ'''
        # setting new set-point
        return [dqExtractTransV3(self.dq_sp).reshape(1,3).tolist()[0] ,dq2rpy(self.dq_sp)]
    

    def srvcllbck_getPoseCtrlGain(self, req):
        '''Service that gets controller gains'''
        return [self.kp_tr, self.kp_rot]
    

    @staticmethod
    def getParamWithWait(path_param):
        """Waits until a param exists so retrieves it"""
        while not  rospy.has_param(path_param):
            rospy.loginfo("[manager] Waiting for param: %s", path_param)
        return rospy.get_param(path_param)


    @staticmethod
    def computeU2(flpJointPos_l, flpJointPosSp_l, kmu_l):
        aux = [ ki * (jsp - jcurr) for jcurr, jsp, ki in zip(flpJointPos_l, flpJointPosSp_l, kmu_l)]
        return np.array(aux).reshape(4,1)


if __name__ == '__main__':
    node_name = 'pose_reg_base_cmd_vel'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    rospy.loginfo('Actually, articulationC_control_1 node initiated!!! Testing purposes only.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass



    