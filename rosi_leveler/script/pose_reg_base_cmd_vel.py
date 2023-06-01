#!/usr/bin/env python3
''' This is a ROSI algorithm
It receives imu input, a pose set-point , and provides and generates a 
rosi base cmdVel ctrl signal in operational space to correct it
'''
import rospy

from rosi_common.msg import TwistStamped, Vector3ArrayStamped, DualQuaternionStamped
from sensor_msgs.msg import Imu

import numpy as np
import quaternion
from dqrobotics import *

from rosi_common.dq_tools import quat2rpy, rpy2quat, quatAssurePosW, trAndOri2dq, dq2rpy
from rosi_common.dq_tools import *

from rosi_common.node_status_tools import nodeStatus
from rosi_common.srv import SetNodeStatus, GetNodeStatusList, setPoseSetPointVec, setPoseCtrlGain, getPoseCtrlGain, getPoseSetPointVec

class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        ##==== PARAMETERS

        #-----------------------------------------------
        # Pose set-point
        sp_pos = [0, 0, 0.33]
        sp_ori = rpy2quat(np.deg2rad([0, 0, 0]))
        self.dq_sp = trAndOri2dq(sp_pos, sp_ori, 'trfirst')
        #-----------------------------------------------

        # corrective imu rotation
        # TODO retirar talvez esta correcao de quaternion. Ela so eh necessaria quando a IMU esta instalada em orientacao errada
        aux_imu_correct = rpy2quat(np.deg2rad([0, 0, 0]))
        self.q_imu_correct = np.quaternion(aux_imu_correct[0], aux_imu_correct[1], aux_imu_correct[2], aux_imu_correct[3]) 

        # rotational controller kp for each dof [rol, pitch, yaw]
        #self.kp_rot = [3.5, 6.5, 0]
        self.kp_rot = [2.0, 4.0, 0]

        # rotational dead-band (the controller do not correct if error is below this value)
        self.deadBand_rot = 0.01

        # translational controller kp for each dof [x, y, z]
        #self.kp_tr = [0, 0, 1.25]
        self.kp_tr = [0, 0, 0.8]

        # translational dead-band (the controller do not correct if error is below this value)
        self.deadBand_tr = 0.01

        # rosi direction side
        self.drive_side_param_path = '/rosi/forward_side'
        self.drive_side = self.getParamWithWait(self.drive_side_param_path)

    
        ##=== Useful variables
        # node status object
        self.ns = nodeStatus(node_name)
        self.ns.resetActive() # this node is disabled by default

        # stores IMU current orientation
        self.q_ori = None

        # stores the robot base distance from the ground
        self.p_grnd = None

        ##==== ROS interfaces

        # publishers
        self.pub_imuCtrlSig = rospy.Publisher('/rosi/base/space/cmd_vel/ctrl_signal', TwistStamped, queue_size=5)
        self.pub_dqError = rospy.Publisher('/rosi/base/pose_reg/dq_error', DualQuaternionStamped, queue_size=5)
        self.pub_dqSetPoint = rospy.Publisher('/rosi/base/pose_reg/set_point', DualQuaternionStamped, queue_size=5)
        self.pub_dqPoseCurr = rospy.Publisher('/rosi/base/pose', DualQuaternionStamped, queue_size=5)
        self.pub_ctrlGain = rospy.Publisher('/rosi/base/pose_reg/ctrl_gain', TwistStamped, queue_size=5)

        # subscribers
        sub_imu = rospy.Subscriber('/mti/sensor/imu', Imu, self.cllbck_imu)
        sub_grndDist = rospy.Subscriber('/rosi/model/base_ground_distance', Vector3ArrayStamped, self.cllbck_grndDist)

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

                # only runs control if at least one valid IMU and distance to ground messages has been received
                if self.q_ori is not None and self.p_grnd is not None :

                    # creating current pose dual-quaternion
                    dq_x = trAndOri2dq(self.p_grnd, self.q_ori*self.q_imu_correct, 'trfirst')

                    # converting the pose dq to roll-pitch-yaw format
                    rpy = dq2rpy(dq_x)

                    # creates a dual-quaternion with the compensation of the yaw component, as the pose reg controls only rol, pitch angles and vertical position
                    aux = rpy2quat([0, 0, -rpy[2]])
                    dq_transform_yaw = trAndOri2dq([0,0,0], aux, 'trfirst')

                    # rotates the pose dq by the yaw compensating element to zerate yaw component
                    dq_x_noYaw = dq_x * dq_transform_yaw

                    # computing the pose error
                    dq_error = self.dq_sp.conj() * dq_x_noYaw
                    
                    # extracting rotation quaternion and translation vector from the error dual-quaternion
                    tr_e, q_e = dq2trAndQuatArray(dq_error)


                    ##=== Generating rotational control signal
                    # assuring the quaternion has a positive w (considering its double conver on R3 - we want ijk components signal to reflect the necessary rotation to zerate the error)
                    q_e = quatAssurePosW(q_e)

                    # extracting rotational velocity command signal from the error quaternion
                    rot_ang_error = np.append(quaternion.as_float_array(q_e)[1:3], 0) # zero velocity to yaw

                    # computing rotational velocity control signal (aroung x, y, and z base axis)
                    baseRotCtrlSig = -rot_ang_error * self.kp_rot

                    # applyes the dead band to the rotational control signal  (controller do not correct if it is below a threshold)
                    baseRotCtrlSig_db = [cmd if abs(cmd) >= self.deadBand_rot else 0.0 for cmd in baseRotCtrlSig]
        

                    ##=== Generating translational control signal
                    # extracting only z component from translational error as flippers only cope with this dof
                    tr_e = np.array([0, 0, tr_e[2][0]])

                    # computing translational velocity control signal
                    baseTrCtrlSig = - tr_e * self.kp_tr 

                    # applyes the dead band to the translational control signal (controller do not correct if it is below a threshold)
                    baseTrCtrlSig_db = [cmd if abs(cmd) >= self.deadBand_tr else 0.0 for cmd in baseTrCtrlSig]

                    # receiving ROS time
                    ros_time = rospy.get_rostime()

                    # updates drive param
                    self.drive_side = rospy.get_param(self.drive_side_param_path)

                    ##=== Publishing the control command signal
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
                    self.pub_ctrlGain.publish(m)


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


if __name__ == '__main__':
    node_name = 'pose_reg_base_cmd_vel'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass



    