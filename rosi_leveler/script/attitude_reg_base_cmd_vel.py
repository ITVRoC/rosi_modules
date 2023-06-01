#!/usr/bin/env python3
''' This is a ROSI algorithm
It receives imu input, an orientation set-point, and provides and generates a 
rosi base cmdVel ctrl signal in operational space to correct it
'''
import rospy

from rosi_common.msg import TwistStamped
from sensor_msgs.msg import Imu

import numpy as np
import quaternion

from rosi_common.dq_tools import quat2rpy, rpy2quat, quatAssurePosW

from rosi_common.node_status_tools import nodeStatus
from rosi_common.srv import SetNodeStatus, GetNodeStatusList

class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        ##==== PARAMETERS

        # orientation set-point
        self.q_sp = np.quaternion(1, 0, 0, 0)

        # controller kp
        self.kp = 100

        # dead-band (the controller do not correct if error is below this value)
        self.deadBand = 0.001


        ##=== Useful variables
        # node status object
        self.ns = nodeStatus(node_name)

        ##==== ROS interfaces

        self.pub_imuCtrlSig = rospy.Publisher('/rosi/base/space/cmd_vel/ctrl_signal', TwistStamped, queue_size=5)

        sub_imu = rospy.Subscriber('/mti/sensor/imu', Imu, self.cllbck_imu)

        # services
        srv_setActive = rospy.Service(self.ns.getSrvPath('active', rospy), SetNodeStatus, self.srvcllbck_setActive)
        srv_getStatus = rospy.Service(self.ns.getSrvPath('getNodeStatus', rospy), GetNodeStatusList, self.srvcllbck_getStatus)

        rospy.spin()

    
    def cllbck_imu(self, msg):

         if self.ns.getNodeStatus()['active']: # only runs if node is active

            # creating a numpy quaternion element
            q_ori = np.quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)

            # converting the quaternion to roll-pitch-yaw format
            rpy_ori = quat2rpy(quaternion.as_float_array(q_ori))

            # creates a quaternion with the compensation of the yaw component, as the attitude reg controls only roll and pitch
            aux = rpy2quat([0, 0, -rpy_ori[2]])
            q_yaw = np.quaternion(aux[0], aux[1], aux[2], aux[3])

            # rotates the IMU input quaternion by the yaw component to zerate it
            q_ori_noYaw = q_ori * q_yaw

            # computing the attitude error
            q_error = self.q_sp.conj() * q_ori_noYaw

            # assuring the quaternion has a positive w (considering its double conver on R3 - we want ijk components signal to reflect the necessary rotation to zerate the error)
            q_error = quatAssurePosW(q_error)

            # extracting rotational velocity command signal from the error quaternion
            rot_ang_error = np.append(quaternion.as_float_array(q_error)[1:3], 0) # zero velocity to yaw

            # base rotation control signal (aroung x, y, and z base axis)
            baseRotCtrlSig = -rot_ang_error * self.kp

            # applyes the dead band (controller do not correct if it is below a threshold)
            baseRotCtrlSig_db = [cmd if abs(cmd) >= self.deadBand else 0.0 for cmd in baseRotCtrlSig]

            # publishing the Twist message
            m = TwistStamped()
            m.header.stamp = rospy.get_rostime()
            m.header.frame_id = 'ctrl_cmd_vel'
            m.twist.linear.x = 0.0
            m.twist.linear.y = 0.0
            m.twist.linear.z = 0.0
            m.twist.angular.x = baseRotCtrlSig_db[0]
            m.twist.angular.y = baseRotCtrlSig_db[1]
            m.twist.angular.z = baseRotCtrlSig_db[2] 
            self.pub_imuCtrlSig.publish(m)

            #print(m)


    ''' === Service Callbacks === '''
    def srvcllbck_setActive(self, req):
        ''' Method for setting the active node status flag'''
        return self.ns.defActiveServiceReq(req, rospy)

    def srvcllbck_getStatus(self, req):
        ''' Method for returning the node status flag list'''
        return self.ns.getNodeStatusSrvResponse()


if __name__ == '__main__':
    node_name = 'attitude_reg_base_cmd_vel'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass


    