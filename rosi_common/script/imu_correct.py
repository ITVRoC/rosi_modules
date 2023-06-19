#!/usr/bin/env python3
'''this node read the imu message and corrects the orientation

'''
import rospy
import numpy as np
import quaternion

from sensor_msgs.msg import Imu

from rosi_common.node_status_tools import nodeStatus
from rosi_common.dq_tools import rpy2quat

from rosi_common.srv import SetNodeStatus, GetNodeStatusList

class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        ##=== Parameters

        # imu orientation rotation  correction
        imu_correct_rpy_deg = [0, 0, 0]


        ##=== Useful variables
        # node status object
        self.ns = nodeStatus(node_name)
        #self.ns.resetActive() # this node is disabled by default

        ##=== One time calculations

        # creating the correction quaternion object
        q_corr_l = rpy2quat(np.deg2rad(imu_correct_rpy_deg))
        self.q_imu_offset = np.quaternion(q_corr_l[0], q_corr_l[1], q_corr_l[2], q_corr_l[3]) 

        ##=== ROS interfaces

        # publishers
        self.pub_imu = rospy.Publisher('/sensor/imu_corrected', Imu, queue_size=5)

        # subscribers
        sub_imu = rospy.Subscriber('/mti/sensor/imu', Imu, self.cllbck_imu)

        # services
        srv_setActive = rospy.Service(self.ns.getSrvPath('active', rospy), SetNodeStatus, self.srvcllbck_setActive)
        srv_getStatus = rospy.Service(self.ns.getSrvPath('getNodeStatus', rospy), GetNodeStatusList, self.srvcllbck_getStatus)
        srv_setHaltCmd = rospy.Service(self.ns.getSrvPath('haltcmd', rospy), SetNodeStatus, self.srvcllbck_setHaltCmd)

        ##=== Looping the node
        rospy.loginfo('[%s] node in entering in spin mode.', self.node_name)
        rospy.spin()


    def cllbck_imu(self, msg):
        '''IMU message callback method'''
        
        # rotating current IMU reading by the offset
        q_imu = np.quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
        q_corr = q_imu * self.q_imu_offset
       

        # mounting and publishing the message
        auxq = q_corr.components
        m = msg
        m.orientation.w = auxq[0]
        m.orientation.x = auxq[1]
        m.orientation.y = auxq[2]
        m.orientation.z = auxq[3]
        self.pub_imu.publish(m)
 

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


if __name__ == '__main__':
    node_name = 'imu_correct'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass



    