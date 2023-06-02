#!/usr/bin/env python3
''' This is a ROSI algorithm
    It receives Twist velocity commands to the base from different sources and sums them up
'''
import rospy

from rosi_common.msg import TwistStamped

import numpy as np


from rosi_common.node_status_tools import nodeStatus
from rosi_common.srv import SetNodeStatus, GetNodeStatusList

class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        ##=== PARAMETERS
        # time window to discard last received command
        self.discardWindow = rospy.Duration.from_sec(0.3)

        # rosi direction side
        self.drive_side_param_path = '/rosi/forward_side'
        self.drive_side = self.getParamWithWait(self.drive_side_param_path)

        ##=== Useful variables
        # node status object
        self.ns = nodeStatus(node_name)

        # possible frame_ids to require ROSI base Twist cmdvel commands
        self.pubNodes = (
                    'pose_reg_base_cmd_vel',
                    'joy_base_cmdVel',
                    'autnav'
        )

        # last TwistStamped received from each possible node
        self.lastTwist = {
                    self.pubNodes[0]: TwistStamped(),
                    self.pubNodes[1]: TwistStamped(),
                    self.pubNodes[2]: TwistStamped()
        }

        ##==== ROS interfaces

        # publisher
        self.pub_baseSpaceCmdVel = rospy.Publisher('/rosi/base/space/cmd_vel', TwistStamped, queue_size=5)

        # subscribers
        sub_cmdVelJoy = rospy.Subscriber('/rosi/base/space/cmd_vel/joy', TwistStamped, self.cllbck_cmdVelJoy)
        sub_cmdVelCtrl = rospy.Subscriber('/rosi/base/space/cmd_vel/ctrl_signal', TwistStamped, self.cllbck_cmdVelCtrl)
        sub_cmdAutNav = rospy.Subscriber('/rosi/base/space/cmd_vel/autnav', TwistStamped, self.cllbck_cmdAutNav )

        # services
        srv_setActive = rospy.Service(self.ns.getSrvPath('active', rospy), SetNodeStatus, self.srvcllbck_setActive)
        srv_getStatus = rospy.Service(self.ns.getSrvPath('getNodeStatus', rospy), GetNodeStatusList, self.srvcllbck_getStatus)

        # calling node main method
        self.nodeMain()


    def nodeMain(self):
        '''Node main method'''

        # node rate sleep
        node_rate_sleep = rospy.Rate(10)

        rospy.loginfo('entering in eternal loop.')
        while not rospy.is_shutdown():

            if self.ns.getNodeStatus()['active']: # only runs if node is active

                # auxiliary sums arrays
                cmd_lin = np.zeros(3)
                cmd_ang = np.zeros(3)

                # receives ros time
                time_current = rospy.get_rostime()

                '''print('---')
                print(time_current)
                print(self.lastTwist[self.pubNodes[0]].header.stamp)
                print(time_current - self.lastTwist[self.pubNodes[0]].header.stamp)
                print(self.discardWindow)'''


                # sums last received twist from different sources if they are within the time window
                if (time_current - self.lastTwist[self.pubNodes[0]].header.stamp) <= self.discardWindow: # for the ctrl_cmd_vel node
                    cmd_lin, cmd_ang = self.sumTwist(cmd_lin, cmd_ang, self.lastTwist[self.pubNodes[0]])

                if (time_current - self.lastTwist[self.pubNodes[1]].header.stamp) <= self.discardWindow:
                    cmd_lin, cmd_ang = self.sumTwist(cmd_lin, cmd_ang, self.lastTwist[self.pubNodes[1]]) # for the joy_base_cmdVel node

                if (time_current - self.lastTwist[self.pubNodes[2]].header.stamp) <= self.discardWindow:
                    cmd_lin, cmd_ang = self.sumTwist(cmd_lin, cmd_ang, self.lastTwist[self.pubNodes[2]]) # for the autonomous navigation node


                # updates drive param
                self.drive_side = rospy.get_param(self.drive_side_param_path)
                
                # mounting message to publish
                if self.drive_side == 'a':
                    m = TwistStamped()
                    m.header.stamp = time_current
                    m.header.frame_id = 'base_cmdVel_sum'
                    m.twist.linear.x = cmd_lin[0]
                    m.twist.linear.y = cmd_lin[1]
                    m.twist.linear.z = cmd_lin[2]
                    m.twist.angular.x = cmd_ang[0]
                    m.twist.angular.y = cmd_ang[1]
                    m.twist.angular.z = cmd_ang[2]
                    self.pub_baseSpaceCmdVel.publish(m)

                elif self.drive_side == 'b':
                    m = TwistStamped()
                    m.header.stamp = time_current
                    m.header.frame_id = 'base_cmdVel_sum'
                    m.twist.linear.x = -cmd_lin[0]
                    m.twist.linear.y = cmd_lin[1]
                    m.twist.linear.z = cmd_lin[2]
                    m.twist.angular.x = -cmd_ang[0]
                    m.twist.angular.y = -cmd_ang[1]
                    m.twist.angular.z = cmd_ang[2]
                    self.pub_baseSpaceCmdVel.publish(m)
                else:
                    rospy.logerr("[base_cmd_vel_sum] wrong param "+self.drive_side_param_path+" value. Valid values are 'a' and 'b'")

                #print(m)

            # sleeping the node
            node_rate_sleep.sleep()


    def cllbck_cmdVelJoy(self, msg):
        '''Callback for the joystick Twist command topic'''
        self.lastTwist[msg.header.frame_id] = msg


    def cllbck_cmdVelCtrl(self, msg):
        ''' Callback for the controller Twist command topic'''
        self.lastTwist[msg.header.frame_id] = msg


    def cllbck_cmdAutNav(self, msg):
        ''' Callback for the autonomous navigation (Gilmarzaooo) Twist command topic'''
        self.lastTwist[msg.header.frame_id] = msg


    ''' === Service Callbacks === '''
    def srvcllbck_setActive(self, req):
        ''' Method for setting the active node status flag'''
        return self.ns.defActiveServiceReq(req, rospy)

    def srvcllbck_getStatus(self, req):
        ''' Method for returning the node status flag list'''
        return self.ns.getNodeStatusSrvResponse()

    @staticmethod
    def sumTwist(cmd_lin, cmd_ang, t_in):
        cmd_lin += np.array([t_in.twist.linear.x, t_in.twist.linear.y, t_in.twist.linear.z])
        cmd_ang += np.array([t_in.twist.angular.x, t_in.twist.angular.y, t_in.twist.angular.z])
        return cmd_lin, cmd_ang
    

    @staticmethod
    def getParamWithWait(path_param):
        """Waits until a param exists so retrieves it"""
        while not  rospy.has_param(path_param):
            rospy.loginfo("[manager] Waiting for param: %s", path_param)
        return rospy.get_param(path_param)


if __name__ == '__main__':
    node_name = 'base_cmdVel_sum'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass


    