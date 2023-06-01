#!/usr/bin/env python3
import sys
import rospy
import rosnode
import numpy as np

from rosi_common.node_status_tools import nodeStatus
from rosi_common.srv import SetNodeStatus, GetNodeStatusList
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from rosi_common.gen3_tools import gen3RotFkin

from sensor_msgs.msg import Joy, JointState
from kortex_driver.msg import TwistCommand, Base_JointSpeeds, JointSpeed
from control_msgs.msg import GripperCommandActionGoal

from rosi_joy.srv import ModeChangeReq, ModeChangeReqResponse, ModeReq, ModeReqResponse

from rosi_common.rosi_tools import *

class NodeClass():

    # class constructor
    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        # checking if kinova gen3 nodes are running
        nodes_list = rosnode.get_node_names()
        if '/my_gen3_driver' in nodes_list:
            self.flag_gen3_nodesEnabled = True
        else:
            self.flag_gen3_nodesEnabled = False

        ##==== Useful variables
        # node status object
        self.ns = nodeStatus(node_name)
        self.ns.resetActive() # this node is disabled by default

        # parameters
        self._lin_spd_reduction_factor = 0.1
        self._ang_spd_reduction_factor = 0.3
        self._joint_spc_reduction_factor = 0.4
        self._finger_max_effort = 0.001
        self._finger_pos_open = 0.0
        self._finger_pos_closed = 0.8

        # constants
        self.op_modes = {
            "linear": int(0),
            "angular": int(1),
            "joints": int(2)
        }

        # default operation mode
        self.op_mode_current = self.op_modes["angular"]
        rospy.loginfo("[gen3] Gen3 cartesian command in angular mode.")

        # class variables initializatio
        self.gen3_cart_cmd = {"vx": 0.0, "vy": 0.0, "vz": 0.0,
                         "wx": 0.0, "wy": 0.0, "wz": 0.0, "finger_vel": 0.0, "ref_frame": 0}
        self.gen3_joint_cmd = 0.0
        self.gripper_state = "open"

        # buttons
        self.btn_gripper = {"current": 0, "last": 0}
        self.btn_modeLinear = {"current": 0, "last": 0}
        self.btn_modeAngular = {"current": 0, "last": 0}
        self.btn_modeJoint = {"current": 0, "last": 0}
        self.btn_chngJointSpcCmd = {"current": 0, "last": 0}
        self.btn_clearFault = {"current": 0, "last": 0}

        self.jointState = {'position':(), 'velocity':(), 'effort':()}
        self.node_enabled = True
        self.jointSpcCmdCurrIndex = 0
        

        # ROS topics
        self.sub_joy = rospy.Subscriber('/joy', Joy, self.callback_Joy)
        self.sub_gen3JointState = rospy.Subscriber('/base_feedback/joint_state', JointState, self.cllbck_gen3JointState)

        self.pub_gen3_cartCmd = rospy.Publisher('/in/cartesian_velocity', TwistCommand, queue_size=3)
        self.pub_gen3_jointsCmd = rospy.Publisher('/in/joint_velocity', Base_JointSpeeds, queue_size=3)
        self.pub_gripper = rospy.Publisher('/robotiq_2f_85_gripper_controller/gripper_cmd/goal', GripperCommandActionGoal, queue_size=1, latch=True)
        self.pub_clearFault = rospy.Publisher('/in/clear_faults', Empty, queue_size=3)

        # ROS services
        #s_modeChangeReq = rospy.Service("/"+self.node_name+"/change_cartesian_mode", ModeChangeReq, self.cllbckSrv_modeChangeReq)
        s_modeReq = rospy.Service("/"+self.node_name+"/request_cartesian_mode", ModeReq, self.cllbckSrv_modeReq)

        # node management services
        srv_getStatus = rospy.Service(self.ns.getSrvPath('getNodeStatus', rospy), GetNodeStatusList, self.srvcllbck_getStatus)
        srv_setActive = rospy.Service(self.ns.getSrvPath('active', rospy), SetNodeStatus, self.srvcllbck_setActive)
        srv_setHaltCmd = rospy.Service(self.ns.getSrvPath('haltcmd', rospy), SetNodeStatus, self.srvcllbck_setHaltCmd)

        # setting gripper to home position
        self.gripperCmd(self.gripper_state)

        # calls node main script
        self.nodeMain()


    def nodeMain(self):
        ''' Node main script'''

        ''' ==== ETERNAL LOOP ==== '''
        # defining the eternal loop frequency
        node_sleep_rate = rospy.Rate(10)

        # true loop
        rospy.loginfo("[gen3] Node in spin mode.")
        while not rospy.is_shutdown():

            # only works if node is enabled
            if self.ns.getNodeStatus()['active'] and self.flag_gen3_nodesEnabled == True: 

                # receiving ROS time
                time_ros = rospy.get_rostime()

                # in case of cartesian operational mode
                if self.op_mode_current == self.op_modes["linear"] or self.op_mode_current == self.op_modes["angular"]:
                    # mounting publishing message
                    m = TwistCommand()
                    m.reference_frame = self.gen3_cart_cmd["ref_frame"] 
                    if not self.ns.getNodeStatus()['haltcmd']:  
                        m.twist.linear_x = self.gen3_cart_cmd["vx"]
                        m.twist.linear_y = self.gen3_cart_cmd["vy"]
                        m.twist.linear_z = self.gen3_cart_cmd["vz"]
                        m.twist.angular_x = self.gen3_cart_cmd["wx"]
                        m.twist.angular_y = self.gen3_cart_cmd["wy"]
                        m.twist.angular_z = self.gen3_cart_cmd["wz"]
                    else:   # sends halt cmd if node is halted
                        m.twist.linear_x = 0.0
                        m.twist.linear_y = 0.0
                        m.twist.linear_z = 0.0
                        m.twist.angular_x = 0.0
                        m.twist.angular_y = 0.0
                        m.twist.angular_z = 0.0

                    # publishing the cmd
                    self.pub_gen3_cartCmd.publish(m)
                    #print(m)

                elif self.op_mode_current == self.op_modes["joints"]:

                    mjs = JointSpeed()
                    js_l = [JointSpeed(id, 0.0, 0) for id in range(7)]
                    if not self.ns.getNodeStatus()['haltcmd']: # only sends the command if the robot is not halted
                        js_l[self.jointSpcCmdCurrIndex].value = self.gen3_joint_cmd 
                    m = Base_JointSpeeds()
                    m.joint_speeds = js_l
                    m.duration = 0
                    self.pub_gen3_jointsCmd.publish(m)
                    #print(m)
                    
            # sleep for a while
            node_sleep_rate.sleep()


    # joystick callback function
    def callback_Joy(self, msg):

        # only sends commands if node is enabled 
        if self.ns.getNodeStatus()['active'] and not self.ns.getNodeStatus()['haltcmd']:

            # destrinchando axis joints variables
            # ** trecho util no caso da troca de joysticks
            ax_lx = msg.axes[0]
            ax_ly = msg.axes[1]
            ax_rx = msg.axes[3]
            ax_ry = msg.axes[4]
            ax_z = (msg.axes[2] - msg.axes[5])/2
            self.btn_gripper["current"] = msg.buttons[5] # R1

            # updating btn variables values
            self.btn_modeLinear["current"] = msg.buttons[2] # quadrado
            self.btn_modeAngular["current"] = msg.buttons[0] # X
            self.btn_modeJoint["current"] = msg.buttons[1]  # bolinha
            self.btn_chngJointSpcCmd["current"] = msg.buttons[3] #triangulo       
            self.btn_clearFault["current"] = msg.buttons[9]


            # defining gen3 cartesian commands based on current cartesian operational mode (linear or angular)
            if self.op_mode_current == self.op_modes["linear"]:

                # mounting the velocity command vector from joystick input
                v_tcp = np.array([  self._lin_spd_reduction_factor * ax_lx, 
                                    self._lin_spd_reduction_factor * ax_ly, 
                                    self._lin_spd_reduction_factor * ax_ry]).reshape(3,1)

                # computes the tcp rotation of tcp wrt base with direct kinematics
                rot_tcp_base = gen3RotFkin(self.jointState['position'])

                # puts the velocity in the base frase
                v_base = rot_tcp_base.dot(v_tcp)

                # fills the Twist command
                self.gen3_cart_cmd["vx"] = v_base[0][0]
                self.gen3_cart_cmd["vy"] = v_base[1][0]
                self.gen3_cart_cmd["vz"] = v_base[2][0]
                self.gen3_cart_cmd["wx"] = 0.0
                self.gen3_cart_cmd["wy"] = 0.0
                self.gen3_cart_cmd["wz"] = 0.0
                self.gen3_cart_cmd["ref_frame"] = 0 # base reference frame

            elif self.op_mode_current == self.op_modes["angular"]:

                # fills the Twist command
                self.gen3_cart_cmd["vx"] = 0.0
                self.gen3_cart_cmd["vy"] = 0.0
                self.gen3_cart_cmd["vz"] = 0.0
                self.gen3_cart_cmd["wx"] = self._ang_spd_reduction_factor * ax_ly
                self.gen3_cart_cmd["wy"] = self._ang_spd_reduction_factor * ax_rx
                self.gen3_cart_cmd["wz"] = -self._ang_spd_reduction_factor * ax_lx
                self.gen3_cart_cmd["ref_frame"] = 8 # base reference frame


            # treats the gripper button command
            # btn has just been pressed
            if self.btn_gripper["current"] == 1 and self.btn_gripper["last"] == 0:
                self.gripperChangeState()


            # changes to requested gen3 mode
            if self.btn_modeAngular['current']==1 and self.btn_modeAngular['last']==0:
                if not self.op_mode_current == self.op_modes["angular"]:
                    rospy.loginfo('[gen3] Gen3 in angular mode.')
                    self.op_mode_current = self.op_modes["angular"]
                else: 
                    rospy.logwarn("[gen3] Current mode is already angular mode.")
        
            elif self.btn_modeLinear['current']==1 and self.btn_modeLinear['last']==0:
                if not self.op_mode_current == self.op_modes["linear"]:
                    rospy.loginfo('[gen3] Gen3 in linear mode.')
                    self.op_mode_current = self.op_modes["linear"]
                else: 
                    rospy.logwarn("[gen3] Current mode is already linear mode.")

            elif self.btn_modeJoint['current']==1 and self.btn_modeJoint['last']==0:
                if not self.op_mode_current == self.op_modes["joints"]:
                    rospy.loginfo('[gen3] Gen3 in joint space mode.')
                    self.op_mode_current = self.op_modes["joints"]
                    self.jointSpcCmdCurrIndex = 0
                    rospy.loginfo("[gen3] Current active joint: %d", self.jointSpcCmdCurrIndex+1)
                else: 
                    rospy.logwarn("[gen3] Current mode is already joints mode.")


            # for treating currently joint active in joint space
            if self.op_mode_current == self.op_modes["joints"]:
                # treats the change of joint index to control in joint space operational mode
                if self.btn_chngJointSpcCmd["current"] == 1 and self.btn_chngJointSpcCmd["last"] == 0:
                    self.jointSpcCmdCurrIndex += 1
                    if self.jointSpcCmdCurrIndex > 6:
                        self.jointSpcCmdCurrIndex = 0
                    rospy.loginfo("[gen3] Current active joint: %d", self.jointSpcCmdCurrIndex+1)

                # saves the joint cmd
                self.gen3_joint_cmd = ax_z * self._joint_spc_reduction_factor
            

            # button for clearing the fault state
            if self.btn_clearFault["current"] == 1 and self.btn_clearFault["last"] == 0:
                m = Empty()
                self.pub_clearFault.publish(m)
            
            ###=== UPDATING BUTTONS
            self.btn_gripper["last"] = self.btn_gripper["current"]
            self.btn_modeLinear["last"] = self.btn_modeLinear["current"]
            self.btn_modeAngular["last"] = self.btn_modeAngular["current"]
            self.btn_modeJoint["last"] = self.btn_modeJoint["current"]
            self.btn_chngJointSpcCmd["last"] = self.btn_chngJointSpcCmd["current"]      
            self.btn_clearFault["last"] = self.btn_clearFault["current"]


    def cllbck_gen3JointState(self, msg):
        '''Saves gen3 joint states in a list'''
        self.jointState['position'] = msg.position
        self.jointState['velocity'] = msg.velocity
        self.jointState['effort'] = msg.effort   


    def cllbckSrv_modeChangeReq(self, req):
        """ Service to treat changes in current gen3 cartesian mode
        Modes are
            0: linear
            1: angular
        """

        # creating the response msg
        r = ModeChangeReqResponse()

        # treating the service request
        if req.req_mode == self.op_modes["linear"]:
            self.op_mode_current = self.op_modes["linear"]
            rospy.loginfo("[gen3] Gen3 cartesian command in linear mode.")
            r.ok = True

        elif req.req_mode == self.op_modes["angular"]:
            self.op_mode_current = self.op_modes["angular"]
            rospy.loginfo("[gen3] Gen3 cartesian command in angular mode.")
            r.ok = True

        else:
            self.op_mode_current = self.op_modes["linear"]
            rospy.loginfo("[gen3] Cartesian service received a bad mode request. Setting to defaul linear mode")
            r.ok = False

        # giving a response
        r.current_mode = self.op_mode_current
        return r 


    def cllbckSrv_modeReq(self, req):
        """ Service for requesting gen3 current cartesian linear mode"""

        # creating the response msg
        r = ModeReqResponse()
        r.current_mode = self.op_mode_current
        return r


    def gripperCmd(self, cmd):
        '''Method for opening/closing the finger'''

        # mounting goal msg
        g = GripperCommandActionGoal()
        g.goal.command.max_effort = self._finger_max_effort

        if cmd == "close":
            g.goal.command.position = self._finger_pos_closed
        elif cmd == "open":
            g.goal.command.position = self._finger_pos_open
        else:
            rospy.loginfo("[gen3] gripperCmd received wrong command. Opening the gripper by default.")
            g.goal.command.position = self._finger_pos_open

        # only sends command if someone is subscribed to the topic
        while self.pub_gripper.get_num_connections() == 0 and self.flag_gen3_nodesEnabled == True:
            rospy.loginfo("[gen3] Waiting for subscribers in gripper/goal topic for publishing.")
            rospy.sleep(0.5)

        # publishes the command
        self.pub_gripper.publish(g)


    def gripperChangeState(self):
        ''' Changes the gripper current state (open/close)'''

        if self.gripper_state == "open":
            self.gripperCmd("close")
            self.gripper_state = "close"
            rospy.loginfo("[gen3] closing gripper.")

        elif self.gripper_state == "close":
            self.gripperCmd("open")
            self.gripper_state = "open"
            rospy.loginfo("[gen3] opening gripper.")


    '''
    def swapOpMode(self):
        """ Method that swaps current cartesian mode """

        # halts the arm
        for i in range(5):
            m = TwistCommand()
            m.reference_frame = 8 
            m.twist.linear_x = 0.0
            m.twist.linear_y = 0.0
            m.twist.linear_z = 0.0
            m.twist.angular_x = 0.0
            m.twist.angular_y = 0.0
            m.twist.angular_z = 0.0
            self.pub_gen3_cartCmd.publish(m)

        # changes the mode flag
        if  self.op_mode_current == self.op_modes["angular"]:
            self.op_mode_current = self.op_modes["linear"]
            rospy.loginfo("Gen3 in cartesian linear mode.")

        elif self.op_mode_current == self.op_modes["linear"]:
            self.op_mode_current = self.op_modes["joints"]
            rospy.loginfo("Gen3 in joint space mode.")
            rospy.loginfo("Current joint in control: %d", self.jointSpcCmdCurrIndex+1)

        elif  self.op_mode_current == self.op_modes["joints"]:
            self.op_mode_current = self.op_modes["angular"]
            rospy.loginfo("Gen3 in cartesian angular mode.")'''

                
    ''' === Service Callbacks === '''
    def srvcllbck_getStatus(self, req):
        ''' Method for returning the node status flag list'''
        return self.ns.getNodeStatusSrvResponse()


    def srvcllbck_setActive(self, req):
        ''' Method for setting the active node status flag'''
        return self.ns.defActiveServiceReq(req, rospy)


    def srvcllbck_setHaltCmd(self, req):
        ''' Method for setting the haltCmd node status flag'''
        return self.ns.defHaltCmdServiceReq(req, rospy)


if __name__ == '__main__':
    node_name = 'joy_gen3'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass