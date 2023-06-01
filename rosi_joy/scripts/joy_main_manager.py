#!/usr/bin/env python3
import rospy

from rosi_common.node_status_tools import nodeStatus
from rosi_common.srv import *
from sensor_msgs.msg import Joy

class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        ##==== Useful variables
        # node status object
        self.ns = nodeStatus(node_name)

        # button holders
        self.btn_start = {"current": 0, "last": 0}
        self.btn_select = {"current": 0, "last": 0}
        self.btn_dpad_left = {"current": 0, "last": 0}
        self.btn_dpad_right = {"current": 0, "last": 0}
        self.btn_dpad_up = {"current": 0, "last": 0}
        self.btn_dpad_down = {"current": 0, "last": 0}

        # rosi_main_manager operative modes
        self.d_operative_mode = {
            'base_flp_jnt_spc': 1,
            'base_flp_base_spc': 2,
            'base_posture_cntrl': 3,
            'gen3': 4
        }

        ###=== ROS SERVICES
        self.sh_getOnOff = self.getServiceHandle('/rosi/manager/get_on_off', GetNodeStatus)
        self.sh_setOnOff = self.getServiceHandle('/rosi/manager/set_on_off', SetNodeStatus)
        self.sh_getOpState = self.getServiceHandle('/rosi/manager/get_operational_state', GetInt)
        self.sh_setOpState = self.getServiceHandle('/rosi/manager/set_operational_state', SetNodeIntState)

        ###=== ROS INTERFACES
        self.sub_joy = rospy.Subscriber('/joy', Joy, self.cllbck_joy)

        ###=== NODE SPIN
        rospy.loginfo("Node in spin mode.")
        rospy.spin()


    def cllbck_joy(self, msg):
        '''Callback for the joy message'''

        ###=== RETRIEVING CURRENT BUTTON STATES
        self.btn_start['current'] = msg.buttons[7]
        self.btn_select['current'] = msg.buttons[6]
        self.btn_dpad_left['current'] = 1 if msg.axes[6] == 1 else 0 # flp joint space mode
        self.btn_dpad_right['current'] = 1 if msg.axes[6] == -1 else 0  # flp base space mode
        self.btn_dpad_up['current'] = 1 if msg.axes[7] == 1 else 0  # posture controller mode
        self.btn_dpad_down['current'] = 1 if msg.axes[7] == -1 else 0   # gen3 mode


        ###=== IN CASE OF REQUEST TO POWER ON/OFF THE ROBOT
        if self.btn_start['current']==1 and self.btn_start['last']==0 and self.btn_select['current']==1 and self.btn_select['last']:
            state_onOff = self.sh_getOnOff().node_status
            if state_onOff == False: # to turn on the robot
                ret = self.sh_setOnOff(True)
                if ret.return_status:
                    rospy.loginfo('ROSI is now operating!')
                else:
                    rospy.loginfo('Something went wrong. The main manager did not turn ON the robot.')
            elif state_onOff == True: # to turn off the robot
                ret = self.sh_setOnOff(False)
                if not ret.return_status:
                    rospy.loginfo('ROSI has turned OFF!')
                else:
                    rospy.loginfo('Something went wrong. The main manager did not turn OFF the robot.')

        
        ###=== FOR SELECTING DESIRED MODE
        if self.btn_dpad_left['current']==1 and  self.btn_dpad_left['last']==0:
            rospy.loginfo('Setting ROSI to base_flp_jnt_spc operative mode.')
            self.sh_setOpState(self.d_operative_mode['base_flp_jnt_spc'])

        elif self.btn_dpad_up['current']==1 and  self.btn_dpad_up['last']==0:
            rospy.loginfo('Setting ROSI to base_flp_base_spc operative mode.')
            self.sh_setOpState(self.d_operative_mode['base_flp_base_spc'])
            
        elif self.btn_dpad_right['current']==1 and  self.btn_dpad_right['last']==0:
            rospy.loginfo('Setting ROSI to base_posture_cntrl operative mode.')
            self.sh_setOpState(self.d_operative_mode['base_posture_cntrl'])

        elif self.btn_dpad_down['current']==1 and  self.btn_dpad_down['last']==0:
            rospy.loginfo('Setting ROSI to gen3 operative mode.')
            self.sh_setOpState(self.d_operative_mode['gen3'])

        ###=== UPDATING BUTTON STATES
        self.btn_start['last'] = self.btn_start['current'] 
        self.btn_select['last'] = self.btn_select['current']
        self.btn_dpad_left['last'] =self.btn_dpad_left['current']
        self.btn_dpad_right['last'] = self.btn_dpad_right['current']
        self.btn_dpad_up['last'] =self.btn_dpad_up['current'] 
        self.btn_dpad_down['last'] = self.btn_dpad_down['current'] 
        

    @staticmethod
    def getServiceHandle(service_path, service_type):
        '''Retrieves a service proxi after confirming it exists
        Input:
            - h_node_status: handler to the service that retrieves the node status'''
        rospy.loginfo("Waiting for service: %s", service_path)
        rospy.wait_for_service(service_path)
        rospy.loginfo(" Service %s found!", service_path)
        return rospy.ServiceProxy(service_path, service_type)

    
if __name__ == '__main__':
    node_name = 'joy_main_manager'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass