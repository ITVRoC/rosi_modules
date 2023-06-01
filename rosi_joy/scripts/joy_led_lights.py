#!/usr/bin/env python3
'''this is a ROSI node to command led lights from the joystick

'''
import rospy

from std_msgs.msg import Int32
from sensor_msgs.msg import Joy

class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        ###=== PARAMETERS

        # rosi direction side
        self.drive_side_param_path = '/rosi/forward_side'
        self.drive_side = self.getParamWithWait(self.drive_side_param_path)


        ###=== USEFUL VARIABLES
        self.btn_frontLight = {"current": 0, "last": 0}
        self.btn_backLight = {"current": 0, "last": 0}
        
        self.frontLight_value = 0
        self.backLight_value = 0

        ###=== ROS INTERFACES

        self.pub_frontLight = rospy.Publisher('/frontLight', Int32, queue_size=5)
        self.pub_backLight = rospy.Publisher('/backLight', Int32, queue_size=5)

        sub_Joy = rospy.Subscriber('/joy', Joy, self.cllbck_joy)


        # spinning the node
        rospy.loginfo('['+self.node_name+'] entering in eternal loop.')
        rospy.spin()


    def cllbck_joy(self, msg):
        """Joy callback method"""

        # updates drive param
        self.drive_side = rospy.get_param(self.drive_side_param_path)

        # updates buttons state
        if self.drive_side == 'a':
            self.btn_frontLight["current"] = msg.buttons[10]
            self.btn_backLight["current"] = msg.buttons[9]
        else:
            self.btn_frontLight["current"] = msg.buttons[9]
            self.btn_backLight["current"] = msg.buttons[10]


        # updates front light
        if self.btn_frontLight["current"] == 1 and self.btn_frontLight["last"] == 0:
            self.frontLight_value = self.changeLightValue(self.frontLight_value)
            self.pub_frontLight.publish(self.mountInt32Msg(self.frontLight_value))
            rospy.loginfo('updating front light to '+str(self.frontLight_value)+'%.')

        # updates back light
        if self.btn_backLight["current"] == 1 and self.btn_backLight["last"] == 0:
            self.backLight_value = self.changeLightValue(self.backLight_value)
            self.pub_backLight.publish(self.mountInt32Msg(self.backLight_value))
            rospy.loginfo('updating back light to '+str(self.backLight_value)+'%.')
            

        # updates last button state
        self.btn_frontLight["last"] = self.btn_frontLight["current"]
        self.btn_backLight["last"] = self.btn_backLight["current"]
        

    @staticmethod
    def changeLightValue(val):
        """Simply updates val to discrete state"""
        if val == 0:
            val = 33
        elif val == 33:
            val = 66
        elif val == 66:
            val = 90
        else:
            val = 0
        return val
    
    @staticmethod
    def mountInt32Msg(val):
        m = Int32()
        m.data = val
        return m


    @staticmethod
    def getParamWithWait(path_param):
        """Waits until a param exists so retrieves it"""
        while not  rospy.has_param(path_param):
            rospy.loginfo("[manager] Waiting for param: %s", path_param)
        return rospy.get_param(path_param)

if __name__ == '__main__':
    node_name = 'joy_led_lights'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass