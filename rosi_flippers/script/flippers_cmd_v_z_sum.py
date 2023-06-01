#!/usr/bin/env python3
'''this is a ROSI flippers node that sums velocities to the z vector in 
the propulsion frame.
'''
import rospy
from collections import deque

from rosi_common.msg import Float32Array

from rosi_common.node_status_tools import nodeStatus

class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        ##=== Parameters

        # node status object
        self.ns = nodeStatus(node_name)
        #self.ns.resetActive() # this node is disabled by default

        # maximum input queue size
        self.deque_queue_max_elements = 10

        # time window to discard last received command
        self.timeWindowToDiscardCmd = rospy.Duration.from_sec(0.2)

        ##=== Useful variables
        # callback messages
        self.msg_cmdz_joy = None
        self.msg_cmdz_lev = None


        ##=== ROS interfaces
        # publishers
        self.pub_cmdVel_cmdVelVzPi = rospy.Publisher('/rosi/flippers/space/cmd_v_z', Float32Array, queue_size=5)

        # subscribers
        sub_cmdVel_cmdVelVzPi_joy = rospy.Subscriber('/rosi/flippers/space/cmd_v_z/joy', Float32Array, self.cllbck_cmdVelVzPi_joy)
        sub_cmdVel_cmdVelVzPi_leveler = rospy.Subscriber('/rosi/flippers/space/cmd_v_z/leveler', Float32Array, self.cllbck_cmdVelVzPi_leveler)


        ##=== Main method 
        self.nodeMain()


    def nodeMain(self):
        '''Node main script'''

        # rate sleep
        node_rate_sleep = rospy.Rate(10)

        # eternal loop
        while not rospy.is_shutdown():

            if self.ns.getNodeStatus()['active']: # only runs if node is active

                # receives current ros time
                time_current = rospy.get_rostime()
                
                # empty command vector that will receive the sum
                cmd_v_z_sum = 4*[0]

                # treats the command received by the joystick subsystem
                if self.msg_cmdz_joy is not None:
                    if (time_current - self.msg_cmdz_joy.header.stamp) <= self.timeWindowToDiscardCmd: 
                            cmd_v_z_sum = [x+y for x,y in zip(cmd_v_z_sum, self.msg_cmdz_joy.data)]

                # treats the command received by the leveler subsystem
                if self.msg_cmdz_lev is not None:
                    if (time_current - self.msg_cmdz_lev.header.stamp) <= self.timeWindowToDiscardCmd: 
                            cmd_v_z_sum = [x+y for x,y in zip(cmd_v_z_sum, self.msg_cmdz_lev.data)]


                # publishing message
                m = Float32Array()
                m.header.stamp = time_current
                m.header.frame_id = self.node_name
                m.data = cmd_v_z_sum
                self.pub_cmdVel_cmdVelVzPi.publish(m)
                #print(m)


    def cllbck_cmdVelVzPi_joy(self, msg): 
        """ Callback for a ROS topic"""
        self.msg_cmdz_joy = msg      


    def cllbck_cmdVelVzPi_leveler(self, msg): 
        """ Callback for a ROS topic"""
        self.msg_cmdz_lev = msg


    """@staticmethod
    def sumsToCmdIfOk(cmd_in, queue, time_current, time_window):
        '''Pops an element from the queu and sums the command to cmd_l if it is
        within allowed time window
        Input:
            - cmd_in <list>: the list with current commands. This variable may be directly modified by reference
            - queue <collection.deque>: a queue with a buffer of received commands
            - time_current <genpy.time>: current ros time to compare with the last received message
            - time_window <genpy.duration>: the allowed time window
        Output
            - the summed command list <list>
            
        '''
        # treats for the command received by the joystick branch
        if len(queue) > 0: # treats if there is a valid value here
            msg_curr = queue.popleft()
            if time_current - msg_curr['stamp'] <= time_window:
                cmd_out = [x+y for x,y in zip(cmd_in, msg_curr['data'])]
                return cmd_out
            else:
                rospy.logwarn('input cmd discarded due time window is bigger than allowed.')
                return cmd_in
        else:
            return cmd_in"""


if __name__ == '__main__':
    node_name = 'flippers_cmd_v_z_sum'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass