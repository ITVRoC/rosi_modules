#!/usr/bin/env python3
'''this is a ROSI analysis node
It generates topics proper for RVIZ plot containing base command vectors

'''
import rospy

from geometry_msgs.msg import PolygonStamped, Polygon, Point32
from rosi_common.msg import Vector3ArrayStamped


class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        ##=== Useful variables
        self.msg_cntctPnt = None


        ##=== ROS interfaces
        self.pub_vis_cntctPln = rospy.Publisher('/rosi/visual/support_polygon', PolygonStamped, queue_size=5)

        sub_cntctPnt = rospy.Subscriber('/rosi/model/contact_point_wrt_base', Vector3ArrayStamped, self.cllbck_cntctPnt)

        ##=== Node main method
        self.nodeMain()


    def nodeMain(self):
        '''Node main method'''

        # rate sleep
        node_rate_sleep = rospy.Rate(3)

        while not rospy.is_shutdown():
            
            if self.msg_cntctPnt is not None: # only runs if valid messages has been received

                # message for publishing
                m = PolygonStamped()
                m.header.stamp = rospy.get_rostime()
                m.header.frame_id = 'dq_world_base'
                m.polygon.points = [(Point32(a.x,a.y,a.z)) for a in self.msg_cntctPnt.vec]

                # publishing the polygon
                self.pub_vis_cntctPln.publish(m)

                #print(m.header.stamp)


    def cllbck_cntctPnt(self, msg):
        '''Callback for the ground contact points wrt frame {R}'''

        # swaping two last elements for a proper plot of polygon in Rviz
        msg.vec[2], msg.vec[3] = msg.vec[3], msg.vec[2]
        self.msg_cntctPnt = msg


if __name__ == '__main__':
    node_name = 'disp_rviz_cntct_pln'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass