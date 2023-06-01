#!/usr/bin/env python3
''' This is a ROSI model algorithm
It receives ground contact point vector w.r.t. rosi base frame. 
Then, it computes planes from theses point and obtains and publishes the distances from {R} to found planes.
'''
import rospy

import itertools

from rosi_common.msg import Vector3ArrayStamped
from geometry_msgs.msg import Vector3

from rosi_common.geometry_tools import plane_from_three_points, dist_point_to_plane_along_vector, dist_frame_origin_to_plane

from rosi_common.node_status_tools import nodeStatus
from rosi_common.srv import SetNodeStatus, GetNodeStatusList


class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        ##=== Parameters

        # number of contact points to generate planes
        n_contact_points = 4

        # method for publishing the distance
        self.chosen_dist = 'max' # 'max' publishes the greater distance to the floor found. 'min' publishes the minor

        ##=== Useful variables
        # node status object
        self.ns = nodeStatus(node_name)

        ##=== Computations

        #  analysis of all possible four points indexes 3-groups combinations for computing the plane
        self.combIndex = list(itertools.combinations(list(range(0,n_contact_points)), 3))

        ##=== ROS Interfaces
        self.pub_grndDist = rospy.Publisher('/rosi/model/base_ground_distance', Vector3ArrayStamped, queue_size=5)

        sub_cntctPnt = rospy.Subscriber('/rosi/model/contact_point_wrt_base', Vector3ArrayStamped, self.cllbck_cntctPnt)

        # services
        srv_setActive = rospy.Service(self.ns.getSrvPath('active', rospy), SetNodeStatus, self.srvcllbck_setActive)
        srv_getStatus = rospy.Service(self.ns.getSrvPath('getNodeStatus', rospy), GetNodeStatusList, self.srvcllbck_getStatus)

        # spinning the node
        rospy.loginfo('Node in spin mode.')
        rospy.spin()


    def cllbck_cntctPnt(self, msg):
        '''Callback message for received contact points'''
        
        if self.ns.getNodeStatus()['active']: # only runs if node is active

            # storing n contact points in a list
            points = [[p.x, p.y, p.z] for p in msg.vec]

            # computing generated planes for all possible combinations among the n contact points
            planes = [plane_from_three_points(points[ind[0]], points[ind[1]], points[ind[2]]) for ind in self.combIndex]

            # computes the distance from frame {R} origin to computed planes
            distances = [dist_frame_origin_to_plane(plane_i) for plane_i in planes]
            #distances = [l[0] for l in [dist_point_to_plane_along_vector([0,0,0], plane_i, [0,0,-1]) for plane_i in planes]] # we want only the first return from the function
            
            # discovers the distance to the ground based on desired method
            d_grnd = max(distances) if self.chosen_dist == 'max' else min(distances)

            # publishing the message
            m = Vector3ArrayStamped()
            m.header.stamp = rospy.get_rostime()
            m.header.frame_id = self.node_name
            m.vec = [Vector3(0, 0, d_grnd)] # sends the distance to the ground as a 3D vector
            self.pub_grndDist.publish(m)
            
            #print(m)


    ''' === Service Callbacks === '''
    def srvcllbck_setActive(self, req):
        ''' Method for setting the active node status flag'''
        return self.ns.defActiveServiceReq(req, rospy)

    def srvcllbck_getStatus(self, req):
        ''' Method for returning the node status flag list'''
        return self.ns.getNodeStatusSrvResponse()



if __name__ == '__main__':
    node_name = 'base_ground_distance_from_contact_point'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass


    