#!/usr/bin/env python3
''' This is a ROSI model algorithm
It receives ground contact point vector w.r.t. rosi base frame. 
Then, it computes planes from theses point and obtains and publishes the distances from {R} to found planes.
'''
import rospy

import itertools
import numpy as np

from rosi_common.msg import Vector3ArrayStamped
from geometry_msgs.msg import Vector3, Vector3Stamped

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

        # node rate sleep
        self.p_rateSleep = 10

        ##=== Useful variables
        # node status object
        self.ns = nodeStatus(node_name)

        self.msg_gravVec = None
        self.msg_cntctPnt = None

        ##=== Computations

        #  analysis of all possible four points indexes 3-groups combinations for computing the plane
        self.combIndex = list(itertools.combinations(list(range(0,n_contact_points)), 3))

        ##=== ROS Interfaces
        # publishers
        self.pub_grndDist = rospy.Publisher('/rosi/model/base_ground_distance', Vector3ArrayStamped, queue_size=5)
        self.pub_cntctPlnVec = rospy.Publisher('/rosi/model/contact_plane_normal_vec', Vector3Stamped, queue_size=5)

        # subscribers
        sub_cntctPnt = rospy.Subscriber('/rosi/model/contact_point_wrt_base', Vector3ArrayStamped, self.cllbck_cntctPnt)
        sub_gravVec = rospy.Subscriber('/rosi/model/grav_vec_wrt_frame_r', Vector3Stamped, self.cllbck_gravVec)
        

        # services
        srv_setActive = rospy.Service(self.ns.getSrvPath('active', rospy), SetNodeStatus, self.srvcllbck_setActive)
        srv_getStatus = rospy.Service(self.ns.getSrvPath('getNodeStatus', rospy), GetNodeStatusList, self.srvcllbck_getStatus)

        # spinning the node
        self.nodeMain()


    def nodeMain(self):
        '''Node main function'''

        while not rospy.is_shutdown():

            # defining the eternal loop rate
            node_rate_sleep = rospy.Rate(self.p_rateSleep)

            if self.ns.getNodeStatus()['active']: # only runs if node is active
                if self.msg_gravVec is not None and self.msg_cntctPnt is not None: # only runs if valid messages have been received
                    
                    # storing n contact points in a list
                    points = [[p.x, p.y, p.z] for p in self.msg_cntctPnt.vec]

                    # computing generated planes for all possible combinations among the n contact points
                    planes = [plane_from_three_points(points[ind[0]], points[ind[1]], points[ind[2]]) for ind in self.combIndex]

                    # computes the distance from frame {R} origin to computed planes
                    distances = [dist_frame_origin_to_plane(plane_i) for plane_i in planes]
                    
                    # discovers the distance to the ground based on desired method
                    d_grnd = max(distances) if self.chosen_dist == 'max' else min(distances)

                    # discovers the index of the chosen component
                    i_grnd = distances.index(d_grnd)

                    # computes the vector normal to the plane
                    v_coefs = np.array([planes[i_grnd]['a'], planes[i_grnd]['b'], planes[i_grnd]['c']]).reshape(3,1)
                    n_cp = v_coefs / np.linalg.norm(v_coefs)

                    # treats if the normal vector is pointing downwardsvector
                    grav_vec = np.array([self.msg_gravVec.vector.x, self.msg_gravVec.vector.y, self.msg_gravVec.vector.z]).reshape(3,1)
                    costheta_n_g = np.dot(n_cp.T, grav_vec)
                    n_cp = -1*n_cp if costheta_n_g > 0 else n_cp # the normal is towards gravity vector if the cos of their inner angle > 0.

                    # retrieving ros time
                    ros_time = rospy.get_rostime()

                    # publishing the distance vector from {R} to the contact plane
                    m = Vector3ArrayStamped()
                    m.header.stamp = ros_time
                    m.header.frame_id = self.node_name
                    m.vec = [Vector3(0, 0, d_grnd)] # sends the distance to the ground as a 3D vector
                    self.pub_grndDist.publish(m)

                    # publishing the contact plane normal vector
                    m = Vector3Stamped()
                    m.header.stamp = ros_time
                    m.header.frame_id = self.node_name
                    m.vector = Vector3(n_cp[0][0], n_cp[1][0], n_cp[2][0])
                    self.pub_cntctPlnVec.publish(m)
            
            # sleeping the node
            node_rate_sleep.sleep()


    def cllbck_cntctPnt(self, msg):
        '''Callback message for received contact points'''
        self.msg_cntctPnt = msg
            

    def cllbck_gravVec(self, msg):
        ''' Callback function for the gravity vector element'''
        self.msg_gravVec = msg


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


    