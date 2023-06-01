'''This file provides tools for treating transforms in ROS'''
import  tf2_ros

from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

import dqrobotics
import numpy as np
import quaternion # pip install numpy-quaternion

from rosi_common.dq_tools import *

# creates the tf2 ROS broadcaster
br = tf2_ros.TransformBroadcaster()

# define visual properties for Rviz Marker vector broadcast components
listColors = {
    'error_angle': {'alpha': 1.0, 'red': 0.84, 'green': 0.68, 'blue': 0.08},
    'base_to_flpr_vec': {'alpha': 1.0, 'red': 0.53, 'green': 0.14, 'blue': 0.09},
    'corr_dir_vec': {'alpha': 1.0, 'red': 0.70, 'green': 0.47, 'blue': 0.39},
    'flp_corr_act_vec': {'alpha': 1.0, 'red': 0.43, 'green': 0.23, 'blue': 0.26},
    'base_cmd_vel_lin': {'alpha': 1.0, 'red': 0.9, 'green': 1, 'blue': 0.0},
    'red': {'alpha': 1.0, 'red': 1.0, 'green': 0.0, 'blue': 0.0},
    'green': {'alpha': 1.0, 'red': 0.0, 'green': 1.0, 'blue': 0.0},
    'blue': {'alpha': 1.0, 'red': 0.0, 'green': 0.0, 'blue': 1.0}
}

_visualRviz = {'shaft_diameter': 0.03,'head_diameter': 0.05, 'head_length': 0}


##=== BROADCAST METHODS

def BroadcastRvizTransform(ros_time, id_parent, id_this, dq_in):
    ''' Broadcasts a transform to tf2 server
    - dq_in: input transforms in dual-quaternion format. It may be a dictionary or list with many dq elements.
    '''

    ''' Tests if the input is a dictionary'''
    if isinstance(dq_in, dict):
        if isinstance(id_parent, list):
            for id, parent, dq in zip(id_this, id_parent, dq_in.values()):
                t = mountTrMsg(ros_time, parent, id, dq) 
                br.sendTransform(t)
        else:
            for id, dq in zip(id_this, dq_in.values()):
                t = mountTrMsg(ros_time, id_parent, id, dq) 
                br.sendTransform(t)

    elif isinstance(dq_in, dqrobotics._dqrobotics.DQ):
        t = mountTrMsg(ros_time, id_parent, id_this, dq_in) 
        br.sendTransform(t)

    elif isinstance(dq_in, list) and isinstance(id_this, list):
        if isinstance(id_parent, list):
            for id, parent, dq in zip(id_this, id_parent, dq_in):
                t = mountTrMsg(ros_time, parent, id, dq) 
                br.sendTransform(t)
        else:
            for id, dq in zip(id_this, dq_in):
                t = mountTrMsg(ros_time, id_parent, id, dq) 
                br.sendTransform(t)


def BroadcastRvizVector(ros_time, ref_frame_id, marker_ns, marker_id, pub, v_origin, v_coord, color):
    ''' Publishes vectors as Markers to a desired ROS publisherd
        The function can receive a single vector or an array of vectors.
        Input:
            - ros_time <ros_time>: ROS time 
            - ref_frame_id <string>: the tf2 frame which the marker will use as base. If input is an array of vectors, ref_frame_id may
                                    be a single string (same reference frame for all vectors), or a list of different reference frame ids.
            - marker_ns <string>: Marker namespace 
            - marker_id <integer>: Marker id (if you are sending a MarkerArray, each id must be unique; marker name is namespace+id)
            - pub <ros_pub>: the ROS publishing element
            - v_origin <nd.array or list of nd.array>: vector origin
            - v_coord <nd.array or list of nd.array>: vector coordinate elements
    '''

    if isinstance(v_coord, list): # in case of a list of vector as input
        ref_frame_id = [ref_frame_id]*4 if isinstance(ref_frame_id, str) else ref_frame_id# treats different ref_frame_id inputs
        mA = MarkerArray()
        for ref_id, m_id, origin, vector in zip(ref_frame_id, marker_id, v_origin, v_coord):
            mA.markers.append(mountMarkerVectorMsg(ros_time, ref_id, marker_ns, m_id, Marker.ADD, origin, vector, _visualRviz, color))
        pub.publish(mA)
    elif isinstance(v_coord, np.ndarray):
        m = mountMarkerVectorMsg(ros_time, ref_frame_id, marker_ns, marker_id, Marker.ADD, v_origin, v_coord, _visualRviz, color)
        pub.publish(m)
    else:
        raise Exception("tf_tools.BroadcastRvizVector received a not supported type: "+str(type(v_coord)))


def BroadcastRvizMesh(ros_time, tf_id_ref, pub, mesh_name, mesh_extension, offset):
    ''' Publishes one or many mesh Markers for rendering in Rviz 
    Input:
        - ros_time: ros time stamp
        - tf_id_ref <string>: frame id w.r.t. the mesh is rendered
        - pub <rospy.pub>: ROS publisher handler
        - mesh_name <string>: mesh name, the mesh file should be located at <rosi_model>/meshes. There is no need for adding '.stl' extension here.
        - mesh_extension <string>: mesh file extension
        - offset <dict>: a dictionary containing the mesh pose offset to its related TF frame
                        ex: offset = {'px': 0.0, 'py': 0.0, 'pz': -0.08, 'qw': 1.0, 'qx': 0.0,'qy': 0.0,'qz': 0.0}
        
    One remarks that this method loads an specific offset considering the mesh_name input. Be sure there is an offset variable set in this code, 
    or a condition for null return, considering the mesh_name you input.
    '''

    # in case of one mesh to plot
    if isinstance(tf_id_ref, str):
        # retrieves Marker message with mesh reference
        m = mountMeshMsg(ros_time, tf_id_ref, mesh_name, 0, mesh_name+mesh_extension, offset)
        pub.publish(m)

    # in case of a list of frames of same mesh to plot
    if isinstance(tf_id_ref, list):
        mA = MarkerArray()
        for id_ref, i, off in zip(tf_id_ref, range(len(tf_id_ref)), offset.values()):
            mA.markers.append(mountMeshMsg(ros_time, id_ref, mesh_name, i+1, mesh_name+mesh_extension, off))
        pub.publish(mA)    


##==== Mount Marker Messages

def mountTrMsg(ros_time, id_parent, id_this, dq_in):
    ''' Mounts the Transform message for broadcasting
    '''

    # Alterei uma dimensao de numpy em dq_tools. Pode ser que aqui de
    tr, qrot = dq2trAndQuatArray(dq_in)
    qrot = qrot.components

    t = TransformStamped()
    t.header.stamp = ros_time
    t.header.frame_id = id_parent
    t.child_frame_id = id_this
    t.transform.translation.x = tr[0]
    t.transform.translation.y = tr[1]
    t.transform.translation.z = tr[2]
    t.transform.rotation.w = qrot[0]
    t.transform.rotation.x = qrot[1]
    t.transform.rotation.y = qrot[2]
    t.transform.rotation.z = qrot[3]

    return t


def mountMarkerVectorMsg(ros_time, frame_id, ns, id, mAction, origin, vec, vis, colorize):
    ''' Mounts a Marker msg
        Input:
            - ros_time: ROS time header component
            - id: ROS id header component
            - ns: Namespace for the marker
            - mType: Marker type (see Marker msg components)
            - mAction: Marker action (see Marker msg components)
            - origin: R^3 vector with vector origin coordinate
            - vec: R^3 vector containing the vector dimension coordinates
            - vis: vector visual properties in Rviz
            - colorize: dictionary containing marker colorize elements
    '''

    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = ros_time
    m.ns = ns
    m.id = id
    m.type = Marker.ARROW
    m.action = mAction

    # mesh orientation
    m.pose.orientation.w = 1
    m.pose.orientation.x = 0
    m.pose.orientation.y = 0
    m.pose.orientation.z = 0

    # vector coordinates
    aux_points = []
    # vector origin
    aux_p = Point()
    aux_p.x = origin[0]
    aux_p.y = origin[1]
    aux_p.z = origin[2]
    m.points.append(aux_p)

    # vector components
    aux_p = Point()
    aux_p.x = vec[0]
    aux_p.y = vec[1]
    aux_p.z = vec[2]
    m.points.append(aux_p)

    # vector visual dimensions
    m.scale.x = vis['shaft_diameter']
    m.scale.y = vis['head_diameter']
    m.scale.z = vis['head_length']

    # vector visual color
    m.color.a = colorize['alpha']
    m.color.r = colorize['red']
    m.color.g = colorize['green']
    m.color.b = colorize['blue']

    return m


def mountMeshMsg(ros_time, frame_id, ns, id, mesh, offset):
    ''' Mounts a Marker message for rendering a mesh in Rviz
    It loads meshes located at package <rosi_model>/meshes/ folder
    Input:
        - ros_time: ros time stamp
        - frame_id <string>: tf frame id w.r.t. the mesh is rendered
        - ns <string>: marker name space
        - id <int>: marker id 
        - mesh <string>: mesh.stl file name located at <rosi_model>/meshes/
        - offset <dict>: mesh offset for rendering wr.t. frame_id frame.
    '''

    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = ros_time 
    m.ns = ns 
    m.id = id
    m.type = Marker.MESH_RESOURCE
    m.mesh_resource = "package://rosi_model/meshes/"+mesh

    # mesh position
    m.pose.position.x = offset['px']
    m.pose.position.y = offset['py']
    m.pose.position.z = offset['pz']

    # mesh orientation
    m.pose.orientation.w = offset['qw']
    m.pose.orientation.x = offset['qx']
    m.pose.orientation.y = offset['qy']
    m.pose.orientation.z = offset['qz']

    # mesh visual scale
    m.scale.x = 0.001
    m.scale.y = 0.001
    m.scale.z = 0.001

    # mesh visual color
    m.color.a = 1
    m.color.r = 1
    m.color.g = 0
    m.color.b = 0

    return m
    

              