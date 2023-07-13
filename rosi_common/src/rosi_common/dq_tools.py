from xml.dom.minidom import parseString
import rospy

import numpy as np
import quaternion # pip install numpy-quaternion

from dqrobotics import *
import numpy as np
import quaternion

from rosi_common.math_tools import *

from rosi_common.msg import DualQuaternionStamped

def rpy2dq(v_in):
    '''Converts a 3x1 np.array containing RPY angles in radians to a pure rotation dual-quaternion format'''

    # converting to the quaternion format
    ret = rpy2quat(v_in)

    return DQ([ret[0], ret[1], ret[2], ret[3], 0, 0, 0, 0])


def rpy2quat(v_in):
    '''Converts a 3x1 np.array containing RPY angles in radians to the quaternion format as a 
    numpy.array'''
   
    # extracting rpy components from input
    roll = v_in[0]
    pitch = v_in[1]
    yaw = v_in[2]

    # computing recurrent variables
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)

    # computing rotation quaternion components 
    w = (cr * cp * cy) + (sr * sp * sy)
    x = (sr * cp * cy) - (cr * sp * sy)
    y = (cr * sp * cy) + (sr * cp * sy)
    z = (cr * cp * sy) - (sr * sp * cy)

    return [w, x, y, z]



def dq2rpy(dq_in):
    '''Extracts the rotation of a dual-quaternion to RPY angles in radians.'''

    # extracting the orientation quaterion components
    w = dq_in.vec8()[0]
    x = dq_in.vec8()[1]
    y = dq_in.vec8()[2]
    z = dq_in.vec8()[3]

    # converts and return the quaternion to RPY
    return quat2rpy([w, x, y, z])


def quat2rpy(q_in):
    '''Converts a quaternion to RPY angles in radians.
    Quaternion must enter as an array [w x y z]'''

    # treats the input if it is a quaternion
    if isinstance(q_in, quaternion.quaternion):
        q_in = q_in.components

    # extracting the orientation quaterion components
    w = q_in[0]
    x = q_in[1]
    y = q_in[2]
    z = q_in[3]

    # roll
    sinr_cosp = 2 * (w*x + y*z)
    cosr_cosp = 1 - 2 * (x*x + y*y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # pitch
    sinp = 2 * (w*y - z*x)
    if np.abs(sinp) >= 1:
        pitch = np.pi/2 * np.sign(sinp)
    else:
        pitch = np.arcsin(sinp)
    
    #yaw
    siny_cosp = 2 * (w*z + x*y)
    cosy_cosp = 1 - 2 * (y*y + z*z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return [roll, pitch, yaw]


def tr2dq(v_in):
    ''' Converts a 3D translation vector to the dual-quaternion format'''

    # transforms the input into a nd.array if it is a list
    if isinstance(v_in, list):
        v_in = np.array(v_in)

    aux = 0.5 * v_in

    return DQ(1, 0, 0, 0, 0, aux[0], aux[1], aux[2])


def quat2dq(q_in):
    '''Converts a rotation quaternion element into a pure-rotation dual-quaternion'''

    # treating the input quaternion
    if isinstance(q_in, quaternion.quaternion):
        q_in = q_in.components

    return DQ(q_in[0], q_in[1], q_in[2], q_in[3], 0, 0, 0, 0)


def trAndOri2dq(tr, quat, order):
    ''' Transforms a translation 3D vector and an orientation in quaternion
    into a dual-quaternion
        Input:
            - tr: 3D vector with translation
            - quat: 4D list or np.quaternion element containing orientation
            - order: Transform order
                    'rotfirst': first rotates by quat, then translates by tr (current frame)
                    'trfirst': first translates by tr, then rotates by quat
    '''

    # converting input translation vector into a pure-translation dual-quaternion
    dq_tr = tr2dq(tr)

    # converting input rotation quaternion into a pure-rotation dual-quaternion
    dq_rot = quat2dq(quat)

    # composing pure transforms considering desired input
    if order == 'rotfirst':
        return dq_rot * dq_tr
    elif order == 'trfirst':
        return dq_tr * dq_rot 
    else:
        return None



def removeYaw(dq_in):
    ''' Given a dual-quaternion representing a pose, this function removes the yaw component out of it'''

    # translates input dual-quaternion to roll-pitch-yaw format
    rpy = dq2rpy(dq_in)

    # creating a dual-quaternion to cancel the yaw component

    dq_yaw = rpy2dq([0, 0, -rpy[2]])

    # rotating dq_in to cancel the yaw component
    return dq_yaw * dq_in 


def dq2trAndQuatArray(dq_in):
    ''' Extracts the translation as a R3 vector and orientation as a quaternion R4 array'''

    q_p, q_d = dqExtractQuaternions(dq_in)
    tr = dqExtractTransV3(dq_in)
    return tr, q_p


def dq2qRot(dq_in):
    ''' Extract the orientation quaternion (primary quaternion) from a dual-quaternion element'''
    q_p, _ = dqExtractQuaternions(dq_in)
    return q_p


def dqExtractTransV3(dq_in):
    ''' Extracts the translation from a dual-quaternionelement 
    as a R3 vector '''
    q_p, q_d = dqExtractQuaternions(dq_in)
    q_tr = 2 * q_d * q_p.conj()
    return q_tr.components[1:4].reshape(3,1)
    

def dqExtractQuaternions(dq_in):
    '''Extracts the primary and dual quaternion components from a dual-quaternion object'''
    a = dq_in.vec8()
    q_p = np.quaternion(a[0], a[1], a[2], a[3]) # primary quaternion
    q_d = np.quaternion(a[4], a[5], a[6], a[7]) # dual quaternion
    return q_p, q_d


def dqExtractRotQuaternion(dq_in):
    '''Extracts the orientation quaternion from a dual-quaternion element'''
    q_p, _ = dqExtractQuaternions(dq_in)
    return q_p


def dqRotfromdq(dq_in):
    '''Constructs a rotation dual quaternion by exctracting the rotation component from an input dual quaternion'''
    a = dq_in.vec8()
    return DQ(a[0], a[1], a[2], a[3], 0, 0, 0, 0)


def dqExtractRotM(dq_in):
    '''Extracts the rotation matrix from a dual quaternion'''

    # extracting rotating quaternion as an array
    q = dq_in.rotation().vec4()

    # saving components for brevity (notation as in Siciliano pg 55)
    n = q[0]
    ex = q[1]
    ey = q[2]
    ez = q[3]

    # creating the rotation matrix
    rotm = np.array([[2*(n**2+ex**2)-1, 2*(ex*ey-n*ez), 2*(ex*ez+n*ey)],
                    [2*(ex*ey+n*ez), 2*(n**2+ey**2)-1, 2*(ey*ez-n*ex)],
                    [2*(ex*ez-n*ey), 2*(ey*ez+n*ex), 2*(n**2+ez**2)-1]])

    return rotm


def angleAxis2dqRot(angle, axis):
    '''Creates a pure rotation dual-quaternion
        - angle: rotation angle in radians.
        - axis: rotation axis as a 3position list'''

    # auxiliary variable
    sin_angle_div_2 = np.sin(angle/2)

    # rotating quaternion
    w = np.cos(angle/2)
    x = axis[0] * sin_angle_div_2
    y = axis[1] * sin_angle_div_2
    z = axis[2] * sin_angle_div_2

    return DQ(w, x, y, z, 0, 0, 0, 0)


def angleAxis2qRot(angle, axis):
    '''Creates a rotation quaternion
        - angle: rotation angle in radians.
        - axis: rotation axis as a 3position list'''

    # auxiliary variable
    sin_angle_div_2 = np.sin(angle/2)

    # rotating quaternion
    w = np.cos(angle/2)
    x = axis[0] * sin_angle_div_2
    y = axis[1] * sin_angle_div_2
    z = axis[2] * sin_angle_div_2

    return [w, x, y, z]


def angleAxis2npqRot(angle, axis):
    '''Creates a rotation quaternion in numpy format
        - angle: rotation angle in radians.
        - axis: rotation axis as a 3position list'''
    q = angleAxis2qRot(angle, axis)
    return np.quaternion(q[0], q[1], q[2], q[3])


def dqExtractTH(dq_in):
    '''Converts a dual-quaternion to the Homogeneous Transform matrix format'''
    rotm = dqExtractRotM(dq_in)
    tr = dqExtractTransV3(dq_in)
    return np.concatenate((np.concatenate((rotm, tr),axis=1),np.array([0,0,0,1]).reshape(1,4)), axis=0)


def quatAssurePosW(q_in):
    '''This function assures that the input quaterion has a positive w component'''
    w = quaternion.as_float_array(q_in)[0]
    if w > 0:
        return q_in
    else:
        return -q_in


def quat2tr(q_in):
    '''Converts a numpy.quaternion element into a 3D vector'''
    return q_in.components[1:]


def imuROSData2dq(d_in):
    ''' Converts a IMU data received from ROS to dual-quaternion format
        d_in is in `sensor_msgs/Imu` format, received by a callback'''
    
    # orientation dual-quaternion
    dq_world_base_ori = DQ([d_in.orientation.w,
                            d_in.orientation.x,
                            d_in.orientation.y,
                            d_in.orientation.z,
                            0, 0, 0, 0])

    # rotational dual-velocity
    omega_world_base_world = DQ([0,
                                d_in.angular_velocity.x,
                                d_in.angular_velocity.y,
                                d_in.angular_velocity.z,
                                0, 0, 0, 0])

    # linear dual-acceleration
    accel_world_base_base = DQ([0, 0, 0, 0, 0,
                                d_in.linear_acceleration.x,
                                d_in.linear_acceleration.y,
                                d_in.linear_acceleration.z])

    return dq_world_base_ori, omega_world_base_world, accel_world_base_base


def rotateVecByQua(qr, v):
    '''Rotates an input vector by a rotation quaternion
    Input: 
        - qr <np.quaternion> or <list>: unit quaternion that encodes desired rotation. If is is a list, it should be [qw qx qy qz]
        - v <np.array>: input vector to rotate
    Output:
        - rotated vector <np.array>
    '''
    # creates input quaternion as a np.quaternion element, if it is not
    if isinstance(qr, list):
        qr = np.quaternion(qr[0], qr[1], qr[2], qr[3])

    qr = qr / np.abs(qr) # assuring unit norm
    qv = np.quaternion(0, v[0], v[1], v[2])
    qrot = qr * qv * qr.conj()  # rotating quaternion
    return qrot.components[1:]


def dqElementwiseMul(qa, qb):
    '''Performs the elementwise multiplication of to input dual-quaternion
    Input
        - qa <dqrobotics.DQ>: input dual-quaternion element a
        - qb <dqrobotics.DQ>: input dual-quaternion element b
    Output
        - a <dqrobotics.DQ> element  with the result.
    '''
    a_qa = qa.vec8()
    a_qb = qb.vec8()
    return DQ(np.multiply(a_qa, a_qb).tolist())



def twist2Dq(twist):
    '''Converts a ROS twist message to the dual-quaternion format
    Input
        - twist <geometry_msgs/Twist>: The twist ROS message
    Output
        the rescpetive pose in <dq> format'''
    
    tr = [twist.transform.translation.x, twist.transform.translation.y, twist.transform.translation.z]
    ori_q = [twist.transform.rotation.w, twist.transform.rotation.x, twist.transform.rotation.y, twist.transform.rotation.z]
    return trAndOri2dq(tr, ori_q, 'trfirst')


def dq2DualQuaternionStampedMsg(dq, ros_time, frame_id):
    '''Converts a dual-quaternion variable into a ROS DualQuaternionStamped message
    Input
        - dq<DQ>: the dual quaternion variable
    Output
        - an object <DualQuaternionStamped> as a ROS message.'''
    aux = dq.vec8()
    m = DualQuaternionStamped()
    m.header.stamp = ros_time
    m.header.frame_id = frame_id
    m.wp = aux[0]
    m.xp = aux[1]
    m.yp = aux[2]
    m.zp = aux[3]
    m.wd = aux[4]
    m.xd = aux[5]
    m.yd = aux[6]
    m.zd = aux[7]
    return m


def DualQuaternionStampedMsg2dq(msg):
    return DQ(msg.wp, msg.xp, msg.yp, msg.zp, msg.wd, msg.xd, msg.yd, msg.zd)
