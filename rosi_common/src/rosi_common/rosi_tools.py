''' This file defines rosi common variables and parameters'''
import numpy as np
import quaternion

from dqrobotics import * # pip install dqrobotics
from rosi_model.rosi_description import *
from rosi_common.dq_tools import rotateVecByQua, angleAxis2qRot


''' -------------------'''
''' ---> Programming useful variables <---'''

'''Base vectors'''
x_axis = np.array([1, 0, 0]).reshape(3,1)
y_axis = np.array([0, 1, 0]).reshape(3,1)
z_axis = np.array([0, 0, 1]).reshape(3,1)

''' Control type modes for sending to rosi_controller ros nodelet '''
ctrlType = {
            "NotControlled": int(0),
            "Brake": int(1),
            "Velocity": int(2),
            "Torque": int(3), 
            "Position": int(4), 
            "Unchanged": int(5)
        }


''' -------------------'''
''' ---> LOCOMOTION MECHANISMS <---'''

def clipFlipVel(l_cmd, dict_flprsVelLimits):
    ''' Clips flippers velocitiy given current velocity and the operational range'''
    l_cmd_out = []
    for cmd in l_cmd:
        if cmd < dict_flprsVelLimits['negative']:
            l_cmd_out.append(dict_flprsVelLimits['negative'])
        elif cmd > dict_flprsVelLimits['positive']:
            l_cmd_out.append(dict_flprsVelLimits['positive'])
        else:
            l_cmd_out.append(cmd)
    return l_cmd_out


def clipFlipPos(cmd_vel, flp_pos, dict_flprsPosLimits, dt):
    ''' Clips a flipper velocity command if it is outside angular position operational limits'''

    cut_mask = []
    for cmd, pos in zip(cmd_vel, flp_pos):
        pos_next = pos + cmd*dt # predicts next flpr angular position considering given angular velocity cmd.

        if pos_next > dict_flprsPosLimits['max'] and cmd > 0: # case of next pos is bigger than maximum angular position and cmd is to increase angular position
            cut_mask.append(0)
        elif pos_next < dict_flprsPosLimits['min'] and cmd < 0: # case of next pos is smaller than minimum ang position and cmd is to decrease angular position
            cut_mask.append(0)
        else:
            cut_mask.append(1)

    # computing output considering the mask
    cmd_out = [a*b for a,b in zip(cut_mask, cmd_vel)]

    return cmd_out, cut_mask # inerses cut_mask for publishing


def correctFlippersJointSignal(d_in):
    ''' Correct rosi joints signal'''
    correction = np.array([1, -1, -1, 1])
    return np.multiply(d_in, correction)


def correctFlippersJointSignal2(d_in):
    ''' Correct rosi joints signal after the correction of flipper frames orientation.
    Created a version 2 of this method and maintaned the last for legacy purporses.
    '''
    correction = np.array([1, 1, -1, -1])
    return np.multiply(d_in, correction)


def correctTractionJointSignal(d_in):
    ''' Correct rosi Traction joints signal
        This modelling considers that positive traction joints velocities makes
        ROSI moves forwards (in direction of its x axis)'''
    correction = np.array([1, -1, 1, -1])
    return np.multiply(d_in, correction)


def jointStateData2dict(d_in):
    ''' Splits a joints_state data received from ROS to a dictionary and correct
    joints signal'''

    tr = {"pos": np.ndarray.tolist(correctTractionJointSignal(d_in.position[0:4])),
          "vel": np.ndarray.tolist(correctTractionJointSignal(d_in.velocity[0:4])),
          "frc": np.ndarray.tolist(correctTractionJointSignal(d_in.effort[0:4]))
    }

    flp = {"pos": np.ndarray.tolist(correctFlippersJointSignal(d_in.position[4:8])),
          "vel": np.ndarray.tolist(correctFlippersJointSignal(d_in.velocity[4:8])),
          "frc": np.ndarray.tolist(correctFlippersJointSignal(d_in.effort[4:8]))
    }

    return tr, flp


def tractionLinVelGivenJointSpeed(jointVel, mechanism):
    '''Returns the traction linear velocity given input joint velocity
    Input
        jointVel: traction joint velocity in rad/s
        mechanism: mechanism current touching the ground
            'wheel': wheel
            'flipper': flipper'''

    if mechanism == 'wheel':
        coefs = coefs_baseLinVel_wrt_trJointSpeed_wheels
    elif mechanism == 'flipper':
        coefs = coefs_baseLinVel_wrt_trJointSpeed_tracks
    else:
        print('[ERRO] input in mechanism variable not recognizable')
        return None

    return coefs['a']*jointVel + coefs['b']


def tractionJointSpeedGivenLinVel(linVel, mechanism):
    '''Returns the joints speed  given input traction linear velocity
    Input
        linVel: traction linear velocity in m/s
        mechanism: mechanism current touching the ground
            'wheel': wheel
            'flipper': flipper
    '''
    if mechanism == 'wheel':
        coefs = coefs_baseLinVel_wrt_trJointSpeed_wheels
    elif mechanism == 'flipper':
        coefs = coefs_baseLinVel_wrt_trJointSpeed_tracks
    else:
        print('[ERRO] input in mechanism variable not recognizable')
        return None

    return (linVel-coefs['b'])/coefs['a']


def computeKinematicMatrixA(var_lambda, wheel_radius, ycir):
    '''Method for compute the skid-steer A kinematic matrix 
    See Mandow - Skid-steering for more'''

    # kinematic A matrix 
    matrix_A = np.array([[var_lambda*wheel_radius/2, var_lambda*wheel_radius/2],
                        [(var_lambda*wheel_radius)/(2*ycir), -(var_lambda*wheel_radius)/(2*ycir)]])

    return matrix_A



''' ------------------------'''
''' ---> CONTACT POINTS <---'''

def flippersContactPoint(thetaq_l, gxz):
    ''' THIS METHOD IS DEPRECATED
    THE METHOD TO COMPUTE CONTACT POINTS IS NOT RIGHT ANYMORE HERE
    GO TO THE NODE groud_contact_point... for the current version
    
    Computes the flipper contact point given its lever axis angle and the 
    unit vector of the gravity projection onto the plane formed by x_P z_P 
    Input
        - thetaq_l <list> or <number>: flipper lever joint angle in radians, which is the angle from z_P to z_Q
        - gxz <np.array>: unit vector of the gravity projection onto the plane formed by x_P z_P 
    Output
        - the flipper contact vector <np.array> or as a list of such.
    '''
    
    ##--- computing c_2f
    # computes the rotation quaternion from Pf to Qf, which is a rotation of the lever joint axis angular state around y_Pf axis
    q_r_l = [ angleAxis2qRot(theta_i, [0, 1, 0]) for theta_i in thetaq_l ]

    # computes the z_Qf vector expressed in Pf
    z_pf_qf_l = [ rotateVecByQua(q_ri, np.array([0, 0, 1])) for q_ri in q_r_l ]   # rotates the z_Qf vector aligned with z_Pf to its new angular position

    # c_2f contact point candidate
    c_2f_l = [(l_s * z_pf_qf_i + r_ss).reshape(3,1) for z_pf_qf_i in z_pf_qf_l]

    ##--- elects the contact point as the farthest from {P} between candidates flipper elbow and point
    c_f = []
    for c_2f_i in c_2f_l:
        c_f.append( c_1f if c_1f[2] < c_2f_i[2][0] else c_2f_i )
    return c_f


''' -------------------'''
''' ---> CHASSIS KIN JACOBIANS <---'''

def compute_J_c_dagger(Rotm_R_Pi_l, tr_R_Pi_l):
    '''Computes the Chassis Jacobian pseudo-inverse, that maps the full {R} state velocities [lin_vel_3d; ang_vel_3d]
    to all propulsion
    Input
        - Rotm_R_Pi_l <list>: a list of rotation matrices <np.array> of frame {Pi} wrt {R}
        - tr_R_Pi <list>: a list of translation vectors <np.array>{nx1} of frame {Pi} wrt {R}
    Output
        - Chassis jacobian matrix <np.array>{3xnxlen(Rotm_R_Pi_l), 6}
    '''
    J_l = []
    for Rotm_R_Pi, tr_R_Pi in zip(Rotm_R_Pi_l, tr_R_Pi_l):
        c1 = Rotm_R_Pi.T # first column group
        c2 = -np.dot(Rotm_R_Pi.T, skewsim(tr_R_Pi)) # second column group
        J_i = np.hstack((c1, c2))
        J_l.append(J_i)
    return np.concatenate(J_l, axis=0)


def compute_J_art_dagger(tr_R_Pi_l):
    ''' Computes the Chassis Articulation Kinematics pseudo-inverse already CONSIDERING that all propulsion frames {Pi} are 
    aligned wrt {R}
    Input
        - tr_R_Pi_l <list> a list containing tr_R_Pi_l <np.array> 3D translation vector of {Pi} wrt {R}
    Output
        - the Articulation Jacobian <np.array>{len(tr_R_Pi_l)x3}
    '''
    J_l = []
    for tr_R_Pi in tr_R_Pi_l:
        J_l.append([1, tr_R_Pi[1][0], -tr_R_Pi[0][0] ])
    return np.concatenate([J_l], axis=0)


def compute_J_ori_dagger(tr_R_Pi_l):
    ''' Computes the Chassis Orientation Kinematics pseudo-inverse already CONSIDERING that all propulsion frames {Pi} are 
    aligned wrt {R}
    Input
        - tr_R_Pi_l <list> a list containing tr_R_Pi_l <np.array> 3D translation vector of {Pi} wrt {R}
    Output
        - the Articulation Jacobian <np.array>{len(tr_R_Pi_l)x3}
    '''
    J_l = []
    for tr_R_Pi in tr_R_Pi_l:
        J_l.append([tr_R_Pi[1][0], -tr_R_Pi[0][0] ])
    return np.concatenate([J_l], axis=0)


def compute_J_mnvx_dagger(tr_R_Pi_l):
    ''' Computes the Maneuvering Kinematics pseudo-inverse for the x_Pi axis already CONSIDERING that all propulsion frames {Pi} are 
    aligned wrt {R}
    Input
        - tr_R_Pi_l <list> a list containing tr_R_Pi_l <np.array> 3D translation vector of {Pi} wrt {R}
    Output
        - the Maneuvering Jacobian for the x_Pi axis <np.array>{len(tr_R_Pi_l)x2}
    '''
    J_l = []
    for tr_R_Pi in tr_R_Pi_l:
        J_l.append([1, -tr_R_Pi[1][0]])
    return np.concatenate(J_l, axis=0)


def compute_J_mnvy_dagger(tr_R_Pi_l):
    ''' Computes the Maneuvering Kinematics pseudo-inverse for the y_Pi axis already CONSIDERING that all propulsion frames {Pi} are 
    aligned wrt {R}
    Input
        - tr_R_Pi_l <list> a list containing tr_R_Pi_l <np.array> 3D translation vector of {Pi} wrt {R}
    Output
        - the Maneuvering Jacobian for the y_Pi axis <np.array>{len(tr_R_Pi_l)x2}
    '''
    J_l = []
    for tr_R_Pi in tr_R_Pi_l:
        J_l.append([0, tr_R_Pi[0][0]])
    return np.concatenate(J_l, axis=0)


''' -------------------'''
''' ---> PROPULSION KIN JACOBIANS <---'''

def compute_J_traction(r, n_cp):
    '''Computes the Wheel/Flippers tracks Traction Jacobian matrix considering that the rotation is about the y_Pw axis.
    Input
        - r <float>: wheel/sprocket effective radius
        - n_cp <np.array>{3x1}: a 3D unit vector of the ground normal vector expressed in {Pi} frame 
    Output
        - Traction Jacobian <np.array>{ len(Rotm_R_Pi_l).3  x  1}
    '''
    r1 = -r * np.dot(np.cross(n_cp.T, y_axis.T), x_axis)
    r3 = -r * np.dot(np.cross(n_cp.T, y_axis.T), z_axis)
    return np.vstack([r1, 0, r3])

    '''for Rotm_Pi_Qi in Rotm_Pi_Qi_l:

        # extracting basis vector coordinates from the rotation matrix
        x_axis = Rotm_Pi_Qi[:][0].reshape(3,1)
        y_axis = Rotm_Pi_Qi[:][1].reshape(3,1)
        z_axis = Rotm_Pi_Qi[:][2].reshape(3,1)

        # computing both rows
        r1 =  (-r*np.dot(np.cross(y_axis.T, n_cp), x_axis))[0][0]
        r3 = (-r*np.dot(np.cross(y_axis.T, n_cp), z_axis))[0][0]
        J_l.append(  np.array([r1, 0, r3]).reshape(3,1)  )
    return np.concatenate(J_l, axis=0)'''


def compute_J_flpLever(Rotm_Pi_Qi_l, c_fi_l, type):
    '''Computes the Flippers full lever Jacobian matrix considering that the rotation is about the y_Pw axis.
    Input
        - Rotm_Pi_Qi_l <list>: a list of rotation matrices <np.array> of frame {Qi} wrt {Pi}
        - c_P_f <list> of <np.array>{3x1}: flippers ground contact points,
        - type <string>: type of flipper lever jacobian to return. 'full', 'x', or 'z'
    Output
        - Flipper lever joint Jacobian <np.array>{ len(Rotm_R_Pi_l).3  x  1}
    '''
    J_l = []
    for Rotm_Pi_Qi, c_fi in zip([Rotm_Pi_Qi_l], [c_fi_l]):
        
        # corrects contact point c_fi dimension if needed
        c_fi = c_fi.reshape(3,1) if c_fi.shape != (3,1) else c_fi

        # extracting basis vector coordinates from the rotation matrix
        x_axis = Rotm_Pi_Qi[:][0].reshape(3,1)
        y_axis = Rotm_Pi_Qi[:][1].reshape(3,1)
        z_axis = Rotm_Pi_Qi[:][2].reshape(3,1)

        # computing both rows
        r1 = -np.dot(np.cross(y_axis.T, c_fi.T), x_axis)[0][0]
        r3 = -np.dot(np.cross(y_axis.T, c_fi.T), z_axis)[0][0]

        if type=='full':
            J = np.array([r1, 0.0, r3])
        elif type=='x':
            J = np.array([r1, 0.0, 0.0])
        elif type=='z':
            J = np.array([0.0, 0.0, r3])
        else:
            J = None
            print('bad value received for the type of jacobian')

        J_l.append( J.reshape(3,1) )
    return np.concatenate(J_l, axis=0)



''' -------------------'''
''' ---> GRAVITY TOOLS <---'''

def gravityVec(d_imu):
    '''Returns the unit gravity vector given an IMU data. When totally aligned, the gravity vector is g = [0, 0, -1]
    Input
        - d_imu <sensor_msgs/Imu>: IMU data in ROS message format
    Output
        - the 3D gravity vector <np.array>{3}
    '''
    vg = np.array([0, 0, -1])
    qr = np.quaternion(d_imu.orientation.w, d_imu.orientation.x, d_imu.orientation.y, d_imu.orientation.z)
    return rotateVecByQua(qr, vg)

    
def gravityVecProjectedInPlaneXZ(d_imu):
    '''Returns the unit gravity vector projected in the XZ plane of the underlying reference frame.
    Input
        - d_imu <sensor_msgs/Imu>: IMU data in ROS message format
    Output
        - the unit 3D gravity vector projected onto XZ plane <np.array>{3}
    '''
    vg = gravityVec(d_imu)
    aux = np.array([vg[0], 0, vg[2]])   # removes y component from vg
    return aux / np.linalg.norm(aux)    # turns into unit the vector