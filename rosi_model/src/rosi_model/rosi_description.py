'''This file contains rosi description variables and parameters'''
from dqrobotics import *
import numpy as np

from rosi_common.math_tools import skewsim
from rosi_common.dq_tools import *


'''Propellant keys 
    -fl: front left
    -fr: front right
    -rl: rear left
    -rr: rear right
'''
propKeys = ('fl', 'fr', 'rl', 'rr')

##=== Frame Pi (flipper propellants) w.r.t. frame R (Rosi Base)
'''Dual-quaternion of flipper propellants frame w.r.t. base frame'''
dq_base_piFlp = {
    'fl': trAndOri2dq([0.3920, 0.1950, 0], [1, 0, 0, 0], 'trfirst'),
    'fr': trAndOri2dq([0.3920, -0.1950, 0], [1, 0, 0, 0], 'trfirst'),
    'rl': trAndOri2dq([-0.3920, 0.1950, 0], [1, 0, 0, 0], 'trfirst'),
    'rr': trAndOri2dq([-0.3920, -0.1950, 0], [1, 0, 0, 0], 'trfirst')
}

'''Rotation Matrix of propellants frame w.r.t. base frame'''
rotm_base_piFlp = {
    'fl': dqExtractRotM(dq_base_piFlp['fl']),
    'fr': dqExtractRotM(dq_base_piFlp['fr']),
    'rl': dqExtractRotM(dq_base_piFlp['rl']),
    'rr': dqExtractRotM(dq_base_piFlp['rr'])
}

'''Translation of propellants frame w.r.t. base frame'''
tr_base_piFlp = {
    'fl': dqExtractTransV3(dq_base_piFlp['fl']),
    'fr': dqExtractTransV3(dq_base_piFlp['fr']),
    'rl': dqExtractTransV3(dq_base_piFlp['rl']),
    'rr': dqExtractTransV3(dq_base_piFlp['rr'])
}

'''Rotation quaternion of flipper propellants frames w.r.t. the base frame'''
q_base_piFlp = {
    'fl': dqExtractRotQuaternion(dq_base_piFlp['fl']),
    'fr': dqExtractRotQuaternion(dq_base_piFlp['fr']),
    'rl': dqExtractRotQuaternion(dq_base_piFlp['rl']),
    'rr': dqExtractRotQuaternion(dq_base_piFlp['rr'])
}

'''Defines flippers contact assumed contact point wrt'''
th_base_piFlp = {
    'fl': dqExtractTH(dq_base_piFlp['fl']),
    'fr': dqExtractTH(dq_base_piFlp['fr']),
    'rl': dqExtractTH(dq_base_piFlp['rl']),
    'rr': dqExtractTH(dq_base_piFlp['rr'])
}


##=== Frame Pi (traction propellants frame ) w.r.t. frame R (Rosi Base)
'''Dual-quaternion of propellants frame w.r.t. base frame'''
dq_base_piTra = {
    'fl': DQ(1.0, 0.0, 0.0, 0.0,    0.1332, 0.0822, 0.1003, 0.0680).normalize(),
    'fr': DQ(1.0, 0.0, 0.0, 0.0,   -0.1340, 0.0827, -0.1003, 0.0679).normalize(),
    'rl': DQ(1.0, 0.0, 0.0, 0.0,   -0.1344, -0.0830, 0.1002, 0.0680).normalize(),
    'rr': DQ(1.0, 0.0, 0.0, 0.0,    0.1340, -0.0827, -0.1003, 0.0680).normalize()
}

'''Rotation Matrix of propellants frame w.r.t. base frame'''
rotm_base_piTra = {
    'fl': dqExtractRotM(dq_base_piTra['fl']),
    'fr': dqExtractRotM(dq_base_piTra['fr']),
    'rl': dqExtractRotM(dq_base_piTra['rl']),
    'rr': dqExtractRotM(dq_base_piTra['rr'])
}

'''Translation of propellants frame w.r.t. base frame'''
tr_base_piTra = {
    'fl': dqExtractTransV3(dq_base_piTra['fl']),
    'fr': dqExtractTransV3(dq_base_piTra['fr']),
    'rl': dqExtractTransV3(dq_base_piTra['rl']),
    'rr': dqExtractTransV3(dq_base_piTra['rr'])
}

'''Defines flippers contact assumed contact point wrt'''
th_base_piTra = {
    'fl': dqExtractTH(dq_base_piTra['fl']),
    'fr': dqExtractTH(dq_base_piTra['fr']),
    'rl': dqExtractTH(dq_base_piTra['rl']),
    'rr': dqExtractTH(dq_base_piTra['rr'])
}


##=== Frame {Qi} wrt {Pi}

rotm_qi_pi = np.eye(3) # we consider all all {Qi} frames are aligned with their respective {Pi}


##=== Flipper contact point w.r.t. Qi frame (flipper-compliant)
'''translation vector of flipper contact point w.r.t. Qi frame'''
# TODO THE BELOW VARIABLE HAS BEEN DEPRECATED AND SHOULD BE TREATED IN ALL ITS APPEARANCES IN OTHER FILES
tr_qi_flpContact = {
    'fl': np.array([0.43979, 0.06023, -0.01598]).reshape(3,1),
    'fr': np.array([0.43979, -0.06023, -0.01598]).reshape(3,1),
    'rl': np.array([0.43979, -0.06023, -0.01598]).reshape(3,1),
    'rr': np.array([0.43979, 0.06023, -0.01598]).reshape(3,1)
} 

''' dual-quaternion of flipper contact point w.r.t. Qi frame'''
# TODO THE BELOW VARIABLE HAS BEEN DEPRECATED AND SHOULD BE TREATED IN ALL ITS APPEARANCES IN OTHER FILES
dq_qi_flpContact  = {
    'fl': tr2dq(tr_qi_flpContact['fl']),
    'fr': tr2dq(tr_qi_flpContact['fr']),
    'rl': tr2dq(tr_qi_flpContact['rl']),
    'rr': tr2dq(tr_qi_flpContact['rr']),
}


tr_qi_flpContactElbow = {
    'fl': np.array([0.079, 0.119, 0]),
    'fr': np.array([0.079, -0.119, 0]),
    'rl': np.array([0.079, -0.119, 0]),
    'rr': np.array([0.079, 0.119, 0])
}

dq_qi_flpContactElbow = {
    'fl': tr2dq(tr_qi_flpContactElbow['fl']),
    'fr': tr2dq(tr_qi_flpContactElbow['fr']),
    'rl': tr2dq(tr_qi_flpContactElbow['rl']),
    'rr': tr2dq(tr_qi_flpContactElbow['rr']),
}
 

##=== Wheels contact point w.r.t. Pi frame 
'''translation vector of wheel contact point w.r.t. propeller Pi frame'''
tr_pi_wheelContact = {
    'fl': np.array([-0.13201, 0.0, -0.0573]).reshape(3,1),
    'fr': np.array([-0.13201, 0.0, -0.0573]).reshape(3,1),
    'rl': np.array([-0.13201, 0.0, -0.0573]).reshape(3,1),
    'rr': np.array([-0.13201, 0.0, -0.0573]).reshape(3,1)
}

''' dual-quaternion of wheel contact point w.r.t. propeller Pi frame'''
dq_pi_wheelContact = {
    'fl': tr2dq(tr_pi_wheelContact['fl']),
    'fr': tr2dq(tr_pi_wheelContact['fr']),
    'rl': tr2dq(tr_pi_wheelContact['rl']),
    'rr': tr2dq(tr_pi_wheelContact['rr']),
}


##=== USEFUL PARAMETERS

''' polynomial coefficients of the function that receives angular traction joints velocities
to develop linear velocities of the tracks
y = ax + b, where 
                x: traction joint velocity
                y: base linear velocity'''
coefs_baseLinVel_wrt_trJointSpeed_tracks = {
    'a': 0.07373,
    'b': 0.0000
}

''' polynomial coefficients of the function that receives angular traction joints velocities
to develop linear velocities of the wheels
y = ax + b, where 
                x: traction joint velocity
                y: base linear velocity'''
coefs_baseLinVel_wrt_trJointSpeed_wheels = {
    'a': 0.1298,
    'b': 0.0000
}

# flippers angular position limits
dict_flprsPosLimits = {'min': np.deg2rad(20), 'max': np.deg2rad(160)} 


##=== LOCOMOTION MECHANISM PARAMETERS

# flipper elbow contact vector wrt q_f
c_1f = np.array([0.05726, 0.0, -0.14368]) # in [m]

# distance between primary and secondary sprockets' radius
l_s = 0.3832 # in [m]

# secondary sprocket's effective radius
r_ss = 0.08207 # in [m]