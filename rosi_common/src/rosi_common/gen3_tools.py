'''This method provides tools for computing gen3 parameters'''

from rosi_common.math_tools import rotmx, rotmy, rotmz, thFromRotmAndTr
from rosi_common.dq_tools import angleAxis2dqRot, trAndOri2dq

import numpy as np
import quaternion
from dqrobotics import *


gen3_tr_all = { # translations between gen3 frames considering robot in home configuration
    'l0-b': np.array([0.0, 0.0, 0.085]).reshape(3,1),
    'j1-l0': np.array([0.0, 0.0, 0.0714]).reshape(3,1),
    'l1-j1': np.array([0.0003, -0.0008, -0.0869]).reshape(3,1),
    'j2-l1': np.array([-0.0003, -0.0062, 0.0414]).reshape(3,1),
    'l2-j2': np.array([0.0009, -0.0878, -0.008]).reshape(3,1),
    'j3-l2': np.array([-0.001, 0.0016, 0.1225]).reshape(3,1),
    'l3-j3': np.array([0.0010, -0.0015, -0.1260]).reshape(3,1),
    'j4-l3': np.array([-0.0010, -0.0082, 0.08436]).reshape(3,1),
    'l4-j4': np.array([0.0006, -0.0846, -0.0130]).reshape(3,1),
    'j5-l4': np.array([-0.0006, 0.007, 0.1237]).reshape(3,1),
    'l5-j5': np.array([0.0006, -0.0093, -0.0680]).reshape(3,1),
    'j6-l5': np.array([-0.0006, -0.0093, 0.0379]).reshape(3,1),
    'l6-j6': np.array([0.0006, -0.0379, -0.0081]).reshape(3,1),
    'j7-l6': np.array([-0.0006, 0.0081, -0.0379]).reshape(3,1),
    'l7-j7': np.array([0.0006, -0.0198, -0.0288]).reshape(3,1)
}

gen3_qrot_all = { # rotations between gen3 frames considering robot in home configuration
    'l0-b': np.quaternion(1.0, 0.0, 0.0, 0.0),
    'j1-l0': np.quaternion(0.0, 1.0, 0.0, 0.0),
    'l1-j1': np.quaternion(0.0, -1.0, 0.0, 0.0),
    'j2-l1': np.quaternion(-0.7071, 0.7071, 0.0, 0.0),
    'l2-j2': np.quaternion(-0.7071, -0.7071, 0.0, 0.0),
    'j3-l2': np.quaternion(0.0, 1.0, 0.0, 0.0),
    'l3-j3': np.quaternion(0.0, -1.0, 0.0, 0.0),
    'j4-l3': np.quaternion(-0.7071, 0.7071, 0.0, 0.0),
    'l4-j4': np.quaternion(-0.7071, -0.7071, 0.0, 0.0),
    'j5-l4': np.quaternion(0.0, 1.0, 0.0, 0.0),
    'l5-j5': np.quaternion(0.0, -1.0, 0.0, 0.0),
    'j6-l5': np.quaternion(0.7071, -0.7071, 0.0, 0.0),
    'l6-j6': np.quaternion(0.7071, 0.7071, 0.0, 0.0),
    'j7-l6': np.quaternion(0.7071, -0.7071, 0.0, 0.0),
    'l7-j7': np.quaternion(0.0, -1.0, 0.0, 0.0)
}

gen3_dq_all = {
    'l0-b': trAndOri2dq(gen3_tr_all['l0-b'], gen3_qrot_all['l0-b'], 'trfirst'),
    'j1-l0': trAndOri2dq(gen3_tr_all['j1-l0'], gen3_qrot_all['j1-l0'], 'trfirst'),
    'l1-j1': trAndOri2dq(gen3_tr_all['l1-j1'], gen3_qrot_all['l1-j1'], 'trfirst'),
    'j2-l1': trAndOri2dq(gen3_tr_all['j2-l1'], gen3_qrot_all['j2-l1'], 'trfirst'),
    'l2-j2': trAndOri2dq(gen3_tr_all['l2-j2'], gen3_qrot_all['l2-j2'], 'trfirst'),
    'j3-l2': trAndOri2dq(gen3_tr_all['j3-l2'], gen3_qrot_all['j3-l2'], 'trfirst'),
    'l3-j3': trAndOri2dq(gen3_tr_all['l3-j3'], gen3_qrot_all['l3-j3'], 'trfirst'),
    'j4-l3': trAndOri2dq(gen3_tr_all['j4-l3'], gen3_qrot_all['j4-l3'], 'trfirst'),
    'l4-j4': trAndOri2dq(gen3_tr_all['l4-j4'], gen3_qrot_all['l4-j4'], 'trfirst'),
    'j5-l4': trAndOri2dq(gen3_tr_all['j5-l4'], gen3_qrot_all['j5-l4'], 'trfirst'),
    'l5-j5': trAndOri2dq(gen3_tr_all['l5-j5'], gen3_qrot_all['l5-j5'], 'trfirst'),
    'j6-l5': trAndOri2dq(gen3_tr_all['j6-l5'], gen3_qrot_all['j6-l5'], 'trfirst'),
    'l6-j6': trAndOri2dq(gen3_tr_all['l6-j6'], gen3_qrot_all['l6-j6'], 'trfirst'),
    'j7-l6': trAndOri2dq(gen3_tr_all['j7-l6'], gen3_qrot_all['j7-l6'], 'trfirst'),
    'l7-j7': trAndOri2dq(gen3_tr_all['l7-j7'], gen3_qrot_all['l7-j7'], 'trfirst'),
}

gen3_tr_joints = { # translations between gen3 joints considering robot in home configuration
    'j1-b': np.array([0.0, 0.0, 0.1564]).reshape(3,1),
    'j2-j1': np.array([0.0, 0.0054, -0.1284]).reshape(3,1),
    'j3-j2': np.array([0.0, -0.2104, -0.0064]).reshape(3,1),
    'j4-j3': np.array([0.0, 0.0067, -0.2104]).reshape(3,1),
    'j5-j4': np.array([0.0, -0.2084, -0.0061]).reshape(3,1),
    'j6-j5': np.array([0.0, 0.0, -0.1059]).reshape(3,1),
    'j7-j6': np.array([0.0, -0.1059, 0.0]).reshape(3,1),
}

gen3_qrot_joints = {
    'j1-b': np.quaternion(0.0, 1.0, 0.0, 0.0),
    'j2-j1': np.quaternion(0.7071, 0.7071, 0.0, 0.0),
    'j3-j2': np.quaternion(0.7071, -0.7071, 0.0, 0.0),
    'j4-j3': np.quaternion(0.7071, 0.7071, 0.0, 0.0),
    'j5-j4': np.quaternion(0.7071, -0.7071, 0.0, 0.0),
    'j6-j5': np.quaternion(-0.7071, -0.7071, 0.0, -0.0),
    'j7-j6': np.quaternion(-0.70710, 0.7071, 0.0, 0.0),
}

gen3_dq_joints = {
    'j1-b': trAndOri2dq(gen3_tr_joints['j1-b'], gen3_qrot_joints['j1-b'], 'trfirst'),
    'j2-j1': trAndOri2dq(gen3_tr_joints['j2-j1'], gen3_qrot_joints['j2-j1'], 'trfirst'),
    'j3-j2': trAndOri2dq(gen3_tr_joints['j3-j2'], gen3_qrot_joints['j3-j2'], 'trfirst'),
    'j4-j3': trAndOri2dq(gen3_tr_joints['j4-j3'], gen3_qrot_joints['j4-j3'], 'trfirst'),
    'j5-j4': trAndOri2dq(gen3_tr_joints['j5-j4'], gen3_qrot_joints['j5-j4'], 'trfirst'),
    'j6-j5': trAndOri2dq(gen3_tr_joints['j6-j5'], gen3_qrot_joints['j6-j5'], 'trfirst'),
    'j7-j6': trAndOri2dq(gen3_tr_joints['j7-j6'], gen3_qrot_joints['j7-j6'], 'trfirst'),
}


gen3_dqrot_joints_var = { # rotations between gen3 frames considering joints positions
    'j1': lambda jpos: angleAxis2dqRot(jpos, [0, 0, 1]),
    'j2': lambda jpos: angleAxis2dqRot(jpos, [0, 0, 1]),
    'j3': lambda jpos: angleAxis2dqRot(jpos, [0, 0, 1]),
    'j4': lambda jpos: angleAxis2dqRot(jpos, [0, 0, 1]),
    'j5': lambda jpos: angleAxis2dqRot(jpos, [0, 0, 1]),
    'j6': lambda jpos: angleAxis2dqRot(jpos, [0, 0, 1]),
    'j7': lambda jpos: angleAxis2dqRot(jpos, [0, 0, 1])
}


def gen3RotFkin(jpos):
    """Computes the gen3 rotation forward kinematics 
    Input:
        - jpos<list>: list with seven elements containing gen3 joints position in radians
    Output:
        - rot<np.array>: rotation matrix of TCP.  
    """
    rot = np.eye(3)
    rot = rot.dot(rotmz(-jpos[0])) # l0 -> l1
    rot = rot.dot(rotmy(jpos[1])) # l1 -> l2
    rot = rot.dot(rotmz(-jpos[2])) # l2 -> l3
    rot = rot.dot(rotmy(jpos[3])) # l3 -> l4
    rot = rot.dot(rotmz(-jpos[4])) # l4 -> l5
    rot = rot.dot(rotmy(jpos[5])) # l5 -> l6
    rot = rot.dot(rotmz(-jpos[6])) # l6 -> l7
    return rot   


def gen3Fkin(l_jpos, retType):
    """Computes the gen3 forward kinematics in dq
    Input:
        - jpos<list>: list with seven elements containing gen3 joints position in radians 
        - retType: type of return. 
            --'tcp': returns the dq pose of TCP only
            --'joints': returns the dq pose of joints
            --'links': returns the dq pose of links
            --'all': returns all poses
            
    Output:
        - computed pose    
    """
    
    ###=== COMPUTING THE FKIN

    # joints id
    l_j_id = ['b'] + ['j'+str(i)+'-b' for i in range(1,8)] 

    # base dq pose    
    d_dq_j = {} # the pose of each joint and tcp wrt the base
    d_dq_j[l_j_id[0]] = DQ(1,0,0,0,0,0,0,0)

    # computing joints dq pose fkin
    for i, (id, dq_j, dq_j_var, jpos) in enumerate(zip(l_j_id, gen3_dq_joints.values(), gen3_dqrot_joints_var.values(), l_jpos)):
        d_dq_j[l_j_id[i+1]] = d_dq_j[id] * dq_j * dq_j_var(jpos)
    del d_dq_j['b']

    ###=== IF request is solely for tcp pose, returns it
    if retType == 'tcp':
        return {'tcp-b': d_dq_j[l_j_id[-1]] * gen3_dq_joints['l7-j7']}


    ###=== IF request is for joints poses 
    if retType == 'joints':
        return d_dq_j


    ###=== IF request is for links poses 
    # links ids wrt its joint
    aux_id_lj = ['l'+str(i)+'-j'+str(i) for i in range(1,8)]
    aux_id_lb = ['l0-b'] + ['l'+str(i)+'-b' for i in range(1,8)]
    aux_id_jb = ['j'+str(i)+'-b' for i in range(1,8)]

    # computing first link pose wrt base
    d_dq_lb = {}
    d_dq_lb[aux_id_lb[0]] = gen3_dq_all[aux_id_lb[0]]

    # computing inner links pose wrt base
    for id, id_jb, id_lj in zip(aux_id_lb[1:], aux_id_jb, aux_id_lj):
        d_dq_lb[id] =  d_dq_j[id_jb] * gen3_dq_all[id_lj]

    if retType == 'links':
        return d_dq_lb
    

    ###=== IF request is for all poses
    if retType == 'all':
        return d_dq_j, d_dq_lb
    
    ###==== In case retType has not been recognized
    return -1

