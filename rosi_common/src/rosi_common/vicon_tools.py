'''This file implements tools for using with the vicon system'''
import rospy
import numpy as np

from rosi_common.dq_tools import trAndOri2dq, rpy2quat


# ---> Parameters

# ROSI base Vicon marker pose wrt frame {R}
p_marker_R_tr = [-0.14999997615814, 0.0, 0.18100000917912]
p_marker_R_q = [0.70153158903122, 0.0, 0.0, 0.71263885498047]


# marker calibration correction offset
p_corrOffset = {
    'tr_z': 0.31015 - 0.3474,
    'rot_x': np.deg2rad(0.349 - 0.15),
    'rot_y': np.deg2rad(0.021 - 0.745)
}

# ---> Preamble
# computing the dual quaternion from the marker to the robot
p_marker_R_dq = trAndOri2dq(p_marker_R_tr, p_marker_R_q, 'trfirst')

# computing the calibration correction dual quaternion
p_corrOffset_q = rpy2quat([p_corrOffset['rot_x'], p_corrOffset['rot_y'], 0])
p_corrOffset_dq = trAndOri2dq(  [0,0,p_corrOffset['tr_z']], p_corrOffset_q, 'trfirst'  )


# method that returns the base pose dual quaternion given marker current pose
def getBasePoseFromMarkerDq(marker_dq):
    return marker_dq * p_marker_R_dq.conj() * p_corrOffset_dq

