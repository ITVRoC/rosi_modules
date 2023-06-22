'''This file implements tools for using with the vicon system'''
import rospy

from rosi_common.dq_tools import trAndOri2dq


# ROSI base Vicon marker pose wrt frame {R}
p_marker_offset_tr = [-0.14999997615814, 0.0, 0.18100000917912]
p_marker_offset_q = [0.70153158903122, 0.0, 0.0, 0.71263885498047]

# computing the offset dual-quaternion
p_marker_offset_dq = trAndOri2dq(p_marker_offset_tr, p_marker_offset_q, 'trfirst')

# method that returns the base pose dual quaternion given marker current pose
def getBasePoseFromMarkerDq(marker_dq):
    return marker_dq * p_marker_offset_dq.conj()