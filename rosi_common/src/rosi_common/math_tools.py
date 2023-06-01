import numpy as np
import quaternion # pip install numpy-quaternion

from dqrobotics import *

def skewsim(v):
    """Computes the skew-symmetric matrix from a 3D vector 
    Input:
        - v <np.array>: input vector.
    """
    return np.array([[0,-v[2][0],v[1][0]], [v[2][0],0,-v[0][0]], [-v[1][0],v[0][0],0]])


def rotmx(t):
    """Returns an elementary rotation matrix of t radians around x axis"""
    return np.array([[1,0,0],[0,np.cos(t), -np.sin(t)],[0, np.sin(t), np.cos(t)]])


def rotmy(t):
    """Returns an elementary rotation matrix of t radians around x axis"""
    return np.array([[np.cos(t), 0, np.sin(t)],[0, 1, 0],[-np.sin(t), 0, np.cos(t)]])


def rotmz(t):
    """Returns an elementary rotation matrix of t radians around x axis"""
    return np.array([[np.cos(t), -np.sin(t), 0],[np.sin(t), np.cos(t), 0],[0,0,1]])


def thFromRotmAndTr(rotm, tr):  
    """Returns an homogeneous transform matrix given a rotation matrix and a tr
    Input:
        - rotm<np.array>: rotation matrix
        - tr<np.array>: translation array

    """
    return np.vstack((np.hstack((rotm, tr)), np.array([0, 0, 0, 1])))