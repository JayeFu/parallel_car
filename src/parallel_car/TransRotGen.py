#!/usr/bin/env python 

import rospy

import numpy as np



def Rotation(axis, angle, in_degree=False):
    """A funtion to return homegeneous rotation matrix
    
    Arguments:
        axis {string} -- either 'x' or 'y' or 'z'
        angle {float} -- in degrees
        in_degree {bool} -- if True do conversion from degree to radian
    
    Returns:
        Numpy matrix in 4x4 -- 4x4 homogeneous rotation matrix
    """
    if in_degree:
        angle = np.deg2rad(angle)
    else:
        pass
    if axis in ['x', 'y', 'z']:
        if axis == 'x':
            return Rot_X(angle)
        elif axis == 'y':
            return Rot_Y(angle)
        else: # axis == 'z'
            return Rot_Z(angle)
    else:
        rospy.logfatal('Axis wrong! Return identity matrix')
        return np.mat(np.eye(4,4))

def Rot_X(alpha):
    """A function to return homogeneous rotation matrix along x-axis
    
    Arguments:
        alpha {float} -- angle in radians
    
    Returns:
        Numpy matrix in 4x4 -- 4x4 homogeneous rotation matrix along x-axis
    """
    T_rot_X = np.mat(np.eye(4,4))

    T_rot_X[1, 1] = np.cos(alpha)
    T_rot_X[2, 2] = np.cos(alpha)
    T_rot_X[1, 2] = -np.sin(alpha)
    T_rot_X[2, 1] = np.sin(alpha)
    
    return T_rot_X

def Rot_Y(beta):
    """A function to return homogeneous rotation matrix along y-axis
    
    Arguments:
        beta {float} -- angle in radians
    
    Returns:
        Numpy matrix in 4x4 -- 4x4 homogeneous rotation matrix along y-axis
    """
    T_rot_Y = np.mat(np.eye(4,4))

    T_rot_Y[0, 0] = np.cos(beta)
    T_rot_Y[2, 2] = np.cos(beta)
    T_rot_Y[0, 2] = np.sin(beta)
    T_rot_Y[2, 0] = -np.sin(beta)
    
    return T_rot_Y

def Rot_Z(gamma):
    """A function to return homogeneous rotation matrix along z-axis
    
    Arguments:
        gamma {float} -- angle in radians
    
    Returns:
        Numpy matrix in 4x4 -- 4x4 homogeneous rotation matrix along y-axis
    """
    T_rot_Z = np.mat(np.eye(4,4))

    T_rot_Z[0, 0] = np.cos(gamma)
    T_rot_Z[1, 1] = np.cos(gamma)
    T_rot_Z[0, 1] = -np.sin(gamma)
    T_rot_Z[1, 0] = np.sin(gamma)

    return T_rot_Z

def Translation(axis, distance):
    """A funtion to return homegeneous translation matrix
    
    Arguments:
        axis {string} -- either 'x' or 'y' or 'z'
        distance {float} -- the distance to travel along the axis
    
    Returns:
        Numpy matrix in 4x4 -- 4x4 homogeneous translation matrix
    """
    if axis in ['x', 'y', 'z']:
        if axis == 'x':
            return Trans_X(distance)
        elif axis == 'y':
            return Trans_Y(distance)
        else: # axis == 'z'
            return Trans_Z(distance)
    else:
        rospy.logfatal('Axis wrong! Return identity matrix')
        return np.mat(np.eye(4,4))

def Trans_X(dist):
    """A funtion to return homogeneous translation matrix along x-axis
    
    Arguments:
        dist {float} -- distance to travel along x-axis
    
    Returns:
        Numpy matrix in 4x4 -- 4x4 homogeneous translation matrix along x-axis
    """
    T_trans_X = np.mat(np.eye(4,4))

    T_trans_X[0, 3] = dist

    return T_trans_X

def Trans_Y(dist):
    """A funtion to return homogeneous translation matrix along y-axis
    
    Arguments:
        dist {float} -- distance to travel along y-axis
    
    Returns:
        Numpy matrix in 4x4 -- 4x4 homogeneous translation matrix along y-axis
    """
    T_trans_Y = np.mat(np.eye(4,4))

    T_trans_Y[1, 3] = dist

    return T_trans_Y

def Trans_Z(dist):
    """A funtion to return homogeneous translation matrix along z-axis
    
    Arguments:
        dist {float} -- distance to travel along z-axis
    
    Returns:
        Numpy matrix in 4x4 -- 4x4 homogeneous translation matrix along z-axis
    """
    T_trans_Z = np.mat(np.eye(4,4))

    T_trans_Z[2, 3] = dist

    return T_trans_Z


