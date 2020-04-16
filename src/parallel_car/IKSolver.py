#!/usr/bin/env python 

import rospy

import math
import tf2_ros
import geometry_msgs.msg
import copy

import numpy as np

class ParallelPose:
    """A class to contain the pose of the parallel car, given the pose of wx_link
    """
    def __init__(self, x=0.0, y=0.0, theta=0.0, alpha=0.0):
        """Constructor for ParallelPose
        
        Arguments:
            x {float} -- x-axis position
            y {float} -- y-axis position
            theta {float} -- angle in radians rotating along z-axis
            alpha {float} -- angle in radians rotating along the z-axis of wx_link compared
        """
        self.x = x
        self.y = y
        self.theta = theta
        self.alpha = alpha

    def __sub__(self, other):
        """Subtraction function override
        
        Arguments:
            other {ParallelPose} -- other parallel_pose
        
        Returns:
            [ParallelPose] -- result of the subtraction
        """
        parallel_increment = ParallelPose()
        if isinstance(other, ParallelPose):
            parallel_increment.x = self.x - other.x
            parallel_increment.y = self.y - other.y
            parallel_increment.theta = self.theta - other.theta
            parallel_increment.alpha = self.alpha - other.alpha
            return parallel_increment
        else:
            rospy.logfatal("The instance after '-' is not PrallelPose object. Returning all zero PrallelPose!")
            return parallel_increment

    def __add__(self, other):
        add_result = ParallelPose()
        if isinstance(other, ParallelPose):
            add_result.x = self.x + other.x
            add_result.y = self.y + other.y
            add_result.theta = self.theta + other.theta
            add_result.alpha = self.alpha + other.alpha
        else:
            rospy.logfatal("The instance after '+' is not PrallelPose object. Returning all zero PrallelPose!")
            return add_result


class ParallelIKSolver:
    """A parallel mechanism IK solver class for listening to tf msgs and calculating length of poles

    """
    def __init__(self, pole_num=6):
        """Constructor for ParallelIKSolver
        
        Keyword Arguments:
            pole_num {int} -- the number of poles in the parallel mechanism (default: {6})
        """
        # tf series for listening
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)

        # number of poles
        self._pole_num = pole_num

        # a list for pole_num Transform-type transform got from listener
        # origin
        self._transform_list = list()

        # a list for the length of pole_num poles
        self._pole_length_list = list()

        # a list of transfromation from down_link to down_i (i from 1 to 6), fixed tf, no change afterwards
        self._down_to_down_num_tf_list = list()
        # a list of 
        self._T_down_to_down_num_list = list()

        # a list of transfromation from up_link to up_i (i from 1 to 6), fixed tf, no change afterwards
        self._up_to_up_num_tf_list = list()

    def listen_to_tf(self):
        """Use tf listener to get tf from up joint to down joint, i.e. origin is down joint
        
        Returns:
            [bool] -- If exception is raised while looking up for transformation, return False. If all is successful, return True
        """
        old_transform_list = self._transform_list
        try:
            self._transform_list = list()
            for idx in range(0, self._pole_num):
                transform_stamped = self._tfBuffer.lookup_transform('down_'+str(idx+1), 'up_'+str(idx+1), rospy.Time())
                self._transform_list.append(transform_stamped.transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logfatal("Transform from down_i to up_i lookup exception")
            self._transform_list = old_transform_list
            return False
        return True

    def calculate_pole_length_from_inherent(self):
        """ Read from self._transform_list, calculate length of poles and output to self._pole_length_list
        """
        vector3_list = list() # a list to contain the translation of tfs
        for idx in range(0, self._pole_num):
            vector3_list.append(self._transform_list[idx].translation)
        
        self._pole_length_list = list()
        for idx in range(0, self._pole_num):
            x = vector3_list[idx].x
            y = vector3_list[idx].y
            z = vector3_list[idx].z
            length = math.sqrt(math.pow(x,2) + math.pow(y,2) + math.pow(z,2))
            self._pole_length_list.append(length)

    def listen_to_up_down_fixed_tf(self):
        try:
            for idx in range(self._pole_num):
                transform_stamped = self._tfBuffer.lookup_transform('down_link', 'down_'+str(idx+1), rospy.Time())
                self._down_to_down_num_tf_list.append(transform_stamped.transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logfatal("Transform from down_link to down_i lookup exception")
            self._down_to_down_num_tf_list = list()
            return False
        
        try:
            for idx in range(self._pole_num):
                transform_stamped = self._tfBuffer.lookup_transform('up_link', 'up_'+str(idx+1), rospy.Time())
                self._up_to_up_num_tf_list.append(transform_stamped.transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logfatal("Transform from up_link to up_i lookup exception")
            self._up_to_up_num_tf_list = list()
            return False


        return True

    def calculate_pole_length_from_target(self, parallel_pose_desired):
        pole_length_list = list()




    def print_pole_length(self):
        """Print the length of every pole in the parallel mechanism, using loginfo
        """
        for idx in range(0, self._pole_num):
            rospy.loginfo("pole {} is {} meter".format(idx+1, self._pole_length_list[idx]))

    def get_pole_length_list(self):
        """Return a copy version of self._pole_length_list
        
        Returns:
            [list] -- A list of double, each one indicates the length of a pole
        """
        return copy.copy(self._pole_length_list)

    def quaternion_to_rotation_matrix(self, quat):
        """A function to transform quaternion to homogeneous rotation matrix
        
        Arguments:
            rot {Quaternion} -- Quaternion-type msg
        
        Returns:
            Numpy matrix in 4x4  -- Result homogeneous rotation matrix from input quaternion
        """
        # get qx, qy, qz and qw from Quaternion-type msg
        qx = quat.x
        qy = quat.y
        qz = quat.z
        qw = quat.w

        rot_matrix = np.mat(np.eye(4))

        rot_matrix[0, 0] = 1-2*np.square(qy)-2*np.square(qz)
        rot_matrix[0, 1] = 2*qx*qy-2*qz*qw
        rot_matrix[0, 2] = 2*qx*qz+2*qy*qw
        rot_matrix[1, 0] = 2*qx*qy+2*qz*qw
        rot_matrix[1, 1] = 1-2*np.square(qx)-2*np.square(qz)
        rot_matrix[1, 2] = 2*qy*qz-2*qx*qw
        rot_matrix[2, 0] = 2*qx*qz-2*qy*qw
        rot_matrix[2, 1] = 2*qy*qz+2*qx*qw
        rot_matrix[2, 2] = 1-2*np.square(qx)-2*np.square(qy)
        
        return rot_matrix


    def vector3_to_translation_matrix(self, vec3):
        """A function to transfrom from Vector3-type msg to homogeneous translation matrix
        
        Arguments:
            vec3 {Vector3} -- Vector3-type msg
        
        Returns:
            Numpy matrix in 4x4  -- Result homogeneous translation matrix from input vector3
        """
        
        # get x, y and z from Vector3-type msg
        x = vec3.x
        y = vec3.y
        z = vec3.z

        trans_matrix = np.mat(np.eye(4))
        trans_matrix[0, 3] = x
        trans_matrix[1, 3] = y
        trans_matrix[2, 3] = z

        return trans_matrix
