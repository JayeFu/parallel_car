#!/usr/bin/env python 

import rospy

import math
import tf2_ros
import geometry_msgs.msg
import copy

class ParallelPose:
    """A class to contain the pose of the parallel car
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
            rospy.logfatal("Transform lookup exception")
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