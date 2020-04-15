#!/usr/bin/env python 

import rospy

import math
import tf2_ros
import geometry_msgs.msg

class ParallelIKSolver:
    '''
    A parallel mechanism IK solver class for listening to tf msgs and calculating length of poles
    '''
    def __init__(self):
        # tf series for listening
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)

        # number of poles
        self._pole_num = 6

        # a list for n Transform-type transform got from listener
        # origin
        self._transform_list = list()

        # a list for the length of 6 poles
        self._pole_length_list = list()

    def listen_to_tf(self):
        '''
        use tf listener to get tf from up joint to down joint, i.e. origin is down joint
        '''
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

    def calculate_pole_length(self):
        '''
        read from self._transform_list, calculate length of poles and output to self._pole_length_list
        '''
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

    def print_pole_length(self):
        for idx in range(0, self._pole_num):
            rospy.loginfo("pole {} is {} meter".format(idx+1, self._pole_length_list[idx]))