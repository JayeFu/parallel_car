#!/usr/bin/env python 

import rospy

import math
import tf2_ros
from geometry_msgs.msg import Vector3
import copy

from parallel_car.TransRotGen import Rotation, Translation, quaternion_to_rotation_matrix, vector3_to_translation_matrix

import numpy as np

CAR_HEIGHT = 0.36
BAR_LENGTH = 1.0
ADDON_LENGTH = 0.5

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
            rospy.logerr("The instance after '-' is not PrallelPose object. Returning all zero PrallelPose!")
            return parallel_increment

    def __add__(self, other):
        add_result = ParallelPose()
        if isinstance(other, ParallelPose):
            add_result.x = self.x + other.x
            add_result.y = self.y + other.y
            add_result.theta = self.theta + other.theta
            add_result.alpha = self.alpha + other.alpha
        else:
            rospy.logerr("The instance after '+' is not PrallelPose object. Returning all zero PrallelPose!")
            return add_result

    def __str__(self):
        resStr = "at ({}, {}), theta={}, alpha={}".format(self.x, self.y, self.theta, self.alpha)
        return resStr


class ParallelIKSolver:
    """A parallel mechanism IK solver class for listening to tf msgs and calculating length of poles

    """
    def __init__(self, pole_num=6, run_env="rviz"):
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
        # a list of homogeneous transformation matrix from down_link to down_i
        self._T_down_to_down_num_list = list()

        # a list of transfromation from up_link to up_i (i from 1 to 6), fixed tf, no change afterwards
        self._up_to_up_num_tf_list = list()
        # a list of homogeneous transformation matrix from up_link to up_i
        self._T_up_to_up_num_list = list()

        if run_env=='rviz':
            self._o_to_down_height = CAR_HEIGHT/2.0
        else: # run_env== 'gazebo'
            self._o_to_down_height = CAR_HEIGHT

    def listen_to_tf(self):
        """Use tf listener to get tf from up joint to down joint, i.e. origin is down joint
        
        Returns:
            [bool] -- If exception is raised while looking up for transformation, return False. If all is successful, return True
        """
        old_transform_list = self._transform_list
        try:
            self._transform_list = list()
            for idx in range(self._pole_num):
                transform_stamped = self._tfBuffer.lookup_transform('down_'+str(idx+1), 'up_'+str(idx+1), rospy.Time())
                self._transform_list.append(transform_stamped.transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Transform from down_i to up_i lookup exception")
            self._transform_list = old_transform_list
            return False
        return True

    def calculate_pole_length_from_inherent(self):
        """ Read from self._transform_list, calculate length of poles and output to self._pole_length_list
        """
        vector3_list = list() # a list to contain the translation of tfs
        for idx in range(self._pole_num):
            vector3_list.append(self._transform_list[idx].translation)
        
        self._pole_length_list = list()
        for idx in range(self._pole_num):
            x = vector3_list[idx].x
            y = vector3_list[idx].y
            z = vector3_list[idx].z
            length = math.sqrt(math.pow(x,2) + math.pow(y,2) + math.pow(z,2))
            self._pole_length_list.append(length)

    def listen_to_up_down_fixed_tf(self):

        # down
        try:
            for idx in range(self._pole_num):
                transform_stamped = self._tfBuffer.lookup_transform('down_link', 'down_'+str(idx+1), rospy.Time())
                self._down_to_down_num_tf_list.append(transform_stamped.transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Transform from down_link to down_i lookup exception")
            self._down_to_down_num_tf_list = list()
            return False
        
        # up
        try:
            for idx in range(self._pole_num):
                transform_stamped = self._tfBuffer.lookup_transform('up_link', 'up_'+str(idx+1), rospy.Time())
                self._up_to_up_num_tf_list.append(transform_stamped.transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Transform from up_link to up_i lookup exception")
            self._up_to_up_num_tf_list = list()
            return False

        #down
        for idx in range(self._pole_num):
            transform = self._down_to_down_num_tf_list[idx]

            # CAUTION: since the transformation from listener is translated or rotated along the source link coordinate
            # Thus, on right multiplication, first multiply translation matrix, then multiply rotation matrix

            # get the translation matrix
            vec3 = transform.translation # Vector3-type msg
            trans_matrix = vector3_to_translation_matrix(vec3)

            # ge the rotation matrix
            quat = transform.rotation # Quaternion-type msg
            rot_matrix = quaternion_to_rotation_matrix(quat)

            com_matrix = trans_matrix*rot_matrix

            self._T_down_to_down_num_list.append(com_matrix)

        # up
        for idx in range(self._pole_num):
            transform = self._up_to_up_num_tf_list[idx]

            vec3 = transform.translation
            trans_matrix = vector3_to_translation_matrix(vec3)

            quat = transform.rotation
            rot_matrix = quaternion_to_rotation_matrix(quat)

            com_matrix = trans_matrix*rot_matrix
            
            self._T_up_to_up_num_list.append(com_matrix)
                
        
        return True

    def calculate_pole_length_from_target(self, parallel_pose_desired, wx_pose): 
        """A function to compute the pole length from given target
        
        Arguments:
            parallel_pose_desired {ParallelPose} -- the desired parallel pose of the car and mechanism
            wx_pose {Pose} -- a Pose-type msg containing the msg of the 

        Return:
            [tuple] -- a tuple, first item is the list of pole length, second item is the transformations from down_num to up_num
        """
        # a list of pole_length computed from target
        pole_length_list = list()

        # a list of transformations from down_num to up_num
        T_down_num_to_up_num_list = list()

        # although Point-type msg can also be used, however, for clarity, first CONVERT to Vector3-type msg
        T_wx_trans = vector3_to_translation_matrix(Vector3(wx_pose.position.x, wx_pose.position.y, wx_pose.position.z))
        T_wx_rot = quaternion_to_rotation_matrix(wx_pose.orientation)

        # multiply translation matrix first
        T_o_to_wx = T_wx_trans*T_wx_rot

        
        # rospy.loginfo("T_o_to_wx is")
        # print T_o_to_wx

        T_down_trans = vector3_to_translation_matrix(Vector3(parallel_pose_desired.x, parallel_pose_desired.y, self._o_to_down_height))
        T_down_rot = Rotation('z', parallel_pose_desired.theta)

        # multply translation matrix first
        T_o_to_down = T_down_trans*T_down_rot

        # rospy.loginfo("T_o_to_down is")
        # print T_o_to_down

        T_up_to_wx = Translation('z', ADDON_LENGTH) * Rotation('y', np.pi/4.0) * Translation('z', ADDON_LENGTH) * Rotation('z', parallel_pose_desired.alpha)

        # rospy.loginfo("T_up_to_wx is")
        # print T_up_to_wx

        T_down_to_up = T_o_to_down.I * T_o_to_wx * T_up_to_wx.I

        # rospy.loginfo("T_down_to_up is")
        # print T_down_to_up

        # rospy.loginfo("T_down_to_down_1 is ")
        # print self._T_down_to_down_num_list[0]

        for idx in range(self._pole_num):
            T_down_num_to_up_num = self._T_down_to_down_num_list[idx].I * T_down_to_up * self._T_up_to_up_num_list[idx]
            x = T_down_num_to_up_num[0, 3]
            y = T_down_num_to_up_num[1, 3]
            z = T_down_num_to_up_num[2, 3]
            pole_len = np.sqrt(np.square(x)+np.square(y)+np.square(z))
            pole_length_list.append(pole_len)
            T_down_num_to_up_num_list.append(T_down_num_to_up_num)
        
        rospy.loginfo("pole length from target is coming!")
        for idx in range(self._pole_num):
            print "pole {} is {} meter".format(idx+1, pole_length_list[idx])
        
        return (pole_length_list, T_down_num_to_up_num_list)


    def print_pole_length(self):
        """Print the length of every pole in the parallel mechanism
        """
        rospy.loginfo("pole length from inherent!")
        for idx in range(self._pole_num):
            print "pole {} is {} meter".format(idx+1, self._pole_length_list[idx])

    def get_pole_length_list(self):
        """Return a copy version of self._pole_length_list
        
        Returns:
            [list] -- A list of double, each one indicates the length of a pole
        """
        return copy.copy(self._pole_length_list)

    def get_transform(self, source_link, target_link):
        try:
            transform_stamped = self._tfBuffer.lookup_transform(source_link, target_link, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Transform from {} to {} lookup exception".format(source_link, target_link))
            return (False, None)
        return (True, transform_stamped.transform)

class SerialIKSolver:
    """Given transform form down_link to up_link, compute translation of car_to_barX, barX_to_barY, barY_to_barZ and the rotation of barZ_to_littleX, littleX_to_littleY, littleY_to_littleZ
    """
    def __init__(self, run_env="rviz"):
        """Constructor function for class SerialIKSolver
        """

        # tf series for listening
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)

        # to contan the translation of car_to_barX, barX_to_barY, barY_to_barZ
        self._T_down_to_up_trans = np.mat(np.eye(4))

        # to contain the rotation of barZ_to_littleX, littleX_to_littleY, littleY_to_littleZ
        self._T_down_to_up_rot = np.mat(np.eye(4))

        self._Z_OFFSET = 1.075

        if run_env=='rviz':
            self._o_to_down_height = CAR_HEIGHT/2.0
        else: # run_env== 'gazebo'
            self._o_to_down_height = CAR_HEIGHT

    def listen_to_tf(self):
        """Use private _tfBuffer to look up transform from down_link to up_link and store it in _T_down_to_up_trans and _T_down_to_up_rot

        Returns:
            [bool] -- if successfully lookup transform from down_link to up_link and store it, return True, else return False
        """
        try:
            transform_stamped = self._tfBuffer.lookup_transform('down_link', 'up_link', rospy.Time())   
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Transform from down_link to up_link lookup exception")
            return False

        self._T_down_to_up_trans = vector3_to_translation_matrix(transform_stamped.transform.translation)
        self._T_down_to_up_rot = quaternion_to_rotation_matrix(transform_stamped.transform.rotation)

        return True

    def print_T_down_to_up(self):
        """Print stored _T_down_to_up_trans and _T_down_to_up_rot
        """
        
        rospy.loginfo("T_down_to_up_trans is")
        print self._T_down_to_up_trans

        rospy.loginfo("T_down_to_up_rot is")
        print self._T_down_to_up_rot

    def compute_ik_from_inherent(self):
        """Compute inverse kinematics of three prismatic joints and three revolute joints from stored transformation
        """

        trans_x = self._T_down_to_up_trans[0, 3]
        trans_y = self._T_down_to_up_trans[1, 3]
        trans_z = self._T_down_to_up_trans[2, 3] - self._Z_OFFSET

        rospy.loginfo("Translation in x:{}, y:{}, z:{}".format(trans_x, trans_y, trans_z))

        # alpha, beta, gamma are all restricted to (-pi/2, pi/2), so sin(beta) 
        # alpha: rotation about x-axis
        # beta:  rotation about y-axis
        # gamma: rotation about z-axis

        alpha = np.arctan2(-self._T_down_to_up_rot[1, 2], self._T_down_to_up_rot[2, 2])
        
        beta = np.arcsin(self._T_down_to_up_rot[0, 2])
        
        gamma = np.arctan2(-self._T_down_to_up_rot[0, 1], self._T_down_to_up_rot[0, 0])

        rospy.loginfo("Rotation in alpha:{}, beta:{}, gamma:{}".format(alpha, beta, gamma))

    def compute_ik_from_target(self, parallel_pose_desired, wx_pose):

        # although Point-type msg can also be used, however, for clarity, first CONVERT to Vector3-type msg
        T_wx_trans = vector3_to_translation_matrix(Vector3(wx_pose.position.x, wx_pose.position.y, wx_pose.position.z))
        T_wx_rot = quaternion_to_rotation_matrix(wx_pose.orientation)

        # multiply translation matrix first
        T_o_to_wx = T_wx_trans*T_wx_rot

        
        # rospy.loginfo("T_o_to_wx is")
        # print T_o_to_wx

        T_down_trans = vector3_to_translation_matrix(Vector3(parallel_pose_desired.x, parallel_pose_desired.y, self._o_to_down_height))
        T_down_rot = Rotation('z', parallel_pose_desired.theta)

        # multply translation matrix first
        T_o_to_down = T_down_trans*T_down_rot

        # rospy.loginfo("T_o_to_down is")
        # print T_o_to_down

        T_up_to_wx = Translation('z', ADDON_LENGTH) * Rotation('y', np.pi/4.0) * Translation('z', ADDON_LENGTH) * Rotation('z', parallel_pose_desired.alpha)

        # rospy.loginfo("T_up_to_wx is")
        # print T_up_to_wx

        T_down_to_up = T_o_to_down.I * T_o_to_wx * T_up_to_wx.I

        trans_x = T_down_to_up[0, 3]
        trans_y = T_down_to_up[1, 3]
        trans_z = T_down_to_up[2, 3] - self._Z_OFFSET

        rospy.loginfo("Translation in x:{}, y:{}, z:{}".format(trans_x, trans_y, trans_z))

        # alpha, beta, gamma are all restricted to (-pi/2, pi/2), so sin(beta) 
        # alpha: rotation about x-axis
        # beta:  rotation about y-axis
        # gamma: rotation about z-axis

        alpha = np.arctan2(-T_down_to_up[1, 2], T_down_to_up[2, 2])
        
        beta = np.arcsin(T_down_to_up[0, 2])
        
        gamma = np.arctan2(-T_down_to_up[0, 1], T_down_to_up[0, 0])

        rospy.loginfo("Rotation in alpha:{}, beta:{}, gamma:{}".format(alpha, beta, gamma))

    def get_transform(self, source_link, target_link):
        try:
            transform_stamped = self._tfBuffer.lookup_transform(source_link, target_link, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Transform from {} to {} lookup exception".format(source_link, target_link))
            return (False, None)
        return (True, transform_stamped.transform)