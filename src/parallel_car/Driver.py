#!/usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the joint trajectory control action, including the
# goal message and the result message.
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import Transform, Vector3, Quaternion

class BaseAndMechDriver:
    """A class used to control the motion of both base and mechanism onboard
    """
    def __init__(self, file_path="../data/mbx_planned_trajectory.txt", action_ns='/parallel_car/support_and_mech_controller/follow_joint_trajectory'):
        """Construction function for class BaseAndMechDriver

        Keyword Arguments:
            action_ns {str} -- action namespace (default: {'/parallel_car/support_and_mech_controller/follow_joint_trajectory'})
        """

        self._joint_names = ['world_to_supportX', 'supportX_to_supportY', 'supportY_to_car', # for movement of base
         'car_to_barX', 'barX_to_barY', 'barY_to_barZ', # for translation of mech
         'barZ_to_littleX', 'littleX_to_littleY', 'littleY_to_littleZ', # for rotation of mech
         'addon_Tilt_to_wx'] # for rotation of wx about z-axis

        # a list containing the time series
        self._time_list = list()
        
        # a list to contain the target tf from origin to wx_link
        self._o_to_wx_tf_list = list()

        # action client
        self._action_client = actionlib.SimpleActionClient(action_ns, FollowJointTrajectoryAction)

        # wait for the action server until finally detecting
        rospy.loginfo("Start waiting for the action server")
        self._action_client.wait_for_server()
        rospy.loginfo("Server detected")

    def read_trajectory(self):
        with open(self._file_path,'r') as f:
            for line in f.readlines()[1:]:
                # convert string to float
                time_slice = map(float,line.split())

                # index 0 is time
                self._time_list.append(time_slice[0])

                o_to_wx_tf = Transform()
                
                # index 1, 2, 3 are x, y, z
                o_to_wx_tf.translation = Vector3(time_slice[1], time_slice[2], time_slice[3])

                # index 4, 5, 6, 7 are qx, qy, qz, qw
                o_to_wx_tf.rotation = Quaternion(time_slice[4], time_slice[5], time_slice[6], time_slice[7])

                self._o_to_wx_tf_list.append(o_to_wx_tf)

    def send_trajectory_from_controller(self, parallel_pose_desired, serial_pose_desired):
        """A function used to control drive base and mechanism to specified pose

        Arguments:
            parallel_pose_desired {ParallelPose} -- to specify the pose of the base and the angle of wx respective to addon_Tilt_link
            serial_pose_desired {SeiralPose} -- to specify the translation and rotation of the mechanism onboard
        """

        parallel_pose_desired.little_to_zero()
        serial_pose_desired.little_to_zero()
        
        rospy.loginfo("parallel_pose_desired: ")
        print parallel_pose_desired

        rospy.loginfo("serial_pose_desired: ")
        print serial_pose_desired
        
        rospy.loginfo("Start going to the point specified by controller")

        goal = FollowJointTrajectoryGoal()

        # add joint names
        goal.trajectory.joint_names = self._joint_names

        # a joint point in the trajectory
        trajPt = JointTrajectoryPoint()

        # for movement of base
        trajPt.positions.append(parallel_pose_desired.x)
        trajPt.positions.append(parallel_pose_desired.y)
        trajPt.positions.append(parallel_pose_desired.theta)

        # for translation of mech
        trajPt.positions.append(serial_pose_desired.x)
        trajPt.positions.append(serial_pose_desired.y)
        trajPt.positions.append(serial_pose_desired.z)

        # for rotation of mech
        trajPt.positions.append(serial_pose_desired.alpha)
        trajPt.positions.append(serial_pose_desired.beta)
        trajPt.positions.append(serial_pose_desired.gamma)

        # for rotation of wx about z-axis
        trajPt.positions.append(parallel_pose_desired.alpha)

        for idx in range(len(self._joint_names)):
            trajPt.velocities.append(0.0)

        # time to reach the joint trajectory point specified to 2.0 since this will be controlled by my enter
        trajPt.time_from_start = rospy.Duration(secs=8.0)
        # add the joint trajectory point to the goal
        goal.trajectory.points.append(trajPt)

        # go to goal ASAP
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(secs=1.0)

        # send the goal to the action server
        self._action_client.send_goal(goal)

        # wait for the result
        rospy.loginfo("Start waiting for go to the next pose")
        self._action_client.wait_for_result()
        rospy.loginfo("Waiting ends")

        # show the error code
        rospy.loginfo(self._action_client.get_result())

    def send_trajectory_one_by_one(self):
        pass