#!/usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the joint trajectory control action, including the
# goal message and the result message.
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import Transform, Vector3, Quaternion

class DronePose:
    
    def __init__(self, x=0.0, y=0.0, z=0.0, gamma=0.0):
        
        # translation
        self.x = x
        self.y = y
        self.z = z

        # rotation
        self.gamma = gamma

class DroneMechDriver:
    
    def __init__(self, file_path="", action_ns="/drone/drone_mech_controller/follow_joint_trajectory"):

        # self stored file path for read later
        self._file_path = file_path

        # joint names
        self._joint_names = ["/drone/link1_to_link2", "/drone/link2_to_link3", "/drone/link3_to_link4", "/drone/link4_to_drone"]

        # time list for further sending trajectories
        self._time_list = list()
        
        # pose list consisting of DronePose
        self._pose_list = list()
        
        # action client
        self._action_client = actionlib.SimpleActionClient(action_ns, FollowJointTrajectoryAction)

        # wait for the action server until finally detecting
        rospy.loginfo("Start waiting for the action server")
        self._action_client.wait_for_server()
        rospy.loginfo("Server detected")

    def set_poses_for_test(self):
        
        # initial pose
        x0 = 1.0
        y0 = 1.0
        z0 = 1.0
        gamma0 = 0.0

        # step move for x,y,z,alpha, step is 0.05
        for idx in range(20):
            x = x0 - 0.05*idx
            y = y0 - 0.05*idx
            z = z0 - 0.05*idx
            gamma = gamma0 + 0.05*idx
            pose = DronePose(x,y,z,gamma)
            self._pose_list.append(pose)

    def set_time_for_test(self):
        
        # time for 20pts
        for idx in range(20):
            self._time_list.append(idx)

    def init_pose(self):

        # get first pose
        first_pose = self._pose_list[0]
        
        # construct a new goal
        goal = FollowJointTrajectoryGoal()

        # add joint names
        goal.trajectory.joint_names = self._joint_names

        # a joint point in the trajectory
        trajPt = JointTrajectoryPoint()

        # tranlation positions
        trajPt.positions.append(first_pose.x)
        trajPt.positions.append(first_pose.y)
        trajPt.positions.append(first_pose.z)

        # rotation positions
        trajPt.positions.append(first_pose.gamma)

        # append as many velocities as joints
        for idx in range(len(self._joint_names)):
            trajPt.velocities.append(0.0)

        # time to reach the joint trajectory point specified to 6.0 since this will be controlled by my enter
        trajPt.time_from_start = rospy.Duration(secs=6.0)

        # add the joint trajectory point to the goal
        goal.trajectory.points.append(trajPt)

        # go to goal ASAP
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(secs=1.0)

        # send the goal to the action server
        self._action_client.send_goal(goal)

        # wait for the result
        rospy.loginfo("Waiting for go to the first pose")
        self._action_client.wait_for_result()
        rospy.loginfo("Waiting ends")

        # show the error code
        rospy.loginfo(self._action_client.get_result())

    def send_trajectory(self):
        
        # construct a goal
        goal = FollowJointTrajectoryGoal()

        # add joint names
        goal.trajectory.joint_names = self._joint_names
        
        
        for traj_idx in range(len(self._time_list)-1):

            # indexing a pose
            pose = self._pose_list[traj_idx+1]

            rospy.loginfo("At trajectory point {}".format(traj_idx))

            # a joint point in the trajectory
            trajPt = JointTrajectoryPoint()

            # tranlation positions
            trajPt.positions.append(pose.x)
            trajPt.positions.append(pose.y)
            trajPt.positions.append(pose.z)

            # rotation positions
            trajPt.positions.append(pose.gamma)

            # append as many velocities as joints
            for idx in range(len(self._joint_names)):
                trajPt.velocities.append(0.0)

            # time to reach the joint trajectory point specified to 1.0
            trajPt.time_from_start = rospy.Duration(secs=self._time_list[traj_idx+1])

            # add the joint trajectory point to the goal
            goal.trajectory.points.append(trajPt)

        # go to goal ASAP
        goal.trajectory.header.stamp = rospy.Time.now()

        print "This goal has {} points to go".format(len(goal.trajectory.points))

        # send the goal to the action server
        self._action_client.send_goal(goal)

        # wait for the result
        rospy.loginfo("Start waiting for go to the resting poses")
        self._action_client.wait_for_result()
        rospy.loginfo("Waiting ends")

        # show the error code
        rospy.loginfo(self._action_client.get_result())