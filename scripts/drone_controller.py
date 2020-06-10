#!/usr/bin/env python  

import sys

import rospy

from parallel_car.Drone import DroneMechDriver

if __name__ == "__main__":

    # construct a node
    rospy.init_node("drone_controller")

    # construct a drone driver
    drone = DroneMechDriver()

    # set poses for test
    drone.set_poses_for_test()

    # set time for test
    drone.set_time_for_test()

    # go to first pose
    drone.init_pose()

    # go to other poses
    drone.send_trajectory()