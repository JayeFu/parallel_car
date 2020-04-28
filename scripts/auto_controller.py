#!/usr/bin/env python  

import sys

import rospy
from parallel_car.Optimizer import SimpleOptimizer
from parallel_car.IKSolver import SerialIKSolver
from parallel_car.Driver import BaseAndMechDriver
from geometry_msgs.msg import Transform, Vector3, Quaternion

# either 'rviz' or 'gazebo'
RUN_ENV = 'gazebo'

def manual_move():
    while not rospy.is_shutdown():

        try:
            # uncomment raw_input if you want to control the pace of sending goals
            raw_input()
        except EOFError:
            print "Manual Ending"
            sys.exit()
        
        # get the transform from origin to wx_link
        (o_to_wx_succ, o_to_wx_tf) = seri_ik.get_transform(origin, "wx_link")

        if o_to_wx_succ:
            # from o_to_wx_tf get disired parallel pose and serial pose
            (parallel_pose_desired, serial_pose_desired) = seri_ik.compute_ik_from_o_to_wx_tf(o_to_wx_tf)

            print "parallel_pose_desired"
            print parallel_pose_desired

            print "serial_pose_desired"
            print serial_pose_desired
        
        # go to desired pose by driver
        driver.send_trajectory_from_controller(parallel_pose_desired, serial_pose_desired)

def auto_move():

    driver.read_trajectory()

    o_to_wx_tf_list = driver.get_o_to_wx_tf_list()

    for tf_idx in range(len(o_to_wx_tf_list)):
        # get a tf from o_to_wx_tf_list
        o_to_wx_tf = o_to_wx_tf_list[tf_idx]
        # compute the parallel pose and serial pose from modified matrix
        (parallel_pose_desired, serial_pose_desired) = seri_ik.compute_ik_from_o_to_wx_tf(o_to_wx_tf)
        # drive store both parallel pose and serial pose inside
        driver.append_pose_desired(parallel_pose_desired, serial_pose_desired)

    # go to initial pose
    driver.init_pose()

    # send the trajectory point one by one
    # driver.send_trajectory_one_by_one()



if __name__ == "__main__":
    rospy.init_node("auto_controller")

    seri_ik = SerialIKSolver(run_env=RUN_ENV)

    driver = BaseAndMechDriver()

    listened_to_fixed = False

    rate = rospy.Rate(1.0)

    if RUN_ENV == 'rviz':
        origin = 'car_link'
    else: # RUN_ENV == 'gazebo'
        origin = 'world'

    # wait until having successfully listened to fixed tf
    while not listened_to_fixed:
        if seri_ik.listen_to_fixed_tf():
            rospy.loginfo("listend to fixed tf successfully")
            listened_to_fixed = True
        else:
            rospy.logerr("listening to fixed failed")
        rate.sleep()
    
    manual_move()

    # auto_move()