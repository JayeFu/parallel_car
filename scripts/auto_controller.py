#!/usr/bin/env python  

import rospy
from parallel_car.Optimizer import SimpleOptimizer
from parallel_car.IKSolver import SerialIKSolver
from parallel_car.Driver import BaseAndMechDriver
from geometry_msgs.msg import Transform, Vector3, Quaternion

# either 'rviz' or 'gazebo'
RUN_ENV = 'gazebo'

if __name__ == "__main__":
    rospy.init_node("auto_controller")

    seri_ik = SerialIKSolver(run_env=RUN_ENV)

    optimizer = SimpleOptimizer()

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

    driver.read_trajectory()

    o_to_wx_tf_list = driver.get_o_to_wx_tf_list()

    for tf_idx in range(len(o_to_wx_tf_list)):
        # get a tf from o_to_wx_tf_list
        o_to_wx_tf = o_to_wx_tf_list[tf_idx]
        # compute optimal alpha and modified homogeneous transformation matrix which neutralizes the optimal alpha
        (optimal_alpha, T_o_to_wx_modified) = optimizer.compute_optimal_alpha(o_to_wx_tf)
        # compute the parallel pose and serial pose from modified matrix
        (parallel_pose_desired, serial_pose_desired) = seri_ik.compute_ik_from_modified_matrix(T_o_to_wx_modified)
        # need to revert optimal alpha for parallel pose
        parallel_pose_desired.alpha = -optimal_alpha
        # drive store both parallel pose and serial pose inside
        driver.append_pose_desired(parallel_pose_desired, serial_pose_desired)

    
    '''
    # manually give a tf

    o_to_wx_tf = Transform()

    o_to_wx_tf.translation = Vector3(1.57822624983, 0.105762480478, 1.36)

    o_to_wx_tf.rotation = Quaternion(-0.662418020787, 0.246229618558, -0.282773315579, 0.648546523141)

    (optimal_alpha, T_o_to_wx_modified) = optimizer.compute_optimal_alpha(o_to_wx_tf)
    # from T_o_to_wx_modified get disired parallel pose and serial pose
    (parallel_pose_desired, serial_pose_desired) = seri_ik.compute_ik_from_modified_matrix(T_o_to_wx_modified)
    # need to revert optimal alpha for parallel pose
    parallel_pose_desired.alpha = -optimal_alpha

    # go to desired pose by driver
    # driver.send_trajectory_from_controller(parallel_pose_desired, serial_pose_desired)
    '''