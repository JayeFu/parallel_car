#!/usr/bin/env python  

import rospy
from parallel_car.Optimizer import GradientOptimizer ,SimpleOptimizer
from parallel_car.IKSolver import ParallelPose ,ParallelIKSolver, SerialIKSolver
from parallel_car.TransRotGen import quaternion_to_rotation_matrix, vector3_to_translation_matrix, transform_to_matrix, quaternion_to_euler
from geometry_msgs.msg import Pose, Point
from time import sleep

# either 'rviz' or 'gazebo'
RUN_ENV = 'rviz'

if __name__ == "__main__":
    rospy.init_node("auto_controller")

    seri_ik = SerialIKSolver(run_env=RUN_ENV)

    rate = rospy.Rate(1.0)

    if RUN_ENV == 'rviz':
        origin = 'car_link'
    else: # RUN_ENV == 'gazebo'
        origin = 'odom'

    while not rospy.is_shutdown():

        # listen to tf from down_link to up_link
        listen_to_tf_succ = seri_ik.listen_to_tf()
        if listen_to_tf_succ:
            rospy.loginfo("listen to down_link-up_link transform successfully!")
        else:
            rospy.logerr("listening to down_link-up_link transform failed.")

        parallel_pose_desired = ParallelPose()

        # get transfrom from origin to down_link
        (o_to_down_succ, o_to_down_tf) = seri_ik.get_transform(origin, "down_link")
        if o_to_down_succ:
            parallel_pose_desired.x = o_to_down_tf.translation.x
            parallel_pose_desired.y = o_to_down_tf.translation.y
            parallel_pose_desired.theta = quaternion_to_euler(o_to_down_tf.rotation)[0]
        else:
            rospy.logerr("listening to {}-down_link transform failed.".format(origin))

        # get transform from up_link to wx_link
        (up_to_wx_succ, up_to_wx_tf) = seri_ik.get_transform("up_link", "wx_link")
        if up_to_wx_succ:
            # something wrong in quaternion_to_euler
            parallel_pose_desired.alpha = quaternion_to_euler(up_to_wx_tf.rotation)[0]
            rospy.logerr("alpha is {}".format(parallel_pose_desired.alpha))
        else:
            rospy.logerr("listening to up_link-wx_link transform failed.")

        # get transform from origin to wx_link
        (o_to_wx_succ, o_to_wx_tf) = seri_ik.get_transform(origin, "wx_link")
        if o_to_wx_succ:
            wx_pose = Pose()
            wx_pose.position.x = o_to_wx_tf.translation.x
            wx_pose.position.y = o_to_wx_tf.translation.y
            wx_pose.position.z = o_to_wx_tf.translation.z
            wx_pose.orientation = o_to_wx_tf.rotation

        if listen_to_tf_succ and o_to_down_succ and up_to_wx_succ and o_to_wx_succ:
            # seri_ik.print_T_down_to_up()
            seri_ik.compute_ik_from_inherent()
            seri_ik.compute_ik_from_target(parallel_pose_desired, wx_pose)
         
        rate.sleep()