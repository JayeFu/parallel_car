#!/usr/bin/env python  

import rospy
from parallel_car.Optimizer import GradientOptimizer, SimpleOptimizer
from parallel_car.IKSolver import ParallelPose ,ParallelIKSolver, SerialIKSolver
from parallel_car.TransRotGen import quaternion_to_rotation_matrix, vector3_to_translation_matrix, transform_to_matrix, yaw_from_quaternion_only_Z
from geometry_msgs.msg import Pose, Point

# either 'rviz' or 'gazebo'
RUN_ENV = 'rviz'

if __name__ == "__main__":
    rospy.init_node("auto_controller")

    seri_ik = SerialIKSolver(run_env=RUN_ENV)

    optimizer = SimpleOptimizer()

    listened_to_fixed = False

    rate = rospy.Rate(1.0)

    if RUN_ENV == 'rviz':
        origin = 'car_link'
    else: # RUN_ENV == 'gazebo'
        origin = 'world'

    while not rospy.is_shutdown():
        (o_to_wx_succ, o_to_wx_tf) = seri_ik.get_transform(origin, "wx_link")
        if o_to_wx_succ:
            (optimal_alpha, T_o_to_wx_modified) = optimizer.compute_optimal_alpha(o_to_wx_tf)
            (parallel_pose_desired, serial_pose_desired) = seri_ik.compute_ik_from_modified_matrix(T_o_to_wx_modified)
        else:
            rospy.logerr("listening to {}-wx_link transform failed.".format(origin))

        try:
            rate.sleep()
        except ROSInterruptException:
            print "end while sleeping"
            break
        