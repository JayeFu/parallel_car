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

    seri_ik = SerialIKSolver()

    rate = rospy.Rate(1.0)


    while not rospy.is_shutdown():

        if seri_ik.listen_to_tf():
            seri_ik.print_T_down_to_up()
            seri_ik.compute_ik()
         
        rate.sleep()
            