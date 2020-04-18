#!/usr/bin/env python  

import rospy
from parallel_car.IKSolver import ParallelPose ,ParallelIKSolver
from parallel_car.TransRotGen import quaternion_to_rotation_matrix, vector3_to_translation_matrix
from geometry_msgs.msg import Pose

if __name__ == "__main__":
    rospy.init_node("auto_tester")

    para_ik = ParallelIKSolver()

    rate = rospy.Rate(5.0)

    listened_to_fixed = False

    while not rospy.is_shutdown():
        if not listened_to_fixed: # have not listned to fixed tf
            if para_ik.listen_to_up_down_fixed_tf():
                listened_to_fixed = True
            else: # for integrity
                pass
        else: # listned_to_fixed is True
            # listen to tf from down_num to up_num
            if para_ik.listen_to_tf():
                # calculate the num from
                para_ik.calculate_pole_length_from_inherent()
                para_ik.print_pole_length()