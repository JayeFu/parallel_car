#!/usr/bin/env python  
import rospy
from parallel_car.IKSolver import ParallelIKSolver

if __name__ == '__main__':
    rospy.init_node('iksolver')

    para_ik = ParallelIKSolver()

    rate = rospy.Rate(1.0)

    while not rospy.is_shutdown():
        if para_ik.listen_to_tf():
            para_ik.calculate_pole_length_from_inherent()
            para_ik.print_pole_length()
        rate.sleep()
