#!/usr/bin/env python  

import sys

import rospy
from parallel_car.Optimizer import SimpleOptimizer
from parallel_car.IKSolver import SerialIKSolver
from parallel_car.Driver import BaseAndMechDriver
from parallel_car.PrintFunc import print_tf
from geometry_msgs.msg import Transform, Vector3, Quaternion

from iiwa_agv.Executor import Executor

from threading import Thread

# either 'rviz' or 'gazebo'
RUN_ENV = 'gazebo'


def init_pose(obj):
    # let obj go to initial pose
    obj.init_pose()

def send_path(ob):
    # let obj go to rest poses
    ob.send_trajectory()


if __name__ == "__main__":

    # init node
    rospy.init_node("auto_controller")

    # parallel car construction 
    seri_ik = SerialIKSolver(run_env=RUN_ENV)

    mbx_file_path = "../data/mbx_planned_trajectory_zheng.txt"

    para_car = BaseAndMechDriver(file_path=mbx_file_path)

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

    para_car.read_trajectory()

    o_to_wx_tf_list = para_car.get_o_to_wx_tf_list()

    for tf_idx in range(len(o_to_wx_tf_list)):
        # get a tf from o_to_wx_tf_list
        o_to_wx_tf = o_to_wx_tf_list[tf_idx]
        # compute the parallel pose and serial pose from modified matrix
        (parallel_pose_desired, serial_pose_desired) = seri_ik.compute_ik_from_o_to_wx_tf(o_to_wx_tf)
        # drive store both parallel pose and serial pose inside
        para_car.append_pose_desired(parallel_pose_desired, serial_pose_desired)

    # END parallel car construction 

    # iiwa agv construction

    iiwa_car = Executor(file_path='/home/fjw/dual_ws/src/iiwa_agv/data/fwx_planned_trajectory_zheng.txt')

    # listen to initial offset until success
    while not iiwa_car.listen_to_initial_offset():
        rospy.logerr("Waiting for listening to initial offset")
        rate.sleep()
    
    rospy.loginfo("Have successfully listened to initial offset")
    
    # read the joint trajectory from txt file
    iiwa_car.read_trajectory()

    # END iiwa car construction

    # initiate robots using thread
    try:
        iiwa_car_thread = Thread(target=init_pose,args=(iiwa_car,))
        para_car_thread = Thread(target=init_pose,args=(para_car,))
        iiwa_car_thread.start()
        para_car_thread.start()
        iiwa_car_thread.join()
        para_car_thread.join()
    except:
        rospy.logerr('Error in FIRST pose')
        exit()
    
    rospy.loginfo('the robots are initialized! ')
    raw_input("Enter to start the simulation")

    try:
        iiwa_car_thread = Thread(target=send_path,args=(iiwa_car,))
        para_car_thread = Thread(target=send_path,args=(para_car,))
        iiwa_car_thread.start()
        para_car_thread.start()
        iiwa_car_thread.join()
        para_car_thread.join()
    except:
        rospy.logerr('Error in OTHER poses')
        exit()

