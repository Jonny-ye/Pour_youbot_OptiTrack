#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import copy

from pykdl_utils.kdl_kinematics import KDLKinematics
from urdf_parser_py.urdf import URDF
from ik_lib import YoubotArm


def test():
    arm=YoubotArm()

    # 前抓
    # Guess = [ 2.96833824,  1.04957583, -1.05812389,   2.27263849 ,  2.92212518]

    Guess=[1.52, 1.84, -1.26, 2.4, 3.10]
    #x=0.3m的关节量： 
    q_in1=[ 2.96705973,  1.04957583, -1.37863759,  2.27263849,  2.9234265 ]
    #x=0.4m的关节量： 
    q_in2=[ 2.96705973,  1.63038649, -2.0173912 ,  2.3291985 ,  2.9234265 ]

#(array([ 1.27683451,  0.7529159 , -1.37285255,  2.60791827,  3.1       ]), 左侧)


    # youbot_urdf = URDF.from_xml_file("/home/zy/catkin_ws/sript/urdf/youbot_arm.urdf")
    # kin_grasp = KDLKinematics(youbot_urdf, "base_link", "arm_link_5")

    #向右
    pos=np.array([[ -1,0 ,0,0],
                  [ 0 ,0,-1,-0.2 ],
                  [ 0 ,-1,0,0.35],
                  [ 0,0,0, 1  ]])
    q_right=[ 4.54247722,  0.72588121, -1.31810986,  2.5384595 ,  4.49422372]
    #向左
    #pos=np.array([[ 1 ,0 ,0,0],
                #   [ 0 ,0,1,0.2 ],
                #   [ 0 ,-1 ,0,0.35],
                #   [ 0,0,0, 1  ]])
    q_lift=[ 1.39163762,  0.73450602, -1.33782528,  2.5498045 ,  4.49422073]
    home = np.array([4.54247722, 0.11, -0.11, 0.11, 0.12])
    # q_out=arm.ik_solve(pos,copy.deepcopy(Guess))
    # print(copy.deepcopy(Guess))

    # for i in range(5):
    #     arm.publish_arm_joint_positions(q_lift)
    #     rospy.sleep(1)

    for i in range(5):

        arm.publish_arm_joint_positions(home)
        rospy.sleep(1)

    for i in range (5):
        
        arm.publish_arm_joint_positions(q_right)
        rospy.sleep(1)
        
    # print(q_out,'q_out')
    # rospy.sleep(2)

    # arm.publish_arm_joint_positions(q_out)


if __name__=='__main__':
    rospy.init_node('abc',anonymous=True)
    test()