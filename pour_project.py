#!/usr/bin/python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
import copy

from ik_lib import YoubotArm
from ik_lib import alter_pos_to_se3

from planning_control import ListenJointState
from planning_control import IO
from move_base import Move_base

import tf
import tf.transformations as trans

#打开夹子宽度、抓取物体宽度
gripperWidthAtGrasp = 0.0041
gripperWidthOpen = 0.0115
home = np.array([0.11, 0.11, -0.11, 0.11, 0.12])


def listener_arm_obj():
    
    listener = tf.TransformListener()
    rate = rospy.Rate(50.0)   

    (trans,rot) = listener.lookupTransform('arm','object',rospy.Time(0))

    rospy.loginfo('距离远点的位置：x=%f, y=%f,z=%f \n 旋转四元数：a=%f,b=%f,c=%f,d=%f',trans[0],trans[1],trans[2],rot[0],rot[1],rot[2],rot[3])
    
    quat_listen = np.array([rot[0],rot[1],rot[2],rot[3]])
    pos_listen = np.array([trans[0],trans[1],trans[2]])

    rate.sleep() 
    return quat_listen,pos_listen



#order:
#      往前的规划：1
#      往后的规划：0
def tra_planning(q_out,order):

    arm_control=YoubotArm()
    joint_state=ListenJointState()
    move=Move_base()

    #先让机械臂到初始位置状态：q_out[0]
    print('q_out[0]',q_out[0])

    #order=1时，先动底盘关节
    if order==1:
        home_fro=copy.deepcopy(home)
        home_fro[0]=copy.deepcopy(q_out[0,0])
        print(home_fro,"home_fro")
        for i in range(2):
            arm_control.publish_arm_joint_positions(home_fro)
            rospy.sleep(1)
    
    if order==0:
        #移动到物体
        move.move_listen()
        rospy.sleep(1)

    #发布第一个q_out[0]数据
    arm_control.publish_arm_joint_positions(copy.deepcopy(q_out[0]))
    #延迟一下，否则太快发不出去
    rospy.sleep(1)
    


    arm_control.publish_gripper_width(gripperWidthOpen)
    rospy.sleep(2)


    #轨迹跟踪核心内容——基于机械臂速度环的P控制
    k1=np.array([[0,0,0,0,0],[0,2,0,0,0],[0,0,2,0,0],[0,0,0,1,0],[0,0,0,0,0]])
    vel_commmand=np.zeros(5)
     
    for i in range(99):
        joint_state.state_listener()
        joint_pos=joint_state.joint_pos
        # joint_vel=joint_state.joint_vel

        vel_commmand= np.dot(k1,(q_out[i]-joint_pos))

        arm_control.publish_arm_joint_velocities(vel_commmand)
    
    #发布最后一个q_out[98]
    for i in range(2):
        
        arm_control.publish_arm_joint_positions(copy.deepcopy(q_out[98]))
        rospy.sleep(0.5)

    if order==1:
        arm_control.publish_gripper_width(gripperWidthAtGrasp)
        rospy.sleep(1)

    if order==0:
        arm_control.publish_gripper_width(0.)
        rospy.sleep(2)
        
        #先发第2到5个关节量
        home_back=copy.deepcopy(q_out[98])
        home_back[1:5]=copy.deepcopy(home[1:5])
        print(home_back,"home_back")

        arm_control.publish_arm_joint_positions(home_back)
        rospy.sleep(1)
        arm_control.publish_arm_joint_positions(home)
        rospy.sleep(1)


def demo():

    #类的实例化
    io=IO()
    arm = YoubotArm()
    move=Move_base()

    #姿态绕x轴旋转-120
    rot_120 = np.array([[1,0,0,0],[0,-0.5,0.866,0],[0,-0.866,-0.5,0],[0,0,0,1]])
    
    #从tf树上读取四元素quat和位置pos
    # quat, pos = listener_arm_obj()
    # print("xyz",pos) 

    #规划内容：
    q_out_fro=io.loadtxt("规划q_out_fro")
    q_out_back=io.loadtxt("规划q_out_back")

    move.move_listen()
    rospy.sleep(1)

    tra_planning(q_out_fro,1)

    #抓取瓶子后，上升5cm
    guess1 = [2.96705,0.999,-0.68886,1.6358575,2.923426497]
    pos_up = np.array([[0,0,1,0.3],[0,1,0,0],[-1,0,0,0.3],[0,0,0,1]])
    q_out_up = arm.ik_solve(pos_up,guess1)
    arm.publish_arm_joint_positions(q_out_up)
    print(q_out_up,'q_out_up')
    rospy.sleep(3)

    #横向移动0.4m
    move.move_listen_cross()
    rospy.sleep(1)

    #倒水动作——末端夹子绕x轴旋转-120度
    pos_pour=alter_pos_to_se3(rot_120,pos_up)
    q_out_pour = arm.ik_solve(pos_pour,q_out_up)
 
    arm.publish_arm_joint_positions(q_out_pour)
    rospy.sleep(1.5)

    arm.publish_arm_joint_positions(q_out_up)
    rospy.sleep(2)

    #机械臂收回的末端规划
    tra_planning(q_out_back,0)
    rospy.sleep(1)
    
    #移动到终点停放
    move.move_listen_armToEnd()

 
def main():

    rospy.init_node('youbot_arm_control')
    
    try:

        demo()
    except rospy.ROSInterruptException:
        print "EXCEPTION"
        pass
    rospy.spin()


if __name__=="__main__":
    main()
