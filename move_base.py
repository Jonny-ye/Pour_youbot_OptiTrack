#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import math
import copy 

import tf
import tf.transformations as trans

from geometry_msgs.msg import Twist
import geometry_msgs.msg


#限制x、y方向的移动速度上限、下限
def limit_vel(v):
    if v >= 0.2:
        v=0.2
    elif v <= -0.2:
        v=-0.2
    return v

#该类作用：移动底盘控制
class Move_base:

    def __init__(self):
        #发布速度Topic
        self.pub=rospy.Publisher('cmd_vel',Twist,queue_size=3)
        self.listener = tf.TransformListener()

    #前向移动至抓取物体的位置，参考坐标系：object ——> arm
    def move_listen(self):
        
        k11= 0.5
        k22= 0.5
        k33= 0.3

        rate = rospy.Rate(50.0)
        twist=Twist()

        self.listener.waitForTransform("arm", "object", rospy.Time(), rospy.Duration(4.0))

        while not rospy.is_shutdown():
            try:
                (trans,rot) = self.listener.lookupTransform('arm','object',rospy.Time(0))
                eular=tf.transformations.euler_from_quaternion(rot)
                print(eular)
                print("角度 "+str(eular[2]))
                
            except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            
            print("x= "+str(trans[0]))      
            print("y= "+str(trans[1])) 
            
            #object在arm前方：x=0.5m,y=0.02m,Z方向欧拉角维持在弧度1.6
            if abs((trans)[0])<0.5 and abs((trans[1]))<0.02 and abs((eular[2]))<1.6:
                v1=0
                v2=0
                v3=0
                twist.linear.x = v1
                twist.linear.y = v2
                twist.angular.z = v3
                print(v1,v2,v3)
                self.pub.publish(twist)
                rate.sleep() 
                break
    
            else:
                v1 = limit_vel((trans[0]-0.5) * k11)
                v2 = limit_vel(trans[1]* k22)
                v3 = (eular[2]-1.59) * k33

                twist.linear.x = v1
                twist.linear.y = v2
                twist.angular.z = v3
                print("v1= "+str(v1),"v2= "+str(v2),"v3= "+str(v3))
                self.pub.publish(twist)
                self.pub.publish(twist)
                #rate.sleep() 
            

            rate.sleep() 

    def move_listen_cross(self):
        
        k11= 0.5
        k22= 0.5
        k33= 0.3

        rate = rospy.Rate(50.0)
        twist=Twist()

        self.listener.waitForTransform("arm", "object", rospy.Time(), rospy.Duration(4.0))

        while not rospy.is_shutdown():
            try:
                (trans,rot) = self.listener.lookupTransform('arm','object',rospy.Time(0))
                eular=tf.transformations.euler_from_quaternion(rot)
                print(eular)
                print("角度 "+str(eular[2]))
                
            except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            #rospy.loginfo('距离远点的位置：x=%f, y=%f,z=%f \n 角度：a=%f,b=%f,c=%f,d=%f',trans[0],trans[1],trans[2],eular[0],eular[1],eular[2])
            #v1 = trans[0] * k11
            #v2 = trans[1] * k22
            
            #x方向位置限制0.47m～0.5m,y方向移动-0.4m,z方向欧拉角<1.6弧度
            if  abs((trans)[0])<0.5 and abs((trans)[0])>0.47 and abs((trans[1]+0.40))<0.02 and abs((eular[2]))<1.6 :
                v1=0
                v2=0
                v3=0
                twist.linear.x = v1
                twist.linear.y = v2
                twist.angular.z = v3
                print(v1,v2,v3)
                self.pub.publish(twist)
                rate.sleep()
                break

            else:
                v1 = limit_vel((trans[0]-0.5) * k11)
                v2 = limit_vel((trans[1]+0.40)* k22)
                v3 = limit_vel((eular[2]-1.59) * k33)

                twist.linear.x = v1
                twist.linear.y = v2
                twist.angular.z = v3
                print("v1= "+str(v1),"v2= "+str(v2))
                self.pub.publish(twist)
                self.pub.publish(twist)
                #rate.sleep() 
                print('1')
            rate.sleep() 

    #作用：所有操作结束后，回到定点停放——（1.8,-0.17）处，欧拉角维持在-1.78弧度
    #动作过程：将抓取物体放置完成后，先自转，调整姿态，在移动（x,y）方向移动到终点停放。
    def move_listen_armToEnd(self):
            
        k11= 0.5
        k22= 0.5
        k33= 0.35

        rate = rospy.Rate(50.0)
        twist=Twist()
        self.listener.waitForTransform("world", "arm", rospy.Time(), rospy.Duration(4.0))

        while not rospy.is_shutdown():
            try:
                (trans1,rot1) = self.listener.lookupTransform('world','arm',rospy.Time(0))
                eular1=tf.transformations.euler_from_quaternion(rot1)
                print(eular1)
                print("角度 "+str(eular1[1]))
                
            except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            #rospy.loginfo('距离远点的位置：x=%f, y=%f,z=%f \n 角度：a=%f,b=%f,c=%f,d=%f',trans[0],trans[1],trans[2],eular[0],eular[1],eular[2])
            #v1 = trans[0] * k11
            #v2 = trans[1] * k22
            
            
            print("x= "+str(trans1[0]))      
            print("y= "+str(trans1[2])) 

            if abs(-1.78-eular1[1])>0.3:    
                v1=0
                v2=0
                v3=(-1.78-eular1[1]) * k33
                twist.linear.x = v1
                twist.linear.y = v2               
                twist.angular.z = v3
                print(eular1[1])
                self.pub.publish(twist) 
            else:

                if abs(trans1[2]-1.8)<0.15 and abs(trans1[0]+0.017)<0.15:# and abs(-1.78-eular1[1])<0.15:
                    v1=0
                    v2=0
                    v3=0
                    twist.linear.x = v1
                    twist.linear.y = v2
                    twist.angular.z = v3
                    print(v1,v2)
                    self.pub.publish(twist)
                    break
           
                v1 = limit_vel((1.8-trans1[2]) * k11)
                v2 = limit_vel(( -0.017 - trans1[0])* k22)
                v3 = 0
                twist.linear.x = v1
                twist.linear.y = v2
                twist.angular.z = v3
                print("v1= "+str(v1),"v2= "+str(v2))
                self.pub.publish(twist)

def test():

    rospy.init_node('youbot_arm_control')
    
    try:
        demo = Move_base()

        # demo.move_listen()
        # rospy.sleep(2)
        # demo.move_listen_cross()
        demo.move_listen_armToEnd()

    except rospy.ROSInterruptException:
        print "EXCEPTION"
        pass
    rospy.spin()


if __name__ == '__main__':
    test()