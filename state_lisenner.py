#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import copy
import numpy as np

from brics_actuator.msg import JointValue
from sensor_msgs.msg import JointState

# This class is for saving np.array into text, and reading from text.
class IO(object):
    @staticmethod
    def savetxt(filename, data):
        np.savetxt(filename, data, delimiter=",")

    @staticmethod
    def loadtxt(filename):
        return np.loadtxt(filename, delimiter=",",newline='\n')

def callback(JointState):
    io=IO()
    rospy.loginfo('youbot当前状态:position={},velocity={}'.format(JointState.position[0:5],JointState.velocity[0:5]))
    io.savetxt(filename="编码器数据",data=JointState.position[0:5])
   
    # rospy.sleep(5)
def listener():
    
    rospy.init_node('pylistener',anonymous=True)
    
    joint_states_sub = rospy.Subscriber("/joint_states",JointState,callback)
    
    rospy.spin()

listener()