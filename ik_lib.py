#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import copy 
import random
import tf
import tf.transformations as trans

from pykdl_utils.kdl_kinematics import KDLKinematics
from urdf_parser_py.urdf import URDF

from brics_actuator.msg import JointPositions
from brics_actuator.msg import JointValue
from brics_actuator.msg import JointVelocities



# Multiplies multiple matrices together.

def matmult(*x):
    return reduce(np.dot, x)

# Turns the parameter 3-vector into a 3x3 skew symmetric matrix.
#
# Params:
# w - a 3-vector.
#
# Returns:
# A 3x3 skew symmetric matrix.

def hat(w):
    return np.array([[0,-w[2],w[1]],
                     [w[2],0,-w[0]],
                     [-w[1],w[0],0]])

# Calculates a rotation matrix given an axis and rotation angle.
#
# Params:
# w - the axis of rotation
# th - the angle of rotation
#
# Returns:
# An SO(3) rotation matrix.

def expm(w,th):
    return np.identity(3) + hat(w)*np.sin(th) + np.dot(hat(w),hat(w))*(1-np.cos(th))

# Takes in a quaternion and returns the corresponding SO(3) matrix.
#
# Params:
# quat - the quaternion defining a rotation.
#
# Returns:
# An SO(3) matrix.

def quat_to_so3(quat):
    q = np.array(quat[0:3])
    th = quat[3]
    th = 2*np.arccos(th)
    if th != 0:
        q /= np.sin(th/2.0)
    else:
        q *= 0.0
    # return scipy.linalg.expm(hat(q)*th)
    return expm(q,th)


# Takes in a quatnernion and position and returns the corresponding SE(3) matrix.
#
# Params:
# quat - the quaternion defining a rotation.
# pos - a 3-vector defining a (x,y,z) location.
#
# Returns:
# An SE(3) matrix.

def quat_pos_to_se3(quat,pos):
    R = quat_to_so3(quat)
    g = np.hstack((R, np.array([pos.ravel()]).T))
    return np.vstack((g,[0,0,0,1]))

#3*3矩阵：姿态 + pos:位置 —— 4*4描述矩阵
def fix_pos_to_se3(pose,pos):
    R = pose[0:3,0:3]
    g = np.hstack((R, np.array([pos.ravel()]).T))
    return np.vstack((g,[0,0,0,1]))

#固定位置，姿态变换
def alter_pos_to_se3(rotate,pose):
    R = pose[0:3,0:3]
    P = pose[0:3,-1]
    rot=rotate[0:3,0:3]
    
    Rot=matmult(rot,R)
    g = np.hstack((Rot, np.array([P.ravel()]).T))
    return np.vstack((g,[0,0,0,1]))

# # Params:
# kin - the kinematic model
# pose - the desired SE(3) pose of the end-effector.
# q0 - an intial guess for the inverse kinematic solution.
# lam - a tuning parameter for the damping.
# num - maximum number of iterations.
#
# Returns:
# The list of joint angles as a solution to the inverse kinematics.

def dls_ik(kin, pose, q0, lam=0.25, num=100):
    # get position and orientation:
    Rd = pose[0:3,0:3]
    Pd = pose[0:3,-1]
    # setup iterations:
    q = copy.deepcopy(q0)
    # run loop trying to get to target:
    for i in range(num):
        J = kin.jacobian(q)
        g = np.array(kin.forward(q))
        R = g[0:3,0:3]
        P = g[0:3,-1]
        Rdelt = matmult(Rd, R.T)
        rdelt_angles = np.array(trans.euler_from_matrix(Rdelt))
        e = np.hstack((Pd-P, rdelt_angles))
        dq = np.array(matmult(J.T,np.linalg.inv(matmult(J,J.T) + lam*np.eye(J.shape[0]))))
        q += matmult(dq,e)
        ##############################################################
        # should add a break condition and corresponding tolerance!! #
        ##############################################################
    return q

# Computes the inverse kinematics for the position only (no orientation) using damped 
# least squares given a pose, a starting guess, a damping parameter, and a maximum 
# number of iterations.
#
# Params:
# kin - the kinematic model
# pose - the desired SE(3) pose of the end-effector.
# q0 - an intial guess for the inverse kinematic solution.
# lam - a tuning parameter for the damping.
# num - maximum number of iterations.
#
# Returns:
# The list of joint angles as a solution to the inverse kinematics.

def dls_ik_position_only(kin, pose, q0, lam=0.25, num=100):
    # get position and orientation:
    Rd = pose[0:3,0:3]
    Pd = pose[0:3,-1].ravel()
    # setup iterations:
    q = copy.deepcopy(q0)
    # run loop trying to get to target:
    for i in range(num):
        J = kin.jacobian(q)
        J = J[0:3,:]
        g = np.array(kin.forward(q))
        P = g[0:3,-1]
        e = Pd-P.ravel()
        dq = np.array(matmult(J.T,np.linalg.inv(matmult(J,J.T) + lam*np.eye(J.shape[0]))))
        q += matmult(dq,e)
        ##############################################################
        # should add a break condition and corresponding tolerance!! #
        ##############################################################
    return q

#限制关节量
def low_high_limit(value, low_limit, high_limit):
    if low_limit >= high_limit:
        return value

    if value < low_limit:
        return low_limit

    if value > high_limit:
        return high_limit

    return value

#判断输出的关节量是否被限制
def equal_q1_q2(q1,q2):
    for i in range(5):
        if q1[i] != q2[i]:
            return False

    return True




class YoubotArm:

    # Constructor.

    def __init__(self):
        self.jointGuessForGrasp = [ 2.96, 1.42, -1.14, 0, 2.92]
        #最大、最小关节量限定值
        self.jointMax= [5.840139, 2.617989, -0.0157081, 3.42919, 5.641589]
        self.jointMin= [0.01006921, 0.01006921, -5.0264, 0.0221391, 0.11062]

        youbot_urdf = URDF.from_xml_file("/home/zy/catkin_ws/sript/urdf/youbot_arm.urdf")
        self.kin_grasp = KDLKinematics(youbot_urdf, "base_link", "arm_link_5")

        self.arm_joint_pub = rospy.Publisher("arm_1/arm_controller/position_command",JointPositions,queue_size=10)
        self.gripper_pub = rospy.Publisher("arm_1/gripper_controller/position_command", JointPositions, queue_size=10)        
        self.arm_joint_vel_pub = rospy.Publisher("arm_1/arm_controller/velocity_command",JointVelocities,queue_size=10)
        
    # Takes in a list of joint angles for joints 1-5 and publishes them for the YouBot to receive.
    #
    # Params:
    # joint_positions - the list of joint positions to publish.  Should be for arm joints 1-5.
    def publish_arm_joint_positions(self,joint_positions):

        desiredPositions = JointPositions()

        jointCommands = []

        for i in range(5):
            joint = JointValue()
            joint.joint_uri = "arm_joint_" + str(i+1)
            joint.unit = "rad"
            joint.value = joint_positions[i]

            jointCommands.append(joint)
            
        desiredPositions.positions = jointCommands

        self.arm_joint_pub.publish(desiredPositions)


    

    # Publishes the parameter gripper width to the YouBot to set its position.
    #
    # Params:
    # width - the width value to be applied to both gripper fingers.

    def publish_gripper_width(self, width):
                  
        desiredPositions = JointPositions()

        jointCommands = []

        joint = JointValue()
        joint.joint_uri = "gripper_finger_joint_l"
        joint.unit = "m"
        joint.value = width
        jointCommands.append(joint)

        joint = JointValue()
        joint.joint_uri = "gripper_finger_joint_r"
        joint.unit = "m"
        joint.value = width
        jointCommands.append(joint)

        desiredPositions.positions = jointCommands

        self.gripper_pub.publish(desiredPositions)


    #关节速度发布
    def publish_arm_joint_velocities(self,Joint_Velocities):

        desiredVelocities = JointVelocities()

        jointCommands = [] 

        for i in range(5):
            joint = JointValue()
            joint.joint_uri = "arm_joint_" + str(i+1)
            joint.unit = "s^-1 rad"
            joint.value = Joint_Velocities[i]

            jointCommands.append(joint)
            
        desiredVelocities.velocities = jointCommands

        self.arm_joint_vel_pub.publish(desiredVelocities)   

    
    #不断更换求逆解的初始猜测值，直至限位前后逆解值相同，返回求解出五关节量：q_out_grasp
    def ik_solve(self,pose,q0):
        while not rospy.is_shutdown():
            # print("q0",q0)
            q_grasp = dls_ik(self.kin_grasp,pose, q0)
            # print("111",q_grasp)

            #限制最大、最小关节量
            q_out_grasp=self.limit_arm_joints(copy.deepcopy(q_grasp))
            # print("222",q_out_grasp)

            #若第一次求出的逆解的任意关节量被限制，则产生随机初始猜测重新求逆解
            if equal_q1_q2(q_grasp,q_out_grasp) == False:
                print(123)
                for i in range(5):
                    q0[i] = random.uniform(self.jointMin[i],self.jointMax[i])
                continue
            return q_out_grasp
        
    #不断更换求逆解的初始猜测值，直至限位前后逆解值相同，返回求解出五关节量：q_out_grasp
    def ik_solve_position_only(self,pose,q0):
        while not rospy.is_shutdown():
            print("q0",q0)
            q_grasp = dls_ik_position_only(self.kin_grasp, pose, q0)
            print("111",q_grasp)

            #限制最大、最小关节量
            q_out_grasp=self.limit_arm_joints(copy.deepcopy(q_grasp))
            print("222",q_out_grasp)

            #若第一次求出的逆解的任意关节量被限制，则产生随机初始猜测重新求逆解
            if equal_q1_q2(q_grasp,q_out_grasp) == False:
                print(123)
                for i in range(5):
                    q0[i] = random.uniform(self.jointMin[i],self.jointMax[i])
                continue
            return q_out_grasp

   
    def limit_arm_joints(self, joints):
        for i in range(5):
            joints[i] = low_high_limit(joints[i], self.jointMin[i], self.jointMax[i])
            
        return joints



def main():

    rospy.init_node('youbot_arm_control')
    
    try:
        demo = YoubotArm()
        
    except rospy.ROSInterruptException:
        print "EXCEPTION"
        pass
    rospy.spin()


if __name__ == '__main__':
    main()

