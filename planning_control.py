#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import copy
import numpy as np

from ik_lib import YoubotArm

from sensor_msgs.msg import JointState


def joint_trans_judge(q_now,q_pre):

    for i in range(5):
        if abs(q_now[i]-q_pre[i]) >= 0.1:
            return False

    return True


# This class is for saving np.array into text, and reading from text.
class IO(object):
    @staticmethod
    def savetxt(filename, data):
        np.savetxt(filename, data, delimiter=",")

    @staticmethod
    def loadtxt(filename):
        return np.loadtxt(filename, delimiter=",")


class Arm_line_planning(object):
    #机械臂的直线拟合
    #参数：
    #start_point：起始点坐标
    def __init__(self,pos,start_point=np.array([0.2,0.,0.35]),end_point=np.array([0.2,0.,0.45])):
        
        #类的实例化
        self.arm_control=YoubotArm()
       
        self.start_point=start_point
        self.end_point=end_point
        self.pos=pos

        self.line_point=np.zeros((100,3))
        self.position=np.zeros((100,4,4))
        self.q_out = np.zeros((100,5))
        self.Diff = np.zeros((100,5))
        
        self.planning_line()
        self.postoRot()
        self.LineToJointVel()

        # print('111')
        # print(self.line_point)
        # print('222')
        # print(self.position)
        # print('333')
        # print('self.q_out',self.q_out)
        # print('self.Diff',self.Diff)
     

    def planning_line(self):

        k = self.end_point-self.start_point
        dt=0.01
        
        #t在[0,1]之间，采样100次，单位：秒（s）
        for i in range(100):
            if i==0:
                self.line_point[i]=self.start_point+k*dt
                continue
            self.line_point[i] = self.line_point[i-1]+k*dt

    def postoRot(self):

        R=self.pos[0:3,0:3]

        for i in range(100):
            pos=self.line_point[i]
            g = np.hstack((R, np.array([pos.ravel()]).T))

            self.position[i,:,:]=np.vstack((g,[0,0,0,1]))
        

    def LineToJointVel(self):
        
        #得到参考位置position[i],和关节空间（q1~q5）与上一时刻做差分得到关节参考角速度Diff[j]
        Guess = [ 2.96833824,  1.04957583, -1.05812389,   2.27263849 ,  2.92212518]
        
        for i in range(100):
            print(i,'++++++++++++++++++++++++++')
            p0=copy.deepcopy(self.position[i])
            while not rospy.is_shutdown():
                # p0=copy.deepcopy(self.position[i])
                if i==0:
                    j=0
                    self.q_out[i] = self.arm_control.ik_solve(p0,copy.deepcopy(Guess))
                        # if joint_trans_judge(copy.deepcopy(self.q_out[i]),Guess)==True:
                        #     break
                    break

                elif i==99:
                    break

                q0=copy.deepcopy(self.q_out[i-1])
                
                self.q_out[i] = self.arm_control.ik_solve(p0,q0)
                q1=copy.deepcopy(self.q_out[i])
                print(i)
                if joint_trans_judge(q1,q0) == True:
                    print('11111')
                    self.Diff[j] = np.divide(self.q_out[i]-self.q_out[i-1],0.01)
                    j=j+1 
                    break


class ListenJointState(object):

    def __init__(self):

        self.joint_pos = np.zeros(5)
        self.joint_vel = np.zeros(5)
        self.state_listener()

    
    def callback(self,JointState):

        # rospy.loginfo('youbot当前状态:position={}'.format(JointState.position))
        # rospy.loginfo('youbot当前状态:position={},velocity={}'.format(JointState.position,JointState.velocity))
        self.joint_pos = JointState.position[0:5]
    
        # self.joint_vel = JointState.velocity[0:5]

    def state_listener(self):
        
        joint_states_sub = rospy.Subscriber("/joint_states",JointState,self.callback)
        # rospy.spin()



def tra_planning(q_out):

    arm_control=YoubotArm()
    joint_state=ListenJointState()

    #先让机械臂到初始位置状态：q_out[0]
    print('q_out[0]',q_out[0])
    for i in range(10):

        arm_control.publish_arm_joint_positions(copy.deepcopy(q_out[0]))
        #延迟一下，否则太快发不出去
        rospy.sleep(1)

    k1=np.array([[0,0,0,0,0],[0,2,0,0,0],[0,0,2,0,0],[0,0,0,1,0],[0,0,0,0,0]])
    #k1=np.array([[0,0,0,0,0],[0,0.8,0,0,0],[0,0,0.7,0,0],[0,0,0,0.8,0],[0,0,0,0,0]])
    k2=np.array([[0,0,0,0,0],[0,0.02,0,0,0],[0,0,0.03,0,0],[0,0,0,0.05,0],[0,0,0,0,0]])
    vel_commmand=np.zeros(5)
     
    for i in range(99):
        joint_state.state_listener()
        joint_pos=joint_state.joint_pos
        # joint_vel=joint_state.joint_vel

        # if i==99:
        #     rospy.sleep(0.5)
        #     #定住末状态关节量
        #     arm_control.publish_arm_joint_positions(joint_pos)
        #     break
        # rospy.loginfo('youbot当前状态:position={},velocity={}'.format(joint_pos,joint_vel))
        # vel_commmand=-np.dot(k1,(line.q_out[i]-joint_pos)) - np.dot(k2,(line.Diff[i]-joint_vel))
        vel_commmand= np.dot(k1,(q_out[i]-joint_pos))
        print(joint_pos,'===========')
        print(vel_commmand)
        arm_control.publish_arm_joint_velocities(vel_commmand)
    
    
    #上一时刻误差
    err_prv = q_out[98]-copy.deepcopy(joint_pos)
    print(err_prv,'err_prv')
    while not rospy.is_shutdown():

        joint_state.state_listener()
        joint_pos=joint_state.joint_pos
        # joint_vel=joint_state.joint_vel
        
        #当前误差
        err_cur = q_out[98]-copy.deepcopy(joint_pos)
        
        #速度误差：微分
        vel_err = err_cur - err_prv
        #PD控制律
        vel_commmand= np.dot(k1,err_cur) + np.dot(k2, vel_err) 
        err_prv = copy.deepcopy(err_cur)

        print(err_prv,'err_prv')
        print(err_cur,'err_cur')

        # print(vel_err,'vel_err')
        print(joint_pos,'joint_pos')
        # print((q_out[98]-joint_pos),'error')

        arm_control.publish_arm_joint_velocities(vel_commmand)


#规划路径生成、路径跟踪测试程序
def test():

    io=IO()

    #～注～：生成直线规划对应的关节量变化：q_out;  速度差分：Diff
    
#此时，注释部分，是生成离线规划数据。

    # start_point = np.array([0.3,0, 0.25])
    # end_point = np.array([0.25, 0,  0.25])
    # pos=np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0],[0,0,0,1]])

    # line=Arm_line_planning(pos,start_point,end_point)

    # io.savetxt(filename="规划q_out_back",data=line.q_out)

#该部分完成那个离线规划数据的读入和规划动作。
    q_out=io.loadtxt("规划q_out_fro")
    tra_planning(q_out)



    # #JointState读取速度信息
    # io.savetxt(filename="规划Diff",data=line.Diff)
    # Diff=io.loadtxt("规划Diff")

    # print('youbot当前状态:q_out={},\n Diff={}'.format(q_out,Diff))


if __name__=='__main__':
    rospy.init_node('pylistener',anonymous=True)
    test()
