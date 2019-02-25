# pour
ik_lib.py:

    1、逆运动学求逆解
      dls_ik():带有末端姿态、位置的逆解迭代求法。
      dls_ik_position_only():仅有末端位置的逆解求法。
      
    2、YoubotArm:机械臂的类：
      2.1、求逆解，需要安装python的两个库：
         pykdl_utils（KDL运动学库）、urdf_parser_py（URDF参数解析库）
         安装方式见本人简书：https://www.jianshu.com/p/f7a39b43f1bf
         
      2.2、youbot机械臂各关节最大最小限关节量
      2.2、话题的发布：
          机械臂关节量发布、末端夹子发布、机械臂关节速度发布
          
move_base.py: youbot底盘移动控制 

    1、move_listen（）:移动到optitrack监听到物体所贴Marker点的位置，保持在前方0.5m处。
    2、move_listen_cross（）：在物体前方位置，横向移动固定距离。
    3、move_listen_armToEnd（）：移动到机械人停放位置。
   

planning_control.py：

    直线路径的规划，以及youbot机械臂末端的基于速度环的跟踪。
    Arm_line_planning（）：线性规划的类
    1、planning_line（）：直线路径规划函数
    2、LineToJointVel（）：机械臂末端的速度跟踪。
    
    ListenJointState（）：监听机械臂当前的状态的类——速度、关节量、力矩（不涉及）
    
    tra_planning（）：直线路径的跟踪，并给予速度在最后一个时刻通过PD控制维持住当前状态。

pour_project.py：倒水主程序

    1、tra_planning()：根据要求设置的路径跟踪函数——涉及夹子动作、机械臂动作。
    2、demo()：倒水DEMO的类：

离线党的路径规划： .txt文件

    规划q_out_back：机械臂前伸的直线规划的路径——直线上各个采样点处的逆解得出的关节量。
    规划q_out_fro：机械臂收回的直线规划的路径——直线上各个采样点处的逆解得出的关节量。
 
          
