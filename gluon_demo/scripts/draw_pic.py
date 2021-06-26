#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from copy import deepcopy

import rospy, sys
import tf2_ros
from std_msgs.msg import Duration
from geometry_msgs.msg import Pose, TransformStamped, PointStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import moveit_commander
from moveit_commander.conversions import pose_to_list   # moveit 相关
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene, RobotTrajectory, RobotState
from gluon_demo.msg import gluon_demo_msgs

import Geometry
import inv_kinematic

#
# (在末端坐标里移动的点的集合， 末端坐标系的初始位姿)
def trans(waypoints_, transf_ ):
    #transf_ = TransformStamped()
    #waypoints_[0] =  Pose()
    wp = []
    for i in waypoints:
        pose_temp = Pose()
        p = np.array([[i.position.x],
                      [i.position.y],
                      [i.position.z]])

        q = transf_.transform.rotation
        t = transf_.transform.translation
        r = Geometry.Quaternion_2_matrix( [q.x, q.y, q.z, q.w] )
        p = r.dot(p)

        pose_temp.position.x = p[0,0]+t.x
        pose_temp.position.y = p[1,0]+t.y
        pose_temp.position.z = p[2,0]+t.z
        pose_temp.orientation = q

        wp.append(pose_temp)
    return wp

# 定义了一个全局变量 target_pose
target_pose = TransformStamped()  
target_pose.transform.translation.x = 0.28
target_pose.transform.translation.y = -0.17
target_pose.transform.translation.z = 0.10

target_pose.transform.rotation.w = 0.707106781
target_pose.transform.rotation.z = -0.707106781

#
# 读取文件夹下的图片，生成轨迹后执行
if __name__ == '__main__':
    #
    # 初始化 move_group
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_gourp_gluon', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    rospy.loginfo('Moveit scene 初始化完成')

    group_name = "gluon"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_group_real = move_group
    move_group.set_goal_position_tolerance(0.001)
    move_group.set_goal_orientation_tolerance(0.001)
    move_group.set_planning_time(5.0)
    move_group.allow_replanning(True)
    rospy.sleep(1) 
    end_effector_link = move_group.get_end_effector_link()
    rospy.loginfo('Move_group 初始化完成')

    #
    # 开始规划
    tfBuffer = tf2_ros.Buffer() # tf2接收器，接受目标位姿
    listener = tf2_ros.TransformListener(tfBuffer)
    pub_Command =  rospy.Publisher("/manipulator/command", gluon_demo_msgs, queue_size=1)  # 设置控制指令发布 topic句柄
    pub_point = rospy.Publisher("/point_path", PointStamped, queue_size=1)
    rospy.sleep(0.5)
    rospy.loginfo('开始规划')
    control_Command = gluon_demo_msgs()


    while not rospy.is_shutdown():
        key = input()

        #
        # 一、运动到合适的位置，点位控制
        if key == 1:
            pose_goal1 = Pose()
            pose_goal1.position.x = target_pose.transform.translation.x
            pose_goal1.position.y = target_pose.transform.translation.y
            pose_goal1.position.z = target_pose.transform.translation.z
            pose_goal1.orientation.w = target_pose.transform.rotation.w
            pose_goal1.orientation.x = target_pose.transform.rotation.x
            pose_goal1.orientation.y = target_pose.transform.rotation.y
            pose_goal1.orientation.z = target_pose.transform.rotation.z
            
            move_group.set_start_state_to_current_state()# 设置当前位姿为规划起点
            move_group.set_pose_target(pose_goal1)# 设置目标位姿
            
            try:
                rospy.loginfo ('moveit轨迹规划中...')
                traj_moveit = move_group.plan()    # 尝试规划
                # 虚拟械臂
                #move_group.execute(traj_moveit)
                
                # 真实机械臂
                if traj_moveit.joint_trajectory.points:  # 规划出来了轨迹，是非空的list
                    control_Command.traj = traj_moveit.joint_trajectory
                    control_Command.mode = 2
                    pub_Command.publish( control_Command )  # 把轨迹发布给控制节点
                else:
                    rospy.loginfo ("规划超时")
                
            except:
                rospy.loginfo ('规划失败')
                continue
        
        #
        # 二、指定画画的区域
        elif key == 2:
            # 先运动到这里 target_pose
            waypoints = []
            # 平面运动最大工作区，一个长方型，可以抬笔1cm
            point1 = move_group.get_current_pose(end_effector_link).pose
            #waypoints.append(point1)        # 

            point5 = deepcopy(point1)
            point5.position.y = 0.17
            point5.position.z = 0.10
            waypoints.append(point5)        # 

            point2 = deepcopy(point1)
            point2.position.y = 0.17
            point2.position.z = 0.31
            waypoints.append(point2)        # 

            point4 = deepcopy(point1)
            point4.position.y = -0.17
            point4.position.z = 0.31
            waypoints.append(point4)        # 

            waypoints.append(point1)        # 回到一

            # 设置规划参数
            fraction = 0.0   #路径规划覆盖率
            maxtries = 100   #最大尝试规划次数
            attempts = 0     #已经尝试规划次数
            # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
            while fraction < 1.0 and attempts < maxtries:
                #规划路径 ，fraction返回1代表规划成功
                (traj_moveit, fraction) = move_group.compute_cartesian_path (
                                    waypoints,   # waypoint poses，路点列表
                                    0.01,        # eef_step，终端步进值，每隔0.01m计算一次逆解判断能否可达
                                    0.0,         # jump_threshold，跳跃阈值，设置为0代表不允许跳跃
                                    True)        # avoid_collisions，避障规划
                attempts += 1       # 尝试次数累加
            if fraction == 1.0:
                rospy.loginfo("Path computed successfully. Moving the arm.")
                # 虚拟机械臂
                #move_group.execute(traj_moveit)
                # 实际机械臂
                if traj_moveit.joint_trajectory.points:  # 规划出来了轨迹，是非空的list
                    control_Command.traj = traj_moveit.joint_trajectory
                    control_Command.mode = 2
                    for i in range(0,3):
                        pub_Command.publish( control_Command )  # 把轨迹发布给控制节点
                        rospy.sleep( control_Command.traj.points[-1].time_from_start.secs + 
                                    control_Command.traj.points[-1].time_from_start.nsecs*1e-9 + 0.5)
                    
                    rospy.loginfo ("正在执行末端轨迹运动...")
                else:
                    rospy.loginfo ("规划超时")
               
            # 如果路径规划失败，则打印失败信息
            else:
                rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  
            

        #
        # 画画轨迹，开始使用自己的规划方法
        elif key == 3:
            # 
            # 处理图像 生成轮廓
            img=cv2.imread('../data/5.jpeg', 0)
            ret, lap = cv2.threshold(img, 100, 255, cv2.THRESH_BINARY_INV)
            # 先左右翻转，然后上下翻转。写出来的字是正的
            lap = cv2.flip(lap, 0)
            lap = cv2.flip(lap, 1)
            lap, contours, hierarchy = cv2.findContours(lap, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE )
            # 缩放轨迹 得到在平面上的运动
            x = []
            y = []
            z = []
            if lap.shape[0]*2 >= lap.shape[1]: # 按照x缩小
                rate = 0.21/lap.shape[0]
            else:
                rate = 0.34/lap.shape[1]

            for i in contours:
                z += [ i[0,0,1]*rate ]    # 抬起  在当前轮廓上方
                x += [ -i[0,0,0]*rate ]
                y += [ -0.005]

                z += [ i[0,0,1]*rate ]
                x += [ -i[0,0,0]*rate ]
                y += [ -0.01]

                z += [ i[0,0,1]*rate ]
                x += [ -i[0,0,0]*rate ]
                y += [ -0.015]

                z += [ i[0,0,1]*rate ]
                x += [ -i[0,0,0]*rate ]
                y += [ -0.02]

    
                z += [ k[0,1]*rate for k in i ]  # 落下 画出轮廓
                x += [ -k[0,0]*rate for k in i ]
                y += [ 0.0 for k in i ]

                z += [ i[-1,0,1]*rate ]   # 抬起
                x += [ -i[-1,0,0]*rate ]
                y += [ -0.005 ]

                z += [ i[-1,0,1]*rate ]
                x += [ -i[-1,0,0]*rate ]
                y += [ -0.01 ]

                z += [ i[-1,0,1]*rate ]
                x += [ -i[-1,0,0]*rate ]
                y += [ -0.015 ]

                z += [ i[-1,0,1]*rate ]
                x += [ -i[-1,0,0]*rate ]
                y += [ -0.02 ]

            rospy.loginfo("图像轮廓已完成")

            # matplotlib 画出图像的轮廓
            fig = plt.figure()
            ax = Axes3D(fig)
            ax.scatter(x, y, z)
            plt.show()
            

            #
            # 在基坐标系下的 点 和 时间戳
            waypoints = []      # 末端的位置
            speed = 0.03        # 末端移动速度
            time_header = []    # 以固定的速度 遍历这些位置与前一个时刻的间隔

            p = Pose()
            p.orientation.w = 1.0
            for i in range(0,len(x)):
                p1 = deepcopy(p)            # 位置
                p1.position.x = x[i]
                p1.position.y = y[i]
                p1.position.z = z[i]
                waypoints.append(p1)

                Dur_time = Duration()       # 时间戳
                if i == 0:
                    Dur_time.data.secs = 0
                    Dur_time.data.nsecs = 0
                    time_header += [ Dur_time ]
                else:
                    tim = np.sqrt( (x[i]-x[i-1])**2 + (y[i]-y[i-1])**2 + (z[i]-z[i-1])**2 )/speed  # 两点之间的时间间隔
                    tim += time_header[i-1].data.secs + time_header[i-1].data.nsecs*1e-9
                    Dur_time.data.secs = int(tim)
                    Dur_time.data.nsecs = int( (tim-Dur_time.data.secs)*1e9 )
                    time_header += [ Dur_time ]
            
            # 
            # 逆运动学，得到关节轨迹
            # 得到关节轨迹 这里比较复杂，主要任务是加时间戳。这里生成的轨迹，是标准轨迹 JointTrajectory() 
            traj = JointTrajectory()
            traj.joint_names = ['axis_joint_1', 'axis_joint_2', 'axis_joint_3', 'axis_joint_4', 'axis_joint_5', 'axis_joint_6']
            wp = trans(waypoints, target_pose )    # 变换到画画区域坐标系，画画的灵活区域是指定的

            #pb = PointStamped() # 在 RVIZ 中显示轨迹点
            #pb.header.frame_id = "base_link"
            for i in range(0, len(wp)):
                # pb.header.stamp = rospy.Time.now()
                # pb.point.x = wp[i].position.x
                # pb.point.y = wp[i].position.y
                # pb.point.z = wp[i].position.z
                # pb.header.stamp = rospy.Time.now()
                # pub_point.publish(pb)
                # rospy.loginfo pb.point
                # rospy.sleep(0.05)

                try:
                    joint = inv_kinematic.inv_arm( wp[i] )[0]  # 返回的多组解，只要第一组
                    points_ = JointTrajectoryPoint() 
                    points_.positions = joint
                    points_.time_from_start = time_header[i].data
                    #points_.velocities = [speed for i in range(0,len(traj.joint_names))]
                    #points_.accelerations = [0.1 for i in range(0,len(traj.joint_names))]
                    traj.points.append(points_)
                except:
                    rospy.loginfo( inv_kinematic.inv_arm( wp[i] ))
                    pass
            rospy.loginfo("已经得到关节轨迹")


            # 
            # 直线运动到轨迹上方
            waypoints = []      # 末端的位置
            speed = 0.03        # 末端移动速度
            time_header = []    # 以固定的速度 遍历这些位置与前一个时刻的间隔

            p = Pose()
            p.orientation.w = 1.0
            for i in range(0,len(x)):
                p1 = deepcopy(p)            # 位置
                p1.position.x = x[i]
                p1.position.y = y[i]
                p1.position.z = z[i]
                waypoints.append(p1)

                Dur_time = Duration()       # 时间戳
                if i == 0:
                    Dur_time.data.secs = 0
                    Dur_time.data.nsecs = 0
                    time_header += [ Dur_time ]
                else:
                    tim = np.sqrt( (x[i]-x[i-1])**2 + (y[i]-y[i-1])**2 + (z[i]-z[i-1])**2 )/speed  # 两点之间的时间间隔
                    tim += time_header[i-1].data.secs + time_header[i-1].data.nsecs*1e-9
                    Dur_time.data.secs = int(tim)
                    Dur_time.data.nsecs = int( (tim-Dur_time.data.secs)*1e9 )
                    time_header += [ Dur_time ]
            # 
            # 逆运动学，得到关节轨迹
            # 得到关节轨迹 这里比较复杂，主要任务是加时间戳。这里生成的轨迹，是标准轨迹 JointTrajectory() 
            traj = JointTrajectory()
            traj.joint_names = ['axis_joint_1', 'axis_joint_2', 'axis_joint_3', 'axis_joint_4', 'axis_joint_5', 'axis_joint_6']
            wp = trans(waypoints, target_pose )    # 变换到画画区域坐标系，画画的灵活区域是指定的

            #pb = PointStamped() # 在 RVIZ 中显示轨迹点
            #pb.header.frame_id = "base_link"
            for i in range(0, len(wp)):
                # pb.header.stamp = rospy.Time.now()
                # pb.point.x = wp[i].position.x
                # pb.point.y = wp[i].position.y
                # pb.point.z = wp[i].position.z
                # pb.header.stamp = rospy.Time.now()
                # pub_point.publish(pb)
                # rospy.loginfo pb.point
                # rospy.sleep(0.05)

                try:
                    joint = inv_kinematic.inv_arm( wp[i] )[0]  # 返回的多组解，只要第一组
                    points_ = JointTrajectoryPoint() 
                    points_.positions = joint
                    points_.time_from_start = time_header[i].data
                    #points_.velocities = [speed for i in range(0,len(traj.joint_names))]
                    #points_.accelerations = [0.1 for i in range(0,len(traj.joint_names))]
                    traj.points.append(points_)
                except:
                    rospy.loginfo( inv_kinematic.inv_arm( wp[i] ))
                    pass
            rospy.loginfo("已经发送关节轨迹")

            
            

            #
            # moveit 运动到轨迹的上方
            '''
            move_group.set_start_state_to_current_state()# 设置当前位姿为规划起点
            point1 = deepcopy(wp[0])
            #point1.position.x -= 3
            #point1.position.z += 5
            move_group.set_pose_target(point1)# 设置目标位姿
            
            try:
                rospy.loginfo ('moveit轨迹规划中...')
                traj_moveit = move_group.plan()    # 尝试规划
                # 虚拟械臂
                #move_group.execute(traj_moveit)
                # 真实机械臂
                if traj_moveit.joint_trajectory.points:  # 规划出来了轨迹，是非空的list
                    control_Command.traj = traj_moveit.joint_trajectory
                    control_Command.mode = 2
                    pub_Command.publish( control_Command )  # 把轨迹发布给控制节点
                    rospy.sleep( control_Command.traj.points[-1].time_from_start.secs + 
                                    control_Command.traj.points[-1].time_from_start.nsecs*1e-9 + 2)
                else:
                    rospy.loginfo ("规划超时")
            except:
                rospy.loginfo ('规划失败')
                break
            '''

            #
            # moveit 运动到轨迹的上方 直线
            '''
            waypoints2 = []
            point1 = move_group.get_current_pose(end_effector_link).pose
            waypoints2.append(point1)
            point2 = deepcopy(point1)
            point2.position.x -= 5

            #point2 = deepcopy(wp[0])
            #point2.position.y -= 3
            waypoints2.append(point2)        # 

            # 设置规划参数
            fraction = 0.0   #路径规划覆盖率
            maxtries = 100   #最大尝试规划次数
            attempts = 0     #已经尝试规划次数
            # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
            while fraction < 1.0 and attempts < maxtries:
                (traj_moveit, fraction) = move_group.compute_cartesian_path ( waypoints2, 0.01, 0.0, True)  #规划路径 ，fraction返回1代表规划成功
                attempts += 1   # 尝试次数累加
            if fraction == 1.0:
                rospy.loginfo("Path computed successfully. Moving the arm.")
                # 虚拟机械臂
                #move_group.execute(traj_moveit)
                # 实际机械臂
                if traj_moveit.joint_trajectory.points:  # 规划出来了轨迹，是非空的list
                    control_Command.traj = traj_moveit.joint_trajectory
                    control_Command.mode = 2
                    for i in range(0,3):
                        pub_Command.publish( control_Command )  # 把轨迹发布给控制节点
                        rospy.sleep( control_Command.traj.points[-1].time_from_start.secs + 
                                    control_Command.traj.points[-1].time_from_start.nsecs*1e-9 + 0.5)
                    
                    rospy.loginfo ("正在执行末端轨迹运动...")
                else:
                    rospy.loginfo ("规划超时")
               
            # 如果路径规划失败，则打印失败信息
            else:
                rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  
            '''


            #
            # 开始写字轨迹
            # 仿真
            #traj_moveit = RobotTrajectory()
            #traj_moveit.joint_trajectory = traj
            #move_group.execute(traj_moveit)
            # 实际
            control_Command.traj = traj
            control_Command.mode = 2
            pub_Command.publish( control_Command )  # 把轨迹发布给控制节点
            rospy.sleep( control_Command.traj.points[-1].time_from_start.secs + 
                        control_Command.traj.points[-1].time_from_start.nsecs*1e-9 + 0.5)
            rospy.loginfo ("正在绘制轮廓...")
            
        #
        # 零、复位规划，（点位控制）规划出路径返回初始位姿
        elif key == 0:
            move_group.set_start_state_to_current_state()
            move_group.set_named_target('zero')    # 设置 group 的 target
            rospy.loginfo ('复位')
            try:
                rospy.loginfo ('moveit轨迹规划中...')
                traj_moveit = move_group.plan()    # 尝试规划
                if traj_moveit.joint_trajectory.points:  # 规划出来了轨迹，是非空的list
                    control_Command.traj = traj_moveit.joint_trajectory
                    control_Command.mode = 2

                    pub_Command.publish( control_Command )  # 把轨迹发布给控制节点
                    #move_group.execute(traj_moveit)
                else:
                    rospy.loginfo ("规划超时")
                
            except:
                rospy.loginfo ('规划失败')
                continue

        else:
            break
