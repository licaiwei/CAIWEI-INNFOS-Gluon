#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy, sys
import tf2_ros

from geometry_msgs.msg import Pose
from gluon_demo.msg import gluon_demo_msgs

import moveit_commander
from moveit_commander.conversions import pose_to_list   # moveit 相关
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene, RobotTrajectory, RobotState

import numpy as np
import Geometry
from copy import deepcopy

#
# 测试重复定位精度节点。两种模式：一、往返式  二、画格点式
if __name__ == "__main__":
    #
    # 初始化 move_group
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_gourp_six_arm_hand', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    print('11111111111111， moveit scene 初始化完成')

    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_group_real = move_group
    move_group.set_goal_position_tolerance(0.01)
    move_group.set_goal_orientation_tolerance(0.01)
    move_group.set_planning_time(5.0)
    move_group.allow_replanning(True)
    rospy.sleep(1) 
    end_effector_link = move_group.get_end_effector_link()
    print('222222222222222, move_group 初始化完成')

    #
    # 开始规划
    tfBuffer = tf2_ros.Buffer() # tf2接收器，接受目标位姿
    listener = tf2_ros.TransformListener(tfBuffer)
    pub_Command =  rospy.Publisher("/manipulator/command", gluon_demo_msgs, queue_size=1)  # 设置控制指令发布 topic句柄
    rospy.sleep(0.5)
    print('3333333333333，开始规划')
    control_Command = gluon_demo_msgs()

    while not rospy.is_shutdown():
        key = input("测试模式：\n 1：往返  2：画格点\n")

        #
        # 一、确定重复定位精度的两个往返点   抬起点、下落点
        if key == 1:
            print "确定两个往返点。抬起点、下落点"
            pose_goal1 = Pose()
            pose_goal1.position.x = 0.46
            pose_goal1.position.y = -0.39
            pose_goal1.position.z = 1.21
            pose_goal1.orientation.x = -0.00667987
            pose_goal1.orientation.y = -0.70106
            pose_goal1.orientation.z = 0.111819
            pose_goal1.orientation.w = 0.704249
           
            pose_goal2 = Pose()
            pose_goal2.position.x = 0.32
            pose_goal2.position.y = -0.24
            pose_goal2.position.z = 0.96
            pose_goal2.orientation.x = 0.331808
            pose_goal2.orientation.y = -0.597991
            pose_goal2.orientation.z = 0.326632
            pose_goal2.orientation.w = 0.652397

            # 设置当前位姿为规划起点，运动到 抬起点
            move_group.set_start_state_to_current_state()
            move_group.set_pose_target(pose_goal1)  # 设置抬起点
            try:
                print 'moveit轨迹规划中...'
                traj_moveit = move_group.plan()    # 尝试规划
                #move_group.execute(traj_moveit)

                if traj_moveit.joint_trajectory.points:  # 规划出来了轨迹，是非空的list
                    control_Command.traj = traj_moveit.joint_trajectory
                    control_Command.mode = 1
                    pub_Command.publish( control_Command )  # 把轨迹发布给控制节点
                    rospy.sleep( control_Command.traj.points[-1].time_from_start.secs + 
                                control_Command.traj.points[-1].time_from_start.nsecs*1e-9 + 2)
                else:
                    print "规划超时"
            except:
                print '规划失败'
                continue
            

            # 运动到 下降点。并重复
            move_group.set_start_state_to_current_state()
            move_group.set_pose_target(pose_goal2)  # 设置下落点
            try:
                print 'moveit轨迹规划中...'
                traj_moveit = move_group.plan()     # 放下轨迹

                if traj_moveit.joint_trajectory.points:
                    control_Command.mode = 1
                    traj_moveit2 = deepcopy( traj_moveit ) # traj_moveit 的反向轨迹
                    traj_moveit2.joint_trajectory.points[-1].positions = traj_moveit2.joint_trajectory.points[0].positions
                    # 放下
                    control_Command.traj = traj_moveit.joint_trajectory
                    pub_Command.publish( control_Command )  # 把轨迹发布给控制节点
                    rospy.sleep( control_Command.traj.points[-1].time_from_start.secs + 
                                    control_Command.traj.points[-1].time_from_start.nsecs*1e-9 + 0.5)
                    # 重复
                    key2 = input("设置好测试工具后，设置测量次数:")
                    print "开始测量"
                    for i in range(0, key2):
                        control_Command.traj = traj_moveit2.joint_trajectory
                        pub_Command.publish( control_Command )  # 把轨迹发布给控制节点
                        rospy.sleep( control_Command.traj.points[-1].time_from_start.secs + 
                                        control_Command.traj.points[-1].time_from_start.nsecs*1e-9 + 0.5)
                        control_Command.traj = traj_moveit.joint_trajectory
                        pub_Command.publish( control_Command )  # 把轨迹发布给控制节点
                        rospy.sleep( control_Command.traj.points[-1].time_from_start.secs + 
                                        control_Command.traj.points[-1].time_from_start.nsecs*1e-9 + 0.5)
                    # 抬起
                    control_Command.traj = traj_moveit2.joint_trajectory
                    pub_Command.publish( control_Command )  # 把轨迹发布给控制节点
                else:
                    print "规划超时"
            except:
                print '规划失败'
                continue

        #
        # 二、琴键精度演示
        elif key == 2:
            move_group.set_start_state_to_current_state()
            # 设置目标位姿，先点位控制运动到一个合适的位姿
            pose_goal2 = Pose()
            pose_goal2.position.x = 0.32
            pose_goal2.position.y = -0.24
            pose_goal2.position.z = 0.96
            pose_goal2.orientation.x = 0.331808
            pose_goal2.orientation.y = -0.597991
            pose_goal2.orientation.z = 0.326632
            pose_goal2.orientation.w = 0.652397
            move_group.set_pose_target(pose_goal2)
            
            try:
                print 'moveit轨迹规划中...'
                traj_moveit = move_group.plan()    # 尝试规划
                # 实际机械臂
                if traj_moveit.joint_trajectory.points:  # 规划出来了轨迹，是非空的list
                    control_Command.traj = traj_moveit.joint_trajectory
                    control_Command.mode = 2
                    pub_Command.publish( control_Command )  # 把轨迹发布给控制节点
                    print "正在运动到初始位姿..."
                    rospy.sleep(5) # 这个执行不是阻塞的，所以用延时指令
                else:
                    print "规划超时"
                
            except:
                    print '规划失败'
                    continue
            
            # 下面设置笛卡尔直线轨迹的路点  waypoints
            waypoints = []
            point1 = move_group.get_current_pose(end_effector_link).pose
            waypoints.append(point1)        # 第一个 中心点
            
            point2 = deepcopy(point1)
            point2.position.y += -0.115
            waypoints.append(point2)        # 第二个  
            
            point3 = deepcopy(point1)
            point3.position.y += 0.092
            waypoints.append(point3)        # 第三个 
            waypoints.append(point2)
            
            point4 = deepcopy(point1)
            point4.position.y += 0.069
            waypoints.append(point4)        # 第4个 
            waypoints.append(point2)

            point5 = deepcopy(point1)
            point5.position.y += 0.046
            waypoints.append(point5)        # 第5个 
            waypoints.append(point2)

            point6 = deepcopy(point1)
            point6.position.y += 0.023
            waypoints.append(point6)        # 第6个 
            waypoints.append(point2)

            point7 = deepcopy(point1)
            waypoints.append(point7)        # 第7个 
            waypoints.append(point2)

            point8 = deepcopy(point1)
            point8.position.y += -0.023
            waypoints.append(point8)        # 第8个 
            waypoints.append(point2)       

            point9 = deepcopy(point1)
            point9.position.y += -0.046
            waypoints.append(point9)        # 第9个 
            waypoints.append(point2)

            point10 = deepcopy(point1)
            point10.position.y += -0.069
            waypoints.append(point10)        # 第10个 
            waypoints.append(point2)

            point11 = deepcopy(point1)
            point11.position.y += -0.092
            waypoints.append(point11)        # 第11个 
            waypoints.append(point2)

            waypoints.append(point3)
            waypoints.append(point11)
            waypoints.append(point3)
            waypoints.append(point10)
            waypoints.append(point3)
            waypoints.append(point9)
            waypoints.append(point3)
            waypoints.append(point8)
            waypoints.append(point3)
            waypoints.append(point1)
            waypoints.append(point3)
            waypoints.append(point6)
            waypoints.append(point3)
            waypoints.append(point5)
            waypoints.append(point3)
            waypoints.append(point4)
            waypoints.append(point3)

            # 设置规划参数
            fraction = 0.0   #路径规划覆盖率
            maxtries = 100   #最大尝试规划次数
            attempts = 0     #已经尝试规划次数
            
            # 设置机器臂当前的状态作为运动初始状态
            #move_group.set_start_state_to_current_state()

            # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
            while fraction < 1.0 and attempts < maxtries:
                #规划路径 ，fraction返回1代表规划成功
                (traj_moveit, fraction) = move_group.compute_cartesian_path (
                                    waypoints,   # waypoint poses，路点列表
                                    0.01,        # eef_step，终端步进值，每隔0.01m计算一次逆解判断能否可达
                                    0.0,         # jump_threshold，跳跃阈值，设置为0代表不允许跳跃
                                    True)        # avoid_collisions，避障规划
                attempts += 1       # 尝试次数累加
                #if attempts % 10 == 0:   # 打印运动规划进程
                #    rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                        
            # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
            if fraction == 1.0:
                rospy.loginfo("Path computed successfully. Moving the arm.")
                #move_group.execute(traj_moveit)
                # 实际机械臂
                if traj_moveit.joint_trajectory.points:  # 规划出来了轨迹，是非空的list
                    control_Command.traj = traj_moveit.joint_trajectory
                    control_Command.mode = 2
                    
                    #for i in range(0, len(traj_moveit.joint_trajectory.points)):
                    #    print traj_moveit.joint_trajectory.points[i].positions[0]   # 打印出来是空的  TODO
                    #for i in range(0, len(traj_moveit.joint_trajectory.points)):
                    #    print traj_moveit.joint_trajectory.points[i].time_from_start   # 打印出来是空的  TODO

                    pub_Command.publish( control_Command )  # 把轨迹发布给控制节点
                    rospy.sleep( control_Command.traj.points[-1].time_from_start.secs + 
                                control_Command.traj.points[-1].time_from_start.nsecs*1e-9 + 0.5)
                    print "正在运动到初始位姿..."
                else:
                    print "规划超时"

            # 如果路径规划失败，则打印失败信息
            else:
                rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

        #
        # 零、复位规划，（点位控制）规划出路径返回初始位姿
        elif key == 0:
            move_group.set_start_state_to_current_state()
            move_group.set_named_target('pose1')    # 设置 group 的 target
            print '复位'
            try:
                print 'moveit轨迹规划中...'
                traj_moveit = move_group.plan()    # 尝试规划

                #move_group.execute(traj_moveit)
                
                if traj_moveit.joint_trajectory.points:  # 规划出来了轨迹，是非空的list
                    control_Command.traj = traj_moveit.joint_trajectory
                    control_Command.mode = 1
                    pub_Command.publish( control_Command )  # 把轨迹发布给控制节点
                else:
                    print "规划超时"
                
            except:
                print '规划失败'
                continue

        else:
            break