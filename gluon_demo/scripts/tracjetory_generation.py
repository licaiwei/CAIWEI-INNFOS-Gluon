#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy, sys
import tf2_ros

from geometry_msgs.msg import Pose
from gluon_demo.msg import gluon_demo_msgs

import moveit_commander
from moveit_commander.conversions import pose_to_list   # moveit 相关
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene, RobotTrajectory, RobotState, DisplayTrajectory

import numpy as np
import Geometry
from copy import deepcopy

#
# 规划节点，用来控制机械臂执行，产生轨迹，设置机械臂模式
if __name__ == "__main__":
    #
    # 初始化 move_group
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_gourp_gluon', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    rospy.loginfo('moveit scene 初始化完成')

    group_name = "gluon"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_group_real = move_group
    move_group.set_goal_position_tolerance(0.001)
    move_group.set_goal_orientation_tolerance(0.001)
    move_group.set_planning_time(5.0)
    move_group.allow_replanning(True)
    rospy.sleep(1)
    end_effector_link = move_group.get_end_effector_link()
    rospy.loginfo('move_group 初始化完成')

    #
    # 开始规划
    tfBuffer = tf2_ros.Buffer() # tf2接收器，接受目标位姿
    listener = tf2_ros.TransformListener(tfBuffer)
    pub_Command =  rospy.Publisher("/manipulator/command", gluon_demo_msgs, queue_size=1)  # 设置控制指令发布 topic句柄
    rospy.sleep(0.5)
    rospy.loginfo('开始规划')
    control_Command = gluon_demo_msgs()

    while not rospy.is_shutdown():
        key = input()

        #
        # 一、跟踪tf，然后点位规划
        if key == 1:
            try:
                # tf 目标位姿，类型 TransformStamped。 tf2订阅有可能出错，所以用try
                target_pose = tfBuffer.lookup_transform("base_link", "target", rospy.Time()) 

                # 把tf订阅的目标位姿 转换成 Pose() 类型，才能给moveit
                pose_goal1 = Pose()
                pose_goal1.position.x = target_pose.transform.translation.x
                pose_goal1.position.y = target_pose.transform.translation.y
                pose_goal1.position.z = target_pose.transform.translation.z
                pose_goal1.orientation.w = target_pose.transform.rotation.w
                pose_goal1.orientation.x = target_pose.transform.rotation.x
                pose_goal1.orientation.y = target_pose.transform.rotation.y
                pose_goal1.orientation.z = target_pose.transform.rotation.z
                
                # 设置当前位姿为规划起点
                move_group.set_start_state_to_current_state()
                # 设置目标位姿
                move_group.set_pose_target(pose_goal1)
                
                try:
                    rospy.loginfo('moveit轨迹规划中...')
                    traj_moveit = move_group.plan()    # 尝试规划
                    if traj_moveit.joint_trajectory.points:  # 规划出来了轨迹，是非空的list
                        control_Command.traj = traj_moveit.joint_trajectory
                        control_Command.mode = 2

                        #move_group.execute(traj_moveit)
                        pub_Command.publish( control_Command )  # 把轨迹发布给控制节点
                    else:
                        rospy.loginfo("规划超时")
                    
                except:
                    rospy.loginfo('规划失败')
                    continue
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue


        #
        # 二、指定的轨迹控制   用到 movevit 的 waypoint
        elif key == 2:
            # 设置当前位姿为规划起点
            move_group.set_start_state_to_current_state()            
            # 下面设置笛卡尔直线轨迹的路点  waypoints
            waypoints = []
            point1 = move_group.get_current_pose(end_effector_link).pose
            waypoints.append(point1)        # 第一个 中心点
            
            point2 = deepcopy(point1)
            point2.position.y += -0.17
            waypoints.append(point2)        # 第二个  中心点 左17cm
            waypoints.append(point1)        # 回到中点
            
            point3 = deepcopy(point1)
            point3.position.y += 0.17
            waypoints.append(point3)        # 第三个 中心点 右17cm
            waypoints.append(point1)        # 回到中点
            
            point4 = deepcopy(point1)
            point4.position.z += 0.17
            waypoints.append(point4)        # 第四个 中心点 推10cm。前推点
            waypoints.append(point1)        # 回到中点
            
            point5 = deepcopy(point1)
            point5.position.z += -0.10
            waypoints.append(point5)        # 第五个 前推点 左17cm
            waypoints.append(point1)        # 回到 前推点
            # 其他的方向上，规划不出来
            

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
                    
                    pub_Command.publish( control_Command )  # 把轨迹发布给控制节点
                    rospy.sleep( control_Command.traj.points[-1].time_from_start.secs + 
                                control_Command.traj.points[-1].time_from_start.nsecs*1e-9 + 0.5)
                    rospy.loginfo("正在执行末端轨迹运动...")
                else:
                    rospy.loginfo("规划超时")
                                
            # 如果路径规划失败，则打印失败信息
            else:
                rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  
            

        #
        # 零、复位规划，（点位控制）规划出路径返回初始位姿
        elif key == 0:
            move_group.set_start_state_to_current_state()
            move_group.set_named_target('zero')    # 设置 group 的 target
            rospy.loginfo('复位')
            try:
                rospy.loginfo( 'moveit轨迹规划中...')
                traj_moveit = move_group.plan()    # 尝试规划
                if traj_moveit.joint_trajectory.points:  # 规划出来了轨迹，是非空的list
                    control_Command.traj = traj_moveit.joint_trajectory
                    control_Command.mode = 2

                    pub_Command.publish( control_Command )  # 把轨迹发布给控制节点
                    #move_group.execute(traj_moveit)
                else:
                    rospy.loginfo( "规划超时")
                
            except:
                rospy.loginfo( '规划失败')
                continue

        else:
            break