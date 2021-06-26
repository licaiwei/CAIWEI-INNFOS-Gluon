#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy, sys
import tf2_ros

from geometry_msgs.msg import Pose, TransformStamped
from gluon_demo.msg import gluon_demo_msgs

import moveit_commander
from moveit_commander.conversions import pose_to_list   # moveit 相关
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene, RobotTrajectory, RobotState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Duration

import numpy as np
import Geometry
from copy import deepcopy

import cv2
import inv_kinematic

#
# (在末端坐标里移动的点的集合， 末端坐标系的初始位姿)
def trans(waypoints_, transf_ ):
    #transf_ = TransformStamped()
    #waypoints_[0] =  Pose() or TransformStamped()
    try:
        x = waypoints_.transform.translation.x
        y = waypoints_.transform.translation.y
        z = waypoints_.transform.translation.z
        qx = waypoints_.transform.rotation.x
        qy = waypoints_.transform.rotation.y
        qz = waypoints_.transform.rotation.z
        qw = waypoints_.transform.rotation.w
    except:
        x = waypoints_.position.x
        y = waypoints_.position.y
        z = waypoints_.position.z
        qx = waypoints_.orientation.x
        qy = waypoints_.orientation.y
        qz = waypoints_.orientation.z
        qw = waypoints_.orientation.w

    pose_temp = Pose()
    p = np.array([[x],
                [y],
                [z]])
    q = transf_.transform.rotation
    t = transf_.transform.translation
    r = Geometry.Quaternion_2_matrix( [q.x, q.y, q.z, q.w] )
    p = r.dot(p)

    pose_temp.position.x = p[0,0]+t.x
    pose_temp.position.y = p[1,0]+t.y
    pose_temp.position.z = p[2,0]+t.z
    pose_temp.orientation = q

    return pose_temp


# 定义了一个全局变量 target_pose
target_pose = TransformStamped()  
target_pose.transform.translation.x = 0.30
target_pose.transform.translation.y = -0.52
target_pose.transform.translation.z = 0.92
target_pose.transform.rotation.x = 0
target_pose.transform.rotation.y = -0.479425538604
target_pose.transform.rotation.z = 0
target_pose.transform.rotation.w = 0.87758256189

#
# 读取文件夹下的图片，生成轨迹后执行
if __name__ == '__main__':
    #
    # 初始化 move_group
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('draw_pic_node', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    rospy.loginfo('11111111111111， moveit scene 初始化完成')

    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_group_real = move_group
    move_group.set_goal_position_tolerance(0.01)
    move_group.set_goal_orientation_tolerance(0.01)
    move_group.set_planning_time(5.0)
    move_group.allow_replanning(True)
    rospy.sleep(1) 
    end_effector_link = move_group.get_end_effector_link()
    rospy.loginfo('222222222222222, move_group 初始化完成')

    #
    # 开始规划
    tfBuffer = tf2_ros.Buffer() # tf2接收器，接受目标位姿
    listener = tf2_ros.TransformListener(tfBuffer)
    pub_Command =  rospy.Publisher("/manipulator/command", gluon_demo_msgs, queue_size=1)  # 设置控制指令发布 topic句柄
    rospy.sleep(0.5)
    rospy.loginfo('3333333333333，开始规划')
    control_Command = gluon_demo_msgs()


    while not rospy.is_shutdown():
        key = input()

        #
        # 一、运动到合适的位置，点位控制
        if key == 1:
            pose_goal1 = Pose()
            pose_goal1.position.x = target_pose.transform.translation.x-0.01
            pose_goal1.position.y = target_pose.transform.translation.y
            pose_goal1.position.z = target_pose.transform.translation.z
            pose_goal1.orientation.w = target_pose.transform.rotation.w
            pose_goal1.orientation.x = target_pose.transform.rotation.x
            pose_goal1.orientation.y = target_pose.transform.rotation.y
            pose_goal1.orientation.z = target_pose.transform.rotation.z
            
            move_group.set_start_state_to_current_state()# 设置当前位姿为规划起点
            move_group.set_pose_target(pose_goal1)# 设置目标位姿
            
            try:
                rospy.loginfo('moveit轨迹规划中...')
                traj_moveit = move_group.plan()    # 尝试规划
                # 虚拟械臂
                #move_group.execute(traj_moveit)
                
                # 真实机械臂
                if traj_moveit.joint_trajectory.points:  # 规划出来了轨迹，是非空的list
                    control_Command.traj = traj_moveit.joint_trajectory
                    control_Command.mode = 1
                    pub_Command.publish( control_Command )  # 把轨迹发布给控制节点
                else:
                    rospy.loginfo("规划超时")
                
            except:
                rospy.loginfo('规划失败')
                continue
            
            # TODO
            # 判断一下是不是指定的位姿

        #
        # 二、视觉伺服
        elif key == 2:
            while True:
                try:
                    d_pose = tfBuffer.lookup_transform("wristPitch_Link", "chess", rospy.Time())  # 误差姿态
                    trans_pose = tfBuffer.lookup_transform("base_link", "wristPitch_Link", rospy.Time())
                    d = trans( d_pose, trans_pose )

                    dpose = np.array([[ d.position.x-trans_pose.transform.translation.x,
                                        d.position.y-trans_pose.transform.translation.y,
                                        d.position.z-trans_pose.transform.translation.z, 0., 0., 0. ]]).T  # 末端误差旋量
                    rospy.loginfo(dpose)
                    # 求 jacobian 的逆
                    joint_values_list = move_group.get_current_joint_values()           # 当前角度值，类型 list 或是订阅 /joint_states，这两个的数据是相同的
                    jacobian_list = move_group.get_jacobian_matrix(joint_values_list)   # list
                    jacobian_matrix = np.array(jacobian_list)                           # 转换成 array
                    inv_jacobian_matrix = np.linalg.pinv(jacobian_matrix)               # 求广义逆

                    # 求关节误差
                    dth = inv_jacobian_matrix.dot(dpose)
                    if dth.max() > 0.2 or dth.min() < -0.2: # 当角度大于 0.2，防止计算错误导致机械臂震荡
                        break
                    if dth.max() < 0.005 and dth.min() > -0.005: # 结束条件
                        rospy.INF("调整结束")
                        break

                    # 求要移动的关节角度
                    control_Command.joint.name = ['shoulderPitch', 'shoulderRoll', 'shoulderYaw', 'elbowRoll', 'wristYaw', 'wristPitch']
                    control_Command.joint.position = []
                    control_Command.joint.velocity = []
                    for i in range(0, len(control_Command.joint.name)):
                        control_Command.joint.position.append( joint_values_list[i] + dth[i][0] )
                        control_Command.joint.velocity.append( dth[i][0]/1.0)
                    control_Command.mode = 0

                    pub_Command.publish(control_Command)
                    rospy.sleep(0.8)

                except:
                    continue

        #
        # 零、复位规划，（点位控制）规划出路径返回初始位姿
        elif key == 0:
            move_group.set_start_state_to_current_state()
            move_group.set_named_target('pose1')    # 设置 group 的 target
            rospy.loginfo('复位')
            try:
                rospy.loginfo('moveit轨迹规划中...')
                traj_moveit = move_group.plan()    # 尝试规划

                #move_group.execute(traj_moveit)

                if traj_moveit.joint_trajectory.points:  # 规划出来了轨迹，是非空的list
                    control_Command.traj = traj_moveit.joint_trajectory
                    control_Command.mode = 1
                    pub_Command.publish( control_Command )  # 把轨迹发布给控制节点
                else:
                    rospy.loginfo("规划超时")
            except:
                rospy.loginfo('规划失败')
                continue

        else:
            break