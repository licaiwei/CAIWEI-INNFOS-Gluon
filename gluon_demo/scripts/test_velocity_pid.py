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
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Duration
import time

M_PI = 3.14159265

#
# 规划节点，用来控制机械臂执行，产生轨迹，设置机械臂模式
if __name__ == "__main__":
    #
    rospy.init_node('move_gourp_six_arm_hand', anonymous=True)
    print('11111111111111')

    control_Command = gluon_demo_msgs()
    control_Command.mode = 3
    pub_Command =  rospy.Publisher("/manipulator/command", gluon_demo_msgs, queue_size=1)  # 设置控制指令发布 topic句柄

    Dur_time = Duration()       # 时间戳

    traj = JointTrajectory()
    traj.joint_names = ['shoulderPitch', 'shoulderRoll', 'shoulderYaw', 'elbowRoll', 'wristYaw', 'wristPitch']

    time_header = []
    for i in range (0, 600):    # 6s的轨迹，间隔0.01s

        # 时间戳
        Dur_time = Duration()       # 时间戳
        if i == 0:
            Dur_time.data.secs = 0
            Dur_time.data.nsecs = 0
            time_header += [ Dur_time ]
        else:
            tim = 0.01  # 两点之间的时间间隔
            tim += time_header[i-1].data.secs + time_header[i-1].data.nsecs*1e-9
            #print tim
            Dur_time.data.secs = int(tim)
            Dur_time.data.nsecs = int( (tim-Dur_time.data.secs)*1e9 )
            time_header += [ Dur_time ]

        # 位置
        points_ = JointTrajectoryPoint() 
        points_.positions = [0., 0., 0., 0., 0., 0.]
        points_.positions[0] = 3.0 * (1.0 - np.cos( M_PI * i/100.0))   # 在这里改关节 ?  < 是的 >
        points_.velocities = [0., 0., 0., 0., 0., 0.]
        points_.accelerations = [0., 0., 0., 0., 0., 0.]
        points_.effort = [0., 0., 0., 0., 0., 0.]
        #print(type(time_header[i]))
        points_.time_from_start = time_header[i].data
        #points_.time_from_start = Duration(0.1)

        traj.points.append(points_)
 

    control_Command.traj = traj
    #print control_Command.traj
    pub_Command.publish( control_Command )  # 把轨迹发布给控制节点
    # time.sleep(3)
