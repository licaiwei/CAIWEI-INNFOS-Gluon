#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
THIS PROGRAM IS FREE SOFTWARE, IS LICENSED MIT
inv_kinematic.py - Inverse kinematic algorithm for Gluon

@Created on APR 6 2021
@author:菜伟
@email:li_wei_96@163.com
"""

import rospy
import sys
import tf2_ros
import numpy as np
from numpy import sin, cos, pi, arctan2, arccos, arcsin, sqrt
from geometry_msgs.msg import Pose, TransformStamped
import Geometry
import time
from sensor_msgs.msg import JointState

pi_2 = 1.570796325

def inv_arm(T):
    """
    UR3/5/10, gluon通用的求解方法
    @Parameters
    T: geometry_msgs.msg.Pose() or TransformStamped()
    @Returns
    solve: [[th1, th2, th3, th4, th5, th6],[] ] 该算法只求解2组解（去掉了左右手，上下手）
    """
    a = []
    try:
        x = T.transform.translation.x
        y = T.transform.translation.y
        z = T.transform.translation.z
        qx = T.transform.rotation.x
        qy = T.transform.rotation.y
        qz = T.transform.rotation.z
        qw = T.transform.rotation.w
    except:
        x = T.position.x
        y = T.position.y
        z = T.position.z
        qx = T.orientation.x
        qy = T.orientation.y
        qz = T.orientation.z
        qw = T.orientation.w

    r = Geometry.Quaternion_2_matrix([qx, qy, qz, qw])
    solve = []

    # th1
    a1 = x**2+y**2-0.005728325
    if -0.0001 < a1 < 0.0001:
        th1 = - arctan2(y, x)
    else:
        th1 = arctan2(sqrt(a1), 0.0756857) - arctan2(y, x)		# 1解 机械臂没有“左右手”

    # th5
    s1 = sin(th1)
    c1 = cos(th1)
    th5 = arccos(c1*r[0, 0]-s1*r[1, 0])    # 2解 正负
    th5_ = -th5

    for i in range(0, 4):
        if i > 1:
            th5 = th5_
        # th6
        s5 = sin(th5)
        c5 = cos(th5)
        px = r[2, 2]*s5
        py = r[2, 1]*s5
        d = r[2, 0]*c5

        th6 = -arctan2(d, sqrt(px**2+py**2-d**2)) - \
                    arctan2(py, px)   # TODO,在这里，只能解2象限，但是他有4象限
        if i == 1 or i == 3:
            th6 = pi-th6
        if th6 > 3.14:
            th6 = th6-2*pi
        elif th6 < -3.14:
            th6 = th6+2*pi

        # th3
        c6 = cos(th6)
        s6 = sin(th6)
        D = (x-0.08*(r[0, 2]*c6-r[0, 1]*s6))**2 + (y-0.08*(r[1, 2]*c6-r[1, 1]*s6))**2 + \
                    (z-0.08*(r[2, 2]*c6-r[2, 1]*s6)-0.105)**2 - \
                    0.06656121  # - 0.00573049 -0.06083072
        if 0.06083072 < D or D < -0.06083072:
            continue
        else:
            th3 = arccos(-D/0.06083072) - pi  # 1解 机械臂没有“上下手”

        # th2
        th_tmpe = arccos((z-0.08*(r[2, 2]*c6-r[2, 1]*s6)-0.105)/sqrt(D+0.06083072))
        th2 = th_tmpe+th3/2

        # th4
        th_tmpe2 = arctan2((s1*(r[0, 2]*c6-r[0, 1]*s6)+c1 *
                        (r[1, 2]*c6-r[1, 1]*s6)), (r[2, 2]*c6-r[2, 1]*s6))
        th4 = th_tmpe2-th2+th3
        if th4 > 3.14:
            th4 = th4-2*pi
        elif th4 < -3.14:
            th4 = th4+2*pi

        # 验证一下
        c234 = cos(th_tmpe2)  # 正运动学
        dist = (c1*c5-s1*c234*s5-r[0, 0])**2 + (s1*c5+c1 *
                                            c234*s5+r[1, 0])**2 + (sin(th_tmpe2)-r[2, 0])**2
        if dist > 1e-5:
            continue
        if -2.79 < th1 < 2.79 and 0 < th2 < 1.54 and -2.79 < th4 < 2.79 and -2.79 < th5 < 2.79 and -2.79 < th6 < 2.79:     # 关节限位 3 6无限位
            solve += [[th1, th2, th3, th4, th5, th6]]

    return solve


if __name__ == '__main__':
    """
    测试逆解算法，
    1.roslaunch gluon_description display.launch
    2.rosrun gluon_demo keyboard_3D_tf.py
    3.rosrun gluon_demo inv_kinematic.py
    在2号终端里移动 target link 来测试逆解算法的速度
    """
    rospy.init_node('inv_joint_state_publisher_controller', anonymous=False)

    pub_joint_states = rospy.Publisher('/move_group/fake_controller_joint_states', JointState, queue_size=10)  # 设置控制joint_state_publisher的topic

    control_ = JointState()
    control_.name = ['axis_joint_1', 'axis_joint_2', 'axis_joint_3', 'axis_joint_4', 'axis_joint_5', 'axis_joint_6']
    control_.effort = [0., 0., 0., 0., 0., 0.]
    control_.velocity = [0., 0., 0., 0., 0., 0.]

    tfBuffer = tf2_ros.Buffer()  # tf2接收器，接受目标位姿
    listener = tf2_ros.TransformListener(tfBuffer)
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        b_time = time.time()

        target_pose = tfBuffer.lookup_transform("base_link", "target",  rospy.Time())
        a = inv_arm(target_pose)
        e_time = time.time()
        #print a
        rospy.loginfo( "%sms", ( e_time-b_time)/1000)

        if a == []:
            rospy.loginfo("No sulution")
        else:
            for i in a:
                control_.position = i
                pub_joint_states.publish(control_)
                rospy.sleep(0.5)
