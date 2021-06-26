#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
THIS PROGRAM IS FREE SOFTWARE, IS LICENSED MIT
joint_state_publiser_controller.py - Publishing joint state of the real manipulator

@Created on APR 2 2021
@author:菜伟
@email:li_wei_96@163.com
"""

import rospy
import sys
from sensor_msgs.msg import JointState

rate = 36.0           # 电机的转速比
pi = 3.14159665
rate_position = rate / 2.0 / pi
rate_veloity = rate_position * 60.0

rate_position_inv = 1.0/rate_position
rate_veloity_inv = 1.0/rate_veloity

control_ = JointState()     # 发布控制 joint_state_publisher 的消息
Joint_Name_ID = {}          # 字典{"Actuator id"："joint_name",}  将yaml定义从参数服务器读取出来
# 设置控制joint_state_publisher的topic
pub_joint_states = rospy.Publisher('/joint_states', JointState, queue_size=10)


def callback(actuator_states):
    control_.name = []
    for i in actuator_states.name:
        control_.name.append(Joint_Name_ID[i])
    control_.position = [x*rate_position_inv for x in actuator_states.position]
    control_.velocity = [x*rate_position_inv for x in actuator_states.velocity]
    control_.header.stamp = rospy.Time.now()
    pub_joint_states.publish(control_)


# 节点的作用是订阅电机的状态 /INNFOS/actuator_states 主题，用这个状态控制 joint_state_publisher
if __name__ == '__main__':
    rospy.init_node('joint_state_publisher_controller', anonymous=False)
    rospy.Subscriber("/INNFOS/actuator_states", JointState, callback)

    #
    # 加载YAML文建定义的 关节id-名称，得到字典
    param_list = rospy.get_param_names()

    for param in param_list:
        if "/joint_id_list/" in param:
            Joint_Name_ID["Actuator" + str(rospy.get_param(param))] = param.replace("/joint_id_list/", "")

    if len(Joint_Name_ID) == 0:
        rospy.loginfo("没有加载 yaml 配置文件")
        exit(1)

    rospy.spin()
