#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Jan  9 19:10:45 2020
@author: Liwei
@E-mail: li_wei_96@163.com
该节点用于产生一个3D的tf
"""

import rospy
import tf2_ros
import geometry_msgs.msg

import sys, select, tty, termios
from std_msgs.msg import String

import numpy as np 
from numpy import sin, cos, pi, arctan, arccos, sqrt

def Matrix_2_quaternion(m): # 3*3的np.array矩阵 转换成 四元数
  qw_ = sqrt(np.trace(m)+1) / 2
  qx_ = -(m[1,2]-m[2,1]) / 4 / qw_
  qy_ = -(m[2,0]-m[0,2]) / 4 / qw_
  qz_ = -(m[0,1]-m[1,0]) / 4 / qw_
  Q_normed_ = [qx_, qy_, qz_, qw_]
  return Q_normed_

def Euler_2_matrix(thx, thy, thz):
  Rx = np.array([[ 1.,       0.,        0. ],
                 [ 0., cos(thx), -sin(thx) ],
                 [ 0., sin(thx),  cos(thx) ]])
  Ry = np.array([[ cos(thy),  0., sin(thy) ],
                 [       0.,  1.,      0.  ],
                 [-sin(thy),  0., cos(thy) ]])
  Rz = np.array([[ cos(thz), -sin(thz), 0. ],
                 [ sin(thz),  cos(thz), 0. ],
                 [       0.,        0., 1. ]])
  m = Rx.dot(Ry.dot(Rz))
  return m

def set_transform(x, y, z, m, t):  # 给变换t设置平移和旋转
  t.transform.translation.x = x
  t.transform.translation.y = y
  t.transform.translation.z = z
  q = Matrix_2_quaternion(m)
  t.transform.rotation.x = q[0]
  t.transform.rotation.y = q[1]
  t.transform.rotation.z = q[2]
  t.transform.rotation.w = q[3]

if __name__ == "__main__":  
  rospy.init_node('key_3D_tf')

  #
  # tf2发送器
  br = tf2_ros.TransformBroadcaster()  
  t = geometry_msgs.msg.TransformStamped()
  t.header.frame_id = "base_link"#"wristPitch_Link" 
  t.child_frame_id = "target"
  
  #
  # tf初始位姿
  x, y, z = 0.25, 0.0, 0.2
  thx, thy, thz = 0., 0., 0.
  m = np.eye(3)
  set_transform(x, y, z, m, t)
  br.sendTransform(t)

  old_attr = termios.tcgetattr(sys.stdin)     # 备份终端属性,普通的终端属性是需要按下’Enter‘键的。
  tty.setcbreak(sys.stdin.fileno())     # 设置属性。设置成实时监测键盘输入的属性
  print('Please input keys, press Ctrl + C to quit')

  rate = rospy.Rate(20.0)     # 设置发布频率
  while not rospy.is_shutdown():
    thx, thy, thz = 0., 0., 0.
    if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:     #设置超时参数为0，防止阻塞
      key = sys.stdin.read(1) # 获得键值
      #
      # 平移相关
      if key == "w":  # "w"键，欧式空间向前，全局坐标的+x方向。不是沿tf的+X轴
        x += 0.01 
      elif key == "s":  # "s"键，欧式空间向后
        x -= 0.01
      elif key == "a":  # "a"键，欧式空间向前
        y += 0.01
      elif key == "d":  # "d"键，欧式空间向前
        y -= 0.01
      elif key == "x":  # 上键，欧式空间向前
        z += 0.01
      elif key == "c":  # 下键，欧式空间向前
        z -= 0.01
      
      #
      # 旋转相关
      elif key == "u":  # 绕当前x轴顺时针旋转
        thx = 0.1
      elif key == "i":  # 绕当前y轴顺时针旋转
        thy = 0.1
      elif key == "o":  # 绕当前z轴顺时针旋转
        thz = 0.1
      elif key == "j":  # 绕当前x轴逆时针旋转
        thx = -0.1
      elif key == "k":  # 绕当前y轴逆时针旋转
        thy = -0.1
      elif key == "l":  # 绕当前z轴逆时针旋转
        thz = -0.1
      else:
        pass

    dm = Euler_2_matrix(thx, thy, thz)
    m = m.dot(dm)
    set_transform(x, y, z, m, t)
    t.header.stamp = rospy.Time.now()
    br.sendTransform(t)
    rate.sleep()     #使用sleep()函数消耗剩余时间
  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)     #设置为原先的标准模式