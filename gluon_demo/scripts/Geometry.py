#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Geometry.py - Tool kit for common operations in SO(3), SO(4)

@Created on Sat Dec 28 21:07:24 2019
@author:菜伟
@email:li_wei_96@163.com
"""

import numpy as np
from numpy import sin, cos, pi, arctan, arccos, sqrt
from geometry_msgs.msg import Pose, TransformStamped


def QuaternionNorm(Q_raw):
    """
    四元数单位化
    @Parameters
    Q_raw: list or numpy.array([]) eg.[qx, qy, qz, qw]
    @Returns
    Q_normed_: list [qx_, qy_, qz_, qw_]
    """
    qx_temp, qy_temp, qz_temp, qw_temp = Q_raw[0:4]
    qnorm = sqrt(qx_temp*qx_temp + qy_temp*qy_temp +
                 qz_temp*qz_temp + qw_temp*qw_temp)
    qx_ = qx_temp/qnorm
    qy_ = qy_temp/qnorm
    qz_ = qz_temp/qnorm
    qw_ = qw_temp/qnorm
    Q_normed_ = [qx_, qy_, qz_, qw_]
    return Q_normed_


def ferk(n):
    """
    向量反对称化
    @Parameters
    vector: numpy.array([[]]) eg.array([[1], [2], [3]])
    @Returns
    ferk_R: numpy.array([[ 0, -3,  2],
                         [ 3,  0, -1],
                         [-2,  1,  0],) 3x3
    """
    m = np.array([[0, -n[2, 0],  n[1, 0]],
                  [n[2, 0], 0, -n[0, 0]],
                  [-n[1, 0], n[0, 0], 0]])
    return m


def vector_norm(v):
    """
    求向量的方向向量+模长
    @Parameters
    vector: numpy.array([[]]) eg.array([[1], [2], [3]])
    @Returns
    n, norm: numpy.array([[ 0.267261242],
                          [ 0.534522484],
                          [ 0.801783726],), 3.741657387
    """
    norm = sqrt(v[0, 0]**2 + v[1, 0]**2 + v[2, 0]**2)
    n = v/norm
    return n, norm


def inv_(T):
    """
    变换矩阵 T 求逆，4x4
    @Parameters
    T: geometry_msgs.msg.Pose() or TransformStamped()
    @Returns
    inv_max: np.array([[]]) 4x4
    """
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

    r = Quaternion_2_matrix([qx, qy, qz, qw])
    tr = np.c_[r, np.array([[x],
                            [y],
                            [z]])]
    tr = np.r_[tr, np.array([[0., 0., 0., 1.]])]
    #tr = np.array([[]])
    return np.linalg.inv(tr)


def deal_angle(a):
    """
    限定关节角度在 [-pi, pi]范围内
    @Parameters
    angle: Unlimited range
    @Returns
    angle_norm: range in [-pi, pi]
    """
    if np.pi <= a:
        a = a-2*np.pi
    elif a <= -np.pi:
        a = a+2*np.pi
    else:
        return a
    return deal_angle(a)


'''*************************************************************************************************
   旋转矩阵变换到其他
   ***********************************************************************************************'''


def Matrix_2_quaternion(m):
    """
    3*3的np.array矩阵 转换成 四元数
    @Parameters
    m: numpy.array([[]]) SO(3)
    @Returns
    Q_normed_: list [qx_, qy_, qz_, qw_]
    """
    qw_ = sqrt(np.trace(m)+1) / 2
    qx_ = -(m[1, 2]-m[2, 1]) / 4 / qw_
    qy_ = -(m[2, 0]-m[0, 2]) / 4 / qw_
    qz_ = -(m[0, 1]-m[1, 0]) / 4 / qw_
    Q_normed_ = [qx_, qy_, qz_, qw_]
    return Q_normed_


def Matrix_2_axis_angle(m):
    """
    3*3的np.array矩阵 转换成 旋转向量  SO(3)->so(3)
    @Parameters
    m: numpy.array([[]]) SO(3)
    @Returns
    n, th: numpy.array([[]]) 1x3, rotation angle(rad)
    """
    th = arccos((np.trace(m)-1) / 2)
    eigenvalue, featurevector = np.linalg.eig(m)  # 求特征值特征向量
    c = np.where(np.imag(eigenvalue) == 0)  # 旋转向量的特征值 1+0j 的所在行
    n = np.real(featurevector[:, c[0][0]]).reshape(3, 1)
    return n, th  # 方向向量,角度


def Matrix_2_euler(r):
    """
    3*3的np.array矩阵 转换成 欧拉角
    @Parameters
    m: numpy.array([[]]) SO(3)
    @Returns
    thx, thy, thz：euler angle （当前轴z-y-x = 绕固定轴x-y-z）
    """
    return Quaternion_2_euler(Matrix_2_quaternion(r))


'''*************************************************************************************************
   四元数变换到其他
   ***********************************************************************************************'''


def Quaternion_2_matrix(Q_normed_):
    """
    单位四元数 转化成 旋转矩阵 [x,y,z,w]
    @Parameters
    Q_normed_: list [qx_, qy_, qz_, qw_]
    @Returns
    m: numpy.array([[]]) SO(3)
    """
    q0 = Q_normed_[3]
    q1 = Q_normed_[0]
    q2 = Q_normed_[1]
    q3 = Q_normed_[2]
    m = np.array([[1-2*q2**2-2*q3**2,   2*q1*q2-2*q0*q3,   2*q1*q3+2*q0*q2],
                  [2*q1*q2+2*q0*q3, 1-2*q1**2-2*q3**2,   2*q2*q3-2*q0*q1],
                  [2*q1*q3-2*q0*q2,   2*q2*q3+2*q0*q1, 1-2*q1**2-2*q2**2]])
    return m


def Quaternion_2_axis_angle(Q_normed_):
    """
    单位四元数 转化成 旋转向量
    @Parameters
    Q_normed_: list [qx_, qy_, qz_, qw_]
    @Returns
    n, th: numpy.array([[]]) 1x3, rotation angle(rad)
    """
    m = Quaternion_2_matrix(Q_normed_)
    n, th = Matrix_2_axis_angle(m)
    return n, th  # 方向向量, 角度


def Quaternion_2_euler(Q_normed_):
    """
    四元数 转换成 欧拉角
    @Parameters
    Q_normed_: list [qx_, qy_, qz_, qw_]
    @Returns
    thx, thy, thz：euler angle（当前轴z-y-x = 绕固定轴x-y-z）
    """
    q0 = Q_normed_[3]
    q1 = Q_normed_[0]
    q2 = Q_normed_[1]
    q3 = Q_normed_[2]
    thx = np.arctan2(2*(q0*q1+q2*q3), 1-2*(q1**2+q2**2))
    thy = np.arcsin(2*(q0*q2-q1*q3))
    thz = np.arctan2(2*(q0*q3+q1*q2), 1-2*(q2**2+q3**2))
    return thx, thy, thz


'''*************************************************************************************************
   旋转向量变换到其他
   ***********************************************************************************************'''


def Axis_angle_2_matrix(n, th):
    """
    旋转向量 转换成 3*3的np.array矩阵，也就是Rodrigues变换 so(3)->SO(3)
    @Parameters
    n, th: numpy.array([[]]) 1x3, rotation angle(rad)
    @Returns
    m: numpy.array([[]]) SO(3)
    """
    m = cos(th)*np.eye(3) + (1-cos(th))*np.dot(n, n.T) + \
        sin(th)*ferk(n)  # Rodrigues变换
    return m


def Axis_angle_2_quaternion(n, th):
    """
    旋转向量 -> 四元数
    @Parameters
    n, th: numpy.array([[]]) 1x3, rotation angle(rad)
    @Returns
    Q_normed_: list [qx_, qy_, qz_, qw_]
    """
    m = Axis_angle_2_matrix(n, th)
    q = Matrix_2_quaternion(m)
    return q


def Axis_angle_2_euler(n, th):
    """
    旋转向量 -> 欧拉角
    @Parameters
    n, th: numpy.array([[]]) 1x3, rotation angle(rad)
    @Returns
    thx, thy, thz：euler angle （当前轴z-y-x = 绕固定轴x-y-z）
    """
    return Quaternion_2_euler(Axis_angle_2_quaternion(n, th))


'''*************************************************************************************************
   欧拉角变换到其他（当前轴z-y-x = 绕固定轴x-y-z )
   ***********************************************************************************************'''


def Euler_2_matrix(thx, thy, thz):
    """
    欧拉角 转换到 旋转矩阵
    @Parameters
    thx, thy, thz：euler angle  （当前轴z-y-x = 绕固定轴x-y-z）
    @Returns
    m: numpy.array([[]]) SO(3)
    """
    Rx = np.array([[1.,       0.,        0.],
                   [0., cos(thx), -sin(thx)],
                   [0., sin(thx),  cos(thx)]])
    Ry = np.array([[cos(thy),  0., sin(thy)],
                   [0.,  1.,      0.],
                   [-sin(thy),  0., cos(thy)]])
    Rz = np.array([[cos(thz), -sin(thz), 0.],
                   [sin(thz),  cos(thz), 0.],
                   [0.,        0., 1.]])
    m = Rz.dot(Ry.dot(Rx))
    return m


def Euler_2_quaternion(thx, thy, thz):
    """
    欧拉角 转换到 四元数
    @Parameters
    thx, thy, thz：euler angle （当前轴z-y-x = 绕固定轴x-y-z）
    @Returns
    Q_normed_: list [qx_, qy_, qz_, qw_]
    """
    m = Euler_2_matrix(thx, thy, thz)
    q = Matrix_2_quaternion(m)
    return q


def Euler_2_axis_angle(thx, thy, thz):
    """
    欧拉角 转换到 旋转向量
    @Parameters
    thx, thy, thz：euler angle （当前轴z-y-x = 绕固定轴x-y-z）
    @Returns
    n, th: numpy.array([[]]) 1x3, rotation angle(rad)
    """
    m = Euler_2_matrix(thx, thy, thz)
    q = Matrix_2_axis_angle(m)
    return


if __name__ == '__main__':
    """
    测试该工具包
    """
    print(Quaternion_2_euler(Euler_2_quaternion(-0.6, 0, -1.9)))
