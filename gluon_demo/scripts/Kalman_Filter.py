#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Kalman_Filter.py

@Created on JAN 6 2021
@author:菜伟
@email:li_wei_96@163.com
"""
import numpy as np
import cv2

# 卡尔曼滤波器模型
# X(k) = AX(k-1) + BU(k) + W(k)     W ～ N(0, Q)
# Z(k) = HX(k) + V(k)               V ～ N(0, R)

# 递推过程
# X(k|k-1) = AX(k-1|k-1) + BU(k)                # 计算预测值
# P(k|k-1) = AP(k-1|k-1)A' + Q(k)               # 预测值和真实值之间的协方差矩阵
# Kg(k) = P(k|k-1)H' / [HP(k|k-1)H'+R]          # 计算卡尔曼增益
# X(k|k) = X(k|k-1) + Kg(k)[Z(k)-HX(k|k-1)]     # 计算估计值
# P(k|k) = (1-Kg(k)H)P(k|k-1)                   # 计算估计值与真实值的协方差矩阵

class KalmanFilter:
    def __init__(self, dim_, q, r):     # (滤波器维度, 系统误差， 测量误差)
        # 这里是假设A=1，H=1的情况。增益为1。因为输入是像素值 输出也是
        self.A = np.eye(dim_)
        self.H = np.eye(dim_)
        self.I = np.eye(dim_)    # 单位向量

        # 所有的递推变量初始化为 0
        self.xhat      = np.zeros((dim_, 1))         # 估计值 x，滤波器的输出 
        self.P         = np.zeros((dim_, dim_))      # 估计值与真实值的协方差矩阵
        self.Pminus    = np.zeros((dim_, dim_))      # 预测值与真实值的协方差矩阵
        self.xhatminus = np.zeros((dim_, 1))         # 预测值       
        self.K         = np.zeros((dim_, dim_))      # 卡尔曼增益

        self.Q = q*np.eye(dim_)                   # 系统噪声协方差
        self.R = r*np.eye(dim_)                   # 测量噪声协方差 (需要调) 调大：表明系统的测量不准，更相信上一时刻的估计值

        # 将变量全部转化成 mat
        self.A = np.mat(self.A)
        self.H = np.mat(self.H)
        self.xhat = np.mat(self.xhat)
        self.P = np.mat(self.P)
        self.Pminus = np.mat(self.Pminus)
        self.xhatminus = np.mat(self.xhatminus)
        self.K = np.mat(self.K)
        self.Q = np.mat(self.Q)
        self.R = np.mat(self.R)

    def kalmanFilter(self, z):  # 输入z是 array 向量
        #print self.xhatminus
        self.xhatminus  = self.A* self.xhat
        self.Pminus     = self.A * self.P *  self.A.T  + self.Q
        self.K          = self.Pminus * self.H.T * (self.H * self.Pminus * self.H.T + self.R).I
        self.xhat       = self.xhatminus + self.K * ( z - self.H * self.xhatminus )
        self.P          = (self.I - self.K * self.H) * self.Pminus
        return self.xhat


if __name__ == '__main__':

    K = KalmanFilter(2, 0.05, 5.0)             # 2维的卡尔曼滤波器

    cap = cv2.VideoCapture(1)

    while(cap.isOpened()):
        ret_flag, Vshow = cap.read()
        gray = cv2.cvtColor(Vshow, cv2.COLOR_BGR2GRAY)
        circles= cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 100, param1=100, param2=30, minRadius=5, maxRadius=100)

        try:
            for circle in circles[0]:
                x = int(circle[0])
                y = int(circle[1])
                #r = int(circle[2])
                raw_data = np.array([[x], [y]])
                xhat = K.kalmanFilter(raw_data)  # 滤波
                Vshow = cv2.circle(Vshow, (int(xhat[0,0]), int(xhat[1,0])), 20, (0,0,255), -1)
        except:
            pass

        cv2.imshow('Capture', Vshow)
        k=cv2.waitKey(1)
        if k==ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()




