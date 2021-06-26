#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy, sys
import tf2_ros

from geometry_msgs.msg import Pose, TransformStamped
from std_msgs.msg import Duration

import numpy as np
import Geometry
from copy import deepcopy

import cv2
import Kalman_Filter

#
# 读取文件夹下的图片，生成轨迹后执行
if __name__ == '__main__':
    rospy.init_node('detect_Ch_chess')

    K = Kalman_Filter.KalmanFilter(2, 0.05, 5.0)             # 2维的卡尔曼滤波器
    cap=cv2.VideoCapture(1)

    tf_br = tf2_ros.TransformBroadcaster()  
    chess_pose = TransformStamped()
    chess_pose.header.frame_id = "wristPitch_Link"
    chess_pose.child_frame_id = "chess"
    chess_pose.transform.rotation.w = 1.0

    while(cap.isOpened()):
        ret_flag, Vshow = cap.read()
        gray = cv2.cvtColor(Vshow, cv2.COLOR_BGR2GRAY)
        circles= cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 100, param1=150, param2=30, minRadius=30, maxRadius=80)

        try:
            for circle in circles[0]:
                x = 360 - int(circle[0]) 
                y = int(circle[1]) -240
                raw_data = np.array([[x], [y]])
                xhat = K.kalmanFilter(raw_data)  # 滤波

                chess_pose.transform.translation.x = xhat[0,0]*0.0001
                chess_pose.transform.translation.y = xhat[1,0]*0.0001
                #print xhat[0,0]*0.0001, xhat[1,0]*0.0001
                print chess_pose.transform.translation.x, chess_pose.transform.translation.y
                tf_br.sendTransform(chess_pose)

                Vshow = cv2.circle(Vshow, (360-int(xhat[0,0]), int(xhat[1,0])+240), 20, (0,0,255), -1)
        except:
            pass


        Vshow = cv2.line(Vshow, (310,240), (330,240), (0,255,0))
        Vshow = cv2.line(Vshow, (320,230), (320,250), (0,255,0))
        cv2.imshow('Capture', Vshow)
        k=cv2.waitKey(1)
        if k==ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()