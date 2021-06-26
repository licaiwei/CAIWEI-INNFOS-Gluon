#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
THIS PROGRAM IS FREE SOFTWARE, IS LICENSED MIT
manipulator_Zero_pose_calibration.py - 

@Created on APR 6 2021
@author:菜伟
@email:li_wei_96@163.com
"""

import rospy
import sys
# 相关的 service
from actuatorcontroller_ros.srv import GeneralQuery             # 在线的电机，与状态
from actuatorcontroller_ros.srv import ZeroReset                # 当前电机位置设置成零位，设置之前，先设置成 Mode_Homing 模式
from actuatorcontroller_ros.srv import ParametersSave           # 把更改的参数下载到电机上
from actuatorcontroller_ros.srv import AttributeQuery           # 查看 可改参数
# 相关的 Topic
from std_msgs.msg import Int32MultiArray
from actuatorcontroller_ros.msg import ActuatorArray, ActuatorAttribute, ActuatorCommand, ActuatorModes

# 以下 service 用不到
'''
from actuatorcontroller_ros.srv import IDModify                 # 更改一个电机的ID
from actuatorcontroller_ros.srv import AttributeDictionary      # 查看 参数含义与使用方法

from actuatorcontroller_ros.srv import DebugQuery               # 查看 不常用的不可更改参数
from actuatorcontroller_ros.srv import TriviaQuery              # 查看 不可更改参数
'''

if __name__ == '__main__':
    
    rospy.init_node('manipulator_Zero_pose_calibration', anonymous=False)
    pub_enableActuator =  rospy.Publisher("/INNFOS/enableActuator", ActuatorArray, queue_size=1)  # 设置开机句柄
    pub_setControlMode =  rospy.Publisher("/INNFOS/setControlMode", ActuatorModes, queue_size=1)  # 设置工作模式句柄
    srv_query_Motor = rospy.ServiceProxy("/INNFOS/GeneralQuery", GeneralQuery)          # 查询开关机状态 service 句柄
    srv_mode_Motor = rospy.ServiceProxy("/INNFOS/AttributeQuery", AttributeQuery)       # 查询开工作模式 service 句柄
    srv_zero_set = rospy.ServiceProxy("/INNFOS/ZeroReset", ZeroReset)                   # 设置0位 service 句柄
    srv_save = rospy.ServiceProxy("/INNFOS/ParametersSave", ParametersSave)             # 保存设置 service 句柄


    #
    # 1.读取电机的开关状态
    print( "********************************** 1 ******************************************")
    rospy.wait_for_service("/INNFOS/GeneralQuery") 
    try:
        motor_State = srv_query_Motor( True )
        if len(motor_State.ActuatorList)==0:
            print( "没有检测到电机0")
            sys.exit(1)
        print( "检测到在线的电机数量：", len(motor_State.ActuatorList))
        print( "\nID     status    开机（1)、关机（0）")
        for i in range(0, len(motor_State.ActuatorList) ):
            print( motor_State.ActuatorList[i], "       ", motor_State.ActuatorSwitch[i])
    except:
        print("未检测到电机，试试重启")
        sys.exit(1)

    # 
    # 2.提示输入: y继续，n结束
    key = raw_input("\n继续将会打开所有电机：y/n \n")
    if( key=="y" ):
        pass
    elif( key=="n"):
        print("退出...")
        sys.exit(1)
    else:
        print("错误的输入")
        sys.exit(1)
    
    # 
    # 3.打开所有的电机
    ID = ActuatorArray()
    ID.JointIDs = (0,)
    pub_enableActuator.publish(ID)      # 发布 [0] 将会打开在线的所有电机
    print("正在打开所有电机...\n")
    rospy.sleep(0.5)

    #
    # 4.再次读取电机开关状态
    print("**********************************")
    rospy.wait_for_service("/INNFOS/GeneralQuery") 
    try:
        motor_State = srv_query_Motor( True )
        print("\nID     status     开机（1)、关机（0）")
        for i in range(0, len(motor_State.ActuatorList) ):  # motor_State.ActuatorList 是 list 包含在线电机的ID 
            print(motor_State.ActuatorList[i], "       ", motor_State.ActuatorSwitch[i])
    except:
        print("启动失败")
        sys.exit(1)

    # 
    # 5.提示进入归零模式
    rospy.sleep(0.5)
    print( "********************************** 2 ******************************************")
    key = raw_input("\n继续将会进入归零模式：y/n \n")
    if( key=="y" ):
        pass
    elif( key=="n"):
        print( "退出...")
        sys.exit(1)
    else:
        print( "错误的输入")
        sys.exit(1)

    # 
    # 6.设置成 homing 模式
    mode_Set = ActuatorModes()
    mode_Set.JointIDs = motor_State.ActuatorList
    mode_Set.ActuatorMode = 6                       # 6 是电机的归零模式
    pub_setControlMode.publish(mode_Set)
    print( "电机的工作模式有：\n电流模式（1）\n速度模式（2）\n位置模式（3）\n梯形位置模式（4）\n梯形电流模式（5）\n归零模式（6）")
    print( "\n正设置成 归零模式（6）...")
    rospy.sleep(0.5)

    # 
    # 7.查询每个电机的模式，看看是不是都设置成 6 了
    print( "**********************************")
    rospy.wait_for_service("/INNFOS/AttributeQuery") 
    try:
        print( "ID     mode")
        for i in motor_State.ActuatorList:
            motor_mode = srv_mode_Motor( i )
            print( i, "     ", motor_mode.MODE_ID)
    except:
        print( "查询工作模式失败")
        sys.exit(1)

    # 
    # 8.调整位姿
    rospy.sleep(0.5)
    print( "********************************** 3 ******************************************")
    print( "\n现在手动拖动各个关节到零位置")
    while not rospy.is_shutdown():
        key = raw_input("\n调节之后，y将当前位置设置成零位 n退出 v查看当前关节角度：y/n/v \n")
        if( key=="y" ):
            break
        elif( key=="n"):
            print( "退出...")
            sys.exit(1)
        elif( key=="v"):
            pass
        else:
            print( "错误的输入")
            sys.exit(1)

        rospy.wait_for_service("/INNFOS/AttributeQuery") 
        try:
            print( "**********************************")
            print( "ID     degree(度)")
            for i in motor_State.ActuatorList:
                motor_mode = srv_mode_Motor( i )
                print( i, "     ", motor_mode.ACTUAL_POSITION*10.0 )
        except:
            print( "读取角度失败")
            sys.exit(1)
    print( "正在设置当前位置为对应电机的 0位....")
    
    #
    # 9.设置 0 位
    rospy.sleep(0.5)
    print( "********************************** 4 ******************************************")
    rospy.wait_for_service("/INNFOS/ZeroReset")
    print( "设置0位....")
    print( "ID     Successful")
    try:
        for i in motor_State.ActuatorList:  
            set_success = srv_zero_set( i )             # 设置0位，返回是否成功
            print( i, "     ", set_success.isSuccessful)
            # 电流模式
            rospy.set_param("/INNFOS/Actuator/"+str(i)+"/CUR_PROPORTIONAL", 0.3)        # 电流环 PI
            rospy.set_param("/INNFOS/Actuator/"+str(i)+"/CUR_INTEGRAL", 0.15)
            rospy.set_param("/INNFOS/Actuator/"+str(i)+"/CURRENT_LIMIT", 8)             # 

            # 速度模式
            rospy.set_param("/INNFOS/Actuator/"+str(i)+"/VEL_PROPORTIONAL", 4.2)        # 速度环 PI
            rospy.set_param("/INNFOS/Actuator/"+str(i)+"/VEL_INTEGRAL", 0.01)
            rospy.set_param("/INNFOS/Actuator/"+str(i)+"/VEL_OUTPUT_LIMITATION_MAXIMUM", 1)
            rospy.set_param("/INNFOS/Actuator/"+str(i)+"/VEL_OUTPUT_LIMITATION_MINIMUM", -1)
            rospy.set_param("/INNFOS/Actuator/"+str(i)+"/VELOCITY_LIMIT", 3000)         # 速度最大设为 3000

            # 位置模式
            rospy.set_param("/INNFOS/Actuator/"+str(i)+"/POS_PROPORTIONAL", 0.15)       # 位置PID    一组柔和的参数 0.15  0.001  0
            rospy.set_param("/INNFOS/Actuator/"+str(i)+"/POS_INTEGRAL", 0.0001)
            rospy.set_param("/INNFOS/Actuator/"+str(i)+"/POS_DIFFERENTIAL", 0)
            rospy.set_param("/INNFOS/Actuator/"+str(i)+"/POS_OUTPUT_LIMITATION_MAXIMUM", 0.7)   # 最大速度限制
            rospy.set_param("/INNFOS/Actuator/"+str(i)+"/POS_OUTPUT_LIMITATION_MINIMUM", -0.7)
            rospy.set_param("/INNFOS/Actuator/"+str(i)+"/POS_LIMITATION_MAXIMUM", 120)          # 将所有的关节限位设为 -120～120
            rospy.set_param("/INNFOS/Actuator/"+str(i)+"/POS_LIMITATION_MINIMUM", -120)

            # 梯形位置模式  主要用这种模式，熟悉这三个参数
            rospy.set_param("/INNFOS/Actuator/"+str(i)+"/PROFILE_POS_ACC", 120000)      # 梯形位置最大加/减速度
            rospy.set_param("/INNFOS/Actuator/"+str(i)+"/PROFILE_POS_DEC", -120000)
            rospy.set_param("/INNFOS/Actuator/"+str(i)+"/PROFILE_POS_MAX_SPEED", 300)   # 运行速度

            # 梯形速度模式
            rospy.set_param("/INNFOS/Actuator/"+str(i)+"/PROFILE_VEL_ACC", 1200)
            rospy.set_param("/INNFOS/Actuator/"+str(i)+"/PROFILE_VEL_DEC", 1200)
            rospy.set_param("/INNFOS/Actuator/"+str(i)+"/PROFILE_VEL_MAX_SPEED", 3000)
            rospy.sleep(0.2)
    except:
        print( "设置0位失败")
        sys.exit(1)
    rospy.sleep(0.5)

    #
    # 10.查询当前角度
    rospy.wait_for_service("/INNFOS/AttributeQuery") 
    print( "**********************************")
    try:
        print( "ID     degree(度)")
        for i in motor_State.ActuatorList:
            motor_mode = srv_mode_Motor( i )
            print( i, "     ", motor_mode.ACTUAL_POSITION*10.0)
    except:
        print( "读取角度失败")
        sys.exit(1)

    #
    # 11.保存数据到电机
    print( "**********************************")
    print( "将数据保存到电机...")
    rospy.wait_for_service("/INNFOS/ParametersSave") 
    print( "ID     Successful")
    try:
        for i in motor_State.ActuatorList:
            save_success = srv_save( i )
            print( i, "     ", save_success.isSuccessful)
            rospy.sleep(0.5)
    except:
        print( "保存0位数据失败")
        sys.exit(1)
    print( "0位姿校正结束。")