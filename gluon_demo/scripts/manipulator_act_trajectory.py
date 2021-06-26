#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
manipulator_act_trajectory.py - 

@Created on APR 6 2021
@author:菜伟
@email:li_wei_96@163.com
"""
import scipy.interpolate as spi
import numpy as np
import matplotlib.pyplot as plt

import rospy
import sys
from sensor_msgs.msg import JointState
from gluon_demo.msg import gluon_demo_msgs
from geometry_msgs.msg import Pose

from actuatorcontroller_ros.msg import ActuatorArray, ActuatorAttribute, ActuatorCommand, ActuatorModes
from actuatorcontroller_ros.srv import GeneralQuery             # 在线的电机，与状态
from actuatorcontroller_ros.srv import AttributeQuery           # 查看 可改参数
from actuatorcontroller_ros.srv import ParametersSave           # 把更改的参数下载到电机上


rate = 36.0           # 电机的转速比 
pi = 3.14159265
rate_position = rate / 2.0 / pi         # 弧度 -> 电机的 R
rate_veloity = rate_position * 60.0     # 弧度每秒 -> 电机的 R/min

#
# 对moveit得到的轨迹按照时间进行插值，再按照等时间间隔取位置点，用定时中断按照等时间间隔控制电机，控制频率要看CAN总线的电机数量
def spline(points, joint, period_):
    length = len(points)
    x = []
    y = []
    for i in range(length):
        x.append( points[i].time_from_start.secs+points[i].time_from_start.nsecs*1e-9 )
        y.append( points[i].positions[joint] )
    new_x = np.linspace(0, points[length-1].time_from_start.secs + points[length-1].time_from_start.nsecs*1e-9, 
                        int((points[length-1].time_from_start.secs + points[length-1].time_from_start.nsecs*1e-9)/period_) )
    new_y = spi.spline(x, y, new_x, order=3, kind='smoothest')
    
    '''
    fig,(ax1, ax2)=plt.subplots(2, 1, figsize=(10, 12))
    ax1.plot(x, y, 'o', label='sample')
    #ax1.set_ylim(y.min()-1, y.max()+1)
    ax1.legend()
    ax2.plot(new_x, new_y, 'o', label='cubic spline')
    #ax2.set_ylim(y.min()-1, y.max()+1)
    ax2.legend()
    plt.show()
    '''
    
    return new_y

trac = []
idx_traj = 1

#
# 梯形位姿模式控制函数
def callback(a):
    global idx_traj 
    idx_traj += 1 
    if idx_traj < len(trac[0]):    # 这里注意，一定从 1 开始，因为插值算法的开头参数总是 0
        # 设置电机控制指令
        actuator_Target.position = [ x[idx_traj]*rate_position for x in trac]
        actuator_Target.header.stamp = rospy.Time.now()
        # 发布
        pub_actuatorTarget.publish(actuator_Target)

#
# 速度控制模式，相关参数、函数
pid_param = [[2.0, 0.1, 0.0],   # 各个关节的pid参数，关节顺序'shoulderPitch', 'shoulderRoll', 'shoulderYaw', 'elbowRoll', 'wristYaw', 'wristPitch'
             [2.0, 0.1, 0.0],
             [2.0, 0.1, 0.0],
             [2.0, 0.1, 0.0],
             [2.0, 0.1, 0.0],
             [2.0, 0.1, 0.0]]
error = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]         # 在开始之前记得清0
error_sum = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]

#
# 梯形速度模式
def callback2(a):
    global idx_traj 
    idx_traj += 1
    if idx_traj < len(trac[0]):    # 这里注意，一定从 1 开始，因为插值算法的开头参数总是 0
        feedback_joints_states = rospy.wait_for_message("/INNFOS/actuator_states",JointState, timeout=pub_period)
        error = [ x[idx_traj]*rate_position for x in trac ]             # 当前位置 rad -> R
        rospy.loginfo( feedback_joints_states)

        for i in range(0, np.minimum(len(error), len(feedback_joints_states.position))):
            error[i] = error[i] - feedback_joints_states.position[i]    # 误差 R
            error_sum[i] = error_sum[i] + error[i]
    
        # 设置电机控制指令
        actuator_Target.velocity = [ (pid_param[i][0]*error[i] + pid_param[i][1]*error_sum[i]) for i in range(len(pid_param))]   # u(t) = Kp*e(t) + K_i*e_sum(t)
        actuator_Target.position = [ x[idx_traj]*rate_position for x in trac]     # 在速度控制模式下，位置值不起作用，我们用来查看轨迹跟踪效果

        actuator_Target.header.stamp = rospy.Time.now()     # 设置时间戳

        # 发布
        pub_actuatorTarget.publish(actuator_Target)


#
# 节点订阅 控制模式 和 轨迹
# 控制模式有：点位控制（1）、末端直线、圆弧控制（2）、末端伺服（3）、去重力控制（4）
if __name__ == '__main__':
    
    rospy.init_node('manipulator_act_tracjectory', anonymous=False)
    pub_enableActuator =  rospy.Publisher("/INNFOS/enableActuator", ActuatorArray, queue_size=1)    # 设置开机 topic句柄
    pub_setControlMode =  rospy.Publisher("/INNFOS/setControlMode", ActuatorModes, queue_size=1)    # 设置工作模式 topic句柄
    pub_actuatorTarget = rospy.Publisher("/INNFOS/actuator_targets", JointState, queue_size=1)      # 设置电机目标 topic句柄，这是 innfos wiki 推荐的控制方法
    srv_query_Motor = rospy.ServiceProxy("/INNFOS/GeneralQuery", GeneralQuery)          # 查询开关机状态 service 句柄
    srv_mode_Motor = rospy.ServiceProxy("/INNFOS/AttributeQuery", AttributeQuery)       # 查询开工作模式 service 句柄
    srv_save = rospy.ServiceProxy("/INNFOS/ParametersSave", ParametersSave)             # 保存设置 service 句柄
    rospy.sleep(2)

    #
    # 加载YAML文建定义的 关节名称 --- id
    Joint_Name_ID = {}         # 字典{"joint_name":id,}  将yaml定义从参数服务器读取出来
    param_list = rospy.get_param_names()
    for param in param_list:
        if "/joint_id_list/" in param:
            Joint_Name_ID[ param.replace("/joint_id_list/", "") ] = rospy.get_param( param )
    if len(Joint_Name_ID) == 0:
        rospy.loginfo("没有加载 yaml 配置文件")
        exit(1)

    #
    # 1.开机
    ID = ActuatorArray()
    ID.JointIDs = (0,)
    pub_enableActuator.publish(ID)          # 发布 [0] 将会打开在线的所有电机
    rospy.sleep(2)
    #读取电机开关状态
    rospy.wait_for_service("/INNFOS/GeneralQuery") 
    try:
        motor_State = srv_query_Motor( True )
        rospy.loginfo( "检测到在线的电机数量：%s", len(motor_State.ActuatorList) )
        num_on = 0
        for i in range(0, len(motor_State.ActuatorList) ):      # motor_State.ActuatorList 是 list 包含在线电机的ID 
            if motor_State.ActuatorSwitch[i]:
                num_on += 1
        rospy.loginfo( "有%s个电机已开机", num_on)
        if num_on == 0:
            exit(1)
    except:
        rospy.loginfo( "启动失败,检查网线链接")
        sys.exit(1)

    #
    # 2.设置成模式4     梯形位置模式  (这里解释一下刚开机就进入梯形位置模式，是为了在没有指令的时候，机械臂是刚性的)
    mode_Set = ActuatorModes()
    mode_Set.JointIDs = motor_State.ActuatorList
    mode_Set.ActuatorMode = 4
    pub_setControlMode.publish(mode_Set)
    rospy.sleep(1)
    rospy.wait_for_service("/INNFOS/AttributeQuery")
    try:
        for i in motor_State.ActuatorList:
            motor_mode = srv_mode_Motor( i )
            if motor_mode.MODE_ID is not 4:
                rospy.loginfo( "ID%s不是梯形位置模式，重新运行该节点", i)
                sys.exit(1)
    except:
        rospy.loginfo( "查询工作模式失败")
        sys.exit(1)

    #
    # 3.保存4模式数据到电机
    rospy.wait_for_service("/INNFOS/ParametersSave") 
    try:
        for i in motor_State.ActuatorList:
            save_success = srv_save( i )
    except:
        rospy.loginfo( "保存模式4失败")
        sys.exit(1)

    rospy.loginfo( "控制节点已启动，等待控制指令...")

    #
    # 4.订阅 控制模式 与 轨迹，解析ID
    while not rospy.is_shutdown():
        # 订阅2个消息：工作模式 + 轨迹。所以在规划node中，要多次发送同一个：工作模式 + 轨迹
        arm_command = rospy.wait_for_message("/manipulator/command", gluon_demo_msgs)           # int16

        #
        # 0、角度控制模式 （用电机的梯形位置模式，读取arm_command.joint，设定电机ID、速度、目标角度 
        if (arm_command.mode == 0):
            velocities = arm_command.joint.velocity

            actuator_Target = JointState()                  # 电机控制指令
            joint_num = len(arm_command.joint.name)   # 关节数量
            for i in range(0, joint_num):
                rospy.set_param("/INNFOS/Actuator/"+str( Joint_Name_ID[ arm_command.joint.name[i] ] )+"/PROFILE_POS_MAX_SPEED", abs(velocities[i]*rate_veloity))   # 运行速度,将 rad/s 转换成 R/min
                actuator_Target.name.append( str( Joint_Name_ID[ arm_command.joint.name[i] ] ))               # 名称设置成关节电机的ID
            actuator_Target.position = [ x*rate_position for x in arm_command.joint.position]            # 关节的目标值,取的是轨迹的的末端位置
            actuator_Target.velocity = [ 0 for x in range(0, joint_num) ]                   # 等长的 [0, 0, ....]
            actuator_Target.effort = [ 0 for x in range(0, joint_num) ]

            # 发布
            rospy.sleep(0.5)
            pub_actuatorTarget.publish(actuator_Target)
            pass
    
        #
        # 1、点位模式 （用电机的梯形位置模式，读取arm_command.traj.points，设定电机ID、速度、目标角度 ）
        elif (arm_command.mode == 1):
            # 关节速度 list，取的是轨迹的中间点的速度,也就是关节稳定运动时的速度
            velocities = arm_command.traj.points[len(arm_command.traj.points)/2].velocities     
            actuator_Target = JointState()                  # 电机控制指令
            joint_num = len(arm_command.traj.joint_names)   # 关节数量
            for i in range(0, joint_num):
                rospy.set_param("/INNFOS/Actuator/"+str( Joint_Name_ID[ arm_command.traj.joint_names[i] ] )+"/PROFILE_POS_MAX_SPEED", abs(velocities[i]*rate_veloity))   # 运行速度,将 rad/s 转换成 R/min
                actuator_Target.name.append( str( Joint_Name_ID[ arm_command.traj.joint_names[i] ] ))               # 名称设置成关节电机的ID
            actuator_Target.position = [ x*rate_position for x in arm_command.traj.points[-1].positions]            # 关节的目标值,取的是轨迹的的末端位置
            actuator_Target.velocity = [ 0 for x in range(0, joint_num) ]                   # 等长的 [0, 0, ....]
            actuator_Target.effort = [ 0 for x in range(0, joint_num) ]
            # 发布
            rospy.sleep(0.5)
            pub_actuatorTarget.publish(actuator_Target)

        #
        # 2、跟随轨迹，用电机的梯形位置模式，读取arm_command.traj
        elif (arm_command.mode == 2):
            actuator_Target = JointState()                  # 电机控制指令
            joint_num = len(arm_command.traj.joint_names)   # 关节数量
            trac = []                                       # 插值之后的位置 [ 关节1[]，关节2[]，关节3[]... ]
            idx_traj = 1
            pub_period = 0.05                               # 定时发布的周期 s
            for i in range(0, joint_num):                   # 设置 name+速度
                rospy.set_param("/INNFOS/Actuator/"+str( Joint_Name_ID[ arm_command.traj.joint_names[i] ] )+"/PROFILE_POS_MAX_SPEED", 200) # 设置成一个较大的速度，快速相应
                actuator_Target.name.append( str( Joint_Name_ID[ arm_command.traj.joint_names[i] ] ))                                       # 名称设置成关节电机的ID
                trac.append(spline(arm_command.traj.points, i, pub_period)) 
            actuator_Target.velocity = [ 0 for x in range(0, joint_num) ]   # 等长的 [0, 0, ....]
            actuator_Target.effort = [ 0 for x in range(0, joint_num) ]

            timer = rospy.Timer(rospy.Duration(pub_period), callback)
            while(idx_traj<len(trac[0])):
                pass
            timer.shutdown()
            #rospy.loginfo( "跳出")

        # 
        # 3、速度环控制（用电机的梯形位置模式，读取arm_command.traj，跟随轨迹）
        elif (arm_command.mode == 3):
            rospy.loginfo( "arm_command.mode == 3")
            # 设置成模式5  梯形速度模式  当速度设置为0时，机械臂是刚性的
            mode_Set = ActuatorModes()
            mode_Set.JointIDs = motor_State.ActuatorList
            mode_Set.ActuatorMode = 5
            pub_setControlMode.publish(mode_Set)
            rospy.sleep(1)
            rospy.wait_for_service("/INNFOS/AttributeQuery")
            try:
                for i in motor_State.ActuatorList:
                    motor_mode = srv_mode_Motor( i )
                    if motor_mode.MODE_ID is not 5:
                        rospy.loginfo( "ID%s不是梯形位置模式，设置梯形速度模式失败", i)
                        sys.exit(1)
            except:
                rospy.loginfo( "查询工作模式失败222")
                sys.exit(1)

            # 控制量是速度，控制器
            actuator_Target = JointState()                  # 电机控制指令 ********
            joint_num = len(arm_command.traj.joint_names)   # 关节数量
            trac = []                                       # 位置轨迹（输入量） ************
            idx_traj = 1
            pub_period = 0.05                               # 定时发布的周期 s

            for i in range(0, joint_num):                   # 设置 name+速度
                #rospy.set_param("/INNFOS/Actuator/"+str( Joint_Name_ID[ arm_command.traj.joint_names[i] ] )+"/PROFILE_POS_MAX_SPEED", 3000) # 设置成一个较大的速度，快速相应
                actuator_Target.name.append( str( Joint_Name_ID[ arm_command.traj.joint_names[i] ] ))                                       # 名称设置成关节电机的ID
                trac.append(spline(arm_command.traj.points, i, pub_period)) 
 
            # 设置速度模式的PID参数
                # **********

            #actuator_Target.velocity = [ 0 for x in range(0, joint_num) ]   # 等长的 [0, 0, ....]
            #actuator_Target.position = [ 0 for x in range(0, joint_num) ]
            actuator_Target.effort = [ 0 for x in range(0, joint_num) ]

            # 开始 PID 控制电机速度
            error = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]         # 在开始之前记得清0
            error_sum = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
            
            timer = rospy.Timer(rospy.Duration(pub_period), callback2)      # callback2 是速度pid控制的，
            while(idx_traj<len(trac[0])):
                pass
            timer.shutdown()
            
        #
        # 4、直线控制，输入起点、终点位姿，直线插补。必须是无碰撞的轨迹
        elif (arm_command.mode == 4): 
            # 
            # 直线路径
            arm_command = gluon_demo_msgs()
            pose1 = arm_command.pose[0]
            x = np.linspace[arm_command.pose[0].position.x, arm_command.pose[1].position.x, ]
            y = []
            z = []

            
            if lap.shape[0]*2 >= lap.shape[1]: # 按照x缩小
                rate = 0.21/lap.shape[0]
            else:
                rate = 0.34/lap.shape[1]



            np.sqrt( (x[i]-x[i-1])**2 + (y[i]-y[i-1])**2 + (z[i]-z[i-1])**2 )

            for i in range(0,len(x)):
                p1 = deepcopy(p)            # 位置
                p1.position.x = x[i]
                p1.position.y = y[i]
                p1.position.z = z[i]
                waypoints.append(p1)

                Dur_time = Duration()       # 时间戳
                if i == 0:
                    Dur_time.data.secs = 0
                    Dur_time.data.nsecs = 0
                    time_header += [ Dur_time ]
                else:
                    tim = np.sqrt( (x[i]-x[i-1])**2 + (y[i]-y[i-1])**2 + (z[i]-z[i-1])**2 )/speed  # 两点之间的时间间隔
                    tim += time_header[i-1].data.secs + time_header[i-1].data.nsecs*1e-9
                    Dur_time.data.secs = int(tim)
                    Dur_time.data.nsecs = int( (tim-Dur_time.data.secs)*1e9 )
                    time_header += [ Dur_time ]
            # 
            # 逆运动学，得到关节轨迹
            # 得到关节轨迹 这里比较复杂，主要任务是加时间戳。这里生成的轨迹，是标准轨迹 JointTrajectory() 
            traj = JointTrajectory()
            traj.joint_names = ['axis_joint_1', 'axis_joint_2', 'axis_joint_3', 'axis_joint_4', 'axis_joint_5', 'axis_joint_6']
            wp = trans(waypoints, target_pose )    # 变换到画画区域坐标系，画画的灵活区域是指定的

            #pb = PointStamped() # 在 RVIZ 中显示轨迹点
            #pb.header.frame_id = "base_link"
            for i in range(0, len(wp)):
                # pb.header.stamp = rospy.Time.now()
                # pb.point.x = wp[i].position.x
                # pb.point.y = wp[i].position.y
                # pb.point.z = wp[i].position.z
                # pb.header.stamp = rospy.Time.now()
                # pub_point.publish(pb)
                # rospy.loginfo pb.point
                # rospy.sleep(0.05)

                try:
                    joint = inv_kinematic.inv_arm( wp[i] )[0]  # 返回的多组解，只要第一组
                    points_ = JointTrajectoryPoint() 
                    points_.positions = joint
                    points_.time_from_start = time_header[i].data
                    #points_.velocities = [speed for i in range(0,len(traj.joint_names))]
                    #points_.accelerations = [0.1 for i in range(0,len(traj.joint_names))]
                    traj.points.append(points_)
                except:
                    rospy.loginfo( inv_kinematic.inv_arm( wp[i] ))
                    pass
            rospy.loginfo("已经得到关节轨迹")




        #
        # 六、
        elif (arm_command.mode == 5):
            pass

        #
        # 七、
        elif (arm_command.mode == 6):
            pass
