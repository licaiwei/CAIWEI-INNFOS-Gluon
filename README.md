# CAIWEI-INNFOS-Gluon
INNFOS的6自由度机械臂ROS package，我用Python重写了规划算法输出到控制卡的部分，可以用来执行movelit产生的轨迹。可以设置控制模式，整定了机械臂的各个关节执行器的位置控制模式和速度控制模式下的PID参数，在规划过程中可以自由的选择两种控制方法，这个SDK程序还在完善。

![demo_gif](https://github.com/licaiwei/CAIWEI-INNFOS-Gluon/blob/main/assets/demo_gif.gif)


## 软件相关

#### 编程环境 

Ubuntu 18.04 ROS Melodic

##### 获取代码

打开一个`termial`，定位到你的 `workspace/src` 下

```bash
git clone git@github.com:licaiwei/CAIWEI-INNFOS-Gluon.git
```

#### package介绍

##### gluon_description

机械臂的`URDF`描述文件

```bsah
roslaunch gluon_demo display.launch 
```

##### gluon_moveit_config

机械臂的`moveit`配置文件，描述文件

```BASJ
roslaunch gluon_demo display.launch 
```

##### ActuatorController_ROS

[INNFOS驱动器ROS SDK](https://github.com/mintasca/ActuatorController_ROS.git)

##### gluon_demo

`config/Joint_name_2_id.yaml` 更换了执行器，在不改变执行器ID的情况下直接更改该文件中的设置，即可上电使用。

`joint_state_publiser_controller`订阅`actuatorcontroller`发布的执行器状态，转换

`launch/real_driver.launch`加载上面的`yaml`文件，启动`joint_state_publiser_controller`

待更新
最近找工作，生活艰辛，有空一定更新
