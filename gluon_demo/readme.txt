发布的主题
/INNFOS/actuator_states (sensor_msgs::JointState)
提供所有已开机的执行器的位置，速度与电流，单位分别为：Rotations, RPM, 和 Amp。



Subscribed Topics
/INNFOS/enableActuator (ActuatorController_ROS::ActuatorArray)
打开指定的执行器，若输入为空或0，打开所有的执行器。

/INNFOS/disableActuator (ActuatorController_ROS::ActuatorArray)
关闭指定的执行器，若输入为空或0，关闭所有的执行器。

/INNFOS/setControlMode (ActuatorController_ROS::ActuatorModes)
给指定的执行器设定指定的模式, 可通过/INNFOS/Dictionary查询可用的模式

/INNFOS/setTargetPosition (ActuatorController_ROS::ActuatorCommand)
给指定的执行器设定目标位置，只有在执行器模式正确时才会生效。

/INNFOS/setTargetVelocity (ActuatorController_ROS::ActuatorCommand)
给指定的执行器设定目标速度，只有在执行器模式正确时才会生效。

/INNFOS/setTargetCurrent (ActuatorController_ROS::ActuatorCommand)
给指定的执行器设定目标电流，只有在执行器模式正确时才会生效。

/INNFOS/actuator_targets (sensor_msgs::JointState)
接受打包好的关节指令，可同时给多个执行器设置位置，速度或电流。推荐使用这种消息来控制多个执行器，
缓解ROS通信的强度。节点会根须当前执行器的模式筛选指令。



Services
/INNFOS/GeneralQuery (ActuatorController_ROS::GeneralQuery)
Function: 给用户查看所有在线的执行器与其开机状态。
Input: 一个变量，不需要赋值。
Output: 在线的执行器与其开机状态的列表。

/INNFOS/AttributeQuery (ActuatorController_ROS::AttributeQuery)
Function: 给用户查看一些常用可更改参数。 请使用对应的信息或服务更改这些参数 。
Input: 指定的执行器ID。
Output: 指定执行器的一部分常用可更改参数。

/INNFOS/TriviaQuery (ActuatorController_ROS::TriviaQuery)
Function: 给用户查看一些常用不可更改参数。
Input: 指定的执行器ID。
Output: 指定执行器的一部分常用不可更改参数。

/INNFOS/DebugQuery (ActuatorController_ROS::DebugQuery)
Function: 给用户查看一些不常用不可更改参数。
Input: 指定的执行器ID。
Output: 指定执行器的一部分不常用不可更改参数。 检查硬件时使用。

/INNFOS/Dictionary (ActuatorController_ROS::AttributeDictionary)
Function: 让用户查询参数的含义与使用方法
Input: 参数名 (比如MODE_ID)
Output: 参数的解释与使用方法

/INNFOS/IDChange (ActuatorController_ROS::IDModify)
Function: 更改一个执行器的ID
Input: 原执行器ID与新执行器ID
Output: 一个Bool告知用户操作是否成功

/INNFOS/ParametersSave (ActuatorController_ROS::ParametersSave)
Function: 把更改的参数下载到执行器上，确保以后开机也能生效
Input: 指定的执行器ID
Output: 一个Bool告知用户操作是否成功

/INNFOS/ZeroReset (ActuatorController_ROS::ZeroReset)
Function: 把当前的位置设置为执行器的零位，执行器当前必须被设置成Mode_Homing方能生效
Input: 指定的执行器ID
Output: 一个Bool告知用户操作是否成功