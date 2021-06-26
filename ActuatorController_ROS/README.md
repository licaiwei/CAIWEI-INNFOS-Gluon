# ActuatorController_ROS
A standalone ROS package exclusively for INNFOS actuators
A ROS wrapper for INNFOS Actuator Controller





## How to Install
### ROS
If you have not already, follow the instructions on http://wiki.ros.org/ to install ROS version of your choice.

### Install
This package is tested on an X86 system with Ubuntu 16.04 installed.

Enter your catkin workspace's source directory. 
If you have workspaces that used the C++ `Ethernet Communication SDK`, it is recommended that you do NOT put this package in those workspaces.
This package uses a modified version of `Ethernet Communication SDK` and will likely cause issues in this and your program. <br>
You can clone the package with:
```
$ git clone https://github.com/Rayckey/ActuatorController_ROS.git actuatorcontroller_ros
```
Then return to the root of your workspace and build the package
```
$ catkin_make
```
There's a chance that the package may fail on the first time it is built depending on your system, if that happens, retry it couple times. <br>
Remember that whenever you need to use the node/messages/services in a new terminal, you must first `source` at the root of workspace:
```
$ source devel/setup.bash
```
Alternatively, you can source in your `.bashrc` file, which will force the terminal to source every time it boots up. <br>
Now you're ready to use the package!




## Usage Example
For a first time user, it is recommended to read through the INNFOS wiki page for the setup: https://innfos.github.io/wiki/en/#!index.md <br>
You can start the ROS node as soon as the device is connected to the communication bridge. <br>

In a new terminal, start ROS core:
```
$ roscore
```
The most common way to start the ROS node is to use `rosrun`, open a new terminal and start the node:
```
$ rosrun actuatorcontroller_ros innfos_actuator
```
To ensure all the actuators are connected, you should first check the lists of available actuators via the `/INNFOS/GeneralQuery` service:
```
$ rosservice call /INNFOS/GeneralQuery "isQuery: true" 
```
You should get a response such as:
```
ActuatorList: [2, 5]
ActuatorSwitch: [0, 0]
```
In this case, that there are two actuators connected, with IDs of 2 and 5, and neither of them are enabled. <br>
We can enable the actuators of your choice. Here we try to enable all the actuators in one go by using the ID 0:
```
$ rostopic pub -1 /INNFOS/enableActuator actuatorcontroller_ros/ActuatorArray "JointIDs:
- 0" 
``` 
It should be noted that there should not be any actuators with the ID 0. <br>
You can also only launch the actuator you wanted, simply change the `JointIDs` to the ID of your choice. <br>   
By default, the actuators when first launched will be in current mode, you can verify it via the service:
```
$ rosservice call /INNFOS/AttributeQuery "ActuatorID: 2"
``` 
You will get a response similar to:
```
ACTUAL_CURRENT: 0.00266915559769
ACTUAL_VELOCITY: -6.74343109131
ACTUAL_POSITION: -0.251586914062
MODE_ID: 1
ACTUATOR_SWITCH: True
ONLINE_STATUS: True
INIT_STATE: True
```
The `MODE_ID` variable is the active mode of the actuator. <br>
But what about the rest of the terms, what do they mean? <br>
If you come across a term that you need more clarifications of, you can call the service:
```
$ rosservice call /INNFOS/Dictionary "LookupTerm:
  data: 'MODE_ID'" 
```
You will get a detailed explanation for this parameter, like this:
````
TermType: 
  data: "Integer"
isChangeable: True
TermExplanation: 
  data: "The control mode of the actuator currently in effect. Options include: Mode_Cur (1),\
  \ Mode_Vel(2), Mode_Pos(3), Mode_Profile_Pos(4), Mode_Profile_Vel(5), Mode_Homing(6)"
````
That means that we can change the control modes of the actuator to 6 options. <br>
To enforce position control to the robot, you should first change its mode:
```
$ rostopic pub -1 /INNFOS/setControlMode actuatorcontroller_ros/ActuatorModes "JointIDs:
- 2
ActuatorMode: 4" 
``` 
Here we changed the control mode of the actuator 2 to `Mode_Profile_Pos`, which allows us to control the actuator's position with local planner. <br>
Now the position commands will take effects:
```
$ rostopic pub -1 /INNFOS/setTargetPosition actuatorcontroller_ros/ActuatorCommand "JointID: 2
TargetValue: 0.0"
```
You should be able to see the actuator moving now. <br>
Maybe you just got spooked because the actuator was moving way too fast for its own good. Well good news, next we will limit the it's maximum velocity using `rosparam` <br>
First, we can check the acceleration used in `Mode_Profile_pos`:
```
$ rosparam get /INNFOS/Actuator/2/PROFILE_POS_ACC
``` 
And we got the results:
```
2000.0
```
Yikes, that is one quick actuator, you may have a different value, nonetheless let's change it to a more reasonable value:
```
$ rosparam set /INNFOS/Actuator/2/PROFILE_POS_ACC 800.0
```
If it worked, the actuator node should have this output:
```
[ loginfo] [1566789998.845573133]: Change Parameter PROFILE_POS_ACC for actuator 2 to 800.000000
```
We also need to change a couple parameters:
```
$ rosparam set /INNFOS/Actuator/2/PROFILE_POS_DEC -800.0
$ rosparam set /INNFOS/Actuator/2/PROFILE_POS_MAX_SPEED 1200
```
And now you can set another target position can see how fast it moves!
```
$ rostopic pub -1 /INNFOS/setTargetPosition actuatorcontroller_ros/ActuatorCommand "JointID: 2
TargetValue: 10.0"
```
Now the actuator is running smoothly, but the actuator is in position "10" now, and it is not the "10" that we wanted. <br>
Say you want to adjust the `zero position` of the actuator, so you can change the actuator mode to `Mode_Homing`:
```
$ rostopic pub -1 /INNFOS/setControlMode actuatorcontroller_ros/ActuatorModes "JointIDs:
- 2
ActuatorMode: 6"
```
This will let the actuator enter `Mode_Cur` with an internal flag. If you check the `MODE_ID` now it may return as 1, 'fraid not ,just go ahead and put the actuator in your desired zero position by hand. <br>
When you are finished, keep the actuator steady at that position (or just leave it alone) and use the service:
```
$ rosservice call /INNFOS/ZeroReset "JointID: 2"
```
It the operation is successful, it will return this:
```
isSuccessful: True
```
Don't forget to change the position upper and lower limits now since they changed with the zero position:
```
$ rosparam set /INNFOS/Actuator/2/POS_LIMITATION_MAXIMUM 127
$ rosparam set /INNFOS/Actuator/2/POS_LIMITATION_MINIMUM -127
```
Don't forget that if you have changed the parameters, you will have to download them to the actuators for them to take effects next time they are booted up:
```
$ rosservice call /INNFOS/ParametersSave "ActuatorID: 2"
``` 
Similar to the `/INNFOS/ZeroReset` service, it will return a boolean.
```
isSuccessful: True
```
Awesome possum. You've learnt all the basics, now let's do a something a little fancy. <br>
You might have noticed that the actuator states are being broadcast via `joint_states` in the topic `INNFOS/actuator_states`. 
If you wish to command an arbitrary number of actuators at the same time, you can also use `joint_states` to send out commands. <br>
Let's get bring the long neglected actuator along:
```
$ rostopic pub -1 /INNFOS/setControlMode actuatorcontroller_ros/ActuatorModes "JointIDs: [2,5]
ActuatorMode: 4" 
```
Here we set both the actuators to `Mode_Profile_Pos`, and proceed to boss them around:
```
$ rostopic pub /INNFOS/actuator_targets sensor_msgs/JointState "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
name: ['2', '5']
position: [1,1]
velocity: [0,0]
effort: [0.0,0]" 
```
Now both the actuators have gone to position 1.0. <br>
`joint_states` is a common message type used in a lot of ROS packages, so using this method may makes your integration easier. <br>
You can control actuators with different modes in a single `joint_states`, if we set one of the actuator to `Mode_Profile_Vel`:
```
$ rostopic pub -1 /INNFOS/setControlMode actuatorcontroller_ros/ActuatorModes "JointIDs: [5]
ActuatorMode: 5" 
```
We can then send out messages like this:
```
$ rostopic pub /INNFOS/actuator_targets sensor_msgs/JointState "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
name: ['2', '5']
position: [2,0]
velocity: [0,250]
effort: [0.0,0]"
```
When the node receive a `joint_states` command, it will distinguish the active control mode of each actuators and only uses the correct target values from the messages. 
So if your actuator is in mode `Mode_Cur`, it will not respond to its position commands. <br>

And that's the basics, you can check all the options in the following sections.


## Launching the Node
There are a couple of ways to launch the nodes. The node will always have the basic functions, but the users may affect its performance by changing its parameters.
### rosrun with default options
To run the ros node separately and start actuators with the default options, as seen in the example
```
$ roscore
$ rosrun actuatorcontroller_ros innfos_actuator
```

### roslaunch with modified options
You can launch the node with custom parameters if you wish to change the performance of the controller. These launch files are mere examples and their effects can be combined.<br>
```
$ roslaunch actuatorcontroller_ros innfos_no_param.launch
```
This launch file adds the parameter `innfos_no_param` in parameter server. This option will prevent the node from polluting the parameter server when there are too many actuators connected, or prevent other users from altering these parameters.
```
$ roslaunch actuatorcontroller_ros innfos_fixed_100hz.launch
```
This launch file adds the parameter `innfos_fixed_rate` in parameter server.
When this parameter is present, the node will attempt to process the messages at a fixed rate. This allows for higher control rate or lower cpu usage. Please be informed that the actual control rate will be dependent on the equipment setup. 
```
$ roslaunch actuatorcontroller_ros innfos_use_cvp.launch
```
This launch file adds the parameter `innfos_use_cvp` in parameter server.
When this parameter is true, the controller will use a more efficient method when requesting the current states of the actuators. BUT the values will have a slight delay depending on the control rate. This is best used with the `innfos_fixed_rate` parameter. <br>

Beware! These settings will not take effects if the node is already running! (i.e. If you started the node and then added the parameters it will not take effects! These parameters must be present at the start of the node for it to work. Alternatively, you can create your own launch files that combine these parameters.)

## ROS Messages, Services & Parameters
### Parameters of an INNFOS actuator
The parameters of each actuator are separated into four groups: <br>
1.Frequently used modifiable parameters<br>
2.Frequently used unmodifiable parameters<br>
3.Infrequently used modifiable parameters<br>
4.Infrequently used unmodifiable parameters<br>
The most accessed parameters can be inquired or modified using ROS messages and services, the less used parameters are only accessible through the ROS Parameter server. The unmodifiable information can only be viewed using ROS services
You can easily see the messages/services typeis using:
```
$ rostopic type ${TOPIC_NAME}
$ rosservice type ${SERVICE_NAME}
```

### Published Topics

#### /INNFOS/actuator_states (`sensor_msgs::JointState`)
Provide all available actuators' positions, velocities and efforts when the actuators are enabled. Their units are `Rotations`, `RPM`, and `Amp` respectively. <br>

### Subscribed Topics
#### /INNFOS/enableActuator (`ActuatorController_ROS::ActuatorArray`)
Enable the designated actuators, if the input is empty or 0, the node will enable all available actuators. <br>

#### /INNFOS/disableActuator (`ActuatorController_ROS::ActuatorArray`)
Disable the designated actuators, if the input is empty or 0, the node  will disable all available actuators. <br>

#### /INNFOS/setControlMode (` ActuatorController_ROS::ActuatorModes`)
Set the control mode for the designated actuators, you can check the available modes by using the service `/INNFOS/Dictionary`. <br>

#### /INNFOS/setTargetPosition (`ActuatorController_ROS::ActuatorCommand`)
Set the target position for the designated actuator, will only have effects when the actuator is in the correct mode. <br>

#### /INNFOS/setTargetVelocity (`ActuatorController_ROS::ActuatorCommand`)
Set the target velocity for the designated actuator, will only have effects when the actuator is in the correct mode. <br>

#### /INNFOS/setTargetCurrent (`ActuatorController_ROS::ActuatorCommand`)
Set the target current for the designated actuator, will only have effects when the actuator is in the correct mode. <br>

#### /INNFOS/actuator_targets (`sensor_msgs::JointState`)
Receive bundled commands for positions, velocities and efforts, the preferred method for controlling a large number of actuators. The node will pull out the commands from the message based on the actuators nodes <br>



### Services
#### /INNFOS/GeneralQuery (`ActuatorController_ROS::GeneralQuery`)
Function: Allows users to lookup all available actuators and their status. <br>
Input: A placeholder variable, not needed <br>
Output: Provide a list of all available actuators and their status.<br>

#### /INNFOS/AttributeQuery (`ActuatorController_ROS::AttributeQuery`)
Function: Allow users to lookup some frequently used modifiable parameters of the actuators. To modify these parameters, please use the designated services/messages <br>
Input: The designated actuator ID. <br>
Output: Return a list of the frequently used parameters for the designated actuator.<br>

#### /INNFOS/TriviaQuery (`ActuatorController_ROS::TriviaQuery`)
Function: Allow users to lookup some frequently used unmodifiable parameters of the actuators. <br>
Input: The designated actuator ID. <br>
Output: Return a list of the frequently used unmodifiable parameters for the designated actuator.<br>

#### /INNFOS/DebugQuery (`ActuatorController_ROS::DebugQuery`) 
Function: Allow users to lookup some infrequently used unmodifiable parameters of the actuators. <br>
Input: The designated actuator ID. <br>
Output: Return a list of the infrequently used unmodifiable parameters for the designated actuator. Only for debugging. <br>

#### /INNFOS/Dictionary (`ActuatorController_ROS::AttributeDictionary`)
Function: Allows user to look up Attribute terms' meanings and usages. <br>
Input: The attribute term (i.e. `MODE_ID`) in string <br>
Output: The explanation and usage of the term or parameter. <br>

#### /INNFOS/IDChange (`ActuatorController_ROS::IDModify`)
Function: Change the ID of an actuator <br>
Input: The original ID, and the modified ID. <br>
Output: Will return a boolean to indicate whether this operation is successful. <br>

#### /INNFOS/ParametersSave (`ActuatorController_ROS::ParametersSave`)
Function: Permanently download the user's setting into the actuator, allows it to take effect the next time it is powered up. <br>
Input: The designated actuator ID. <br>
Output: Will return a boolean to indicate whether this operation is successful. <br>


#### /INNFOS/ZeroReset (`ActuatorController_ROS::ZeroReset`)
Function: Reset the actuator's absolute zero position to its current position, the actuator has to be in homing mode for it to take effects. (Due to some issues, the actuator may appear to be in current mode even after setting it to homing mode, but it will still have taken effects.) <br>
Input: The designated actuator ID. <br>
Output: Will return a boolean to indicate whether this operation is successful. <br>


### Parameters Server
The parameter server allows the user to check or modify the infrequently used modifiable parameters.<br>
Since each actuator has a number of modifiable parameters, the parameter names on the server are arranged in the format of : <br>
```
/INNFOS/Actuator/${ACTUATOR_ID}/${PARAMETER_NAME}
```
You can look up the explanation of the parameter using the service `/INNFOS/Dictionary`. <br>
Note that any changes will need to be saved using the `/INNFOS/ParametersSave` service for it to take effects in the next boot-up. <br>
If the assignment was unsuccessful, the parameter will be reverted on the server. <br>


## Change logs

<table style="width:500px"><thead><tr style="background:PaleTurquoise"><th style="width:100px">Version number</th><th style="width:150px">Update time</th><th style="width:3800px">Update content</th></tr></thead><tbody><tr><td>v1.1.0</td><td>2019.08.27</td><td> Joint states commands added </td></tr></thead><tbody><tr><td>v1.0.3</td><td>2019.08.26</td><td> More Examples</td></tr></thead><tbody><tr><td>v1.0.2</td><td>2019.08.21</td><td> Included an example in readme </td></tr></thead><tbody><tr><td>v1.0.1</td><td>2019.08.09</td><td>Added readme</td></tr></thead><tbody><tr><td>v1.0.0</td><td>2019.08.09</td><td>Node tested with actuators on Ubuntu 16.04 with ROS Luna, Stable release</td></tbody></table>
