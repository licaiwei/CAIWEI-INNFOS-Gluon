# 控制模式: 0, 1, 2
int16 mode
# 0:角度控制
sensor_msgs/JointState joint
# 1:点位控制 2:轨迹控制 3:速度控制
trajectory_msgs/JointTrajectory traj
# 4:直线插补
geometry_msgs/Pose[2] pose