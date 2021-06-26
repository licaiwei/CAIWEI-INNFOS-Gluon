#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <six_arm_hand_demo/gluon_demo_msgs.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>


namespace rvt = rviz_visual_tools;
moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");

six_arm_hand_demo::gluon_demo_msgs::*traj = &six_arm_hand_demo::gluon_demo_msgs::traj;


void callback(const six_arm_hand_demo::gluon_demo_msgs commond)
{
    // 清除显示的东西
    visual_tools.deleteAllMarkers();
    // Remote 控制，可以通过Rviz按钮实现脚本
    visual_tools.loadRemoteControl();

    // text
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "文字", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();     // 触发 publish
    visual_tools.prompt("显示文字");

    // plans
    //visual_tools.publishAxisLabeled(target_pose1, "pose1");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine( commond.traj, "wristPitch_Link", "arm");
    visual_tools.trigger();
    visual_tools.prompt("显示轨迹");

    // Visualize the plan in RViz
    /*
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();
    visual_tools.prompt("显示路径点");
    */
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::Subscriber traj_sub = node_handle.subscribe("/manipulator/command", 1000, callback);
    ros::spin();
    return 0;
}