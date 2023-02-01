// This script is the test of asynchonous move mode of MoveIt! interface
/* Author: Yifan Yin */
#include <iostream>
#include <fstream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
    // ROS setups
    ros::init(argc, argv, "pick_and_place");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP_ARM = "manipulator";
    
    // Planning setups
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    planning_scene_monitor::LockedPlanningSceneRW scene(visual_tools.getPlanningSceneMonitor());

    scene->getCurrentStateNonConst().update();
    robot_state::RobotState cur_state = scene->getCurrentStateNonConst();   

    robot_model::RobotModelConstPtr robot_model = scene->getRobotModel();

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);

    const robot_state::JointModelGroup* joint_model_group =
            move_group_interface_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

    // Print all available groups in the robot
    ROS_INFO_NAMED("pick_and_place", "Available Planning Groups:");
    std::copy(move_group_interface_arm.getJointModelGroupNames().begin(),
            move_group_interface_arm.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;

    move_group_interface_arm.setMaxVelocityScalingFactor(0.3);
    move_group_interface_arm.setMaxAccelerationScalingFactor(0.01);
    
    // 2. Place the TCP to target 1 (above the blue box)
    geometry_msgs::PoseStamped current_pose;
    current_pose = move_group_interface_arm.getCurrentPose("ee_link");

    geometry_msgs::Pose target_pose;

    target_pose.orientation = current_pose.pose.orientation;
    target_pose.position.x = 0.3;
    target_pose.position.y = 0.5;
    target_pose.position.z = 0.3;

    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("pick_and_place", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_arm.asyncMove();

    std::cout << "Start sleeping for 1.0 second ..." << std::endl;

    std::cout << "Done sleeping; check TCP position" << std::endl;
    std::cout << "Ready to move to target 2" << std::endl;

    // 3. Move the TCP to target 2 (center of the box)
    target_pose.position.z = target_pose.position.z - 0.15;
    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("pick_and_place", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_arm.asyncMove();

    std::cout << "Done sleeping; check TCP position" << std::endl;
    std::cout << "Ready to move to target 3" << std::endl;

    // 4. Move the TCP to target 3 (raise it from the box)
    target_pose.position.z = target_pose.position.z + 0.3;
    move_group_interface_arm.setPoseTarget(target_pose);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("pick_and_place", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_arm.asyncMove();

    std::cout << "Start sleeping for 1.0 second ..." << std::endl;

    ros::Duration(1.0).sleep();

    std::cout << "Done sleeping; check TCP position" << std::endl;

    ros::shutdown();
    return 0;
}
