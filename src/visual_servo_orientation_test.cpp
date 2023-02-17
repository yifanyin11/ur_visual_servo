#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit_msgs/RobotTrajectory.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "visual_servo_controller.cpp"
#include "image_capturer.cpp"
#include "tool_detector.cpp"
#include "Jacobian_updater.cpp"

int main(int argc, char** argv){
    // Ros setups
    ros::init(argc, argv, "visual_servo_orientation_test");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    // MOVEIT planning setups
    static const std::string PLANNING_GROUP_ARM = "manipulator";
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
    const robot_state::JointModelGroup* joint_model_group =
            move_group_interface_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
    move_group_interface_arm.setMaxVelocityScalingFactor(0.0025);
    move_group_interface_arm.setMaxAccelerationScalingFactor(0.01);
    bool success;

    // get current pose
    geometry_msgs::PoseStamped current_pose;
    current_pose = move_group_interface_arm.getCurrentPose("tool0");
    // define target pose and initialize it as current pose
    geometry_msgs::Pose target_pose;
    target_pose.orientation = current_pose.pose.orientation;
    target_pose.position = current_pose.pose.position;
    // get current orientation
    double x, y, z, roll, pitch, yaw;
    x = target_pose.position.x;
    y = target_pose.position.y;
    z = target_pose.position.z;
    tf::Quaternion q_target;
    tf::Transform transform_target;
    visual_servo::JacobianUpdater::poseMsg2Transform(transform_target, target_pose);
    tf::Matrix3x3(transform_target.getRotation()).getRPY(roll, pitch, yaw);

    ros::Rate rate(10);
    // target topics
    std::string t_topic = "/visual_servo/targets";
    // image topics
    std::string img_topic1 = "/visual_servo/camera1/image_raw_1";
    std::string img_topic2 = "/visual_servo/camera2/image_raw_2";

    // detection setups
    visual_servo::ImageCapturer cam1(nh, img_topic1);
    visual_servo::ImageCapturer cam2(nh, img_topic2);

    visual_servo::ToolDetector detector_toolcenter(nh, std::vector<int>{150, 150, 150, 160, 255, 255});
    visual_servo::ToolDetector detector_tooltip(nh, std::vector<int>{20, 100, 100, 30, 255, 255});
    visual_servo::ToolDetector detector_frametip(nh, std::vector<int>{100, 150, 100, 110, 255,255});

    visual_servo::ToolDetector detector_target(nh, std::vector<int>{0, 100, 100, 5, 255, 255});
    visual_servo::ToolDetector detector_target_frametip(nh, std::vector<int>{130, 30, 30, 140, 255, 120});
    visual_servo::ToolDetector detector_target_tooltip(nh, std::vector<int>{76, 100, 150, 96, 255, 255});

    std::cout << "Done setups" << std::endl;

    int num_features = 4;
    int dof = 6;

    cv::Point target1, target2;
    cv::Point target_tooltipPos1, target_tooltipPos2;
    cv::Point target_toolframePos1, target_toolframePos2;

    // visual_servo::VisualServoController servo_controller(nh, t_topic, 5, 0.05);
    visual_servo::VisualServoController servo_controller(nh, t_topic, 5, 0.05);

    std::cout << "Done initialize servo controller" << std::endl;

    Eigen::VectorXd increment;
    increment.resize(dof);

    std::vector<visual_servo::ToolDetector> detector_list{detector_toolcenter, detector_tooltip, detector_frametip};

    while(nh.ok()&&(!servo_controller.stopSign())){
        servo_controller.uniOriDirectionIncrement(increment, cam1, cam2, detector_list);
        // servo_controller.oriDirectionIncrement(increment, cam1, cam2, detector_list);
        std::cout << "Done increment" << std::endl;

        transform_target.setOrigin(tf::Vector3(x, y, z));

        roll = roll+increment(0);
        pitch = pitch+increment(1);
        yaw = yaw+increment(2);
        q_target.setRPY(roll, pitch, yaw);
        q_target.normalize();
        transform_target.setRotation(q_target);

        visual_servo::JacobianUpdater::transform2PoseMsg(transform_target, target_pose);

        move_group_interface_arm.setPoseTarget(target_pose);
        move_group_interface_arm.move();
    }

}
