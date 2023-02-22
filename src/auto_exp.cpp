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

#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <visual_servo_controller.hpp>
#include <pixel_picker.hpp>
#include <image_capturer.hpp>
#include <tool_detector.hpp>
#include <Jacobian_updater.hpp>


// exp parameters
int num_targets = 3;
int num_exps = 1;

int num_features = 4;
int dof = 3;

std::vector<double> targets_raw(num_targets*num_features*2);

void home(ros::NodeHandle& nh){
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate rate(1000);

    // moveit setups for planning
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    moveit::core::RobotStatePtr current_state =  move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    const std::vector<std::string> joint_names = joint_model_group->getActiveJointModelNames();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    move_group.setMaxVelocityScalingFactor(0.02);
    move_group.setMaxAccelerationScalingFactor(0.01);

    // get ros params
    for (int i=0;i<joint_group_positions.size();++i){
        std::string name = "/home_position/"+joint_names[i];
        if (nh.getParam(name, joint_group_positions[i])){
            ROS_INFO("Got param: %s", name.c_str());
            std::cout << joint_group_positions[i] << std::endl;
        }
        else{
            ROS_ERROR("No parameter named %s!", name.c_str());
        }
    }

    // set joint space goal
    move_group.setJointValueTarget(joint_group_positions);

    // plan 
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("robot_homing", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

    //move the robot
    move_group.move();
}

void screw(ros::NodeHandle& nh){
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate rate(1000);

    // moveit setups for planning
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    moveit::core::RobotStatePtr current_state =  move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    const std::vector<std::string> joint_names = joint_model_group->getActiveJointModelNames();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    move_group.setMaxVelocityScalingFactor(0.02);
    move_group.setMaxAccelerationScalingFactor(0.01);

    for (int i=0; i<joint_names.size(); ++i){
        if (joint_names[i]=="wrist_3_joint"){
            joint_group_positions[i]+=M_PI/2;
            std::cout << "reach" << std::endl;
            break;
        }
    }

    // set joint space goal
    move_group.setJointValueTarget(joint_group_positions);

    // plan 
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("robot_homing", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

    //move the robot
    move_group.move();
}


void targetCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
    targets_raw = msg->data;
    // std::cout << targets_raw.size() << std::endl;
}


void flat2eigenVec(Eigen::VectorXd& V, std::vector<double> flat){
    // check input
    int size_flat = flat.size();
    int size_V = V.size();
    if (size_flat!=size_V){
        ROS_ERROR("Dimension inconsistent! Cannot convert to vector (Eigen3).");
        return;
    }
    for (int i=0; i<size_V; ++i){
        V(i) = flat[i];
    }
}

int main(int argc, char** argv){
    // ros init
    ros::init(argc, argv, "auto_exp");
    ros::NodeHandle nh;
    ros::Rate rate(100);
    ros::AsyncSpinner spinner(4);
    spinner.start();

    // tolerance
    double tol_pos = 5.0, tol_ori = 0.05, pred_thred = 100.0, pred_thred_ori = 0.48;
    // double tol_pos = 5.0, tol_ori = 0.05, pred_thred = 50.0, pred_thred_ori = 0.32;

    // paths
    std::string data_root = "/home/sanaria/users/yifan/";

    // topics
    // std::string J_stop_topic = "/visual_servo/Jacobian_update_stop";
    std::string G_stop_topic = "/visual_servo/gradient_update_stop";
    std::string T_topic = "/visual_servo/targets";

    // publishers
    // ros::Publisher J_stop_pub = nh.advertise<std_msgs::Bool>(J_stop_topic, 1000);
    ros::Publisher G_stop_pub = nh.advertise<std_msgs::Bool>(G_stop_topic, 1000);

    // subscribers
    ros::Subscriber T_sub = nh.subscribe(T_topic, 1, &targetCallback);

    // MOVEIT planning setups
    static const std::string PLANNING_GROUP_ARM = "manipulator";
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
    const robot_state::JointModelGroup* joint_model_group =
            move_group_interface_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
    move_group_interface_arm.setMaxVelocityScalingFactor(0.005);
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

    // image topics
    std::string img_topic1 = "/ptzcam/image_raw";
    std::string img_topic2 = "/webcam/image_raw";

    // cameras
    visual_servo::ImageCapturer cam1(nh, img_topic1);
    visual_servo::ImageCapturer cam2(nh, img_topic2);

    // detectors
    visual_servo::ToolDetector detector_tool(nh);

    visual_servo::ToolDetector detector_ori_cam1(nh, std::vector<int>{100, 170, 140, 109, 255, 255}); // blue night
    visual_servo::ToolDetector detector_ori_cam2(nh, std::vector<int>{100, 100, 140, 109, 255, 179});

    std::vector<visual_servo::ToolDetector> detector_list{detector_tool, detector_ori_cam1, detector_ori_cam2};

    // take images
    cv::Mat img_target1 = cam1.getCurrentImage();
    cv::Mat img_target2 = cam2.getCurrentImage();

    // servo controller
    std::unique_ptr<visual_servo::VisualServoController> servo_controller;

    Eigen::VectorXd increment;
    increment.resize(dof);

    std::string name1, name2;

    // img saving path
    std::string img_path = data_root+"img/";

    // msgs
    std_msgs::Bool stopmsg;
    stopmsg.data = true;

    // targets
    Eigen::VectorXd target_full;
    target_full.resize(2*num_features);

    // loop for num_exps times
    for (int j=0; j<num_exps; ++j){
        for (int i=0; i<num_targets; ++i){ 
            std::cout << "Homing ..." << std::endl;
            // go to home pos
            home(nh);
            // loop until converges
            Eigen::VectorXd increment;
            increment.resize(dof);

            // setup current target vector
            std::vector<double> temp(targets_raw.begin() + 2*num_features*i, targets_raw.begin() + 2*num_features*(i+1));
            std::cout << temp.size() << std::endl;
            flat2eigenVec(target_full, temp);

            std::cout << "target_full: \n" << target_full << std::endl;

            // orientation servo
            if (!(j==0&&i==0)){
                // initialize J for ori
                ros::Time begin = ros::Time::now();
                ros::Duration duration = ros::Duration(0.5); 
                ros::Time end = begin + duration;
                // publish for certain period of time
                while(ros::Time::now() < end){
                    G_stop_pub.publish(stopmsg);
                }
                ros::Duration(0.5).sleep(); 
            }

            servo_controller.reset(new visual_servo::VisualServoController(nh, target_full, true, tol_pos, tol_ori, pred_thred, pred_thred_ori));
            
            while(nh.ok()&&(!servo_controller->stopSign())){
                servo_controller->uniOriDirectionIncrement(increment, cam1, cam2, detector_list, true, true);
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

            servo_controller.reset(new visual_servo::VisualServoController(nh, target_full, true, tol_pos, tol_ori, pred_thred, pred_thred_ori));

            // position servo
            while(nh.ok()&&(!servo_controller->stopSign())){
                // servo_controller->directionIncrement(increment, cam1, cam2, detector_list); // dl off
                servo_controller->directionIncrement(increment, cam1, cam2, detector_tool, true); // dl on
                std::cout << "Done increment" << std::endl;
                target_pose.position.x = target_pose.position.x+increment(0);
                target_pose.position.y = target_pose.position.y+increment(1);
                target_pose.position.z = target_pose.position.z+increment(2);
                move_group_interface_arm.setPoseTarget(target_pose);
                move_group_interface_arm.move();
            }
            // take and save images
            cv::Mat img_target1 = cam1.getCurrentImage();
            cv::Mat img_target2 = cam2.getCurrentImage();
            name1 = img_path+"cam1"+std::to_string(i)+".png";
            name2 = img_path+"cam2"+std::to_string(i)+".png";
            cv::imwrite(name1, img_target1);
            cv::imwrite(name2, img_target2);

            // screwing
            ROS_INFO("Start screwing ...");
            screw(nh);
        }
        // data_logger.close();
    }

    ros::shutdown();
    return 0;
}
