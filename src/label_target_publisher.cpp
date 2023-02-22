#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include <std_msgs/Float64MultiArray.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <visual_servo_controller.hpp>
#include <pixel_picker.hpp>
#include <image_capturer.hpp>


void vecEigen2std(Eigen::VectorXd& ve, std::vector<double>& vs){
    if (ve.size()!=vs.size()) vs.resize(ve.size());
    for (int i=0; i<ve.size(); ++i){
        vs[i] = ve(i);
    }
}

void getToolRot(Eigen::VectorXd& toolRot, cv::Point2d& center1, cv::Point2d& tooltip1, cv::Point2d& frametip1, cv::Point2d& center2, cv::Point2d& tooltip2, cv::Point2d& frametip2){
    double theta11 = atan2((tooltip1-center1).y, (tooltip1-center1).x); 
    double theta12 = atan2((frametip1-center1).y, (frametip1-center1).x); 
    double theta21 = atan2((tooltip2-center2).y, (tooltip2-center2).x); 
    double theta22 = atan2((frametip2-center2).y, (frametip2-center2).x); 
    toolRot << theta11, theta12, theta21, theta22;
}

int main(int argc, char** argv){
    // Ros setups
    ros::init(argc, argv, "label_target_publisher");
    ros::NodeHandle nh;
    std::string target_topic = "/visual_servo/targets";
    ros::Publisher target_pub = nh.advertise<std_msgs::Float64MultiArray>(target_topic, 1);

    ros::Rate rate(10);

    int num_features = 4;
    
    // image topics
    std::string img_topic1 = "/ptzcam/image_raw";
    std::string img_topic2 = "/webcam/image_raw";

    // detection setups
    visual_servo::ImageCapturer cam1(nh, img_topic1);
    visual_servo::ImageCapturer cam2(nh, img_topic2);

    // take images
    cv::Mat img_target1 = cam1.getCurrentImage();
    cv::Mat img_target2 = cam2.getCurrentImage();

    // set targets by manual annotation
    PixelPicker picker;
    picker.set_shrink_height(1.0);

    Eigen::VectorXd target_pos, target_ori, target_full;
    target_pos.resize(num_features);
    target_ori.resize(num_features);
    target_full.resize(num_features*2);

    std::cout << "Pick top center for the target in cam1 ..." << std::endl;
    cv::Point2d target1 = picker.pickOne(img_target1);
    std::cout << "Pick base for the target in cam1 ..." << std::endl;
    cv::Point2d target_tip1 = picker.pickOne(img_target1);
    std::cout << "Pick top center for the target in cam2 ..." << std::endl;
    cv::Point2d target2 = picker.pickOne(img_target2);
    std::cout << "Pick base for the target in cam2 ..." << std::endl;
    cv::Point2d target_tip2 = picker.pickOne(img_target2);

    target_pos << target1.x, target1.y, target2.x, target2.y;

    target_tip1 = 2*target1-target_tip1;
    target_tip2 = 2*target2-target_tip2;

    getToolRot(target_ori, target1, target_tip1, target_tip1, target2, target_tip2, target_tip2);
    target_full << target_pos, target_ori;

    std::cout << "Done initialize targets" << target_full << std::endl;

    std_msgs::Float64MultiArray Tmsgs;

    std::vector<double> targets_std(target_full.size(), 0.0);
    vecEigen2std(target_full, targets_std);

    Tmsgs.data = targets_std;

    while(nh.ok()){
        target_pub.publish(Tmsgs);
        ros::spinOnce();
        rate.sleep();
    }

}
