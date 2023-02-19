#include <ros/ros.h>
#include "gradient_updater.hpp"
#include "image_capturer.hpp"
#include "tool_detector.hpp"


int main(int argc, char** argv){
    ros::init(argc, argv, "update_ori_gradient");
    ros::NodeHandle nh;
    
    // image topics
    std::string img_topic1 = "/ptzcam/image_raw";
    std::string img_topic2 = "/webcam/image_raw";

    // detection setups
    visual_servo::ImageCapturer cam1(nh, img_topic1);
    visual_servo::ImageCapturer cam2(nh, img_topic2);

    visual_servo::ToolDetector detector_tool_cam1(nh, std::vector<int>{165, 80, 200, 180, 190, 225}); 
    visual_servo::ToolDetector detector_tool_cam2(nh, std::vector<int>{0, 145, 140, 7, 190, 230});

    visual_servo::ToolDetector detector_ori_cam1(nh, std::vector<int>{103, 220, 140, 109, 255, 179});
    visual_servo::ToolDetector detector_ori_cam2(nh, std::vector<int>{103, 130, 140, 109, 255, 179});

    std::vector<visual_servo::ToolDetector> detector_list{detector_tool_cam1, detector_tool_cam2, detector_ori_cam1, detector_ori_cam2, detector_ori_cam1, detector_ori_cam2};

    std::cout << "Done setups" << std::endl;

    int num_features = 4;

    cv::Point target1, target2;
    cv::Point target_tooltipPos1, target_tooltipPos2;
    cv::Point target_toolframePos1, target_toolframePos2;

    std::string T_topic = "/visual_servo/targets";

    std::string G_topic = "/visual_servo/orientation_gradient";

    std::cout << "Done topic assignment" << std::endl;

    visual_servo::GradientUpdater G_updater(nh, G_topic, T_topic);
    // visual_servo::GradientUpdater G_updater(nh, G_topic, target_ori);

    std::cout << "Done updater initialization" << std::endl;

    G_updater.mainLoop(cam1, cam2, detector_list, true);

    ros::shutdown();
    return 0;

}