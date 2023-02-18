#include <ros/ros.h>
#include "Jacobian_updater.hpp"
#include "image_capturer.hpp"
#include "tool_detector.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "update_image_J");
    ros::NodeHandle nh;

    // image topics
    std::string img_topic1 = "/webcam/image_raw";
    std::string img_topic2 = "/ptzcam/image_raw";

    // detection setups
    visual_servo::ImageCapturer cam1(nh, img_topic1);
    visual_servo::ImageCapturer cam2(nh, img_topic2);

    visual_servo::ToolDetector detector_tool_cam1(nh, std::vector<int>{165, 80, 200, 180, 190, 225}); // red
    visual_servo::ToolDetector detector_tool_cam2(nh, std::vector<int>{0, 145, 140, 7, 190, 230});

    std::vector<visual_servo::ToolDetector> detector_list{detector_tool_cam1, detector_tool_cam2};

    std::string J_topic = "/visual_servo/image_Jacobian";
    visual_servo::JacobianUpdater J_updater(nh, J_topic);

    J_updater.mainLoopPos(cam1, cam2, detector_list);
    // J_updater.mainLoop(cam1, cam2, detector_list);

    ros::shutdown();
    return 0;
}

