#include <ros/ros.h>
#include "Jacobian_updater.hpp"
#include "image_capturer.hpp"
#include "tool_detector.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "update_image_J");
    ros::NodeHandle nh;

    // image topics
    std::string img_topic1 = "/ptzcam/image_raw";
    std::string img_topic2 = "/webcam/image_raw";

    // detection setups
    visual_servo::ImageCapturer cam1(nh, img_topic1);
    visual_servo::ImageCapturer cam2(nh, img_topic2);

    // visual_servo::ToolDetector detector_tool_cam1(nh, std::vector<int>{165, 80, 200, 180, 190, 225}); // red night
    // visual_servo::ToolDetector detector_tool_cam2(nh, std::vector<int>{0, 145, 140, 7, 190, 230});

    visual_servo::ToolDetector detector_tool_cam1(nh, std::vector<int>{165, 60, 200, 180, 190, 255}, true); // red day
    visual_servo::ToolDetector detector_tool_cam2(nh, std::vector<int>{0, 135, 140, 10, 190, 255}, true);

    std::vector<visual_servo::ToolDetector> detector_list{detector_tool_cam1, detector_tool_cam2};

    std::string J_topic = "/visual_servo/image_Jacobian";
    visual_servo::JacobianUpdater J_updater(nh, J_topic);

    J_updater.mainLoopPos(cam1, cam2, detector_list[0], true);
    // J_updater.mainLoopPos(cam1, cam2, detector_list);
    // J_updater.mainLoop(cam1, cam2, detector_list);

    ros::shutdown();
    return 0;
}

