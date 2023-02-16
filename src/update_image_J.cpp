#include <ros/ros.h>
#include "Jacobian_updater.hpp"
#include "image_capturer.hpp"
#include "tool_detector.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "update_image_J");
    ros::NodeHandle nh;

    // image topics
    std::string img_topic1 = "/camera/image_raw";
    std::string img_topic2 = "/usb_cam/image_raw";

    // detection setups
    visual_servo::ImageCapturer cam1(nh, img_topic1);
    visual_servo::ImageCapturer cam2(nh, img_topic2);

    // visual_servo::ToolDetector detector_red(nh, std::vector<int>{0, 100, 100, 5, 255, 255});
    visual_servo::ToolDetector detector_red(nh, std::vector<int>{0, 145, 100, 7, 190, 230});

    std::string J_topic = "/visual_servo/image_Jacobian";
    visual_servo::JacobianUpdater J_updater(nh, J_topic);

    J_updater.mainLoopPos(cam1, cam2, detector_red);
    // J_updater.mainLoop(cam1, cam2, detector_list);

    ros::shutdown();
    return 0;
}

