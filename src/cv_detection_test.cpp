#include <ros/ros.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include "image_capturer.hpp"
#include "tool_detector.hpp"

int main(int argc, char** argv){
    // ros setups
    ros::init(argc, argv, "cv_detection_test");
    ros::NodeHandle nh;
    ros::Rate rate(1000);

    // image topics
    std::string img_topic1 = "/camera/image_raw";
    // std::string img_topic2 = "/visual_servo/camera2/image_raw_2";

    // detection setups
    visual_servo::ImageCapturer cam1(nh, img_topic1);
    // visual_servo::ImageCapturer cam2(nh, img_topic2);

    visual_servo::ToolDetector detector_blue(nh, std::vector<int>{115, 60, 80, 118, 80, 100});
    visual_servo::ToolDetector detector_red(nh, std::vector<int>{0, 120, 110, 5, 160, 190});
    // visual_servo::ToolDetector detector_green(nh, std::vector<int>{60, 30, 100, 80, 60, 130});
    // visual_servo::ToolDetector detector_brown(nh, std::vector<int>{5, 70, 60, 10, 90, 80});
    visual_servo::ToolDetector detector_orange(nh, std::vector<int>{5, 120, 160, 10, 140, 170});
    visual_servo::ToolDetector detector_pink(nh, std::vector<int>{165, 120, 130, 175, 135, 150});

    int count = 0;
    detector_pink.firstDetect(cam1);
    while(nh.ok()){ 
        count++;
        detector_pink.track(cam1, 0.25, 0.25);
        cam1.saveCurrentImage("./", std::to_string(count)+".png");
        detector_pink.drawDetectRes();

        // detector_target.detect(cam2);
        // detector_target.drawDetectRes();

        // detector_target_tip1.detect(cam1);
        // detector_target_tip1.drawDetectRes();
        // detector_target_tip1.detect(cam2);
        // detector_target_tip1.drawDetectRes();

        // detector_target_tip2.detect(cam1);
        // detector_target_tip2.drawDetectRes();
        // detector_target_tip2.detect(cam2);
        // detector_target_tip2.drawDetectRes();
        rate.sleep();
    }

    ros::shutdown();
    return 0;
}