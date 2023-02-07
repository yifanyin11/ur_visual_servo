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
    visual_servo::ToolDetector detector_toolcenter(nh, std::vector<int>{150, 150, 150, 160, 255, 255});
    visual_servo::ToolDetector detector_tooltip(nh, std::vector<int>{20, 100, 100, 30, 255, 255});
    // visual_servo::ToolDetector detector_blue(nh, std::vector<int>{111, 60, 70, 117, 80, 110});
    visual_servo::ToolDetector detector_blue(nh, std::vector<int>{115, 60, 80, 118, 80, 100});

    visual_servo::ToolDetector detector_red(nh, std::vector<int>{0, 100, 100, 5, 255, 255});
    visual_servo::ToolDetector detector_target_tip1(nh, std::vector<int>{130, 30, 30, 140, 255, 120});
    visual_servo::ToolDetector detector_target_tip2(nh, std::vector<int>{76, 100, 150, 96, 255, 255});

    int count = 0;

    while(nh.ok()){
        count++;
        detector_blue.detect(cam1);
        cam1.saveCurrentImage("./", std::to_string(count)+".png");
        detector_blue.drawDetectRes();

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