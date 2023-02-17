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
    std::string img_topic2 = "/usb_cam/image_raw";

    // detection setups
    visual_servo::ImageCapturer cam1(nh, img_topic1);
    visual_servo::ImageCapturer cam2(nh, img_topic2);

    visual_servo::ToolDetector detector_blue_usbcam(nh, std::vector<int>{95, 110, 130, 110, 200, 200});
    visual_servo::ToolDetector detector_blue_cam(nh, std::vector<int>{106, 180, 180, 110, 200, 225});
    // visual_servo::ToolDetector detector_red(nh, std::vector<int>{0, 145, 100, 7, 190, 230});
    visual_servo::ToolDetector detector_red(nh, std::vector<int>{0, 145, 140, 7, 190, 230});
    // visual_servo::ToolDetector detector_red(nh, std::vector<int>{175, 170, 80, 180, 190, 225});

    // visual_servo::ToolDetector detector_purple(nh, std::vector<int>{113, 40, 170, 121, 150, 200});
    visual_servo::ToolDetector detector_purple(nh, std::vector<int>{113, 40, 170, 121, 120, 225});

    visual_servo::ToolDetector detector_purple_target(nh, std::vector<int>{113, 40, 170, 121, 150, 200});
    visual_servo::ToolDetector detector_yellow(nh, std::vector<int>{25, 40, 250, 35, 90, 255});
    // visual_servo::ToolDetector detector_pink(nh, std::vector<int>{165, 120, 130, 175, 135, 150});

    int count = 0;
    // detector_red.firstDetect(cam1);
    // detector_red.firstDetect(cam2);
    // detector_blue_cam.firstDetect(cam1);
    // detector_blue_usbcam.firstDetect(cam2);
    while(nh.ok()){ 
        count++;
        detector_purple.detect(cam1);
        cam1.saveCurrentImage("./cam", std::to_string(count)+".png");
        detector_purple.drawDetectRes();

        detector_purple.detect(cam2);
        cam2.saveCurrentImage("./usbcam", std::to_string(count)+".png");
        detector_purple.drawDetectRes();

        // detector_red.track(cam1, 0.25, 0.25);
        // cam1.saveCurrentImage("./cam", std::to_string(count)+".png");
        // detector_red.drawDetectRes();

        // detector_red.track(cam2, 0.25, 0.25);
        // cam2.saveCurrentImage("./usbcam", std::to_string(count)+".png");
        // detector_red.drawDetectRes();

        // detector_blue_cam.track(cam1, 0.25, 0.25);
        // cam1.saveCurrentImage("./cam", std::to_string(count)+".png");
        // detector_blue_cam.drawDetectRes();

        // detector_blue_usbcam.track(cam2, 0.25, 0.25);
        // cam2.saveCurrentImage("./usbcam", std::to_string(count)+".png");
        // detector_blue_usbcam.drawDetectRes();

        rate.sleep();
    }

    ros::shutdown();
    return 0;
}