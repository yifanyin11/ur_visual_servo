#include <ros/ros.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <dynamic_reconfigure/server.h>
#include <ur_visual_servo/ColorsegConfig.h>
#include "image_capturer.hpp"
#include "tool_detector.hpp"


int cam1_h_low, cam1_s_low, cam1_v_low, cam1_h_high, cam1_s_high, cam1_v_high;
int cam2_h_low, cam2_s_low, cam2_v_low, cam2_h_high, cam2_s_high, cam2_v_high;

void callback(ur_visual_servo::ColorsegConfig &config, uint32_t level) {
    cam1_h_low = config.cam1_h_low;
    cam1_s_low = config.cam1_s_low;
    cam1_v_low = config.cam1_v_low;
    cam1_h_high = config.cam1_h_high;
    cam1_s_high = config.cam1_s_high;
    cam1_v_high = config.cam1_v_high;

    cam2_h_low = config.cam2_h_low;
    cam2_s_low = config.cam2_s_low;
    cam2_v_low = config.cam2_v_low;
    cam2_h_high = config.cam2_h_high;
    cam2_s_high = config.cam2_s_high;
    cam2_v_high = config.cam2_v_high;
}

int main(int argc, char** argv){
    // ros setups
    ros::init(argc, argv, "cv_detection_test");
    ros::NodeHandle nh;
    ros::Rate rate(2000);

    dynamic_reconfigure::Server<ur_visual_servo::ColorsegConfig> server;
    dynamic_reconfigure::Server<ur_visual_servo::ColorsegConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    // image topics
    std::string img_topic1 = "/ptzcam/image_raw";
    std::string img_topic2 = "/webcam/image_raw";

    // detection setups
    visual_servo::ImageCapturer cam1(nh, img_topic1);
    visual_servo::ImageCapturer cam2(nh, img_topic2);

    // visual_servo::ToolDetector detector_red_cam1(nh, std::vector<int>{165, 80, 200, 180, 190, 225}); // night
    // visual_servo::ToolDetector detector_red_cam2(nh, std::vector<int>{0, 145, 140, 7, 190, 230});

    // visual_servo::ToolDetector detector_red_cam1(nh, std::vector<int>{165, 60, 200, 180, 190, 255}); // day
    // visual_servo::ToolDetector detector_red_cam2(nh, std::vector<int>{0, 135, 140, 10, 190, 255});

    // visual_servo::ToolDetector detector_blue_cam1(nh, std::vector<int>{103, 220, 140, 109, 255, 179}); // night
    // visual_servo::ToolDetector detector_blue_cam2(nh, std::vector<int>{103, 130, 140, 109, 255, 179});

    // visual_servo::ToolDetector detector_blue_cam1(nh, std::vector<int>{100, 170, 140, 109, 255, 255}); // day
    // visual_servo::ToolDetector detector_blue_cam2(nh, std::vector<int>{100, 100, 140, 109, 255, 179});

    // visual_servo::ToolDetector detector_purple_cam1(nh, std::vector<int>{120, 30, 200, 145, 90, 255});
    // visual_servo::ToolDetector detector_purple_cam2(nh, std::vector<int>{130, 25, 170, 160, 60, 250});

    // visual_servo::ToolDetector detector_red(nh, std::vector<int>{0, 60, 140, 9, 190, 255});
    // visual_servo::ToolDetector detector_red(nh, std::vector<int>{175, 170, 80, 180, 190, 225});

    int count = 0;
    cv::Point2d tip1, tip2;
    // cv::Mat img;
    // img = cv::imread("/home/sanaria/Downloads/train1_91.png");

    // detector_purple.dlDetect(img, tip1, tip2);
    // detector_purple.drawDetectRes(img, tip1);
    // cv::destroyAllWindows();

    while(nh.ok()){ 
        // visual_servo::ToolDetector detector_cam1(nh, std::vector<int>{cam1_h_low, cam1_s_low, cam1_v_low, cam1_h_high, cam1_s_high, cam1_v_high}, true);
        // visual_servo::ToolDetector detector_cam2(nh, std::vector<int>{cam2_h_low, cam2_s_low, cam2_v_low, cam2_h_high, cam2_s_high, cam2_v_high}, true);
        visual_servo::ToolDetector detector_cam1(nh, std::vector<int>{cam1_h_low, cam1_s_low, cam1_v_low, cam1_h_high, cam1_s_high, cam1_v_high});
        visual_servo::ToolDetector detector_cam2(nh, std::vector<int>{cam2_h_low, cam2_s_low, cam2_v_low, cam2_h_high, cam2_s_high, cam2_v_high});
        count++;

        detector_cam1.detect(cam1);
        // detector_cam1.dlDetect(cam1, tip1, tip2);
        cam1.saveCurrentImage("./cam", std::to_string(count)+".png");
        detector_cam1.drawDetectRes();
        // detector_cam1.drawDetectRes(cam1.getCurrentImage(), tip1);

        detector_cam2.detect(cam2);
        // detector_cam2.dlDetect(cam2, tip1, tip2);
        cam2.saveCurrentImage("./usbcam", std::to_string(count)+".png");
        detector_cam2.drawDetectRes();
        // detector_cam2.drawDetectRes(cam2.getCurrentImage(), tip1);

        ros::spinOnce();

        rate.sleep();
    }

    ros::shutdown();
    return 0;
}