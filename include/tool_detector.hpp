#ifndef TOOL_DETECTOR_H
#define TOOL_DETECTOR_H

#include <ros/ros.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_capturer.hpp>
#include <dl_service/dl_detection.h>

namespace visual_servo{
    class ToolDetector{
    private:
        ros::NodeHandle nh;
        cv::Mat image;
        cv::Point2d tool_center;
        cv::Point2d corner1;
        cv::Point2d corner2;
        cv::Scalar lower_hsv;
        cv::Scalar upper_hsv;

        ros::ServiceClient cli;

    public:
        friend class JacobianUpdater; 
        friend class VisualServoController; 
        // constructor
        ToolDetector(ros::NodeHandle& nh, std::vector<int> hsv_range, bool dl_on=false);
        ToolDetector(ros::NodeHandle& nh); // for dl version
        // destructor
        ~ToolDetector(){};
        // mutators
        // accessors
        cv::Mat getSourceImage(ImageCapturer& cam);
        cv::Point2d getCenter();
        // functions
        void firstDetect(ImageCapturer& cam);
        void detect(ImageCapturer& cam); // update source image with the cur frame from cam, perform detection using that image, update tool_center
        void detect(cv::Mat& img); 
        void dlDetect(cv::Mat& img, cv::Point2d& drivertip, cv::Point2d& screwcup);
        void dlDetect(ImageCapturer& cam, cv::Point2d& drivertip, cv::Point2d& screwcup);
        void drawDetectRes(); 
        void drawDetectRes(cv::Mat img); 
        void drawDetectRes(cv::Mat img, cv::Point2d point); 
    };
} // namespace visual_servo

#endif  