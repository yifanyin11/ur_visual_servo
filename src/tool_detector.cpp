#include "tool_detector.hpp"

visual_servo::ToolDetector::ToolDetector(ros::NodeHandle& nh, std::vector<int> hsv_range, bool dl_on):
nh(nh){
    tool_center.x = -1.0;
    tool_center.y = -1.0;
    corner1.x = -1.0;
    corner1.y = -1.0;
    corner2.x = -1.0;
    corner2.y = -1.0;
    lower_hsv = cv::Scalar(hsv_range[0], hsv_range[1], hsv_range[2]);
    upper_hsv = cv::Scalar(hsv_range[3], hsv_range[4], hsv_range[5]);
    if (dl_on){
        cli = nh.serviceClient<dl_service::dl_detection>("/visual_servo/dl_detection");
        cli.waitForExistence();
    }
}

cv::Mat visual_servo::ToolDetector::getSourceImage(visual_servo::ImageCapturer& cam){
    if (cam.img_ptr->image.empty()){
        ROS_ERROR("No detection recorded yet.");
    }
    return cam.getCurrentImage();
}

cv::Point visual_servo::ToolDetector::getCenter(){
    return tool_center;
}

void visual_servo::ToolDetector::firstDetect(visual_servo::ImageCapturer& cam){
    std::string input = "";
    while(input!="y"){
        // update source image
        image = cam.getCurrentImage();
        // perform detection
        cv::Mat hsv, mask, col_sum, row_sum;
        // convert to hsv colorspace
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
        // find the red color within the boundaries
        cv::inRange(hsv, lower_hsv, upper_hsv, mask);

        // cv::namedWindow("mask");
        // cv::imshow("mask", mask);
        // cv::waitKey(0);
        // cv::destroyAllWindows();

        cv::reduce(mask, col_sum, 0, cv::REDUCE_SUM, CV_64FC1); 
        cv::reduce(mask, row_sum, 1, cv::REDUCE_SUM, CV_64FC1); 
        int start1, end1, start2, end2;
        for (int i=0; i<col_sum.size[1]; ++i){
            if (col_sum.at<double>(0,i)!=0){
                start1 = i;
                break;
            }
        }
        for (int i=col_sum.size[1]-1; i>=0; --i){
            if (col_sum.at<double>(0,i)!=0){
                end1 = i;
                break;
            }
        }
        
        for (int i=0; i<row_sum.size[0]; ++i){
            if (row_sum.at<double>(i,0)!=0){
                start2 = i;
                break;
            }
        }
        for (int i=row_sum.size[0]-1; i>=0; --i){
            if (row_sum.at<double>(i,0)!=0){
                end2 = i;
                break;
            }
        }
        tool_center.x = (start1+end1)/2.0;
        tool_center.y = (start2+end2)/2.0;
        corner1.x = start1;
        corner1.y = end2;
        corner2.x = end1;
        corner2.y = start2;

        while(nh.ok()){
            ros::spinOnce();
            break;
        }
        visual_servo::ToolDetector::drawDetectRes(image);
        std::cout << "Confirm first detection (y/n): ";
        std::cin >> input;
    }

}

void visual_servo::ToolDetector::detect(ImageCapturer& cam){
    // update source image
    image = cam.getCurrentImage();
    // perform detection
    cv::Mat hsv, mask, col_sum, row_sum;
    // convert to hsv colorspace
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    // find the red color within the boundaries
    cv::inRange(hsv, lower_hsv, upper_hsv, mask);

    // cv::namedWindow("mask");
    // cv::imshow("mask", mask);
    // cv::waitKey(0);
    // cv::destroyAllWindows();

    cv::reduce(mask, col_sum, 0, cv::REDUCE_SUM, CV_64FC1); 
    cv::reduce(mask, row_sum, 1, cv::REDUCE_SUM, CV_64FC1); 
    int start1=-1, end1=-1, start2=-1, end2=-1;
    for (int i=0; i<col_sum.size[1]; ++i){
        if (col_sum.at<double>(0,i)!=0){
            start1 = i;
            break;
        }
    }
    for (int i=col_sum.size[1]-1; i>=0; --i){
        if (col_sum.at<double>(0,i)!=0){
            end1 = i;
            break;
        }
    }
    
    for (int i=0; i<row_sum.size[0]; ++i){
        if (row_sum.at<double>(i,0)!=0){
            start2 = i;
            break;
        }
    }
    for (int i=row_sum.size[0]-1; i>=0; --i){
        if (row_sum.at<double>(i,0)!=0){
            end2 = i;
            break;
        }
    }
    if(start1<image.cols && start2<image.rows && end1<image.cols && end2<image.rows){
        tool_center.x = (start1+end1)/2.0;
        tool_center.y = (start2+end2)/2.0;
        corner1.x = start1;
        corner1.y = end2;
        corner2.x = end1;
        corner2.y = start2;
    }

    while(nh.ok()){
        ros::spinOnce();
        break;
    }
}

void visual_servo::ToolDetector::detect(cv::Mat& img){
    cv::Mat hsv, mask, col_sum, row_sum;
    // convert to hsv colorspace
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
    // find the red color within the boundaries
    cv::inRange(hsv, lower_hsv, upper_hsv, mask);

    // cv::namedWindow("mask");
    // cv::imshow("mask", mask);
    // cv::waitKey(0);
    // cv::destroyAllWindows();

    cv::reduce(mask, col_sum, 0, cv::REDUCE_SUM, CV_64FC1); 
    cv::reduce(mask, row_sum, 1, cv::REDUCE_SUM, CV_64FC1); 
    int start1=-1, end1=-1, start2=-1, end2=-1;
    for (int i=0; i<col_sum.size[1]; ++i){
        if (col_sum.at<double>(0,i)!=0){
            start1 = i;
            break;
        }
    }
    for (int i=col_sum.size[1]-1; i>=0; --i){
        if (col_sum.at<double>(0,i)!=0){
            end1 = i;
            break;
        }
    }
    
    for (int i=0; i<row_sum.size[0]; ++i){
        if (row_sum.at<double>(i,0)!=0){
            start2 = i;
            break;
        }
    }
    for (int i=row_sum.size[0]-1; i>=0; --i){
        if (row_sum.at<double>(i,0)!=0){
            end2 = i;
            break;
        }
    }
    if(start1<img.cols && start2<img.rows && end1<img.cols && end2<img.rows){
        tool_center.x = (start1+end1)/2.0;
        tool_center.y = (start2+end2)/2.0;
        corner1.x = start1;
        corner1.y = end2;
        corner2.x = end1;
        corner2.y = start2;
    }

    while(nh.ok()){
        ros::spinOnce();
        break;
    }
}

void visual_servo::ToolDetector::track(ImageCapturer& cam, double roir_width, double roir_height){
    // update source image
    image = cam.getCurrentImage();
    // create a img with a fixed size | dim constraints
    int h = image.rows, w = image.cols;
    if (tool_center.x==-1 || tool_center.y==-1 || tool_center.x>=w || tool_center.y>=h){
        while (tool_center.x==-1 || tool_center.y==-1 || tool_center.x>=image.cols || tool_center.y>=image.rows){
            ROS_ERROR("TRYING TO CORRECT AFTER DETECTION FAILED ...");
            image = cam.getCurrentImage();
            visual_servo::ToolDetector::detect(image);
        }
    }
    else{
        int roi_w = w*roir_width, roi_h = h*roir_height;
        cv::Point roi_origin(int(std::max(tool_center.x-roi_w/2,0)), int(std::max(tool_center.y-roi_h/2,0)));
        cv::Mat roi = image(cv::Range(roi_origin.y, int(std::min(tool_center.y+roi_h/2,h))), 
            cv::Range(roi_origin.x, int(std::min(tool_center.x+roi_w/2,w))));
        // detect inside roi
        visual_servo::ToolDetector::detect(roi);
        tool_center.x+=roi_origin.x;
        tool_center.y+=roi_origin.y;
        corner1.x+=roi_origin.x;
        corner1.y+=roi_origin.y;
        corner2.x+=roi_origin.x;
        corner2.y+=roi_origin.y;
    }
}

void visual_servo::ToolDetector::track(cv::Mat& image, ImageCapturer& cam, double roir_width, double roir_height){
    // create a img with a fixed size | dim constraints
    int h = image.rows, w = image.cols;
    if (tool_center.x==-1 || tool_center.y==-1 || tool_center.x>=w || tool_center.y>=h){
        while (tool_center.x==-1 || tool_center.y==-1 || tool_center.x>=image.cols || tool_center.y>=image.rows){
            ROS_ERROR("TRYING TO CORRECT AFTER DETECTION FAILED ...");
            image = cam.getCurrentImage();
            visual_servo::ToolDetector::detect(image);
        }
    }
    else{
        int roi_w = w*roir_width, roi_h = h*roir_height;
        cv::Point roi_origin(int(std::max(tool_center.x-roi_w/2,0)), int(std::max(tool_center.y-roi_h/2,0)));
        cv::Mat roi = image(cv::Range(roi_origin.y, int(std::min(tool_center.y+roi_h/2,h))), 
            cv::Range(roi_origin.x, int(std::min(tool_center.x+roi_w/2,w))));
        // detect inside roi
        visual_servo::ToolDetector::detect(roi);
        tool_center.x+=roi_origin.x;
        tool_center.y+=roi_origin.y;
        corner1.x+=roi_origin.x;
        corner1.y+=roi_origin.y;
        corner2.x+=roi_origin.x;
        corner2.y+=roi_origin.y;
    }
}

void visual_servo::ToolDetector::dlDetect(cv::Mat& img, cv::Point2d& drivertip, cv::Point2d& screwcup){
    image = img.clone();
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg;
    std_msgs::Header header; // empty header
    header.seq = 0; // user defined counter
    header.stamp = ros::Time::now(); // time
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img);
    img_bridge.toImageMsg(img_msg);

    dl_service::dl_detection srv;
    srv.request.image = img_msg;

    if (!cli.call(srv)){
        ROS_ERROR("failed to call image capture in tool_detector");
        drivertip.x = -1.0;
        drivertip.y = -1.0;
    }
    else{
        drivertip.x = srv.response.coordinates[0];
        drivertip.y = srv.response.coordinates[1];
        std::cout << drivertip.x << " " << drivertip.y << std::endl;
        std::cout << srv.response.confidence << std::endl;
    }
}

void visual_servo::ToolDetector::dlDetect(ImageCapturer& cam, cv::Point2d& drivertip, cv::Point2d& screwcup){
    // update source image
    image = cam.getCurrentImage();
    // cv::namedWindow("img");
    // cv::imshow("img", image);
    // cv::waitKey(0);
    // cv::destroyAllWindows();
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg;
    std_msgs::Header header; // empty header
    header.seq = 0; // user defined counter
    header.stamp = ros::Time::now(); // time
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image);
    img_bridge.toImageMsg(img_msg);

    dl_service::dl_detection srv;
    srv.request.image = img_msg;

    if (!cli.call(srv)){
        ROS_ERROR("failed to call image capture in tool_detector");
        drivertip.x = -1.0;
        drivertip.y = -1.0;
    }
    else{
        drivertip.x = srv.response.coordinates[0];
        drivertip.y = srv.response.coordinates[1];
        std::cout << drivertip.x << " " << drivertip.y << std::endl;
        std::cout << srv.response.confidence << std::endl;
    }
}

void visual_servo::ToolDetector::drawDetectRes(){
    cv::Mat img = image.clone();
    cv::Mat imgr = image.clone();
    cv::rectangle(img, corner1, corner2, cv::Scalar(0, 0, 0), 1, cv::LINE_8);
    cv::circle(img, tool_center, 3, cv::Scalar(255, 0, 0), -1);
    cv::resize(img, imgr, cv::Size(), 0.75, 0.75);
    cv::namedWindow("Detection_Result");
    cv::imshow("Detection_Result", imgr);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

void visual_servo::ToolDetector::drawDetectRes(cv::Mat img_){
    cv::Mat img = img_.clone();
    cv::Mat imgr = img_.clone();
    cv::rectangle(img, corner1, corner2, cv::Scalar(0, 0, 0), 1, cv::LINE_8);
    cv::circle(img, tool_center, 3, cv::Scalar(255, 0, 0), -1);
    cv::resize(img, imgr, cv::Size(), 0.75, 0.75);
    cv::namedWindow("Detection_Result");
    cv::imshow("Detection_Result", imgr);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

void visual_servo::ToolDetector::drawDetectRes(cv::Mat img_, cv::Point2d point){
    cv::Mat img = img_.clone();
    cv::Mat imgr = img_.clone();
    cv::circle(img, point, 3, cv::Scalar(255, 0, 0), -1);
    cv::resize(img, imgr, cv::Size(), 0.75, 0.75);
    cv::namedWindow("Detection_Result");
    cv::imshow("Detection_Result", imgr);
    cv::waitKey(0);
    cv::destroyAllWindows();
}