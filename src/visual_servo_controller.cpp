#include "visual_servo_controller.hpp"

// ####################
// # public functions #
// ####################

// constructors
visual_servo::VisualServoController::VisualServoController(ros::NodeHandle& nh, std::string target_topic, bool pred_, double tol, double tol_ori, double pred_thred_, double pred_thred_ori_) :
    nh(nh){

    tolerance = tol;
    tolerance_ori = tol_ori;

    pred = pred_;

    pred_thred = pred_thred_;
    pred_thred_ori = pred_thred_ori_;

    dof = 3;
    num_features = 4;
    freq = 100;
    servoMaxStep = 0.2;
    servoAngMaxStep = 0.5;
    K = 0.6;
    K_ori = 0.9;
    constJTh = 10;
    constJTh_ori = 0.16;

    JChecked = false;
    GChecked = false;
    continueLoop = true;
    targetReceived = false;
    closeUpdateJOri = false;
    // configure subscribers
    J_sub = nh.subscribe("/visual_servo/image_Jacobian", 1, &VisualServoController::JacobianCallback, this);
    G_sub = nh.subscribe("/visual_servo/orientation_gradient", 1, &VisualServoController::gradientCallback, this);

    target_sub = nh.subscribe(target_topic, 100, &VisualServoController::targetCallback, this);

    // containers initialization
    target_pos.resize(num_features);
    target_ori.resize(num_features);
    targets.resize(num_features*2);
    toolPos.resize(num_features);
    toolRot.resize(num_features);
    toolRotU.resize(num_features/2);
    energy.resize(num_features);
    energyU.resize(num_features/2);
    J.resize(num_features, dof);
    J_ori.resize(num_features, dof);
    G_ori.resize(num_features, dof);
    G_ori_uni.resize(num_features/2, dof);
    controlError.resize(num_features);
    controlError(0)=DBL_MAX; // set norm of control error to max
    controlErrorOri.resize(num_features);
    controlErrorOri(0)=DBL_MAX; // set norm of control error to max
    controlErrorOriU.resize(num_features/2);
    controlErrorOriU(0)=DBL_MAX; 
}

visual_servo::VisualServoController::VisualServoController(ros::NodeHandle& nh, Eigen::VectorXd& targets_, bool pred_, double tol, double tol_ori, double pred_thred_, double pred_thred_ori_) :
    nh(nh){

    tolerance = tol;
    tolerance_ori = tol_ori;
    targets = targets_;

    pred = pred_;

    pred_thred = pred_thred_;
    pred_thred_ori = pred_thred_ori_;

    dof = 3;
    num_features = 4;
    freq = 100;
    servoMaxStep = 0.2;
    servoAngMaxStep = 0.5;
    K = 0.6;
    K_ori = 0.9;
    constJTh = 10;
    constJTh_ori = 0.16;

    JChecked = false;
    GChecked = false;
    continueLoop = true;
    targetReceived = true;
    closeUpdateJOri = false;
    // configure subscribers
    J_sub = nh.subscribe("/visual_servo/image_Jacobian", 1, &VisualServoController::JacobianCallback, this);
    G_sub = nh.subscribe("/visual_servo/orientation_gradient", 1, &VisualServoController::gradientCallback, this);
    // containers initialization
    target_pos.resize(num_features);
    target_ori.resize(num_features);
    toolPos.resize(num_features);
    toolRot.resize(num_features);
    toolRotU.resize(num_features/2);
    energy.resize(num_features);
    energyU.resize(num_features/2);
    J.resize(num_features, dof);
    J_ori.resize(num_features, dof);
    G_ori.resize(num_features, dof);
    G_ori_uni.resize(num_features/2, dof);
    controlError.resize(num_features);
    controlError(0)=DBL_MAX; // set norm of control error to max
    controlErrorOri.resize(num_features);
    controlErrorOri(0)=DBL_MAX; // set norm of control error to max
    controlErrorOriU.resize(num_features/2);
    controlErrorOriU(0)=DBL_MAX; 
    // initialize targets
    target_pos << targets.head(num_features);
    target_ori << targets.tail(num_features);
}

void visual_servo::VisualServoController::VisualServoController::gradientCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
    G_ori_flat = msg->data;
    flat2eigen(G_ori, G_ori_flat);
    G_ori_uni << G_ori.row(0), G_ori.row(2);
    // if (controlErrorOri.norm()>constJTh_ori){
    //     G_ori_flat = msg->data;
    //     flat2eigenVec(G_ori, G_ori_flat);
    // }
    if (!GChecked){
        GChecked = true;
    }
}

void visual_servo::VisualServoController::JacobianCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
    // not updating J when control error is small
    if (msg->data.size()==dof*num_features){
        J_flat = msg->data;
        flat2eigen(J, J_flat);
        // if (controlError.norm()>constJTh){
        // J_flat = msg->data;
        // flat2eigen(J, J_flat);
        // }
    }
    else{
        std::vector<double> temp1(msg->data.begin(), msg->data.begin()+dof*num_features);
        J_flat = temp1;
        flat2eigen(J, J_flat);
        std::vector<double> temp2(msg->data.begin()+dof*num_features, msg->data.end());
        J_ori_flat = temp2;
        flat2eigen(J_ori, J_ori_flat);
    }

    if (!JChecked){
        JChecked = true;
    }
}

void visual_servo::VisualServoController::targetCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
    std::vector<double> temp1(msg->data.begin(), msg->data.begin()+num_features);
    flat2eigenVec(target_pos, temp1);
    std::vector<double> temp2(msg->data.begin()+num_features, msg->data.end());
    flat2eigenVec(target_ori, temp2);
    flat2eigenVec(targets, msg->data);
    if (!targetReceived){
        targetReceived = true;
    } 
}

void visual_servo::VisualServoController::setServoMaxStep(int step){
    servoMaxStep = step;
}

void visual_servo::VisualServoController::setK(double K_){
    K = K_;
}

void visual_servo::VisualServoController::setFreq(int f){
    freq = f;
}

bool visual_servo::VisualServoController::stopSign(){
    return !continueLoop;
}

Eigen::VectorXd visual_servo::VisualServoController::getToolPosition(){
    return toolPos;
}

// update member variables
void visual_servo::VisualServoController::directionIncrement(Eigen::VectorXd& increment, visual_servo::ImageCapturer& cam1, visual_servo::ImageCapturer& cam2, 
visual_servo::ToolDetector& detector, bool dl_on){
    ros::Rate loopRate(freq);

    if (!targetReceived){
        ROS_INFO("Waiting for servo target ......");
        while((nh.ok()) && !targetReceived){
            ros::spinOnce();
            loopRate.sleep();
        }
        ROS_INFO("Servo target received!");
    }
    if (target_pos.size()!=num_features){
        ROS_ERROR("Target vector size inconsistent! Should match: %d", num_features);
    }
    if (!JChecked){
        ROS_INFO("Waiting for Jacobian being initialized ......");
        while((nh.ok()) && !JChecked){
            ros::spinOnce();
            loopRate.sleep();
        }
        ROS_INFO("Jacobian received!");
    }
    
    cv::Point2d toolPos1, toolPos2;

    if (dl_on){
        cv::Point2d targetPos1, targetPos2;
        detector.dlDetect(cam1, toolPos1, targetPos1);
        detector.drawDetectRes(cam1.getCurrentImage(), toolPos1);
        detector.dlDetect(cam2, toolPos2, targetPos2);
        detector.drawDetectRes(cam2.getCurrentImage(), toolPos2);
    }
    else{
        detector.detect(cam1);
        toolPos1 = detector.getCenter();
        detector.drawDetectRes();
        detector.detect(cam2);
        toolPos2 = detector.getCenter();
        detector.drawDetectRes();
    }

    toolPos << toolPos1.x, toolPos1.y, toolPos2.x, toolPos2.y;

    controlError = target_pos-toolPos;

    std::cout << "target: " << target_pos << std::endl;
    std::cout << "toolPos: " << toolPos << std::endl;
    std::cout << "control_error: " << controlError << std::endl;

    if (controlError.norm()<tolerance){
        increment.setZero();
        continueLoop = false;
        ROS_INFO("Reach given accuracy, visual servo stopped!");
        return;
    }

    Eigen::MatrixXd J_pinv = (J.transpose()*J).inverse()*J.transpose();

    if (pred && controlError.norm()<pred_thred){
        increment = J_pinv*controlError;
        std::cout << "Final increment of " << increment << std::endl;
        continueLoop = false;
        ROS_INFO("Reach prediction threshold, final step of increment!");
        return;
    }
    else{
        increment = K*J_pinv*controlError;
        if (increment.norm()>servoMaxStep) limInc(increment, servoMaxStep);
        else ROS_INFO("Refining ---");
        std::cout << "increment" << increment << "\n" << std::endl;
    }
}

void visual_servo::VisualServoController::directionIncrement(Eigen::VectorXd& increment, ImageCapturer& cam1, ImageCapturer& cam2, std::vector<ToolDetector>& detector_list)
{
    ros::Rate loopRate(freq);

    if (!targetReceived){
        ROS_INFO("Waiting for servo target ......");
        while((nh.ok()) && !targetReceived){
            ros::spinOnce();
            loopRate.sleep();
        }
        ROS_INFO("Servo target received!");
    }
    if (target_pos.size()!=num_features){
        ROS_ERROR("Target vector size inconsistent! Should match: %d", num_features);
    }
    if (!JChecked){
        ROS_INFO("Waiting for Jacobian being initialized ......");
        while((nh.ok()) && !JChecked){
            ros::spinOnce();
            loopRate.sleep();
        }
        ROS_INFO("Jacobian received!");
    }

    detector_list[0].detect(cam1);
    cv::Point2d toolPos1 = detector_list[0].getCenter();
    detector_list[0].drawDetectRes();
    detector_list[1].detect(cam2);
    cv::Point2d toolPos2 = detector_list[1].getCenter();
    detector_list[1].drawDetectRes();

    toolPos << toolPos1.x, toolPos1.y, toolPos2.x, toolPos2.y;

    controlError = target_pos-toolPos;

    std::cout << "target: " << target_pos << std::endl;
    std::cout << "toolPos: " << toolPos << std::endl;
    std::cout << "control_error: " << controlError << std::endl;

    if (controlError.norm()<tolerance){
        increment.setZero();
        continueLoop = false;
        ROS_INFO("Reach given accuracy, visual servo stopped!");
        return;
    }

    Eigen::MatrixXd J_pinv = (J.transpose()*J).inverse()*J.transpose();

    if (pred && controlError.norm()<pred_thred){
        increment = J_pinv*controlError;
        std::cout << "Final increment of " << increment << std::endl;
        continueLoop = false;
        ROS_INFO("Reach prediction threshold, final step of increment!");
        return;
    }
    else{
        increment = K*J_pinv*controlError;
        if (increment.norm()>servoMaxStep) limInc(increment, servoMaxStep);
        else ROS_INFO("Refining ---");
        std::cout << "increment" << increment << "\n" << std::endl;
    }

}


void visual_servo::VisualServoController::oriDirectionIncrement(Eigen::VectorXd& increment, visual_servo::ImageCapturer& cam1, visual_servo::ImageCapturer& cam2, 
std::vector<visual_servo::ToolDetector>& detector_list){

    ros::Rate loopRate(freq);

    while(nh.ok()){
        ros::spinOnce();
        break;
    }

    if (!targetReceived){
        ROS_INFO("Waiting for servo target ------");
        while((nh.ok()) && !targetReceived){
            ros::spinOnce();
            loopRate.sleep();
        }
        ROS_INFO("Servo target received!");
    }

    if (target_ori.size()!=num_features){
        ROS_ERROR("Target vector size inconsistent! Should match: %d", num_features);
    }

    if (!GChecked){
        ROS_INFO("Waiting for gradient being initialized ------");
        while((nh.ok()) && !GChecked){
            ros::spinOnce();
            loopRate.sleep();
        }
        ROS_INFO("Gradient received!");
    }

    cv::Mat image1, image2;

    cv::Point2d toolPos1, toolPos2;
    cv::Point2d tooltipPos1, tooltipPos2;
    cv::Point2d toolframePos1, toolframePos2;

    image1 = cam1.getCurrentImage();
    image2 = cam2.getCurrentImage();

    detector_list[0].detect(image1);
    toolPos1 = detector_list[0].getCenter();
    // detector_list[0].drawDetectRes(image1);
    detector_list[0].detect(image2);
    toolPos2 = detector_list[0].getCenter();
    // detector_list[0].drawDetectRes(image2);

    detector_list[1].detect(image1);
    tooltipPos1 = detector_list[1].getCenter();
    // detector_list[1].drawDetectRes(image1);
    detector_list[1].detect(image2);
    tooltipPos2 = detector_list[1].getCenter();
    // detector_list[1].drawDetectRes(image2);

    detector_list[2].detect(image1);
    toolframePos1 = detector_list[2].getCenter();
    // detector_list[2].drawDetectRes(image1);
    detector_list[2].detect(image2);
    toolframePos2 = detector_list[2].getCenter();
    // detector_list[2].drawDetectRes(image2);

    visual_servo::VisualServoController::getToolRot(toolRot, toolPos1, tooltipPos1, toolframePos1, toolPos2, tooltipPos2, toolframePos2);
    std::cout << "got tool rot!" << std::endl;
    visual_servo::VisualServoController::calculateEnergyFunction(toolRot-target_ori, energy);

    controlErrorOri = -1.0*energy; // target is the zero energy
    std::cout << "control error assigned!" << std::endl;

    std::cout << "targets: " << target_ori << std::endl;
    std::cout << "toolRot: " << toolRot << std::endl;

    if (visual_servo::VisualServoController::getRotDis(toolRot, target_ori)<tolerance_ori){
        increment.setZero();
        continueLoop = false;
        ROS_INFO("Final orientation error: %.3f", visual_servo::VisualServoController::getRotDis(toolRot, target_ori));
        ROS_INFO("Reach given accuracy, visual servo stopped!");
        return;
    }

    Eigen::MatrixXd G_ori_pinv = (G_ori.transpose()*G_ori).inverse()*G_ori.transpose();

    if (pred && visual_servo::VisualServoController::getRotDis(toolRot, target_ori)<pred_thred_ori){
        increment = G_ori_pinv*controlErrorOri;
        std::cout << "Final increment of " << increment << std::endl;
        continueLoop = false;
        ROS_INFO("Reach prediction threshold, final step of increment!");
        return;
    }
    else{
        increment = K_ori*G_ori_pinv*controlErrorOri;
        if (increment.norm()>servoAngMaxStep) limInc(increment, servoAngMaxStep);
        else ROS_INFO("Refining ---");
        std::cout << "increment" << increment << "\n" << std::endl;
    }

}

void visual_servo::VisualServoController::uniOriDirectionIncrement(Eigen::VectorXd& increment, ImageCapturer& cam1, ImageCapturer& cam2, std::vector<ToolDetector>& detector_list, bool dl_on, bool sep){
    ros::Rate loopRate(freq);

    while(nh.ok()){
        ros::spinOnce();
        break;
    }

    if (!targetReceived){
        ROS_INFO("Waiting for servo target ------");
        while((nh.ok()) && !targetReceived){
            ros::spinOnce();
            loopRate.sleep();
        }
        ROS_INFO("Servo target received!");
    }

    if (target_ori.size()!=num_features){
        ROS_ERROR("Target vector size inconsistent! Should match: %d", num_features);
    }

    if (!GChecked){
        ROS_INFO("Waiting for gradient being initialized ------");
        while((nh.ok()) && !GChecked){
            ros::spinOnce();
            loopRate.sleep();
        }
        ROS_INFO("Gradient received!");
    }

    // fix target
    Eigen::VectorXd target;
    target.resize(num_features/2);
    target << target_ori(0), target_ori(2);

    // fix G/J


    cv::Mat image1, image2;

    cv::Point2d toolPos1, toolPos2;

    cv::Point2d tooltipPos1, tooltipPos2;
    cv::Point2d toolframePos1, toolframePos2;

    image1 = cam1.getCurrentImage();
    image2 = cam2.getCurrentImage();

    if (dl_on){
        cv::Point2d targetPos1, targetPos2;
        detector_list[0].dlDetect(image1, toolPos1, targetPos1);
        detector_list[0].drawDetectRes(image1, toolPos1);
        detector_list[0].dlDetect(image2, toolPos2, targetPos2);
        detector_list[0].drawDetectRes(image2, toolPos2);

        detector_list[1].detect(image1);
        tooltipPos1 = detector_list[1].getCenter();
        detector_list[1].drawDetectRes(image1);
        detector_list[2].detect(image2);
        tooltipPos2 = detector_list[2].getCenter();
        detector_list[2].drawDetectRes(image2);
    }
    else{
        if (sep){
            detector_list[0].detect(image1);
            toolPos1 = detector_list[0].getCenter();
            detector_list[0].drawDetectRes(image1);
            detector_list[1].detect(image2);
            toolPos2 = detector_list[1].getCenter();
            detector_list[1].drawDetectRes(image2);

            detector_list[2].detect(image1);
            tooltipPos1 = detector_list[2].getCenter();
            detector_list[2].drawDetectRes(image1);
            detector_list[3].detect(image2);
            tooltipPos2 = detector_list[3].getCenter();
            detector_list[3].drawDetectRes(image2);
        }
        else{
            detector_list[0].detect(image1);
            toolPos1 = detector_list[0].getCenter();
            // detector_list[0].drawDetectRes(image1);
            detector_list[0].detect(image2);
            toolPos2 = detector_list[0].getCenter();
            // detector_list[0].drawDetectRes(image2);

            detector_list[1].detect(image1);
            tooltipPos1 = detector_list[1].getCenter();
            // detector_list[1].drawDetectRes(image1);
            detector_list[1].detect(image2);
            tooltipPos2 = detector_list[1].getCenter();
            // detector_list[1].drawDetectRes(image2);
        }
    }

    visual_servo::VisualServoController::getToolRot(toolRot, toolPos1, tooltipPos1, tooltipPos1, toolPos2, tooltipPos2, tooltipPos2);
    std::cout << "got tool rot!" << std::endl;
    energyU.resize(num_features/2);
    toolRotU.resize(num_features/2);
    toolRotU << toolRot(0), toolRot(2);
    visual_servo::VisualServoController::calculateEnergyFunction(toolRotU-target, energyU);

    controlErrorOriU = -1.0*energyU; // target is the zero energy
    std::cout << "control error assigned!" << std::endl;

    std::cout << "target: " << target << std::endl;
    std::cout << "toolRotU: " << toolRotU << std::endl;

    if (visual_servo::VisualServoController::getRotDis(toolRotU, target)<tolerance_ori){
        increment.setZero();
        continueLoop = false;
        ROS_INFO("Final orientation error: %.3f", visual_servo::VisualServoController::getRotDis(toolRotU, target));
        ROS_INFO("Final energy: %.3f", energyU.norm());
        ROS_INFO("Reach given accuracy, visual servo stopped!");
        return;
    }

    Eigen::MatrixXd G_ori_uni_pinv = G_ori_uni.transpose()*(G_ori_uni*G_ori_uni.transpose()).inverse();

    if (pred && visual_servo::VisualServoController::getRotDis(toolRotU, target)<pred_thred_ori){
        increment = G_ori_uni_pinv*controlErrorOriU;
        std::cout << "Final increment of " << increment << std::endl;
        continueLoop = false;
        ROS_INFO("Reach prediction threshold, final step of increment!");
        return;
    }
    else{
        increment = K_ori*G_ori_uni_pinv*controlErrorOriU;
        if (increment.norm()>servoAngMaxStep) limInc(increment, servoAngMaxStep);
        else ROS_INFO("Refining ---");
        std::cout << "increment" << increment << "\n" << std::endl;
    }

}



void visual_servo::VisualServoController::poseDirectionIncrement(Eigen::VectorXd& increment, visual_servo::ImageCapturer& cam1, visual_servo::ImageCapturer& cam2, 
std::vector<visual_servo::ToolDetector>& detector_list){

    ros::Rate loopRate(freq);

    while(nh.ok()){
        ros::spinOnce();
        break;
    }

    if (!targetReceived){
        ROS_INFO("Waiting for servo target ------");
        while((nh.ok()) && !targetReceived){
            ros::spinOnce();
            loopRate.sleep();
        }
        ROS_INFO("Servo target received!");
    }

    if (targets.size()!=num_features*2){
        ROS_ERROR("Target vector size inconsistent! Should match: %d", 2*num_features);
    }

    if (!JChecked){
        ROS_INFO("Waiting for Jacobian being initialized ------");
        while((nh.ok()) && !JChecked){
            ros::spinOnce();
            loopRate.sleep();
        }
        ROS_INFO("Jacobian received!");
    }

    Eigen::VectorXd increment_pos, increment_ori;

    Eigen::VectorXd target_pos = targets.head(4);
    Eigen::VectorXd target_ori = targets.tail(4);

    std::cout << "targets divided!" << std::endl;

    cv::Point2d toolPos1, toolPos2;
    cv::Point2d tooltipPos1, tooltipPos2;
    cv::Point2d toolframePos1, toolframePos2;

    detector_list[0].detect(cam1);
    toolPos1 = detector_list[0].getCenter();
    detector_list[0].detect(cam2);
    toolPos2 = detector_list[0].getCenter();

    std::cout << "target detected!" << std::endl;

    toolPos << toolPos1.x, toolPos1.y, toolPos2.x, toolPos2.y;

    std::cout << "toolPos assigned!" << std::endl;

    controlError = target_pos-toolPos;

    std::cout << "control error assigned!" << std::endl;

    detector_list[1].detect(cam1);
    tooltipPos1 = detector_list[1].getCenter();
    detector_list[1].detect(cam2);
    tooltipPos2 = detector_list[1].getCenter();

    detector_list[2].detect(cam1);
    toolframePos1 = detector_list[2].getCenter();
    detector_list[2].detect(cam2);
    toolframePos2 = detector_list[2].getCenter();

    visual_servo::VisualServoController::getToolRot(toolRot, toolPos1, tooltipPos1, toolframePos1, toolPos2, tooltipPos2, toolframePos2);

    std::cout << "got tool rot!" << std::endl;

    controlErrorOri = target_ori-toolRot;

    if (controlError.norm()<tolerance && controlErrorOri.norm()<tolerance_ori){
        increment.setZero();
        continueLoop = false;
        ROS_INFO("Reach given accuracy, visual servo stopped!");
        return;
    }
    
    std::cout << "target_pos: " << target_pos << std::endl;
    std::cout << "toolPos: " << toolPos << std::endl;
    std::cout << "control_error: " << controlError << std::endl;

    Eigen::MatrixXd J_pinv = (J.transpose()*J).inverse()*J.transpose();
    increment_pos = K*J_pinv*controlError;

    std::cout << "increment_pos: " << increment_pos << std::endl;

    std::cout << "**************" << increment_pos << std::endl;

    std::cout << "target_ori: " << target_ori << std::endl;
    std::cout << "toolRot: " << toolRot << std::endl;
    std::cout << "control_error_ori: " << controlErrorOri << std::endl;

    Eigen::MatrixXd J_ori_pinv = (J_ori.transpose()*J_ori).inverse()*J_ori.transpose();
    increment_ori = K_ori*J_ori_pinv*controlErrorOri;

    std::cout << "increment_ori: " << increment_ori << std::endl;

    if (increment_pos.norm()>servoMaxStep){
        limInc(increment_pos, servoMaxStep);
    }
    else{
        ROS_INFO("Refining position ---");
    }

    if (increment_ori.norm()>servoAngMaxStep){
        limInc(increment_ori, servoAngMaxStep);
    }
    else{
        ROS_INFO("Refining orientation ---");
    }

    increment << increment_pos, increment_ori;

    std::cout << "full increment: " << increment << std::endl;
    // std::cout << "increment after limit" << increment << std::endl;
}


// #################
// ##    Utils    ##
// #################

void visual_servo::VisualServoController::vecEigen2std(Eigen::VectorXd& ve, std::vector<double>& vs){
    if (ve.size()!=vs.size()) vs.resize(ve.size());
    for (int i=0; i<ve.size(); ++i){
        vs[i] = ve(i);
    }
}

void visual_servo::VisualServoController::flat2eigen(Eigen::MatrixXd& M, std::vector<double> flat){
    // check input
    int size_flat = flat.size();
    int size_M = M.cols()*M.rows();
    if (size_flat!=size_M){
        ROS_ERROR("Dimension inconsistent! Cannot convert to Eigen matrix.");
        return;
    }
    int cur = 0;
    for (int i=0; i<M.rows(); ++i){
        for (int j=0; j<M.cols(); ++j){
            M.coeffRef(i,j) = flat[cur];
            cur++;
        }
    }
}

void visual_servo::VisualServoController::flat2eigenVec(Eigen::VectorXd& V, std::vector<double> flat){
    // check input
    int size_flat = flat.size();
    int size_V = V.size();
    if (size_flat!=size_V){
        ROS_ERROR("Dimension inconsistent! Cannot convert to vector (Eigen3).");
        return;
    }
    for (int i=0; i<size_V; ++i){
        V(i) = flat[i];
    }
}

void visual_servo::VisualServoController::limInc(Eigen::VectorXd& v, double stepSize){
    double r;
    // a vector with abs of v
    Eigen::VectorXd abs_v(v.size());
    for (int i=0; i<abs_v.size(); ++i){
        abs_v(i) = abs(v(i));
    }
    // initializations
    double max_abs=abs_v(0);
    // find the maximum absolute value
    for (int i=0; i<abs_v.size(); ++i){
        if (abs_v(i)>max_abs){
            max_abs = abs_v(i);
        }
    }
    
    if (max_abs > stepSize){
        std::cout << "Enter limiting velocity!" << std::endl;
        r = stepSize*1.0/max_abs;
        std::cout << "r: " << r << std::endl;
        v = r*v;
    }
}

void visual_servo::VisualServoController::getToolRot(Eigen::VectorXd& toolRot, cv::Point2d& center1, cv::Point2d& tooltip1, cv::Point2d& frametip1, cv::Point2d& center2, cv::Point2d& tooltip2, cv::Point2d& frametip2){
    double theta11 = atan2((tooltip1-center1).y, (tooltip1-center1).x); 
    double theta12 = atan2((frametip1-center1).y, (frametip1-center1).x); 
    double theta21 = atan2((tooltip2-center2).y, (tooltip2-center2).x); 
    double theta22 = atan2((frametip2-center2).y, (frametip2-center2).x); 
    toolRot << theta11, theta12, theta21, theta22;
}

void visual_servo::VisualServoController::stdAngControlError(Eigen::VectorXd& controlErrorOri){
    for (int i=0; i<controlErrorOri.size(); i++){
        if (abs(controlErrorOri(i))>M_PI){
            controlErrorOri(i) = (signbit(controlErrorOri(i)) ? 1.0 : -1.0)*(2*M_PI-abs(controlErrorOri(i)));
        }
    }
}

void visual_servo::VisualServoController::calculateEnergyFunction(Eigen::VectorXd delToolRot, Eigen::VectorXd& energy){
    for (int i=0; i<energy.size(); ++i){
        // energy(i)=sin(delToolRot(i)/2.0);
        energy(i)=-cos(delToolRot(i))+1;
    }
}

double visual_servo::VisualServoController::getRotDis(Eigen::VectorXd toolRot1, Eigen::VectorXd toolRot2){
    // check dimensions
    if (toolRot1.size()!=toolRot2.size()){
        ROS_ERROR("Error!!! Dimension not consistent!");
    }
    int n = toolRot1.size();
    double dis_sq = 0.0;
    for (int i=0; i<n; ++i){
        bool sign1 = signbit(toolRot1(i)), sign2 = signbit(toolRot2(i));
        if (sign1!=sign2){
            if (sign1) toolRot1(i)+=2*M_PI;
            else toolRot2(i)+=2*M_PI;
        }
        dis_sq+=(toolRot1(i)-toolRot2(i))*(toolRot1(i)-toolRot2(i));
    }
    return sqrt(dis_sq);
}