// Technische Hochschule Ingolstadt

#include "car2x/CameraInput.h"

CameraInput::CameraInput():imageTransport_(node_) {
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
    publish_ = node_.advertise<car2x::parkingStatus>("parkingStatus", 1000);
    this->calibrateCamera();
}

void CameraInput::subscribe() {
    imageTransportSubcribe_ = imageTransport_.subscribe("/cameranode/image_raw", 1, &CameraInput::imageCallback, this);
}

cv_bridge::CvImagePtr& CameraInput::RosImgToOpenCv(const sensor_msgs::Image::ConstPtr& rosImgMsg) {
    try {
        openCvFrame_ = cv_bridge::toCvCopy(rosImgMsg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& err) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", rosImgMsg->encoding.c_str());
    }
    return openCvFrame_;
}

void CameraInput::imageCallback(const sensor_msgs::Image::ConstPtr& cameraMsg) {
    openCvFrame_ = RosImgToOpenCv(cameraMsg);
    this->detectAruco();
}

void CameraInput::publish() {
    publish_.publish(parkingStatus_);
}

void CameraInput::optParkingSpotCalculate() {
    // Aruko marker - 5 (behind the turtlebot, Hence if we detect number 5 it means the bot has parked and no need to monitor).
    // Aruko marker - 1, 2 and 3 (Available Parking spots).
    // Bot enters from the marker number one so it is the shortest path.
    parkingStatus_.parkingSlot1Status_ = 0;
    parkingStatus_.parkingSlot2Status_ = 0;
    parkingStatus_.parkingSlot3Status_ = 0;
    auto it = ids_.begin();
    while (it != ids_.end()) {
        if (*it == 0) {
            parkingStatus_.parkingSlot1Status_ = 1;
        }
        else if (*it == 2) {
            parkingStatus_.parkingSlot2Status_ = 1;
        } 
        else if (*it == 3) {
            parkingStatus_.parkingSlot3Status_ = 1;
        }
        if (*it == 5)
            std::cout << "TurtleBot has parked" << std::endl;
        it++;
    }
    this->publish();
}

void CameraInput::detectAruco() {
    cv::aruco::detectMarkers(openCvFrame_->image, dictionary_, corners_, ids_);
    //std::cout << "Markers detected : " << ids_.size() << std::endl;
    this->optParkingSpotCalculate();
}


void CameraInput::calibrateCamera() {
    cv::FileStorage fileObject("../camera_matrix.yml", cv::FileStorage::READ);
    fileObject["camera_matrix"] >> cameraMatrix_;
    fileObject["distortion_coefficients"] >> distCoeffs_;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "car2xlistener");
    CameraInput CameraInput;
    CameraInput.subscribe();
    ros::spin();
    return 0;
}
