// Technische Hochschule Ingolstadt

#ifndef CAR2X_SRC_INCLUDE_CAR2XSERVER_CAMERAINPUT_H_
#define CAR2X_SRC_INCLUDE_CAR2XSERVER_CAMERAINPUT_H_

#include "Input.h"
#include "car2x/parkingStatus.h"

class CameraInput : public Input {
 private:
    image_transport::ImageTransport imageTransport_;
    image_transport::Subscriber imageTransportSubcribe_;
    cv_bridge::CvImagePtr openCvFrame_;
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;
    std::vector<int> ids_{0};
    car2x::parkingStatus parkingStatus_;
    std::vector<std::vector<cv::Point2f> > corners_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;

 public:
    CameraInput();
    cv_bridge::CvImagePtr& RosImgToOpenCv(const sensor_msgs::Image::ConstPtr& rosImgMsg);
    virtual void publish();
    virtual void subscribe();
    void imageCallback(const sensor_msgs::Image::ConstPtr& cameraMsg);
    virtual void detectAruco();
    virtual void calibrateCamera();
    virtual void optParkingSpotCalculate();
    ~CameraInput() {};
};

#endif  // CAR2X_SRC_INCLUDE_CAR2XSERVER_CAMERAINPUT_H_
