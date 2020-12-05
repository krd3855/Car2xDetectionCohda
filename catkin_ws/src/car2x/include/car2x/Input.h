// Technische Hochschule Ingolstadt

#ifndef CAR2X_SRC_INCLUDE_CAR2XSERVER_INPUT_H_
#define CAR2X_SRC_INCLUDE_CAR2XSERVER_INPUT_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>

class Input {
 protected:
    ros::NodeHandle node_;
    ros::Publisher publish_;
    ros::Subscriber subscribe_;

 public:
    Input() {};
    virtual void publish() = 0;
    virtual void subscribe() = 0;
    virtual ~Input() {};
};

#endif  // CAR2X_SRC_INCLUDE_CAR2XSERVER_INPUT_H_
