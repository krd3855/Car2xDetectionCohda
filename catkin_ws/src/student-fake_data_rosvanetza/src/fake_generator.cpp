//
// Created by rosk on 30.08.19.
//

#include "fake_generator.h"
#include <ros/ros.h>
#include <algorithm>
#include <iostream>
#include <cstdint>
#include <cstring>
#include <random>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include "vanetza/LocalisationVehicleInformation.h"
#include "vanetza/LocalisationBody.h"
#include "vanetza/LocalisationStatus.h"
#include "vanetza/PositionVector.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_data_generator");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ros::Rate r(15);

    ros::Publisher publisher_location_earth = nh.advertise<sensor_msgs::NavSatFix>("localisation_earth", 1000);
    ros::Publisher publisher_vehicle_location = nh.advertise<vanetza::LocalisationVehicleInformation>("localisation_vehicle", 1000);
    ros::Publisher publisher_body = nh.advertise<vanetza::LocalisationBody>("localisation_body", 1000);
    ros::Publisher publisher_status = nh.advertise<vanetza::LocalisationStatus>("localisation_status", 1000);
    ros::Publisher publisher_ellipsoid = nh.advertise<sensor_msgs::NavSatFix>("localisation_ellipsoid", 1000);
    ros::Publisher publisher_position = nh.advertise<vanetza::PositionVector>("position_vector", 1000);


    sensor_msgs::NavSatFix localisationEarth;

    nh_private.getParam("latitude_to_use", localisationEarth.latitude);
    nh_private.getParam("longitude_to_use", localisationEarth.longitude);
    localisationEarth.altitude = 0; //48.765957, 11.434670
    localisationEarth.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN ; //48.765957, 11.434670

    sensor_msgs::NavSatFix localisation_ellipsoid = localisationEarth;

    vanetza::LocalisationVehicleInformation vehicleInformation;
    vehicleInformation.speed = 0.0;
    vehicleInformation.speed_confidence = 1;
    nh_private.getParam("heading_to_use", vehicleInformation.heading);
    // vehicleInformation.heading = 0.0;
    vehicleInformation.heading_confidence = 1;
    vehicleInformation.curvature = 0.0;
    vehicleInformation.curvature_confidence = 1;

    vanetza::LocalisationBody bodyInformation;
    bodyInformation.angular_velocity.x = 0.0;
    bodyInformation.angular_velocity.y = 0.0;
    bodyInformation.angular_velocity.z = 0.0;
    bodyInformation.acceleration.x = 0.0;
    bodyInformation.acceleration.y = 0.0;
    bodyInformation.acceleration.z = 0.0;

    vanetza::LocalisationStatus localisationStatus;
    localisationStatus.algorithm = vanetza::LocalisationStatus::ALGO_NULL;
    localisationStatus.operational_quality_indicator = vanetza::LocalisationStatus::OQI_BEST;

    vanetza::PositionVector positionVector;
    positionVector.latitude = 48.765957;
    positionVector.longitude = 11.434670;
    positionVector.altitude = localisationEarth.altitude;
    positionVector.altitude_confidence = 1;
    positionVector.semi_major_confidence = 1;
    positionVector.semi_minor_confidence = 1;
    positionVector.semi_major_orientation = vehicleInformation.heading;
    positionVector.speed = 0;
    positionVector.speed_confidence = 1;
    positionVector.speed_confidence = vehicleInformation.heading;
    positionVector.course_confidence = 1;

    bool fake_position_earth;
    nh_private.getParam("fake_position_earth", fake_position_earth);
    bool fake_localisation_vehicle;
    nh_private.getParam("fake_localisation_vehicle", fake_localisation_vehicle);
    bool fake_localisation_body;
    nh_private.getParam("fake_localisation_body", fake_localisation_body);
    bool fake_localisation_status;
    nh_private.getParam("fake_localisation_status", fake_localisation_status);
    bool fake_localisation_ellipsoid;
    nh_private.getParam("fake_localisation_ellipsoid", fake_position_earth);
    bool fake_position_vector;
    nh_private.getParam("fake_position_vector", fake_position_vector);

    while(ros::ok()){
        if(fake_position_earth){
            sensor_msgs::NavSatFix msgToSendNav;
            //std::memcpy(&msgToSendNav, &localisationEarth, sizeof(localisationEarth));
            publisher_location_earth.publish(localisationEarth);
        }
        if(fake_localisation_vehicle){
            vanetza::LocalisationVehicleInformation msgToSendVehInf;
            //std::memcpy(&msgToSendVehInf, &vehicleInformation, sizeof(vehicleInformation));
            publisher_vehicle_location.publish(vehicleInformation);
        }
        if(fake_localisation_body){
            vanetza::LocalisationBody msgToSendBody;
            //std::memcpy(&msgToSendBody, &bodyInformation, sizeof(bodyInformation));
            publisher_body.publish(bodyInformation);
        }
        if(fake_localisation_status){
            vanetza::LocalisationStatus msgToSendStatus;
            //std::memcpy(&msgToSendStatus, &localisationStatus, sizeof(localisationStatus));
            publisher_status.publish(localisationStatus);
        }

        if(fake_localisation_ellipsoid) {
            publisher_ellipsoid.publish(localisation_ellipsoid);
        }

        if(fake_position_vector){
            publisher_position.publish(positionVector);
        }

        r.sleep();
    }

    //sensor_msgs::NavSatFixPtr location_earth;
    //location_earth->status = sensor_msgs::NavSatStatus::STATUS_FIX; //sensor_msgs::NavSatStatus::STATUS_FIX
}
