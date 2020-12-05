//
// Created by rosk on 30.08.19.
//

#include "denm_client.h"
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
#include "denm_client/DenmTx.h"
#include "car2x/parkingStatus.h"
#include <ros/service.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <string>


void create_denm(denm_client::DenmTx& srv){
    srv.request.request = 0; // For TRIGGER DENM

    //DENM
    srv.request.denm.header.stamp = ros::Time::now();

    //DENM-ManagementContainer
    srv.request.denm.management.action_id.station_id = 1;
    srv.request.denm.management.action_id.sequence_number = 1;
    srv.request.denm.management.detection_time = ros::Time::now().toSec();
    srv.request.denm.management.reference_time = ros::Time::now().toSec();
    srv.request.denm.management.event_position.latitude = 555;
    srv.request.denm.management.event_position.longitude = 111;
    srv.request.denm.management.event_position.position_confidence.semi_major_confidence = 1;
    srv.request.denm.management.event_position.position_confidence.semi_minor_confidence = 1;
    srv.request.denm.management.relevance_distance.value = 0; // less than 50m
    srv.request.denm.management.relevance_traffic_direction.value = 0; // All direction
    srv.request.denm.management.validity_duration = 600; //Duration in s
    srv.request.denm.management.transmission_interval = 1000; //Transmission interval in ms
    srv.request.denm.management.station_type.value = 10; //Special vehicle
    srv.request.denm.has_situation = false; //Special vehicle
    srv.request.denm.has_location = true; //Special vehicle

    srv.request.denm.location.event_speed.value = 0.0;
    srv.request.denm.location.event_speed.confidence = 1;
    srv.request.denm.location.event_position_heading.value = 0;
    srv.request.denm.location.event_position_heading.confidence = 1;

    srv.request.denm.location.traces = std::vector<etsi_its_msgs::PathHistory>();
    srv.request.denm.location.traces.push_back(etsi_its_msgs::PathHistory());

    srv.request.denm.location.traces.at(0).points = std::vector<etsi_its_msgs::PathPoint>();

    // temp object to store information
    etsi_its_msgs::PathPoint temp = etsi_its_msgs::PathPoint();
    temp.path_position.delta_latitude =   1;
    temp.path_position.delta_longitude =   2;
    temp.path_position.delta_altitude =   1;
    temp.path_delta_time.value = 10;
    srv.request.denm.location.traces.at(0).points.push_back(temp);

    srv.request.denm.location.road_type = 255;

    //DENM-LOCATION

    //GeonetArea container -> see description in msg/
    // It contains the information about the location of the obstacle
    srv.request.destination_area.type = 2; //Describe a rectangle
    srv.request.destination_area.latitude = 48.765957;
    srv.request.destination_area.longitude = 11.434670;
    srv.request.destination_area.distance_a = 1; // [m] small bisector
    srv.request.destination_area.distance_b = 2; // [m] big bisector
    srv.request.destination_area.angle = 0;

    //TrafficClass
    srv.request.traffic_class.store_carry_forwarding = false;
    srv.request.traffic_class.channel_offloading = false;
    srv.request.traffic_class.id = 2;

    //Durations
    srv.request.repetition_interval = ros::Duration(1); //1 second repetition interval
    srv.request.repetition_duration = ros::Duration(600); //Message should be sent during 1min
}

void parkingStatusCallBack(const car2x::parkingStatus::ConstPtr& parkingSts)
{
  // TODO 1. Recive this msg and update the loc and lat variables dynamically
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "den_client");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<denm_client::DenmTx>("den_request");
    ros::Subscriber subscriber = nh.subscribe("parkingStatus", 1000, parkingStatusCallBack);
    denm_client::DenmTx srv;
    create_denm(srv);

    if(client.call(srv))
    {

        std::cout << "confirm_den " << srv.response.confirm_den << "confirm_btp " << srv.response.confirm_btp << std::endl;
        std::cout << "\n\n\n\n" << std::endl;
    }
    else
    {
        ROS_ERROR("Failed to call service");
        return 1;
    }

    return 0;
}
    //sensor_msgs::NavSatFixPtr location_earth;
    //location_earth->status = sensor_msgs::NavSatStatus::STATUS_FIX; //sensor_msgs::NavSatStatus::STATUS_FIX

