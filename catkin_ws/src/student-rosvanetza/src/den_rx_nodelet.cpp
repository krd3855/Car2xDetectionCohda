#include "den_rx_nodelet.hpp"
#include "vanetza/BtpDataIndication.h"
#include <etsi_its_msgs/DENM.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/node_handle.h>
#include <vanetza/btp/ports.hpp>

PLUGINLIB_EXPORT_CLASS(vanetza::DenRxNodelet, nodelet::Nodelet)

namespace vanetza
{

void assignManagementContainer(etsi_its_msgs::ManagementContainer&, const ManagementContainer&);
void assignSituationContainer(etsi_its_msgs::SituationContainer&, const SituationContainer&);
void assignLocationContainer(etsi_its_msgs::LocationContainer&, const LocationContainer&);
void assignDeltaReferencePosition(etsi_its_msgs::DeltaReferencePosition&, const DeltaReferencePosition&);
void assignPathDeltaTime(etsi_its_msgs::PathDeltaTime&, const PathDeltaTime_t*);

void DenRxNodelet::onInit()
{
    auto& nhp = getPrivateNodeHandle();
    port_ = nhp.param<int>("port", btp::ports::DENM.host());

    auto& nh = getNodeHandle();
    sub_btp_ = nh.subscribe<BtpDataIndication>("btp_indication", 20, boost::bind(&DenRxNodelet::onIndication, this, _1));
    pub_denm_ = nh.advertise<etsi_its_msgs::DENM>("denm_received", 20);
}

void DenRxNodelet::onIndication(BtpDataIndicationConstPtr indication)
{
    if (indication->btp_type == BtpDataIndication::BTP_TYPE_B && indication->destination_port == port_)
    {
        vanetza::asn1::Denm denm;
        if (denm.decode(indication->data))
        {
            publish(denm);
        }
        else
        {
            NODELET_DEBUG("Dropped received DENM: ASN.1 decoding failed");
        }
    }   
}

void DenRxNodelet::publish(const vanetza::asn1::Denm& asn1)
{
    auto msg = boost::make_shared<etsi_its_msgs::DENM>();

    // etsi_its_msgs/DENM header fields
    msg->header.stamp = ros::Time::now();
    msg->its_header.protocol_version = asn1->header.protocolVersion;
    msg->its_header.station_id = asn1->header.stationID;

    // management container (mandatory)
    assignManagementContainer(msg->management, asn1->denm.management);

    // situation container (optional)
    if (asn1->denm.situation)
    {
        msg->has_situation = true;
        assignSituationContainer(msg->situation, *asn1->denm.situation);
    }
    else
    {
        msg->has_situation = false;
    }

    // location container (optional)
    if (asn1->denm.location)
    {
        msg->has_location = true;
        assignLocationContainer(msg->location, *asn1->denm.location);
    }
    else
    {
        msg->has_location = false;
    }

    pub_denm_.publish(msg);
}

void assignManagementContainer(etsi_its_msgs::ManagementContainer& msg, const ManagementContainer& asn1)
{
    msg.action_id.station_id = asn1.actionID.originatingStationID;
    msg.action_id.sequence_number = asn1.actionID.sequenceNumber;
    asn_INTEGER2ulong(&asn1.detectionTime, &msg.detection_time);
    asn_INTEGER2ulong(&asn1.referenceTime, &msg.reference_time);
    msg.termination = asn1.termination ?
        *asn1.termination :
        etsi_its_msgs::ManagementContainer::TERMINATION_UNAVAILABLE;

    msg.event_position.altitude.value = asn1.eventPosition.altitude.altitudeValue;
    msg.event_position.altitude.confidence = asn1.eventPosition.altitude.altitudeConfidence;
    msg.event_position.latitude = asn1.eventPosition.latitude;
    msg.event_position.longitude = asn1.eventPosition.longitude;
    msg.event_position.position_confidence.semi_major_confidence = asn1.eventPosition.positionConfidenceEllipse.semiMajorConfidence;
    msg.event_position.position_confidence.semi_minor_confidence = asn1.eventPosition.positionConfidenceEllipse.semiMinorConfidence;
    msg.event_position.position_confidence.semi_major_orientation = asn1.eventPosition.positionConfidenceEllipse.semiMajorOrientation;

    msg.relevance_distance.value = asn1.relevanceDistance ?
        *asn1.relevanceDistance :
        etsi_its_msgs::RelevanceDistance::UNAVAILABLE;

    msg.relevance_traffic_direction.value = asn1.relevanceTrafficDirection ?
        *asn1.relevanceTrafficDirection :
        etsi_its_msgs::RelevanceTrafficDirection::UNAVAILABLE;

    msg.validity_duration = asn1.validityDuration ?
        *asn1.validityDuration :
        etsi_its_msgs::ManagementContainer::VALIDITY_DURATION_DEFAULT;

    msg.transmission_interval = asn1.transmissionInterval ?
        *asn1.transmissionInterval :
        etsi_its_msgs::ManagementContainer::TERMINATION_UNAVAILABLE;

    msg.station_type.value = asn1.stationType;
}

void assignSituationContainer(etsi_its_msgs::SituationContainer& msg, const SituationContainer& asn1)
{
    msg.information_quality.value = asn1.informationQuality;
    msg.event_type.cause_code = asn1.eventType.causeCode;
    msg.event_type.sub_cause_code = asn1.eventType.subCauseCode;
    if (asn1.linkedCause)
    {
        msg.has_linked_cause = true;
        msg.linked_cause.cause_code = asn1.linkedCause->causeCode;
        msg.linked_cause.sub_cause_code = asn1.linkedCause->subCauseCode;
    }
    else
    {
        msg.has_linked_cause = false;
    }
    if (asn1.eventHistory)
    {
        for (int i = 0; i < asn1.eventHistory->list.count; ++i)
        {
            const EventPoint* asn1_point = asn1.eventHistory->list.array[i];
            etsi_its_msgs::EventPoint point;
            assignDeltaReferencePosition(point.event_position, asn1_point->eventPosition);
            assignPathDeltaTime(point.event_delta_time, asn1_point->eventDeltaTime);
            point.information_quality.value = asn1_point->informationQuality;
            msg.event_history.emplace_back(std::move(point));
        }
    }
}

void assignLocationContainer(etsi_its_msgs::LocationContainer& msg, const LocationContainer& asn1)
{
    if (asn1.eventSpeed)
    {
        msg.event_speed.value = asn1.eventSpeed->speedValue;
        msg.event_speed.confidence = asn1.eventSpeed->speedConfidence;
    }
    else
    {
        msg.event_speed.value = etsi_its_msgs::Speed::VALUE_UNAVAILABLE;
        msg.event_speed.confidence = etsi_its_msgs::Speed::CONFIDENCE_UNAVAILABLE;
    }

    if (asn1.eventPositionHeading)
    {
        msg.event_position_heading.value = asn1.eventPositionHeading->headingValue;
        msg.event_position_heading.confidence = asn1.eventPositionHeading->headingConfidence;
    }
    else
    {
        msg.event_position_heading.value = etsi_its_msgs::Heading::VALUE_UNAVAILABLE;
        msg.event_position_heading.confidence = etsi_its_msgs::Heading::CONFIDENCE_UNAVAILABLE;
    }

    for (int i = 0; i < asn1.traces.list.count; ++i)
    {
        const PathHistory* asn1_history = asn1.traces.list.array[i];
        etsi_its_msgs::PathHistory history;
        for (int j = 0; j < asn1_history->list.count; ++j)
        {
            const PathPoint* asn1_point = asn1_history->list.array[i];
            etsi_its_msgs::PathPoint point;
            assignDeltaReferencePosition(point.path_position, asn1_point->pathPosition);
            assignPathDeltaTime(point.path_delta_time, asn1_point->pathDeltaTime);
            history.points.emplace_back(std::move(point));
        }
        msg.traces.emplace_back(std::move(history));
    }

    if (asn1.roadType)
    {
        msg.road_type = *asn1.roadType;
    }
    else
    {
        msg.road_type = etsi_its_msgs::LocationContainer::ROAD_TYPE_UNAVAILABLE;
    }
}

void assignDeltaReferencePosition(etsi_its_msgs::DeltaReferencePosition& msg, const DeltaReferencePosition& asn1)
{
    msg.delta_latitude = asn1.deltaLatitude;
    msg.delta_longitude = asn1.deltaLongitude;
    msg.delta_altitude = asn1.deltaAltitude;
}

void assignPathDeltaTime(etsi_its_msgs::PathDeltaTime& msg, const PathDeltaTime_t* asn1)
{
    if (asn1)
    {
        msg.value = *asn1;
    }
    else
    {
        msg.value = etsi_its_msgs::PathDeltaTime::UNAVAILABLE;
    }
}

} // namespace vanetza
