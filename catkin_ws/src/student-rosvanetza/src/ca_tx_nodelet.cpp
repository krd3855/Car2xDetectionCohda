#include "ca_message.hpp"
#include "ca_tx_nodelet.hpp"
#include "tai_clock.hpp"
#include "vanetza/BtpData.h"
#include "vanetza/SecurityIdChange.h"
#include <boost/algorithm/string.hpp>
#include <boost/math/constants/constants.hpp>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>
#include <vanetza/asn1/cam.hpp>
#include <vanetza/btp/ports.hpp>
#include <vanetza/common/its_aid.hpp>
#include <vanetza/facilities/cam_functions.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>

PLUGINLIB_EXPORT_CLASS(vanetza::CaTxNodelet, nodelet::Nodelet)

namespace vanetza
{

CaTxNodelet::CaTxNodelet() :
    cam_rate_min_(0.1), cam_rate_max_(1.0), cam_rate_(cam_rate_max_),
    cam_rate_counter_max_(3), cam_rate_counter_(cam_rate_counter_max_)
{
}

void CaTxNodelet::onInit()
{
    ros::NodeHandle& nh = getNodeHandle();

    sub_cam_provider_ = nh.subscribe<etsi_its_msgs::CAM>("cam_provided", 10,
        [this](etsi_its_msgs::CAMConstPtr cam) {
            checkTriggers(cam);
        });
    sub_id_change_ = nh.subscribe<vanetza::SecurityIdChange>("id_change", 3,
        [this](vanetza::SecurityIdChangeConstPtr change) {
            if (change->phase == vanetza::SecurityIdChange::PHASE_COMMIT)
            {
                station_id_ = change->id[4] << 24 | change->id[5] << 16 | change->id[6] << 8 | change->id[7];
                // TODO drop path history
            }
        });
    pub_cam_transmitter_ = nh.advertise<etsi_its_msgs::CAM>("cam_transmitted", 10);

    client_data_request_ = nh.serviceClient<vanetza::BtpData>("btp_request");
    NODELET_INFO("Cooperative Awareness transmitter nodelet initialised");
}

void CaTxNodelet::checkTriggers(etsi_its_msgs::CAMConstPtr cam)
{
    if (!station_id_)
    {
        NODELET_WARN_THROTTLE(1, "No station id assigned, skip CAM transmission");
        return;
    }
    else if (!client_data_request_.exists())
    {
        NODELET_WARN_THROTTLE(1, "BTP-DATA.request service is not available, skip CAM transmission");
        return;
    }

    auto heading_trigger = [this, cam]()
    {
        using etsi_its_msgs::Heading;
        using vanetza::units::Angle;
        using vanetza::units::degree;

        static const Angle delta_heading { 4.0 * degree };
        const Heading& prev_cam_heading = cam_transmission_->high_frequency_container.heading;
        const Heading& curr_cam_heading = cam->high_frequency_container.heading;

        if (prev_cam_heading.value != Heading::VALUE_UNAVAILABLE && curr_cam_heading.value != Heading::VALUE_UNAVAILABLE)
        {
            const Angle prev_heading { prev_cam_heading.value / 10.0 * degree };
            const Angle curr_heading { curr_cam_heading.value / 10.0 * degree };
            return !vanetza::facilities::similar_heading(prev_heading, curr_heading, delta_heading);
        }
        else
        {
            return false;
        }
    };

    auto speed_trigger = [this, cam]()
    {
        using etsi_its_msgs::Speed;
        static const double delta_speed = 0.5;
        const Speed& prev_cam_speed = cam_transmission_->high_frequency_container.speed;
        const Speed& curr_cam_speed = cam->high_frequency_container.speed;

        if (prev_cam_speed.value != Speed::VALUE_UNAVAILABLE && curr_cam_speed.value != Speed::VALUE_UNAVAILABLE)
        {
            const double prev_speed = prev_cam_speed.value / 100.0;
            const double curr_speed = curr_cam_speed.value / 100.0;
            return std::abs(prev_speed - curr_speed) > delta_speed;
        }
        else
        {
            return false;
        }
    };

    auto position_trigger = [this, cam]()
    {
        static const double delta_dist = 4.0;

        // calculate distance using haversine formula
        namespace mdc = boost::math::double_constants;
        static const double r = 6378137.0; // equatorial earth radius of WGS84
        const double lat1 = 1e-7 * cam_transmission_->reference_position.latitude * mdc::degree;
        const double lon1 = 1e-7 * cam_transmission_->reference_position.longitude * mdc::degree;
        const double lat2 = 1e-7 * cam->reference_position.latitude * mdc::degree;
        const double lon2 = 1e-7 * cam->reference_position.longitude * mdc::degree;
        const double i = std::sin(0.5 * (lat2 - lat1));
        const double j = std::sin(0.5 * (lon2 - lon1));
        const double dist = 2.0 * r * std::asin(std::sqrt(i * i + std::cos(lat1) * std::cos(lat2) * j *j ));
        return dist > delta_dist;
    };

    ros::Duration cam_elapsed = ros::Time::now() - last_cam_transmission_;
    // TODO check DCC transmission interval
    if (!cam_transmission_ || heading_trigger() || speed_trigger() || position_trigger())
    {
        transmitMessage(generateMessage(cam));
        cam_rate_ = std::min(cam_elapsed, cam_rate_max_);
        cam_rate_counter_ = 1;

    }
    else if (cam_elapsed >= cam_rate_)
    {
        transmitMessage(generateMessage(cam));
        if (++cam_rate_counter_ > cam_rate_counter_max_)
        {
            cam_rate_ = cam_rate_max_;
        }
    }
}

void CaTxNodelet::transmitMessage(etsi_its_msgs::CAMConstPtr cam)
{
    using DataRequest = vanetza::BtpDataRequest;
    vanetza::BtpData btp_data;
    DataRequest& request = btp_data.request;
    request.btp_type = DataRequest::BTP_TYPE_B;
    request.destination_port = vanetza::btp::ports::CAM.host();
    request.destination_port_info = 0;
    request.transport_type = DataRequest::TRANSPORT_TYPE_SHB;
    request.destination.type = vanetza::GeoNetDestination::TYPE_NONE;
    request.its_aid = vanetza::aid::CA;
    request.traffic_class.store_carry_forwarding = false;
    request.traffic_class.channel_offloading = false;
    request.traffic_class.id = vanetza::TrafficClass::ID_DCC_DP2;

    try
    {
        asn1::Cam asn1_cam = convertCam(cam);
        request.data = asn1_cam.encode();
    }
    catch (const std::exception& e)
    {
        NODELET_ERROR("Encoding CAM failed: %s", e.what());
        return;
    }

    if (client_data_request_.call(btp_data))
    {
        if (btp_data.response.confirm == vanetza::BtpDataResponse::CONFIRM_ACCEPTED)
        {
            const auto now = ros::Time::now();
            cam_transmission_ = cam;
            pub_cam_transmitter_.publish(cam_transmission_);

            // debug output
            if (last_cam_transmission_.isValid())
            {
                NODELET_DEBUG("CAM update generated with dt = %f", (now - last_cam_transmission_).toSec());
            }

            // update time stamps when containers have been transmitted the last time
            last_cam_transmission_ = now;
            if (cam_transmission_->has_low_frequency_container)
            {
                last_low_frequency_container_ = now;
            }
        }
        else
        {
            NODELET_ERROR("CAM transmission has been rejected by BTP-DATA.request");
        }
    }
    else
    {
        NODELET_ERROR("Passing CAM to BTP-DATA.request service failed");
    }
}

etsi_its_msgs::CAMPtr CaTxNodelet::generateMessage(etsi_its_msgs::CAMConstPtr provided) const
{
    etsi_its_msgs::CAM::Ptr cam = boost::make_shared<etsi_its_msgs::CAM>(*provided);

    // add low frequency container just every 500 ms
    const ros::Time now = ros::Time::now();
    static const ros::Duration low_frequency_interval { 0.5 };
    if (cam->has_low_frequency_container) {
        if (last_low_frequency_container_ + low_frequency_interval > now) {
            cam->has_low_frequency_container = false;
        }
    }

    // overwrite ITS header no matter what we have been provided with
    using etsi_its_msgs::ItsPduHeader;
    cam->its_header.protocol_version = 2; // as per EN 302 637-2 v1.4.1
    cam->its_header.message_id = ItsPduHeader::MESSAGE_ID_CAM;
    cam->its_header.station_id = station_id_.value_or(0);

    // TODO maybe we should use cam_provision_'s header.stamp instead?
    const auto tai_now = time_point_cast<vanetza::Clock>(tai_clock::now());
    const auto tai_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(tai_now.time_since_epoch());
    cam->generation_delta_time = tai_since_epoch.count();

    return cam;
}

} // namespace vanetza
