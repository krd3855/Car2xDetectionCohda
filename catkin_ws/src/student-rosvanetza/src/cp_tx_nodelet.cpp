#include "cp_message.hpp"
#include "cp_tx_nodelet.hpp"
#include "ca_tx_nodelet.hpp"
#include <etsi_its_msgs/CPM.h>
#include "tai_clock.hpp"
#include "vanetza/BtpData.h"
#include "vanetza/SecurityIdChange.h"
#include <boost/algorithm/string.hpp>
#include <boost/math/constants/constants.hpp>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>
#include <vanetza/asn1/cpm.hpp>
#include <vanetza/asn1/cam.hpp>
#include <vanetza/btp/ports.hpp>
#include <vanetza/common/its_aid.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>


PLUGINLIB_EXPORT_CLASS(vanetza::CpTxNodelet, nodelet::Nodelet)

namespace vanetza
{

CpTxNodelet::CpTxNodelet() :
    cpm_rate_min_(1), cpm_rate_max_(0.1), cpm_rate_(cpm_rate_max_),
    cpm_rate_counter_max_(3), cpm_rate_counter_(cpm_rate_counter_max_)
{
}


void CpTxNodelet::onInit()
{
    ros::NodeHandle& nh = getNodeHandle();

    sub_cpm_provider_ = nh.subscribe<etsi_its_msgs::CPM>("cpm_provided", 10,
                                                         [this](etsi_its_msgs::CPMConstPtr cpm) {
                                                             checkTriggers(cpm);
                                                         });
    sub_id_change_ = nh.subscribe<vanetza::SecurityIdChange>("id_change", 3,
                                                             [this](vanetza::SecurityIdChangeConstPtr change) {
                                                                 if (change->phase == vanetza::SecurityIdChange::PHASE_COMMIT)
                                                                 {
                                                                     station_id_ = change->id[4] << 24 | change->id[5] << 16 | change->id[6] << 8 | change->id[7];
                                                                     // TODO drop path history
                                                                 }
                                                             });
    pub_cpm_transmitter_ = nh.advertise<etsi_its_msgs::CPM>("cpm_transmitted", 10);

    client_data_request_ = nh.serviceClient<vanetza::BtpData>("btp_request");
    NODELET_INFO("Collective Perception transmitter nodelet initialised");
}


void CpTxNodelet::checkTriggers(etsi_its_msgs::CPMConstPtr cpm)
{
    if (!station_id_)
    {
        NODELET_WARN_THROTTLE(1, "No station id assigned, skip CPM transmission");
        return;
    }
    else if (!client_data_request_.exists())
    {
        NODELET_WARN_THROTTLE(1, "BTP-DATA.request service is not available, skip CPM transmission");
        return;
    }

    ros::Duration cpm_elapsed = ros::Time::now() - last_cpm_transmission_;
    // TODO check DCC transmission interval
    if (cpm_elapsed >= cpm_rate_)
    {
        transmitMessage(generateMessage(cpm));
    }
}


void CpTxNodelet::transmitMessage(etsi_its_msgs::CPMConstPtr cpm)
{
    using DataRequest = vanetza::BtpDataRequest;
    vanetza::BtpData btp_data;
    DataRequest& request = btp_data.request;
    request.btp_type = DataRequest::BTP_TYPE_B;
    request.destination_port = vanetza::btp::ports::CPM.host();
    request.destination_port_info = 0;
    request.transport_type = DataRequest::TRANSPORT_TYPE_SHB;
    request.destination.type = vanetza::GeoNetDestination::TYPE_NONE;
    request.its_aid = vanetza::aid::CP;
    request.traffic_class.store_carry_forwarding = false;
    request.traffic_class.channel_offloading = false;
    request.traffic_class.id = vanetza::TrafficClass::ID_DCC_DP2;

    try
    {
        asn1::Cpm asn1_cpm = convertCpm(cpm);
        request.data = asn1_cpm.encode();
    }
    catch (const std::exception& e)
    {
        NODELET_ERROR("Encoding CPM failed: %s", e.what());
        return;
    }

    if (client_data_request_.call(btp_data))
    {
        if (btp_data.response.confirm == vanetza::BtpDataResponse::CONFIRM_ACCEPTED)
        {
            const auto now = ros::Time::now();
            cpm_transmission_ = cpm;
            pub_cpm_transmitter_.publish(cpm_transmission_);

            // debug output
            if (last_cpm_transmission_.isValid())
            {
                NODELET_DEBUG("CPM update generated with dt = %f", (now - last_cpm_transmission_).toSec());
            }

            // update time stamps when containers have been transmitted the last time
            last_cpm_transmission_ = now;
        }
        else
        {
            NODELET_ERROR("CPM transmission has been rejected by BTP-DATA.request");
        }
    }
    else
    {
        NODELET_ERROR("Passing CPM to BTP-DATA.request service failed");
    }
}


etsi_its_msgs::CPMPtr CpTxNodelet::generateMessage(etsi_its_msgs::CPMConstPtr provided) const
{
    etsi_its_msgs::CPM::Ptr cpm = boost::make_shared<etsi_its_msgs::CPM>(*provided);

    // overwrite ITS header no matter what we have been provided with
    using etsi_its_msgs::ItsPduHeader;
    cpm->its_header.protocol_version = 2; // as per EN 302 637-2 v1.4.1
    cpm->its_header.message_id = 20;//ItsPduHeader__messageID_cpm;
    cpm->its_header.station_id = station_id_.value_or(0);

    // TODO maybe we should use cam_provision_'s header.stamp instead?
    const auto tai_now = time_point_cast<vanetza::Clock>(tai_clock::now());
    const auto tai_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(tai_now.time_since_epoch());
    cpm->generation_delta_time = tai_since_epoch.count();

    return cpm;
}

} // namespace vanetza
