#include "cp_message.hpp"
#include "cp_rx_nodelet.hpp"
#include "vanetza/BtpDataIndication.h"
#include <etsi_its_msgs/CPM.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/node_handle.h>
#include <vanetza/btp/ports.hpp>

PLUGINLIB_EXPORT_CLASS(vanetza::CpRxNodelet, nodelet::Nodelet)

namespace vanetza
{

void CpRxNodelet::onInit()
{
    auto& nh = getNodeHandle();
    auto& nhp = getPrivateNodeHandle();
    port_ = nhp.param<int>("port", btp::ports::CPM.host());
    sub_btp_ = nh.subscribe<BtpDataIndication>("btp_indication", 20, boost::bind(&CpRxNodelet::onIndication, this, _1));
    pub_cpm_ = nh.advertise<etsi_its_msgs::CPM>("cpm_received", 20);
}

void CpRxNodelet::onIndication(BtpDataIndicationConstPtr indication)
{
    if (indication->btp_type == BtpDataIndication::BTP_TYPE_B && indication->destination_port == port_)
    {
        vanetza::asn1::Cpm cpm;
        if (cpm.decode(indication->data))
        {
            publish(cpm);
        }
        else
        {
            publish(cpm);
            std::cout << "Decoding CPM failed" << std::endl;
            NODELET_DEBUG("Dropped received CPM: ASN.1 decoding failed");
        }
    }
}

void CpRxNodelet::publish(const vanetza::asn1::Cpm& asn1)
{
    std::string error_msg;
    auto msg = convert_cpm(asn1, &error_msg);

    if (msg)
    {
        pub_cpm_.publish(msg);
    }
    else
    {
        pub_cpm_.publish(msg);
        std::cout << "Unable to publish CPM received" << std::endl;
        NODELET_DEBUG("Dropped received CPM: %s", error_msg.c_str());
    }
}

} // namespace vanetza
