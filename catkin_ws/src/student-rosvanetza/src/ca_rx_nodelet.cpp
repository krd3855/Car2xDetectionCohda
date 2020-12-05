#include "ca_message.hpp"
#include "ca_rx_nodelet.hpp"
#include "vanetza/BtpDataIndication.h"
#include <etsi_its_msgs/CAM.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/node_handle.h>
#include <vanetza/btp/ports.hpp>

PLUGINLIB_EXPORT_CLASS(vanetza::CaRxNodelet, nodelet::Nodelet)

namespace vanetza
{

void CaRxNodelet::onInit()
{
    auto& nh = getNodeHandle();
    auto& nhp = getPrivateNodeHandle();
    port_ = nhp.param<int>("port", btp::ports::CAM.host());
    sub_btp_ = nh.subscribe<BtpDataIndication>("btp_indication", 20, boost::bind(&CaRxNodelet::onIndication, this, _1));
    pub_cam_ = nh.advertise<etsi_its_msgs::CAM>("cam_received", 20);
}

void CaRxNodelet::onIndication(BtpDataIndicationConstPtr indication)
{
    if (indication->btp_type == BtpDataIndication::BTP_TYPE_B && indication->destination_port == port_)
    {
        vanetza::asn1::Cam cam;
        if (cam.decode(indication->data))
        {
            publish(cam);
        }
        else
        {
            NODELET_DEBUG("Dropped received CAM: ASN.1 decoding failed");
        }
    }
}

void CaRxNodelet::publish(const vanetza::asn1::Cam& asn1)
{
    std::string error_msg;
    auto msg = convertCam(asn1, &error_msg);

    if (msg)
    {
        pub_cam_.publish(msg);
    }
    else
    {
        NODELET_DEBUG("Dropped received CAM: %s", error_msg.c_str());
    }
}

} // namespace vanetza
