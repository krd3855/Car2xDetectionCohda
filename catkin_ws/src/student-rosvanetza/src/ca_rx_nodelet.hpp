#ifndef CA_RX_NODELET_HPP_ALRJUMFH
#define CA_RX_NODELET_HPP_ALRJUMFH

#include <nodelet/nodelet.h>
#include <ros/message_forward.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <vanetza/asn1/cam.hpp>
#include <cstdint>

namespace vanetza
{

ROS_DECLARE_MESSAGE(BtpDataIndication);

class CaRxNodelet : public nodelet::Nodelet
{
public:
    void onInit() override;

private:
    void onIndication(BtpDataIndicationConstPtr);
    void publish(const vanetza::asn1::Cam&);

    uint16_t port_;
    ros::Subscriber sub_btp_;
    ros::Publisher pub_cam_;
};

} // namespace vanetza

#endif /* CA_RX_NODELET_HPP_ALRJUMFH */

