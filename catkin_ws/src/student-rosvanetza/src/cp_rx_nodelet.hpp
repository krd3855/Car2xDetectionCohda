#ifndef CP_RX_NODELET_HPP_ALRJUMFH
#define CP_RX_NODELET_HPP_ALRJUMFH

#include <nodelet/nodelet.h>
#include <ros/message_forward.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <vanetza/asn1/cpm.hpp>
#include <cstdint>

namespace vanetza
{

ROS_DECLARE_MESSAGE(BtpDataIndication);

class CpRxNodelet : public nodelet::Nodelet
{
public:
    void onInit() override;

private:
    void onIndication(BtpDataIndicationConstPtr);
    void publish(const vanetza::asn1::Cpm&);

    uint16_t port_;
    ros::Subscriber sub_btp_;
    ros::Publisher pub_cpm_;
};

} // namespace vanetza

#endif /* CP_RX_NODELET_HPP_ALRJUMFH */

