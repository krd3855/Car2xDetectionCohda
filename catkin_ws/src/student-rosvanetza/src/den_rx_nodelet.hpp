#ifndef DEN_RX_NODELET_HPP_UEPX34FR
#define DEN_RX_NODELET_HPP_UEPX34FR

#include <nodelet/nodelet.h>
#include <ros/message_forward.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <vanetza/asn1/denm.hpp>
#include <cstdint>

namespace vanetza
{

ROS_DECLARE_MESSAGE(BtpDataIndication);

class DenRxNodelet : public nodelet::Nodelet
{
public:
    void onInit() override;

private:
    void onIndication(BtpDataIndicationConstPtr);
    void publish(const vanetza::asn1::Denm&);

    uint16_t port_;
    ros::Subscriber sub_btp_;
    ros::Publisher pub_denm_;
};

} // namespace vanetza

#endif /* DEN_RX_NODELET_HPP_UEPX34FR */
