#ifndef DEN_TX_NODELET_HPP_5RSQZ6HQ
#define DEN_TX_NODELET_HPP_5RSQZ6HQ

#include "message_table.hpp"
#include <boost/optional/optional.hpp>
#include <nodelet/nodelet.h>
#include <ros/service.h>
#include <ros/subscriber.h>
#include <ros/timer.h>
#include <cstdint>
#include <vector>

namespace etsi_its_msgs
{
ROS_DECLARE_MESSAGE(DENM);
} // namespace etsi_its_msgs

namespace vanetza
{

ROS_DECLARE_MESSAGE(BtpDataResponse);
ROS_DECLARE_MESSAGE(DenmTxRequest);
ROS_DECLARE_MESSAGE(DenmTxResponse);

namespace asn1
{
class Denm;
} // namespace asn1

class DenTxNodelet : public nodelet::Nodelet
{
public:
    void onInit() override;
    bool callback(DenmTxRequest&, DenmTxResponse&);

private:
    BtpDataResponse transmitMessage(const DenmTxRequest&, std::vector<std::uint8_t>&&);
    vanetza::asn1::Denm convertMessage(const etsi_its_msgs::DENM&) const;

    boost::optional<std::uint32_t> station_id_;
    std::uint16_t sequence_number_;
    MessageTable message_table_;
    ros::ServiceClient btp_service_;
    ros::ServiceServer den_service_;
    ros::Timer purge_timer_;
    ros::Subscriber sub_id_change_;
};

} // namespace vanetza

#endif /* DEN_TX_NODELET_HPP_5RSQZ6HQ */

